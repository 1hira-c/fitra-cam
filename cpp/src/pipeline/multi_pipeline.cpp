#include "pipeline/multi_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

#include "pipeline/lifecycle_filter_history.hpp"
#include "util/logging.hpp"

namespace fitra::pipeline {

namespace {

constexpr std::size_t kSyncQueueDepth = 6;
// This is a lifecycle debounce, not a synchronization tolerance. At 30 fps it
// allows roughly three missing cross-camera opportunities before invalidating a
// track, while the configured sync_window_ms remains the quality gate.
constexpr double kSyncLossTimeoutMs = 100.0;

void populate_floor_stats(Skeleton3DStats& stats,
                          bool enabled,
                          double floor_z_m,
                          const lift::FloorContactReport& report,
                          bool fresh) {
    stats.floor_stability_enabled = enabled;
    stats.floor_z_m = floor_z_m;
    stats.floor_contact_fresh = enabled && fresh;
    stats.floor_contact_left = report.feet[0].contact;
    stats.floor_contact_right = report.feet[1].contact;
    stats.floor_evidence_left = fresh && report.feet[0].evidence_valid;
    stats.floor_evidence_right = fresh && report.feet[1].evidence_valid;
    for (std::size_t side = 0; side < report.feet.size(); ++side) {
        stats.floor_corrections_m[side] = report.feet[side].correction_m;
    }
    stats.floor_correction_left_m = cv::norm(stats.floor_corrections_m[0]);
    stats.floor_correction_right_m = cv::norm(stats.floor_corrections_m[1]);
}

}  // namespace

MultiCameraDriver::MultiCameraDriver(
    std::vector<std::unique_ptr<camera::FrameSource>> sources,
    infer::RtmPose& rtmpose,
    SnapshotBus& bus)
    : MultiCameraDriver(std::move(sources), rtmpose, bus, ThreeDConfig{}) {}

MultiCameraDriver::MultiCameraDriver(
    std::vector<std::unique_ptr<camera::FrameSource>> sources,
    infer::RtmPose& rtmpose,
    SnapshotBus& bus,
    ThreeDConfig threed)
    : sources_{std::move(sources)},
      rtmpose_{rtmpose},
      bus_{bus},
      threed_{threed},
      kalman_{},
      ik_{[&]() {
          lift::IkSolver::Options opts;
          opts.bone_calib_frames = std::max(1, threed_.bone_calib_frames);
          opts.subject_height_m = threed_.subject_height_m;
          opts.has_subject_profile = threed_.has_subject_profile;
          opts.subject_profile = threed_.subject_profile;
          return lift::IkSolver{opts};
      }()},
      floor_contact_{threed_.floor_contact},
      latest_per_cam_(sources_.size()),
      sync_queue_(sources_.size(), kSyncQueueDepth),
      per_cam_(sources_.size()) {
    for (const auto& source : sources_) {
        if (!source || !source->prebakes_pose()) {
            throw std::invalid_argument(
                "MultiCameraDriver requires FrameSource with RTMPose prebaking enabled");
        }
    }
    for (auto& source : sources_) source->set_ready_signal(&ready_signal_);
    if ((threed_.triangulator || threed_.bus) && (!threed_.triangulator || !threed_.bus)) {
        throw std::invalid_argument("3D pipeline requires both triangulator and Skeleton3DBus");
    }
    if (threed_.fusion_pose && !threed_.pose_gate) {
        throw std::invalid_argument(
            "fusion pose bus requires the PoseGate lifecycle owner");
    }
    if (threed_.tracker_axis_lineage && !threed_.fusion_pose) {
        throw std::invalid_argument(
            "tracker axis lineage requires the FusionPose lifecycle source");
    }
    if (threed_.triangulator && threed_.bus) {
        FITRA_LOG_INFO(
            "3D floor-contact stability {} (floor_z={}m enter={}m/{}mps exit={}m/{}mps grace={}s)",
            threed_.floor_contact_stability ? "ENABLED" : "disabled",
            threed_.floor_contact.floor_z_m,
            threed_.floor_contact.enter_height_m,
            threed_.floor_contact.enter_speed_mps,
            threed_.floor_contact.exit_height_m,
            threed_.floor_contact.exit_speed_mps,
            threed_.floor_contact.exit_grace_s);
    }
}

MultiCameraDriver::~MultiCameraDriver() {
    try { stop(); } catch (...) {}
}

void MultiCameraDriver::start() {
    for (auto& s : sources_) s->start();
    stop_.store(false);
    worker_ = std::thread{&MultiCameraDriver::loop, this};
    FITRA_LOG_INFO("multi-camera driver started ({} cameras)", sources_.size());
}

void MultiCameraDriver::set_frame_tap(FrameTapFn fn) {
    std::lock_guard<std::mutex> g(tap_mu_);
    frame_tap_ = std::move(fn);
}

void MultiCameraDriver::set_skeleton3d_tap(Skeleton3DTapFn fn) {
    std::lock_guard<std::mutex> g(tap_mu_);
    skeleton3d_tap_ = std::move(fn);
}

void MultiCameraDriver::set_idle_gate(const std::atomic<bool>* idle_flag,
                                      double idle_tick_hz) {
    idle_flag_    = idle_flag;
    idle_tick_hz_ = idle_tick_hz;
}

void MultiCameraDriver::stop() {
    if (!worker_.joinable() && sources_.empty()) return;
    stop_.store(true);
    // The loop may be parked on the aggregate ready signal; wake it so stop_ is
    // observed without waiting out the timeout. Order: flag -> wake -> join.
    ready_signal_.wake();
    if (worker_.joinable()) worker_.join();
    for (auto& s : sources_) s->stop();
}

void MultiCameraDriver::loop() {
    struct PendingCam {
        std::size_t   idx;
        std::size_t   person_offset;
        std::size_t   person_count;
    };
    std::vector<PendingCam>                       pending;
    std::vector<infer::RtmPose::PrebakedRequest>  reqs;

    // Rolling stage breakdown (debug aid; prints every ~3s of work).
    int    iter_count = 0;
    double sum_poll_ms = 0.0, sum_rtm_ms = 0.0, sum_snap_ms = 0.0;
    int    sum_reqs = 0;
    // E2E per-stage frame-age accumulators (capture-relative, summed per
    // pending camera-frame; divided by sum_frames at flush). Distinct from
    // the central thread's own poll/rtm/snap wall time above.
    double sum_cap_dec = 0.0, sum_dec_det = 0.0, sum_det_bake = 0.0;
    double sum_bake_pose = 0.0, sum_pose_pub = 0.0, sum_cap_pub = 0.0;
    long   sum_frames = 0;
    auto   stats_anchor = std::chrono::steady_clock::now();
    std::vector<bool> warned_missing_prebake(sources_.size(), false);
    const std::size_t rtmpose_per_item =
        infer::RtmPose::blob_floats_per_item(rtmpose_.options());
    camera::FrameReadySignal::Ticket ready_ticket = 0;

    while (!stop_.load()) {
        // Block for any camera before scanning all slots. Publications that
        // accumulated while RTMPose/3D was busy advance the shared generation,
        // so the next wait returns immediately and active operation needs one
        // scan per batch rather than an extra empty scan before every wait.
        ready_ticket = ready_signal_.wait(
            ready_ticket, stop_, std::chrono::milliseconds(100));
        if (stop_.load()) break;
        pending.clear();
        reqs.clear();
        auto iter_start = std::chrono::steady_clock::now();

        // One callback snapshot per batch. The old code locked/copy-constructed
        // the same std::function once per ready camera (three mutex round-trips
        // for a typical synchronized batch) even when the tap was empty.
        FrameTapFn frame_tap_local;
        {
            std::lock_guard<std::mutex> g(tap_mu_);
            frame_tap_local = frame_tap_;
        }

        // Idle/standby gate (issue #37). While idle the heavy 3D update is
        // skipped and the loop throttles to idle_tick_hz; cameras + decode stay
        // warm (frames keep arriving and are dropped-old), so resume is the
        // next frame. The flag is a hint that flips a few times a second.
        const bool idle =
            idle_flag_ && idle_flag_->load(std::memory_order_relaxed);
        if (idle != idle_active_) {
            handle_idle_transition(idle);
            idle_active_ = idle;
        }

        // Pass 1: pull the latest (frame, bboxes) from each FrameSource.
        // Decode + YOLOX already ran in the per-camera worker thread.
        for (std::size_t i = 0; i < sources_.size(); ++i) {
            if (stop_.load()) break;
            if (!sources_[i]->try_pop_latest_decoded(latest_per_cam_[i])) continue;

            // Invoke without the lock held -- the user callback may take its
            // own mutex or re-enter set_frame_tap().
            if (frame_tap_local) {
                auto now_tap = std::chrono::steady_clock::now();
                if (!loop_t0_set_) {
                    loop_t0_ = now_tap;
                    loop_t0_set_ = true;
                }
                double ts_ms = std::chrono::duration<double, std::milli>(
                                  latest_per_cam_[i].captured_at - loop_t0_).count();
                frame_tap_local(i, latest_per_cam_[i].bgr, ts_ms);
            }

            PendingCam pc;
            pc.idx           = i;
            pc.person_offset = reqs.size();
            pc.person_count  = 0;
            // FrameSource has already pre-baked the RTMPose inputs.
            const auto& cam_df = latest_per_cam_[i];
            if (!cam_df.has_prebaked_pose_inputs(rtmpose_per_item)) {
                if (!warned_missing_prebake[i]) {
                    FITRA_LOG_WARN(
                        "camera {} has {} bboxes but invalid RTMPose prebaked buffers; "
                        "skipping pose inference for these frames",
                        i, cam_df.bboxes.size());
                    warned_missing_prebake[i] = true;
                }
                pending.push_back(pc);
                continue;
            }
            pc.person_count = cam_df.bboxes.size();
            // All-GPU front-end: when the worker produced a device CHW buffer,
            // point the request at device memory (TRT is fed via D2D, no H2D);
            // otherwise use the host CPU-prebake blob.
            const bool dev = static_cast<bool>(cam_df.chw_dev);
            for (std::size_t bi = 0; bi < cam_df.bboxes.size(); ++bi) {
                infer::RtmPose::PrebakedRequest pr;
                if (dev)
                    pr.chw_dev = cam_df.chw_dev->ptr + bi * rtmpose_per_item;
                else
                    pr.chw     = cam_df.chw_concat.data() + bi * rtmpose_per_item;
                pr.M_inv = cam_df.M_invs[bi];
                pr.bbox  = cam_df.bboxes[bi];
                reqs.push_back(pr);
            }
            pending.push_back(pc);
        }

        if (pending.empty()) {
            // A timeout/spurious wake can legitimately find no work. Return to
            // the aggregate wait without a fixed polling sleep. The 3D matcher
            // is also given this timer tick so a fully stopped camera can cross
            // its one-shot loss boundary even when no new frame arrives.
            if (!idle) {
                maybe_update_3d(std::chrono::steady_clock::now(),
                                std::chrono::system_clock::now());
            }
            continue;
        }
        auto t_after_poll = std::chrono::steady_clock::now();

        // Pass 2: one batched RTMPose call across all cameras' bboxes.
        // Preprocess already ran on per-camera worker threads — this is
        // just memcpy + GPU enqueue + sync + SimCC decode.
        std::vector<infer::Person> all_persons;
        if (!idle && !reqs.empty()) {
            all_persons = rtmpose_.infer_prebaked(reqs);
        }
        auto t_after_rtm = std::chrono::steady_clock::now();

        // Pass 3: distribute + update snapshot bus.
        auto wall_now = std::chrono::system_clock::now();
        auto now      = std::chrono::steady_clock::now();
        for (const auto& pc : pending) {
            auto& cs       = per_cam_[pc.idx];
            const auto& df = latest_per_cam_[pc.idx];
            update_stats(cs, now, df.captured_at);

            // E2E per-stage deltas for this frame. t_pose = t_after_rtm (batched
            // RTMPose finished), t_publish = now. bake->pose folds in the slot
            // wait + central poll, which is exactly the inter-thread latency.
            sum_cap_dec   += std::chrono::duration<double, std::milli>(df.t_decode  - df.captured_at).count();
            sum_dec_det   += std::chrono::duration<double, std::milli>(df.t_detect  - df.t_decode).count();
            sum_det_bake  += std::chrono::duration<double, std::milli>(df.t_prebake - df.t_detect).count();
            sum_bake_pose += std::chrono::duration<double, std::milli>(t_after_rtm  - df.t_prebake).count();
            sum_pose_pub  += std::chrono::duration<double, std::milli>(now          - t_after_rtm).count();
            sum_cap_pub   += std::chrono::duration<double, std::milli>(now          - df.captured_at).count();
            ++sum_frames;

            CameraSnapshot snap;
            snap.id  = static_cast<int>(pc.idx);
            snap.w   = sources_[pc.idx]->options().width;
            snap.h   = sources_[pc.idx]->options().height;
            snap.seq = df.seq;
            snap.captured_at = df.captured_at;
            snap.captured_mono_ns = df.captured_mono_ns;
            snap.v4l2_timestamp = df.v4l2_timestamp;
            auto lag = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now - df.captured_at);
            snap.captured_wall = wall_now - lag;
            // Bounds guard: while idle (or any frame RTMPose was skipped)
            // all_persons is empty though person_count was set in Pass 1, so
            // slicing it would read past the end.
            if (pc.person_count > 0 &&
                all_persons.size() >= pc.person_offset + pc.person_count) {
                snap.persons.assign(
                    all_persons.begin() + pc.person_offset,
                    all_persons.begin() + pc.person_offset + pc.person_count);
            }
            snap.bboxes          = df.bboxes;
            snap.recv_fps        = sources_[pc.idx]->recv_fps();
            snap.recent_pose_fps = cs.stats.recent_pose_fps;
            snap.avg_pose_fps    = cs.stats.avg_pose_fps;
            snap.processed       = cs.stats.processed_count;
            std::uint64_t recv = sources_[pc.idx]->total_received();
            snap.pending         = recv > snap.processed ? recv - snap.processed : 0;
            snap.stage_ms        = cs.stats.last_stage_ms;
            bus_.update(snap);
            if (threed_.triangulator && threed_.bus) {
                last_sync_input_at_ = now;
                sync_queue_.push(pc.idx, snap.captured_at, std::move(snap));
            }
        }
        if (!idle) maybe_update_3d(now, wall_now);
        auto t_after_snap = std::chrono::steady_clock::now();

        ++iter_count;
        sum_poll_ms += std::chrono::duration<double, std::milli>(t_after_poll  - iter_start).count();
        sum_rtm_ms  += std::chrono::duration<double, std::milli>(t_after_rtm   - t_after_poll).count();
        sum_snap_ms += std::chrono::duration<double, std::milli>(t_after_snap  - t_after_rtm).count();
        sum_reqs    += static_cast<int>(reqs.size());

        auto elapsed = std::chrono::duration<double>(t_after_snap - stats_anchor).count();
        if (elapsed >= 3.0) {
            double iter_ms = (sum_poll_ms + sum_rtm_ms + sum_snap_ms) / iter_count;
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "breakdown iter_ms=%.2f poll=%.2f rtm=%.2f (per-cam avg %.2f reqs) snap=%.2f",
                          iter_ms,
                          sum_poll_ms / iter_count,
                          sum_rtm_ms  / iter_count,
                          static_cast<double>(sum_reqs) / iter_count,
                          sum_snap_ms / iter_count);
            FITRA_LOG_INFO("{}", buf);
            if (sum_frames > 0) {
                double n = static_cast<double>(sum_frames);
                char ebuf[256];
                std::snprintf(ebuf, sizeof(ebuf),
                              "e2e_ms cap->dec=%.2f dec->det=%.2f det->bake=%.2f "
                              "bake->pose=%.2f pose->pub=%.2f | cap->pub=%.2f",
                              sum_cap_dec / n, sum_dec_det / n, sum_det_bake / n,
                              sum_bake_pose / n, sum_pose_pub / n, sum_cap_pub / n);
                FITRA_LOG_INFO("{}", ebuf);
            }
            iter_count = 0;
            sum_poll_ms = sum_rtm_ms = sum_snap_ms = 0.0;
            sum_reqs = 0;
            sum_cap_dec = sum_dec_det = sum_det_bake = 0.0;
            sum_bake_pose = sum_pose_pub = sum_cap_pub = 0.0;
            sum_frames = 0;
            stats_anchor = t_after_snap;
        }

        // Standby throttle: cap the central loop at idle_tick_hz so we stop
        // spinning on every decoded frame. Cameras + per-camera decode keep
        // running (latest-frame-wins drops the backlog); only this loop sleeps.
        // Split the sleep into short slices so a resume (idle flag cleared) or
        // stop is observed within ~10ms — a single sleep_for(1/tick_hz) would
        // pin the central loop idle for up to 0.5s (or longer at a low
        // idle_tick_hz), breaking the "<100ms / next frame" resume contract.
        if (idle) {
            const double hz = idle_tick_hz_ > 0.1 ? idle_tick_hz_ : 0.1;
            const auto deadline = std::chrono::steady_clock::now() +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(1.0 / hz));
            const auto slice = std::chrono::milliseconds(10);
            while (!stop_.load()
                   && idle_flag_ && idle_flag_->load(std::memory_order_relaxed)) {
                auto now = std::chrono::steady_clock::now();
                if (now >= deadline) break;
                std::this_thread::sleep_for(std::min<std::chrono::steady_clock::duration>(
                    slice, deadline - now));
            }
        }
    }
}

void MultiCameraDriver::update_stats(CamState& cs,
                                     std::chrono::steady_clock::time_point now,
                                     std::chrono::steady_clock::time_point captured_at) {
    ++cs.stats.processed_count;
    cs.recent.push_back(now);
    while (cs.recent.size() > 60) cs.recent.pop_front();
    if (cs.recent.size() >= 2) {
        auto span = std::chrono::duration<double>(cs.recent.back() - cs.recent.front()).count();
        if (span > 0) cs.stats.recent_pose_fps = (cs.recent.size() - 1) / span;
    }
    auto elapsed = std::chrono::duration<double>(now - cs.start_time).count();
    if (elapsed > 0) cs.stats.avg_pose_fps = cs.stats.processed_count / elapsed;
    cs.stats.last_stage_ms = std::chrono::duration<double, std::milli>(now - captured_at).count();
}

void MultiCameraDriver::maybe_update_3d(std::chrono::steady_clock::time_point now,
                                        std::chrono::system_clock::time_point wall_now) {
    std::shared_ptr<lift::Triangulator> triangulator;
    Skeleton3DBus* bus = nullptr;
    PoseGateBus* pose_gate = nullptr;
    FusionPoseBus* fusion_pose = nullptr;
    TrackerAxisLineageBus* lineage_bus = nullptr;
    bool pose_gate_single_subject = true;
    {
        std::lock_guard<std::mutex> g(threed_mu_);
        triangulator = threed_.triangulator;
        bus = threed_.bus;
        pose_gate = threed_.pose_gate;
        fusion_pose = threed_.fusion_pose;
        lineage_bus = threed_.tracker_axis_lineage;
        pose_gate_single_subject = threed_.pose_gate_single_subject;
    }
    if (!triangulator || !bus) return;
    if (sync_queue_.camera_count() < 2) return;

    const auto sync_event = sync_queue_.poll(
        now, threed_.sync_window_ms, kSyncLossTimeoutMs, last_sync_input_at_);
    if (sync_event.kind == SynchronizedFrameEventKind::None) return;

    // Static camera placements (world frame) for the 3D viewer's frustums.
    // Resent on every snapshot (including the one-shot sync-loss boundary) so
    // the markers stay visible.
    std::vector<CameraPose3D> camera_poses;
    {
        const auto& poses = triangulator->camera_poses();
        camera_poses.reserve(poses.size());
        for (const auto& p : poses) {
            CameraPose3D cp;
            cp.id = p.id;
            cp.pos[0] = p.center_w[0];
            cp.pos[1] = p.center_w[1];
            cp.pos[2] = p.center_w[2];
            cp.quat_wxyz[0] = p.quat_wxyz[0];
            cp.quat_wxyz[1] = p.quat_wxyz[1];
            cp.quat_wxyz[2] = p.quat_wxyz[2];
            cp.quat_wxyz[3] = p.quat_wxyz[3];
            camera_poses.push_back(std::move(cp));
        }
    }

    auto populate_common_stats = [&](Skeleton3DStats& stats) {
        stats.enabled = true;
        stats.subject_height_m = threed_.subject_height_m;
        stats.profile_loaded = ik_.profile_loaded();
        stats.subject_id = ik_.subject_id();
        stats.profile_quality_status = ik_.profile_quality_status();
        stats.processed = tri_processed_;
        stats.sync_miss = tri_sync_miss_;
        stats.ik_locked = ik_.locked();
        if (pose_gate) stats.pose_gate = pose_gate->diagnostics();
        populate_floor_stats(stats, threed_.floor_contact_stability,
                             threed_.floor_contact.floor_z_m,
                             last_floor_report_, false);
    };

    if (sync_event.kind == SynchronizedFrameEventKind::UnavailableBoundary) {
        // Keep a short skew in the waiting state. Only this timeout crossing
        // invalidates the downstream lifecycle, and it is emitted once until
        // a fresh synchronized match is acquired.
        ++tri_sync_miss_;
        Skeleton3DSnapshot miss;
        miss.ts = wall_now;
        miss.stats.sync_dt_ms = sync_event.sync_dt_ms.value_or(0.0);
        populate_common_stats(miss.stats);
        miss.cameras = camera_poses;
        if (pose_gate) {
            const auto boundary = pose_gate->publish_unavailable(
                std::nullopt, PoseGateSourceState::Unavailable,
                "sync_miss", sync_event.sync_dt_ms);
            if (fusion_pose) {
                miss.tracker_axis_lineage = make_tracker_axis_lineage(
                    fusion_pose->publish_boundary(boundary));
                if (lineage_bus) {
                    miss.tracker_axis_lineage = lineage_bus->publish(
                        *miss.tracker_axis_lineage);
                }
            }
            miss.stats.pose_gate = pose_gate->diagnostics();
        }
        bus->update(miss);
        return;
    }

    if (sync_event.frames.size() != sync_queue_.camera_count()) return;
    std::chrono::steady_clock::time_point min_ts{};
    std::optional<std::uint64_t> content_mono_ns;
    std::vector<camera::V4l2CaptureTimestamp> fusion_timestamps;
    fusion_timestamps.reserve(sync_event.frames.size());
    bool all_have_content_mono_ns = true;
    bool first = true;
    auto t0 = std::chrono::steady_clock::now();
    std::vector<lift::PerCameraObservation> observations;
    observations.reserve(sync_event.frames.size());
    for (std::size_t i = 0; i < sync_event.frames.size(); ++i) {
        const auto& snap = sync_event.frames[i];
        if (first) {
            min_ts = snap.captured_at;
            first = false;
        } else {
            min_ts = std::min(min_ts, snap.captured_at);
        }
        if (snap.captured_mono_ns == 0) {
            all_have_content_mono_ns = false;
        } else if (!content_mono_ns || snap.captured_mono_ns < *content_mono_ns) {
            content_mono_ns = snap.captured_mono_ns;
        }
        fusion_timestamps.push_back(snap.v4l2_timestamp);
        if (snap.persons.empty()) continue;
        observations.push_back(lift::PerCameraObservation{
            static_cast<int>(i), &snap.persons[0]});
    }
    if (!all_have_content_mono_ns) content_mono_ns.reset();
    const auto fusion_capture =
        make_fusion_capture_interval(fusion_timestamps);

    auto tri = triangulator->triangulate(observations);
    // Keep the lifecycle decision paired with this exact raw triangulation.
    // FusionPose commits only after the post-Kalman/IK skeleton is available,
    // but it still receives `tri` unchanged for raw position/quality fields.
    std::optional<TrackerAxisLineage> tracker_lineage;
    std::optional<PoseGateFrame> lifecycle;
    if (pose_gate) {
        lifecycle = pose_gate->observe(
            tri, content_mono_ns, pose_gate_single_subject,
            sync_event.sync_dt_ms);
    }
    // A non-Fresh PoseGate frame is a destructive lifecycle boundary. Reset
    // before feeding this frame's measurement to Kalman or floor contact so a
    // reacquired/new subject or coordinate epoch cannot inherit the previous
    // lifecycle's joint, contact, anchor, or release-correction state. The
    // boundary frame itself is hidden from FusionPose; its measurement seeds
    // the new state for the following ordinary Fresh frame.
    if (lifecycle) {
        reset_lifecycle_filter_history_if_boundary(
            lifecycle->source_state, kalman_, floor_contact_,
            last_floor_report_, has_last_3d_update_);
    }
    infer::Skeleton3D skel = tri.skeleton;
    double dt_s = 1.0 / 30.0;
    if (has_last_3d_update_) {
        dt_s = std::chrono::duration<double>(now - last_3d_update_).count();
    }
    if (threed_.kalman_enabled) {
        skel = kalman_.update(skel, dt_s);
    }
    // Subject calibration classifies the pose from anatomical joint *angles* on
    // the measured (pre-IK) skeleton: feeding the post-IK skeleton would let a
    // hinge/length clamp manufacture elbow/knee flexion the subject is not
    // actually holding (a straight arm reported as bent). So capture the
    // measured skeleton before ik_.update() mutates it below -- only when a tap
    // is actually listening, to avoid the copy on the normal live path.
    //
    // The drift value paired with it, however, must be the *post-IK*
    // bone_drift_pct, not the pre-IK one. PoseRecognizer gates on
    // max_bone_drift_pct (~10%); the post-IK skeleton is clamped to the model so
    // its drift is ~0 (an effectively lenient gate, which is how the wizard was
    // validated), whereas raw pre-IK triangulation easily drifts >10% and would
    // trip the gate every frame -- making pose hold impossible.
    Skeleton3DTapFn skel_tap_local;
    {
        std::lock_guard<std::mutex> g(tap_mu_);
        skel_tap_local = skeleton3d_tap_;
    }
    const bool tap_active = skel_tap_local && tri.valid_joints > 0;
    infer::Skeleton3D measured_skel;
    if (tap_active) measured_skel = skel;

    double drift = 0.0;
    if (threed_.ik_enabled) {
        skel = ik_.update(skel);
        drift = ik_.bone_drift_pct(skel);
    } else if (ik_.locked()) {
        drift = ik_.bone_drift_pct(skel);
    }

    // Calibration tap: measured (pre-IK) skeleton for angles + post-IK drift for
    // the gate. Published stats below use the same post-IK drift.
    if (tap_active) {
        skel_tap_local(measured_skel, drift);
    }

    // Formal FusionPose seam: preserve raw tri.skeleton/capture/quality while
    // attaching the same-frame post-Kalman/IK positions.  Deliberately take
    // this copy before floor-contact/root output corrections; those are for
    // WebUI/VR presentation and must not be represented as fusion evidence.
    infer::Skeleton3D post_kalman_ik_skel;
    if (fusion_pose && lifecycle) {
        post_kalman_ik_skel = skel;
        tracker_lineage = make_tracker_axis_lineage(
            fusion_pose->observe(tri, fusion_capture, *lifecycle,
                                 &post_kalman_ik_skel));
        if (lineage_bus) {
            tracker_lineage = lineage_bus->publish(*tracker_lineage);
        }
    }

    lift::FloorContactReport floor_report;
    if (threed_.floor_contact_stability) {
        // Final 3D stage: keep subject-calibration measurements and IK state
        // untouched while publishing one grounded skeleton to both WebUI and
        // the shared VR tracker extractor.
        floor_report = floor_contact_.update(skel, dt_s);
        last_floor_report_ = floor_report;
    }

    auto t1 = std::chrono::steady_clock::now();

    ++tri_processed_;
    tri_recent_.push_back(now);
    while (tri_recent_.size() > 60) tri_recent_.pop_front();
    double tri_fps = 0.0;
    if (tri_recent_.size() >= 2) {
        auto span = std::chrono::duration<double>(tri_recent_.back() - tri_recent_.front()).count();
        if (span > 0.0) tri_fps = (tri_recent_.size() - 1) / span;
    }

    Skeleton3DSnapshot out;
    out.seq = tri_processed_;
    out.ts = wall_now;
    out.t_capture_oldest = min_ts;
    out.tracker_axis_lineage = std::move(tracker_lineage);
    if (tri.valid_joints > 0) {
        out.persons.push_back(skel);
    }
    out.stats.enabled = true;
    out.stats.ik_locked = ik_.locked();
    out.stats.valid_joints = tri.valid_joints;
    out.stats.tri_fps = tri_fps;
    out.stats.reproj_err_med_px = tri.median_reproj_px;
    out.stats.bone_len_drift_pct = drift;
    out.stats.sync_dt_ms = sync_event.sync_dt_ms.value_or(0.0);
    out.stats.stage_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    out.stats.subject_height_m = threed_.subject_height_m;
    out.stats.profile_loaded = ik_.profile_loaded();
    out.stats.subject_id = ik_.subject_id();
    out.stats.profile_quality_status = ik_.profile_quality_status();
    out.stats.processed = tri_processed_;
    out.stats.sync_miss = tri_sync_miss_;
    if (pose_gate) out.stats.pose_gate = pose_gate->diagnostics();
    populate_floor_stats(out.stats, threed_.floor_contact_stability,
                         threed_.floor_contact.floor_z_m,
                         floor_report, true);
    out.cameras = std::move(camera_poses);
    bus->update(out);
    last_3d_update_ = now;
    has_last_3d_update_ = true;
}

void MultiCameraDriver::handle_idle_transition(bool now_idle) {
    sync_queue_.clear();
    last_sync_input_at_.reset();
    if (now_idle) {
        // Entering standby: push one "no fresh data" 3D snapshot (enabled, no
        // persons) so the tracker extractor + publishers + WS viewer drop the
        // last live pose instead of holding a frozen skeleton for the whole
        // idle period. 2D-only runs have no 3D bus → nothing to emit.
        Skeleton3DBus* bus = nullptr;
        PoseGateBus* pose_gate = nullptr;
        FusionPoseBus* fusion_pose = nullptr;
        TrackerAxisLineageBus* lineage_bus = nullptr;
        {
            std::lock_guard<std::mutex> g(threed_mu_);
            bus = threed_.bus;
            pose_gate = threed_.pose_gate;
            fusion_pose = threed_.fusion_pose;
            lineage_bus = threed_.tracker_axis_lineage;
        }
        Skeleton3DSnapshot snap;
        if (pose_gate) {
            const auto boundary = pose_gate->publish_unavailable(
                std::nullopt, PoseGateSourceState::Unavailable, "idle");
            if (fusion_pose) {
                snap.tracker_axis_lineage = make_tracker_axis_lineage(
                    fusion_pose->publish_boundary(boundary));
                if (lineage_bus) {
                    snap.tracker_axis_lineage = lineage_bus->publish(
                        *snap.tracker_axis_lineage);
                }
            }
        }
        if (!bus) return;
        snap.ts = std::chrono::system_clock::now();
        snap.stats.enabled = true;
        // ik_locked=false (even when the subject is calibrated) so both the VMT
        // and the VMT publisher's `!ik_locked` gate skips this frame: otherwise
        // VMT's degeneracy "hold" mode would keep re-sending the frozen pose for
        // the whole idle period instead of dropping it.
        snap.stats.ik_locked = false;
        snap.stats.subject_height_m = threed_.subject_height_m;
        snap.stats.profile_loaded = ik_.profile_loaded();
        snap.stats.subject_id = ik_.subject_id();
        snap.stats.profile_quality_status = ik_.profile_quality_status();
        snap.stats.processed = tri_processed_;
        snap.stats.sync_miss = tri_sync_miss_;
        if (pose_gate) snap.stats.pose_gate = pose_gate->diagnostics();
        populate_floor_stats(snap.stats, threed_.floor_contact_stability,
                             threed_.floor_contact.floor_z_m,
                             last_floor_report_, false);
        bus->update(snap);
    } else {
        // Resuming from standby: drop the Kalman's stale pre-idle state so the
        // first post-idle measurement re-anchors instead of interpolating from
        // the frozen pose (no lurch). dt also resets to the nominal step. IK
        // lock / bone lengths (subject calibration) are preserved. Runs on the
        // loop thread, the only writer of kalman_ / has_last_3d_update_.
        kalman_.reset();
        floor_contact_.reset();
        last_floor_report_ = {};
        has_last_3d_update_ = false;
    }
}

}  // namespace fitra::pipeline
