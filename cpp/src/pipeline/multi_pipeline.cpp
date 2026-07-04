#include "pipeline/multi_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

#include "util/logging.hpp"

namespace fitra::pipeline {

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
      kalman_{[&]() {
          // Default process noise = byte-identical when st_filter is off.
          // The M-C3 seed weakened the chain Kalman ×100 when st_filter was on
          // (premise: IK/rigid take the structure, so the Kalman need only
          // predict/hold). The M-C4 still-clip measurement FALSIFIED that at
          // rest: ×100 injects orientation jitter the tracker filter can't
          // absorb (rel angular velocity 2–3×, held-leg roll +300–1400%), while
          // giving no position benefit (both hit the ~6 mm upstream floor). So
          // the data-driven default is NO weakening (×1); the factor is kept as
          // a named knob for the motion/lag sweep (M-C4-D). See
          // docs/design/pose-3d-spatiotemporal-filter.md.
          lift::SkeletonKalman::Options kopts;
          if (threed_.st_filter) {
              constexpr double kStWeaken = 1.0;  // M-C4: ×100 was harmful at rest
              kopts.q_pos        *= kStWeaken;
              kopts.q_vel        *= kStWeaken;
              kopts.q_pos_offset *= kStWeaken;
              kopts.q_vel_offset *= kStWeaken;
          }
          return lift::SkeletonKalman{kopts};
      }()},
      ik_{[&]() {
          lift::IkSolver::Options opts;
          opts.bone_calib_frames = std::max(1, threed_.bone_calib_frames);
          opts.subject_height_m = threed_.subject_height_m;
          opts.has_subject_profile = threed_.has_subject_profile;
          opts.subject_profile = threed_.subject_profile;
          return lift::IkSolver{opts};
      }()},
      latest_per_cam_(sources_.size()),
      latest_snapshots_(sources_.size()),
      last_3d_input_seqs_(sources_.size(), 0),
      per_cam_(sources_.size()) {
    for (const auto& source : sources_) {
        if (!source || !source->prebakes_pose()) {
            throw std::invalid_argument(
                "MultiCameraDriver requires FrameSource with RTMPose prebaking enabled");
        }
    }
    if ((threed_.triangulator || threed_.bus) && (!threed_.triangulator || !threed_.bus)) {
        throw std::invalid_argument("3D pipeline requires both triangulator and Skeleton3DBus");
    }
    // Spatial pelvis rigid fit (M-A): build the static pelvis template from the
    // subject profile's segment distances. Active only under Halpe26 with a
    // loaded profile and a non-degenerate triangle; otherwise a no-op (the 3D
    // path stays byte-identical to pre-M-A).
    if (threed_.rigid_pelvis && threed_.has_subject_profile &&
        lift::active_keypoint_format() == lift::KeypointFormat::Halpe26) {
        const auto& p = threed_.subject_profile;
        pelvis_template_ = lift::RigidTemplate::from_distances(
            p.bone_lengths_m[11], p.bone_lengths_m[12], p.hip_width_m);
        rigid_pelvis_active_ = pelvis_template_.valid;
        FITRA_LOG_INFO("3D pelvis rigid fit {} (template valid={})",
                       rigid_pelvis_active_ ? "ENABLED (spatial-first)" : "requested but degenerate template",
                       pelvis_template_.valid);
    } else if (threed_.rigid_pelvis) {
        FITRA_LOG_WARN("3D pelvis rigid fit requested but inactive "
                       "(needs Halpe26 + a loaded subject profile)");
    }
    // Floor-contact grounding (M-D): opts from config; active only under Halpe26
    // (needs toe/heel sole points — a no-op on COCO17 regardless of the flag).
    floor_opts_.floor_z_m      = threed_.floor_z_m;
    floor_opts_.stance_vel_mps = threed_.floor_stance_vel_mps;
    floor_opts_.snap_band_m    = threed_.floor_snap_band_m;
    if (threed_.floor_grounding) {
        const bool halpe = lift::active_keypoint_format() == lift::KeypointFormat::Halpe26;
        FITRA_LOG_INFO("3D floor grounding {} (floor_z={}m band={}m stance<{}m/s)",
                       halpe ? "ENABLED" : "requested but inactive (needs Halpe26 toe/heel)",
                       threed_.floor_z_m, threed_.floor_snap_band_m, threed_.floor_stance_vel_mps);
    }
    for (std::size_t i = 0; i < latest_snapshots_.size(); ++i) {
        latest_snapshots_[i].id = static_cast<int>(i);
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
    // The loop may be parked in sources_[0]->wait_available; wake it so stop_
    // is observed without waiting out the timeout. Order: flag -> wake -> join.
    for (auto& s : sources_) if (s) s->wake();
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

    while (!stop_.load()) {
        pending.clear();
        reqs.clear();
        auto iter_start = std::chrono::steady_clock::now();

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
            camera::DecodedFrame df;
            if (!sources_[i]->try_pop_latest_decoded(df)) continue;

            latest_per_cam_[i] = std::move(df);

            // Frame tap. Read the callback into a local std::function under
            // the lock, then invoke without the lock held -- the user callback
            // may take its own mutex and we must not let it back into tap_mu_
            // via a re-entrant set_frame_tap.
            FrameTapFn tap_local;
            {
                std::lock_guard<std::mutex> g(tap_mu_);
                tap_local = frame_tap_;
            }
            if (tap_local) {
                auto now_tap = std::chrono::steady_clock::now();
                if (!loop_t0_set_) {
                    loop_t0_ = now_tap;
                    loop_t0_set_ = true;
                }
                double ts_ms = std::chrono::duration<double, std::milli>(
                                  latest_per_cam_[i].captured_at - loop_t0_).count();
                tap_local(i, latest_per_cam_[i].bgr, ts_ms);
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
            // Single-camera: block until the sole source publishes a new
            // decoded frame (event-driven; removes the fixed 2ms poll tax and
            // its jitter). wait_available doesn't consume -- the next Pass-1
            // iteration picks it up via try_pop_latest_decoded.
            // Multi-camera keeps the short poll: a shared wakeup across N
            // sources is left as a follow-up (see design doc).
            if (sources_.size() == 1) {
                sources_[0]->wait_available(stop_, std::chrono::milliseconds(100));
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
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
            latest_snapshots_[pc.idx] = std::move(snap);
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
    {
        std::lock_guard<std::mutex> g(threed_mu_);
        triangulator = threed_.triangulator;
        bus = threed_.bus;
    }
    if (!triangulator || !bus) return;
    if (latest_snapshots_.size() < 2) return;

    // Static camera placements (world frame) for the 3D viewer's frustums.
    // Resent on every snapshot (incl. sync-miss) so the markers stay visible.
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

    std::chrono::steady_clock::time_point min_ts{};
    std::chrono::steady_clock::time_point max_ts{};
    bool first = true;
    for (const auto& snap : latest_snapshots_) {
        if (snap.processed == 0) return;
        if (first) {
            min_ts = max_ts = snap.captured_at;
            first = false;
        } else {
            min_ts = std::min(min_ts, snap.captured_at);
            max_ts = std::max(max_ts, snap.captured_at);
        }
    }
    bool has_new_input = false;
    for (std::size_t i = 0; i < latest_snapshots_.size(); ++i) {
        if (latest_snapshots_[i].seq != last_3d_input_seqs_[i]) {
            has_new_input = true;
            break;
        }
    }
    if (!has_new_input) return;

    const double sync_dt_ms = std::chrono::duration<double, std::milli>(max_ts - min_ts).count();
    const double sync_window = std::max(0.0, threed_.sync_window_ms);
    if (sync_dt_ms > sync_window) {
        ++tri_sync_miss_;
        Skeleton3DSnapshot miss;
        miss.ts = wall_now;
        miss.stats.enabled = true;
        miss.stats.sync_dt_ms = sync_dt_ms;
        miss.stats.subject_height_m = threed_.subject_height_m;
        miss.stats.profile_loaded = ik_.profile_loaded();
        miss.stats.subject_id = ik_.subject_id();
        miss.stats.profile_quality_status = ik_.profile_quality_status();
        miss.stats.sync_miss = tri_sync_miss_;
        miss.stats.processed = tri_processed_;
        miss.stats.ik_locked = ik_.locked();
        miss.cameras = camera_poses;
        bus->update(miss);
        for (std::size_t i = 0; i < latest_snapshots_.size(); ++i) {
            last_3d_input_seqs_[i] = latest_snapshots_[i].seq;
        }
        return;
    }

    auto t0 = std::chrono::steady_clock::now();
    std::vector<lift::PerCameraObservation> observations;
    observations.reserve(latest_snapshots_.size());
    for (std::size_t i = 0; i < latest_snapshots_.size(); ++i) {
        const auto& snap = latest_snapshots_[i];
        if (snap.persons.empty()) continue;
        observations.push_back(lift::PerCameraObservation{
            static_cast<int>(i), &snap.persons[0]});
    }

    auto tri = triangulator->triangulate(observations);
    infer::Skeleton3D skel = tri.skeleton;

    double dt_s = 1.0 / 30.0;
    if (has_last_3d_update_) {
        dt_s = std::chrono::duration<double>(now - last_3d_update_).count();
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

    double drift = 0.0;
    // Spatial-first only when the pelvis rigid fit actually applied this frame.
    // apply_segment_rigid_fit() returns false (leaving skel untouched) when the
    // flag/profile gate is off OR a pelvis joint is missing this frame; in that
    // case fall through to the proven baseline order (Kalman -> IK) with no
    // rigid benefit. NOTE: a fit-failure frame is byte-identical to the pre-M-A
    // path only in its STATELESS stages — the shared Kalman carries state
    // trained on the other branch's input regime (post-IK vs raw tri), so
    // marginal pelvis visibility toggling the branch per frame feeds the one
    // filter an alternating measurement stream (step inputs at each
    // transition). Known trade-off while rigid_pelvis is default OFF; revisit
    // (single order on failure, or a Kalman soft-reinit) before default-ON.
    const bool fit_applied =
        rigid_pelvis_active_ &&
        lift::apply_segment_rigid_fit(skel, tri, pelvis_template_, {19, 11, 12});
    if (fit_applied) {
        // Spatial-first (spatial-filtering M-A, design A): tri -> pelvis rigid
        // fit -> IK -> Kalman. The rigid fit averages the pelvis joints'
        // independent jitter (no lag); IK enforces limb lengths/hinges relative
        // to the now-rigid pelvis; a light Kalman takes only the residual. Only
        // reached when a subject profile is loaded (calib-subject has none, so
        // its measured-angle tap semantics are untouched).
        if (tap_active) measured_skel = skel;  // pre-IK
        if (threed_.ik_enabled) {
            skel = ik_.update(skel);
            drift = ik_.bone_drift_pct(skel);
        } else if (ik_.locked()) {
            drift = ik_.bone_drift_pct(skel);
        }
        if (threed_.kalman_enabled) skel = kalman_.update(skel, dt_s);
        if (tap_active) skel_tap_local(measured_skel, drift);  // post-IK drift for the gate
        // Published stat: re-measure on the skeleton actually published (the
        // trailing Kalman can re-stretch the IK-clamped bones). Matches the
        // baseline branch (IK last → drift describes the final skeleton) and
        // the offline harness's drift_after, keeping live vs offline A/B
        // comparable. The tap above keeps the lenient post-IK value.
        if (threed_.ik_enabled || ik_.locked()) drift = ik_.bone_drift_pct(skel);
    } else {
        // Baseline (pre-M-A, design B): tri -> Kalman -> IK. Byte-identical to
        // the historical path (also the fit-failure fallback).
        if (threed_.kalman_enabled) skel = kalman_.update(skel, dt_s);
        if (tap_active) measured_skel = skel;  // pre-IK
        if (threed_.ik_enabled) {
            skel = ik_.update(skel);
            drift = ik_.bone_drift_pct(skel);
        } else if (ik_.locked()) {
            drift = ik_.bone_drift_pct(skel);
        }
        // Calibration tap: measured (pre-IK) skeleton for angles + post-IK drift
        // for the gate. Published stats below use the same post-IK drift.
        if (tap_active) skel_tap_local(measured_skel, drift);
    }

    // Floor-contact grounding (M-D): LAST 3D stage, after Kalman + IK, so no
    // downstream smoother can re-sink the foot and the post-IK sole adjustment
    // is output-only (not fed back into Kalman/IK state). No-op on COCO17 / when
    // the flag is off (byte-identical). Uses the same dt_s as the Kalman.
    if (threed_.floor_grounding) {
        lift::apply_floor_grounding(skel, floor_state_, dt_s, floor_opts_);
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
    if (tri.valid_joints > 0) {
        out.persons.push_back(skel);
    }
    out.stats.enabled = true;
    out.stats.ik_locked = ik_.locked();
    out.stats.valid_joints = tri.valid_joints;
    out.stats.tri_fps = tri_fps;
    out.stats.reproj_err_med_px = tri.median_reproj_px;
    out.stats.bone_len_drift_pct = drift;
    out.stats.sync_dt_ms = sync_dt_ms;
    out.stats.stage_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    out.stats.subject_height_m = threed_.subject_height_m;
    out.stats.profile_loaded = ik_.profile_loaded();
    out.stats.subject_id = ik_.subject_id();
    out.stats.profile_quality_status = ik_.profile_quality_status();
    out.stats.processed = tri_processed_;
    out.stats.sync_miss = tri_sync_miss_;
    out.cameras = std::move(camera_poses);
    bus->update(out);
    for (std::size_t i = 0; i < latest_snapshots_.size(); ++i) {
        last_3d_input_seqs_[i] = latest_snapshots_[i].seq;
    }
    last_3d_update_ = now;
    has_last_3d_update_ = true;
}

void MultiCameraDriver::handle_idle_transition(bool now_idle) {
    if (now_idle) {
        // Entering standby: push one "no fresh data" 3D snapshot (enabled, no
        // persons) so the tracker extractor + publishers + WS viewer drop the
        // last live pose instead of holding a frozen skeleton for the whole
        // idle period. 2D-only runs have no 3D bus → nothing to emit.
        Skeleton3DBus* bus = nullptr;
        {
            std::lock_guard<std::mutex> g(threed_mu_);
            bus = threed_.bus;
        }
        if (!bus) return;
        Skeleton3DSnapshot snap;
        snap.ts = std::chrono::system_clock::now();
        snap.stats.enabled = true;
        // ik_locked=false (even when the subject is calibrated) so both the VMT
        // and SlimeVR publishers' `!ik_locked` gate skips this frame: otherwise
        // VMT's degeneracy "hold" mode would keep re-sending the frozen pose for
        // the whole idle period instead of dropping it.
        snap.stats.ik_locked = false;
        snap.stats.subject_height_m = threed_.subject_height_m;
        snap.stats.profile_loaded = ik_.profile_loaded();
        snap.stats.subject_id = ik_.subject_id();
        snap.stats.profile_quality_status = ik_.profile_quality_status();
        snap.stats.processed = tri_processed_;
        snap.stats.sync_miss = tri_sync_miss_;
        bus->update(snap);
    } else {
        // Resuming from standby: drop the Kalman's stale pre-idle state so the
        // first post-idle measurement re-anchors instead of interpolating from
        // the frozen pose (no lurch). dt also resets to the nominal step. IK
        // lock / bone lengths (subject calibration) are preserved. Runs on the
        // loop thread, the only writer of kalman_ / has_last_3d_update_.
        kalman_.reset();
        floor_state_.reset();  // drop stale stance anchors so speed re-anchors post-idle
        has_last_3d_update_ = false;
    }
}

}  // namespace fitra::pipeline
