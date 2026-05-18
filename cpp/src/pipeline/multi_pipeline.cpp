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
      kalman_{},
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

void MultiCameraDriver::stop() {
    if (!worker_.joinable() && sources_.empty()) return;
    stop_.store(true);
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
    auto   stats_anchor = std::chrono::steady_clock::now();
    std::vector<bool> warned_missing_prebake(sources_.size(), false);
    const std::size_t rtmpose_per_item =
        infer::RtmPose::blob_floats_per_item(rtmpose_.options());

    while (!stop_.load()) {
        pending.clear();
        reqs.clear();
        auto iter_start = std::chrono::steady_clock::now();

        // Pass 1: pull the latest (frame, bboxes) from each FrameSource.
        // Decode + YOLOX already ran in the per-camera worker thread.
        for (std::size_t i = 0; i < sources_.size(); ++i) {
            if (stop_.load()) break;
            camera::DecodedFrame df;
            if (!sources_[i]->try_pop_latest_decoded(df)) continue;

            latest_per_cam_[i] = std::move(df);

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
            for (std::size_t bi = 0; bi < cam_df.bboxes.size(); ++bi) {
                infer::RtmPose::PrebakedRequest pr;
                pr.chw   = cam_df.chw_concat.data() + bi * rtmpose_per_item;
                pr.M_inv = cam_df.M_invs[bi];
                pr.bbox  = cam_df.bboxes[bi];
                reqs.push_back(pr);
            }
            pending.push_back(pc);
        }

        if (pending.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        auto t_after_poll = std::chrono::steady_clock::now();

        // Pass 2: one batched RTMPose call across all cameras' bboxes.
        // Preprocess already ran on per-camera worker threads — this is
        // just memcpy + GPU enqueue + sync + SimCC decode.
        std::vector<infer::Person> all_persons;
        if (!reqs.empty()) {
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

            CameraSnapshot snap;
            snap.id  = static_cast<int>(pc.idx);
            snap.w   = sources_[pc.idx]->options().width;
            snap.h   = sources_[pc.idx]->options().height;
            snap.seq = df.seq;
            snap.captured_at = df.captured_at;
            auto lag = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now - df.captured_at);
            snap.captured_wall = wall_now - lag;
            if (pc.person_count > 0) {
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
        maybe_update_3d(now, wall_now);
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
            iter_count = 0;
            sum_poll_ms = sum_rtm_ms = sum_snap_ms = 0.0;
            sum_reqs = 0;
            stats_anchor = t_after_snap;
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
    if (!threed_.triangulator || !threed_.bus) return;
    if (latest_snapshots_.size() < 2) return;

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
        threed_.bus->update(miss);
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

    auto tri = threed_.triangulator->triangulate(observations);
    infer::Skeleton3D skel = tri.skeleton;
    if (threed_.kalman_enabled) {
        double dt_s = 1.0 / std::max(1.0, per_cam_[0].stats.recent_pose_fps);
        skel = kalman_.update(skel, dt_s);
    }
    double drift = ik_.bone_drift_pct(skel);
    if (threed_.ik_enabled) {
        skel = ik_.update(skel);
        drift = ik_.bone_drift_pct(skel);
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
    threed_.bus->update(out);
}

}  // namespace fitra::pipeline
