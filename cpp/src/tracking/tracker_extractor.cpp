#include "tracking/tracker_extractor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "lift/keypoint_format.hpp"

namespace fitra::tracking {

namespace {

const infer::Skeleton3D* pick_skeleton(const pipeline::Skeleton3DSnapshot& snap) {
    if (snap.persons.empty()) return nullptr;
    return &snap.persons.front();
}

// |dot(p, q)| ∈ [0, 1]; covers both halves of the quaternion double cover.
float quat_abs_dot(const cv::Vec4f& a, const cv::Vec4f& b) {
    float d = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
    return std::fabs(d);
}

float quat_angle_rad(const cv::Vec4f& a, const cv::Vec4f& b) {
    float d = std::min(1.0f, std::max(-1.0f, quat_abs_dot(a, b)));
    return 2.0f * std::acos(d);
}

float percentile_inplace(std::vector<float>& samples, float pct) {
    if (samples.empty()) return 0.0f;
    std::size_t n = samples.size();
    std::size_t k = static_cast<std::size_t>(
        std::max(0.0f, std::min(static_cast<float>(n - 1), pct * (n - 1))));
    std::nth_element(samples.begin(), samples.begin() + k, samples.end());
    return samples[k];
}

bool same_lifecycle(const pipeline::TrackerAxisLineage& a,
                    const pipeline::TrackerAxisLineage& b) {
    return a.stream_id == b.stream_id &&
           a.subject_track_id == b.subject_track_id &&
           a.coordinate_epoch == b.coordinate_epoch &&
           a.continuity_epoch == b.continuity_epoch;
}

}  // namespace

TrackerExtractor::TrackerExtractor(pipeline::Skeleton3DBus& skeleton_bus,
                                   TrackerBus&         tracker_bus,
                                   TrackerExtractorOptions  opts,
                                   TrackerAxisBus* tracker_axis_bus,
                                   pipeline::TrackerAxisLineageBus* lineage_bus)
    : skel_bus_(skeleton_bus),
      tracker_bus_(tracker_bus),
      tracker_axis_bus_(tracker_axis_bus),
      lineage_bus_(lineage_bus),
      opts_(opts) {
    for (auto& q : prev_quat_) q = cv::Vec4f{1.0f, 0.0f, 0.0f, 0.0f};
    for (auto& p : prev_pos_)  p = cv::Vec3f{0.0f, 0.0f, 0.0f};
    for (auto& q : last_emitted_quat_) q = cv::Vec4f{1.0f, 0.0f, 0.0f, 0.0f};

    // Pre-allocate ring buffers + percentile scratch so the run loop is
    // allocation-free in steady state.
    const int win = std::max(1, opts_.stats_window);
    for (auto& s : stats_) {
        s.ang_vel_ring.assign(static_cast<std::size_t>(win), 0.0f);
        s.conf_ring.assign(static_cast<std::size_t>(win), 0.0f);
        s.leakage_ring.assign(static_cast<std::size_t>(win), 0);
        s.freeze_ring.assign(static_cast<std::size_t>(win), 0);
        s.percentile_scratch.reserve(static_cast<std::size_t>(win));
        s.head = 0;
        s.fill = 0;
    }
}

TrackerExtractor::~TrackerExtractor() {
    stop();
}

void TrackerExtractor::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) return;
    stop_.store(false, std::memory_order_relaxed);
    thread_ = std::thread([this]() { run_loop(); });
}

void TrackerExtractor::stop() {
    if (!running_.load()) return;
    stop_.store(true, std::memory_order_relaxed);
    // In event-driven mode run_loop may be parked in skel_bus_.wait_for_update;
    // wake it so stop_ is observed without waiting out the timeout.
    skel_bus_.wake();
    if (thread_.joinable()) thread_.join();
    running_.store(false);
}

void TrackerExtractor::reset_smoothing() {
    // Mirror the constructor's history init: identity quats, zero positions,
    // no first-frame anchor. The One Euro / position contexts and FK foot
    // anchors are also dropped so the next valid frame seeds fresh. Rolling
    // stats rings are left alone (they self-flush via the window).
    for (auto& q : prev_quat_)         q = cv::Vec4f{1.0f, 0.0f, 0.0f, 0.0f};
    for (auto& p : prev_pos_)          p = cv::Vec3f{0.0f, 0.0f, 0.0f};
    for (auto& q : last_emitted_quat_) q = cv::Vec4f{1.0f, 0.0f, 0.0f, 0.0f};
    have_last_emitted_ = false;
    quat_ctx_    = QuatSmoothingContext{};
    pos_ctx_     = PosSmoothingContext{};
    extract_ctx_ = ExtractContext{};
    last_smoothed_lineage_.reset();
}

void TrackerExtractor::run_loop() {
    using clk = std::chrono::steady_clock;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, opts_.extract_rate_hz));
    const auto period_d = std::chrono::duration_cast<clk::duration>(period);
    const float nominal_dt_s = static_cast<float>(period.count());
    const int   nominal_dt_ms = static_cast<int>(period.count() * 1000.0);
    const auto  timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(period_d);

    auto next = clk::now() + period_d;
    auto last_tick = clk::now();
    std::uint64_t last_update_seq = 0;
    bool was_idle = false;

    while (!stop_.load(std::memory_order_relaxed)) {
        // dt for angular-velocity / freeze stats. Fixed-rate mode uses the
        // nominal period; event-driven mode measures the real interval (which
        // varies with the triangulation cadence) and clamps it so a post-idle
        // gap doesn't blow up the stats.
        float dt_s;
        int   dt_ms;
        if (opts_.event_driven) {
            // React to each new 3D frame; the timeout still ticks at
            // extract_rate_hz so stale trackers get cleared when 3D is quiet.
            skel_bus_.wait_for_update(last_update_seq, stop_, timeout_ms);
            if (stop_.load(std::memory_order_relaxed)) break;
            auto now = clk::now();
            double measured = std::chrono::duration<double>(now - last_tick).count();
            last_tick = now;
            measured = std::min(0.5, std::max(1e-3, measured));
            dt_s  = static_cast<float>(measured);
            dt_ms = static_cast<int>(measured * 1000.0);
        } else {
            std::this_thread::sleep_until(next);
            next += period_d;
            dt_s  = nominal_dt_s;
            dt_ms = nominal_dt_ms;
        }

        // Idle/standby edge: on resume (idle->active) drop the stale pre-idle
        // smoothing history so the first fresh frame re-anchors instead of
        // lerping from the frozen pose. Checked AFTER the wait/sleep and
        // immediately before snapshot: a resume that lands *during* the wait
        // (idle clears and the first post-idle 3D frame arrives while we are
        // blocked in wait_for_update / sleep_until) is caught in this same
        // iteration. Checking at the top of the loop instead would let the
        // reset slip to the next iteration, smoothing that first post-idle
        // frame against the frozen pre-idle pose and leaving a one-frame lurch.
        const bool idle =
            idle_flag_ && idle_flag_->load(std::memory_order_relaxed);
        if (was_idle && !idle) reset_smoothing();
        was_idle = idle;

        auto snap = skel_bus_.snapshot();

        // Lifecycle boundaries travel on a dedicated FIFO because the
        // Skeleton3DBus is latest-only. Consume and publish them before any
        // extraction/smoothing. A boundary may arrive while the skeleton bus
        // still exposes an older Fresh snapshot, so retain a monotonic
        // watermark and refuse to let that snapshot reseed the reset state.
        std::vector<pipeline::TrackerAxisLineage> pending_boundaries;
        if (lineage_bus_) {
            pending_boundaries = lineage_bus_->drain_boundaries();
            for (const auto& boundary : pending_boundaries) {
                lifecycle_boundary_publish_mono_ns_ = std::max(
                    lifecycle_boundary_publish_mono_ns_,
                    boundary.source_publish_mono_ns);
            }
        }

        const auto* current_lineage = snap.tracker_axis_lineage
            ? &*snap.tracker_axis_lineage : nullptr;
        const bool current_is_boundary = current_lineage &&
            (current_lineage->event_type !=
                 pipeline::FusionPoseEventType::Pose ||
             current_lineage->source_state !=
                 pipeline::FusionPoseSourceState::Fresh);
        if (current_is_boundary) {
            lifecycle_boundary_publish_mono_ns_ = std::max(
                lifecycle_boundary_publish_mono_ns_,
                current_lineage->source_publish_mono_ns);
        }
        const bool current_is_stale = current_lineage &&
            !current_is_boundary &&
            current_lineage->source_publish_mono_ns <=
                lifecycle_boundary_publish_mono_ns_;
        const bool current_changes_lifecycle = current_lineage &&
            !current_is_boundary && !current_is_stale &&
            last_smoothed_lineage_ &&
            !same_lifecycle(*last_smoothed_lineage_, *current_lineage);
        if (!pending_boundaries.empty() ||
            (current_is_boundary && last_smoothed_lineage_) ||
            current_changes_lifecycle) {
            reset_smoothing();
        }

        std::array<TrackerPose, kTrackerCount> boundary_trackers{};
        if (tracker_axis_bus_) {
            for (const auto& boundary : pending_boundaries) {
                tracker_axis_bus_->publish(boundary_trackers, boundary);
            }
            if (current_is_boundary) {
                tracker_axis_bus_->publish(
                    boundary_trackers, snap.tracker_axis_lineage);
            }
        }

        const bool lifecycle_allows_smoothing =
            !current_lineage || (!current_is_boundary && !current_is_stale);
        const infer::Skeleton3D* sk =
            snap.stats.enabled && lifecycle_allows_smoothing
                ? pick_skeleton(snap) : nullptr;
        const bool halpe = fitra::lift::active_keypoint_format() ==
                           fitra::lift::KeypointFormat::Halpe26;

        // Publish on EVERY tick — even when the skeleton snapshot is empty
        // / disabled / in the wrong KP format — so the bus does not retain
        // stale `has_data=true` trackers from a previous successful frame.
        // Without this, /ws3d would keep rendering old AxesHelpers and
        // VMT can keep sending last-known tracker poses while
        // the actual subject is out of view.
        //
        // The "no data" case is signalled by marking every tracker
        // valid=false: apply_quat_smoothing replaces each curr.quat with
        // its prev_quat (so smoothing continuity is preserved for when the
        // subject reappears), the publisher skips them all (no rotations
        // on the wire), and the WebUI fades the axes via the existing
        // valid→opacity mapping.
        std::array<TrackerPose, kTrackerCount> raw_trackers{};
        for (std::size_t i = 0; i < kTrackerCount; ++i) {
            raw_trackers[i].role = static_cast<TrackerRole>(i);
        }
        // Hip context for hip-relative hold. Default is hip_valid=false so
        // an empty / non-Halpe / hip-dropped snapshot falls back to
        // world-absolute hold (the legacy behavior).
        pos_ctx_.hip_valid = false;
        if (sk != nullptr && halpe) {
            raw_trackers = extract_trackers_with_floor_corrections(
                *sk, snap.stats.floor_corrections_m, &extract_ctx_,
                opts_.foot_pos_mode, opts_.chest_height_frac,
                opts_.waist_height_frac, opts_.limb_extension);
            // Halpe26 idx 19 = hip_center. This is the anatomical pelvis anchor
            // for the hip-relative position hold; the waist tracker is built
            // from the same joint (offset up the spine by waist_height_frac), so
            // hip_center stays the stable reference both share.
            constexpr std::size_t kHipCenter = 19;
            const auto& hc = sk->joints[kHipCenter];
            if (hc.valid) {
                pos_ctx_.current_hip_pos = cv::Vec3f{hc.x, hc.y, hc.z};
                pos_ctx_.hip_valid       = true;
            }
        }
        pos_ctx_.dt_s = dt_s;

        // Save validity from the RAW extraction (apply_quat_smoothing will
        // mask invalid trackers with the held quat but valid=false stays).
        std::array<bool, kTrackerCount> raw_valid{};
        for (std::size_t i = 0; i < kTrackerCount; ++i) {
            raw_valid[i] = raw_trackers[i].valid;
        }

        // Frame-rate-independent smoothing: pass the real step dt and the
        // nominal cadence so the time constant is constant regardless of source
        // rate. Fixed-rate mode passes dt_s == nominal_dt_s (behavior unchanged);
        // event-driven mode passes the measured interval so high-fps frames get
        // proportionally gentler per-step smoothing instead of over-damping.
        auto trackers = raw_trackers;
        if (opts_.one_euro) {
            // Speed-adaptive: low cutoff (smooth) at rest, high cutoff
            // (responsive) in motion. quat_smooth/pos_smooth are ignored.
            apply_quat_smoothing(trackers, prev_quat_, quat_ctx_,
                                 opts_.quat_one_euro, dt_s, nominal_dt_s);
            apply_pos_smoothing (trackers, prev_pos_,  pos_ctx_,
                                 opts_.pos_one_euro, nominal_dt_s);
        } else {
            apply_quat_smoothing(trackers, prev_quat_, opts_.quat_smooth, dt_s, nominal_dt_s);
            apply_pos_smoothing (trackers, prev_pos_,  pos_ctx_, opts_.pos_smooth, nominal_dt_s);
        }

        // ------ Per-tracker rolling stats ------------------------------
        TrackerStats stats_out{};
        stats_out.window_frames = opts_.stats_window;

        for (std::size_t i = 0; i < kTrackerCount; ++i) {
            auto& st = stats_[i];

            // 1. Angular velocity (between post-smoothing quats).
            float dv = 0.0f;
            if (have_last_emitted_) {
                float ang = quat_angle_rad(last_emitted_quat_[i], trackers[i].quat_wxyz);
                dv = ang / std::max(1e-6f, dt_s);
            }

            // 2. Confidence + leakage band.
            float conf = trackers[i].roll_confidence;
            std::uint8_t leakage_flag = (conf > 1e-3f && conf < 0.999f) ? 1 : 0;

            // 3. Freeze / dropout edges.
            //
            // `prev_was_valid_seen` makes the very first sample exempt from
            // dropout counting — a tracker that's invalid on the first frame
            // is "born invalid", not a valid→invalid transition. freeze_max_ms
            // is updated on EVERY invalid frame (including the first
            // valid→invalid edge) so a 1-frame freeze does not report max=0.
            bool was_valid = raw_valid[i];
            std::uint8_t freeze_flag = was_valid ? 0 : 1;
            if (was_valid) {
                st.freeze_current_ms = 0;
            } else {
                bool dropout_edge = st.prev_was_valid_seen && st.prev_was_valid;
                if (dropout_edge) {
                    st.dropout_count += 1;
                    st.freeze_current_ms = dt_ms;
                } else {
                    st.freeze_current_ms += dt_ms;
                }
                if (st.freeze_current_ms > st.freeze_max_ms) {
                    st.freeze_max_ms = st.freeze_current_ms;
                }
            }
            st.prev_was_valid      = was_valid;
            st.prev_was_valid_seen = true;

            // 4. Push into ring buffers.
            st.ang_vel_ring[st.head] = dv;
            st.conf_ring[st.head]    = conf;
            st.leakage_ring[st.head] = leakage_flag;
            st.freeze_ring[st.head]  = freeze_flag;
            st.head = (st.head + 1) % st.ang_vel_ring.size();
            if (st.fill < st.ang_vel_ring.size()) st.fill += 1;

            // 5. Reduce to scalars for this publish.
            if (st.fill > 0) {
                // Mean confidence + leakage_pct + freeze_pct.
                double conf_sum = 0.0;
                std::size_t leak_count = 0;
                std::size_t frozen_count = 0;
                for (std::size_t k = 0; k < st.fill; ++k) {
                    conf_sum     += st.conf_ring[k];
                    leak_count   += st.leakage_ring[k];
                    frozen_count += st.freeze_ring[k];
                }
                float n = static_cast<float>(st.fill);
                stats_out.roll_confidence_avg[i] = static_cast<float>(conf_sum / n);
                stats_out.leakage_pct[i]         = static_cast<float>(leak_count) / n;
                stats_out.freeze_pct[i]          = static_cast<float>(frozen_count) / n;

                // Percentiles via nth_element on the per-tracker scratch
                // buffer (pre-reserved to stats_window in the ctor, so
                // assign() reuses storage without allocation). Each
                // percentile_inplace call runs nth_element independently;
                // the second call accepts the partially-reordered scratch
                // left by the first as a valid (unsorted) input.
                st.percentile_scratch.assign(st.ang_vel_ring.begin(),
                                              st.ang_vel_ring.begin() + st.fill);
                stats_out.angular_velocity_rad_s_p50[i] =
                    percentile_inplace(st.percentile_scratch, 0.50f);
                stats_out.angular_velocity_rad_s_p95[i] =
                    percentile_inplace(st.percentile_scratch, 0.95f);
            }
            stats_out.freeze_current_ms[i] = st.freeze_current_ms;
            stats_out.freeze_max_ms[i]     = st.freeze_max_ms;
            stats_out.dropout_count[i]     = st.dropout_count;
        }

        // Remember post-smoothing quats for next iteration's angular velocity.
        for (std::size_t i = 0; i < kTrackerCount; ++i) {
            last_emitted_quat_[i] = trackers[i].quat_wxyz;
        }
        have_last_emitted_ = true;

        tracker_bus_.publish(trackers, stats_out);
        if (tracker_axis_bus_) {
            if (current_lineage && !current_is_boundary && !current_is_stale) {
                tracker_axis_bus_->publish(trackers,
                                           snap.tracker_axis_lineage);
            }
        }
        if (current_lineage && !current_is_boundary && !current_is_stale) {
            last_smoothed_lineage_ = *current_lineage;
        }
    }
}

}  // namespace fitra::tracking
