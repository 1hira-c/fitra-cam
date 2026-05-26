#include "slimevr/tracker_extractor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "lift/keypoint_format.hpp"

namespace fitra::slimevr {

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

}  // namespace

TrackerExtractor::TrackerExtractor(pipeline::Skeleton3DBus& skeleton_bus,
                                   SlimeTrackerBus&         tracker_bus,
                                   TrackerExtractorOptions  opts)
    : skel_bus_(skeleton_bus), tracker_bus_(tracker_bus), opts_(opts) {
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
    if (thread_.joinable()) thread_.join();
    running_.store(false);
}

void TrackerExtractor::run_loop() {
    using clk = std::chrono::steady_clock;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, opts_.extract_rate_hz));
    const auto period_d = std::chrono::duration_cast<clk::duration>(period);
    const float dt_s = static_cast<float>(period.count());
    const int   dt_ms = static_cast<int>(period.count() * 1000.0);

    auto next = clk::now() + period_d;

    while (!stop_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_until(next);
        next += period_d;

        auto snap = skel_bus_.snapshot();
        const infer::Skeleton3D* sk =
            snap.stats.enabled ? pick_skeleton(snap) : nullptr;
        const bool halpe = fitra::lift::active_keypoint_format() ==
                           fitra::lift::KeypointFormat::Halpe26;

        // Phase 13 (Codex P2): publish on EVERY tick — even when the
        // skeleton snapshot is empty / disabled / in the wrong KP format —
        // so the bus does not retain stale `has_data=true` trackers from a
        // previous successful frame. Without this, /ws3d would keep
        // rendering old AxesHelpers and NativePublisher would keep sending
        // last-known rotations while the actual subject is out of view
        // (regression vs. the pre-refactor publisher's pick_skeleton skip).
        //
        // The "no data" case is signalled by marking every tracker
        // valid=false: apply_quat_smoothing replaces each curr.quat with
        // its prev_quat (so smoothing continuity is preserved for when the
        // subject reappears), the publisher skips them all (no rotations
        // on the wire), and the WebUI fades the axes via the existing
        // valid→opacity mapping.
        std::array<SlimeTracker, kTrackerCount> raw_trackers{};
        for (std::size_t i = 0; i < kTrackerCount; ++i) {
            raw_trackers[i].role = static_cast<TrackerRole>(i);
        }
        if (sk != nullptr && halpe) {
            raw_trackers = extract_trackers(*sk);
        }

        // Save validity from the RAW extraction (apply_quat_smoothing will
        // mask invalid trackers with the held quat but valid=false stays).
        std::array<bool, kTrackerCount> raw_valid{};
        for (std::size_t i = 0; i < kTrackerCount; ++i) {
            raw_valid[i] = raw_trackers[i].valid;
        }

        auto trackers = raw_trackers;
        apply_quat_smoothing(trackers, prev_quat_, opts_.quat_smooth);
        apply_pos_smoothing (trackers, prev_pos_,  opts_.pos_smooth);

        // ------ Phase 13 M2: per-tracker rolling stats -----------------
        SlimeTrackerStats stats_out{};
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
            // Phase 13 (Codex P3 / Copilot): `prev_was_valid_seen` makes the
            // very first sample exempt from dropout counting — a tracker
            // that's invalid on the first frame is "born invalid", not a
            // valid→invalid transition. Also: freeze_max_ms is now updated
            // on EVERY invalid frame including the first valid→invalid edge
            // (previously the first invalid frame set freeze_current_ms but
            // skipped the max update, so a 1-frame freeze reported max=0).
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
    }
}

}  // namespace fitra::slimevr
