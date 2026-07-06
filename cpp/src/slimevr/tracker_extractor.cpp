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
    st_pos_state_   = StPosState{};
    st_twist_state_ = StTwistState{};
}

void TrackerExtractor::run_loop() {
    using clk = std::chrono::steady_clock;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, opts_.extract_rate_hz));
    const auto period_d = std::chrono::duration_cast<clk::duration>(period);
    const float nominal_dt_s = static_cast<float>(period.count());
    const int   nominal_dt_ms = static_cast<int>(period.count() * 1000.0);
    const auto  timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(period_d);
    const auto  stale_clear_after = std::chrono::milliseconds(250);

    auto next = clk::now() + period_d;
    auto last_tick = clk::now();
    std::uint64_t last_update_seq = 0;
    std::uint64_t last_source_update_seq = 0;
    std::uint64_t duplicate_ticks = 0;
    std::uint64_t stale_clears = 0;
    double fresh_hz_ema = 0.0;
    bool have_publish_tick = false;
    bool have_fresh_update = false;
    bool stale_clear_published = false;
    clk::time_point last_fresh_update{};
    bool was_idle = false;

    while (!stop_.load(std::memory_order_relaxed)) {
        // dt for angular-velocity / freeze stats. Fixed-rate mode uses the
        // nominal period; event-driven mode measures the real interval (which
        // varies with the triangulation cadence) and clamps it so a post-idle
        // gap doesn't blow up the stats.
        float dt_s;
        int   dt_ms;
        bool  force_clear = false;
        clk::time_point tick_now{};
        if (opts_.event_driven) {
            // React to each new 3D frame; the timeout still ticks at
            // extract_rate_hz so the source can be marked stale when 3D is quiet.
            const bool fresh_update =
                skel_bus_.wait_for_update(last_update_seq, stop_, timeout_ms);
            if (stop_.load(std::memory_order_relaxed)) break;
            tick_now = clk::now();
            if (!fresh_update) {
                if (have_fresh_update) ++duplicate_ticks;
                const bool stale_due =
                    have_fresh_update &&
                    (tick_now - last_fresh_update >= stale_clear_after);
                if (!stale_due || stale_clear_published) {
                    continue;
                }
                force_clear = true;
                stale_clear_published = true;
                ++stale_clears;
            } else {
                if (have_fresh_update) {
                    const double gap_s =
                        std::chrono::duration<double>(tick_now - last_fresh_update).count();
                    if (gap_s > 1.0e-3) {
                        const double hz = 1.0 / gap_s;
                        fresh_hz_ema = fresh_hz_ema <= 0.0
                                           ? hz
                                           : (0.2 * hz + 0.8 * fresh_hz_ema);
                    }
                }
                have_fresh_update = true;
                last_fresh_update = tick_now;
                stale_clear_published = false;
            }
            double measured = have_publish_tick
                                  ? std::chrono::duration<double>(tick_now - last_tick).count()
                                  : nominal_dt_s;
            last_tick = tick_now;
            have_publish_tick = true;
            measured = std::min(0.5, std::max(1e-3, measured));
            dt_s  = static_cast<float>(measured);
            dt_ms = static_cast<int>(measured * 1000.0);
        } else {
            std::this_thread::sleep_until(next);
            next += period_d;
            tick_now = clk::now();
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
        if (opts_.event_driven) {
            if (!force_clear) last_source_update_seq = snap.update_seq;
        } else if (snap.update_seq != 0) {
            if (last_source_update_seq == snap.update_seq) {
                ++duplicate_ticks;
            } else {
                last_source_update_seq = snap.update_seq;
            }
        }
        const infer::Skeleton3D* sk =
            (!force_clear && snap.stats.enabled) ? pick_skeleton(snap) : nullptr;
        const bool halpe = fitra::lift::active_keypoint_format() ==
                           fitra::lift::KeypointFormat::Halpe26;

        // Publish every processed source frame — even when the skeleton snapshot
        // is empty / disabled / in the wrong KP format — so the bus does not
        // retain stale `has_data=true` trackers from a previous successful
        // frame. In event-driven mode, timeout-only loops are skipped; a single
        // forced invalid publish is emitted only after the 3D bus stays quiet.
        //
        // The "no data" case is signalled by marking every tracker
        // valid=false: apply_quat_smoothing replaces each curr.quat with
        // its prev_quat (so smoothing continuity is preserved for when the
        // subject reappears), and consumers can fade / hold / skip according to
        // their existing degeneracy policy.
        std::array<SlimeTracker, kTrackerCount> raw_trackers{};
        for (std::size_t i = 0; i < kTrackerCount; ++i) {
            raw_trackers[i].role = static_cast<TrackerRole>(i);
        }
        // Hip context for hip-relative hold. Default is hip_valid=false so
        // an empty / non-Halpe / hip-dropped snapshot falls back to
        // world-absolute hold (the legacy behavior).
        pos_ctx_.hip_valid = false;
        if (sk != nullptr && halpe) {
            raw_trackers = extract_trackers(*sk, &extract_ctx_, opts_.foot_pos_mode,
                                            opts_.chest_height_frac,
                                            opts_.waist_height_frac,
                                            opts_.roll_hysteresis);
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
        if (opts_.st_filter) {
            // Spatiotemporal filter (priority st_filter > one_euro > EMA).
            // Position = distance×velocity regime in the waist-relative frame;
            // inferred-roll twist = regime-driven for the ARM group only
            // (legs kept out per M-C4/M-C5 — see st_filter make_default_config).
            // The twist override is computed from the RAW orientations (and the
            // held prev_quat_) BEFORE apply_quat_smoothing mutates them; swing
            // still rides the One Euro / fixed base so it is unchanged.
            std::array<float, kTrackerCount> twist_override;
            fill_st_twist_overrides(trackers, prev_quat_, st_twist_state_, st_cfg_,
                                    dt_s, nominal_dt_s, twist_override);
            if (opts_.one_euro) {
                apply_quat_smoothing(trackers, prev_quat_, quat_ctx_,
                                     opts_.quat_one_euro, dt_s, nominal_dt_s, &twist_override);
            } else {
                apply_quat_smoothing(trackers, prev_quat_, opts_.quat_smooth,
                                     dt_s, nominal_dt_s, &twist_override);
            }
            const cv::Vec3f* waist_fallback =
                pos_ctx_.hip_valid ? &pos_ctx_.current_hip_pos : nullptr;
            apply_pos_st_filter(trackers, st_pos_state_, st_cfg_, dt_s, nominal_dt_s,
                                waist_fallback);
        } else if (opts_.one_euro) {
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

        double source_age_ms = 0.0;
        if (snap.updated_at.time_since_epoch().count() != 0) {
            source_age_ms =
                std::chrono::duration<double, std::milli>(tick_now - snap.updated_at).count();
            source_age_ms = std::max(0.0, source_age_ms);
        }
        SlimeTrackerStreamStats stream_out{};
        stream_out.mode = opts_.event_driven ? "event" : "fixed";
        stream_out.source_update_seq = snap.update_seq;
        stream_out.source_pose_seq = snap.seq;
        stream_out.source_age_ms = source_age_ms;
        stream_out.filter_dt_ms = static_cast<double>(dt_s) * 1000.0;
        stream_out.fresh_hz = opts_.event_driven ? fresh_hz_ema : snap.stats.tri_fps;
        stream_out.duplicate_ticks = duplicate_ticks;
        stream_out.stale_clears = stale_clears;
        stream_out.source_stale = force_clear;

        tracker_bus_.publish(trackers, stats_out, stream_out);
    }
}

}  // namespace fitra::slimevr
