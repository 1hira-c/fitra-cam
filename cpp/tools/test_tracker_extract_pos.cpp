// test_tracker_extract_pos — exercise apply_pos_smoothing.
//
// TrackerExtractor maintains a position EMA (mirroring the existing per-tracker
// quat smoothing). We verify:
//   1. pos_smooth=1.0 passes raw curr.pos through unchanged
//   2. pos_smooth=0.5 converges toward the target after a few frames
//   3. valid=false holds prev_pos (curr.pos snaps to prev, prev untouched)
//   4. dropout/recovery does NOT push the cached position back through (0,0,0)
//      — important because prev_pos is the only "last good" cache available
//      to the publisher during a brief occlusion.
//
// All checks use cv::Vec3f and the same shape of helpers as
// test_tracker_extract.

#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

#include "slimevr/tracker_extract.hpp"

namespace {

constexpr float kEps = 1.0e-5f;

using fitra::slimevr::PosSmoothingContext;
using fitra::slimevr::SlimeTracker;
using fitra::slimevr::TrackerRole;
using fitra::slimevr::apply_pos_smoothing;
using fitra::slimevr::kTrackerCount;

void check(bool cond, const std::string& msg) {
    if (!cond) throw std::runtime_error(msg);
}

void check_close(float got, float want, const std::string& label, float eps = kEps) {
    if (std::abs(got - want) > eps) {
        char buf[200];
        std::snprintf(buf, sizeof(buf), "%s: got=%.6f want=%.6f", label.c_str(), got, want);
        throw std::runtime_error(buf);
    }
}

void check_vec3(const cv::Vec3f& got, const cv::Vec3f& want, const std::string& label,
                float eps = kEps) {
    for (int i = 0; i < 3; ++i) check_close(got[i], want[i], label + "[" + std::to_string(i) + "]", eps);
}

// All-valid trackers at a uniform pos for the smoothing tests. role values
// don't matter for apply_pos_smoothing.
std::array<SlimeTracker, kTrackerCount> make_trackers(const cv::Vec3f& pos, bool valid) {
    std::array<SlimeTracker, kTrackerCount> ts{};
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        ts[i].role  = static_cast<TrackerRole>(i);
        ts[i].pos   = pos;
        ts[i].valid = valid;
    }
    return ts;
}

void test_no_smooth_passthrough() {
    auto ts = make_trackers({1.0f, 2.0f, 3.0f}, /*valid=*/true);
    std::array<cv::Vec3f, kTrackerCount> prev{};  // zero-init
    apply_pos_smoothing(ts, prev, /*base_alpha=*/1.0f);
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        check_vec3(ts[i].pos, {1.0f, 2.0f, 3.0f}, "no-smooth.curr[" + std::to_string(i) + "]");
        check_vec3(prev[i],   {1.0f, 2.0f, 3.0f}, "no-smooth.prev[" + std::to_string(i) + "]");
    }
}

void test_smooth_convergence() {
    // Target = (4, 0, 0), prev starts at (0,0,0), alpha=0.5.
    // 1 frame:  prev = 2.0     ; rel error = 50%
    // 2 frames: prev = 3.0     ; rel error = 25%
    // 6 frames: prev = 4*(1-0.5^6) = 3.9375; rel error ≈ 1.6% (< 5% threshold)
    std::array<cv::Vec3f, kTrackerCount> prev{};
    const cv::Vec3f target{4.0f, 0.0f, 0.0f};
    for (int frame = 0; frame < 6; ++frame) {
        auto ts = make_trackers(target, /*valid=*/true);
        apply_pos_smoothing(ts, prev, /*base_alpha=*/0.5f);
    }
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        const float err = std::abs(prev[i][0] - target[0]) / std::abs(target[0]);
        check(err < 0.05f, "convergence rel err > 5% at i=" + std::to_string(i));
    }
}

void test_invalid_holds_prev() {
    // Seed prev to a known good pos, then submit an invalid frame at a
    // wildly different raw pos. Smoothed curr should be the prev (not the
    // raw), and prev itself should stay unchanged.
    std::array<cv::Vec3f, kTrackerCount> prev{};
    for (auto& p : prev) p = cv::Vec3f{0.5f, 0.6f, 0.7f};

    auto ts = make_trackers({99.0f, -99.0f, 0.0f}, /*valid=*/false);
    apply_pos_smoothing(ts, prev, /*base_alpha=*/0.5f);

    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        check_vec3(ts[i].pos, {0.5f, 0.6f, 0.7f}, "invalid.curr[" + std::to_string(i) + "]");
        check_vec3(prev[i],   {0.5f, 0.6f, 0.7f}, "invalid.prev[" + std::to_string(i) + "]");
    }
}

void test_dropout_recovery() {
    // Realistic occlusion timeline:
    //   frame 1-3: valid pos (1,0,0), prev converges
    //   frame 4-5: invalid (occluded) — prev must NOT drift back to 0
    //   frame 6-8: valid pos (1.5,0,0) — converges from the held prev,
    //              not from (0,0,0)
    std::array<cv::Vec3f, kTrackerCount> prev{};
    const float alpha = 0.5f;

    // Step 1: valid at (1,0,0)
    for (int f = 0; f < 3; ++f) {
        auto ts = make_trackers({1.0f, 0.0f, 0.0f}, /*valid=*/true);
        apply_pos_smoothing(ts, prev, alpha);
    }
    // After 3 frames with alpha=0.5 toward 1.0 from 0: prev = 1 - 0.5^3 = 0.875
    check_close(prev[0][0], 0.875f, "dropout.step1.prev[0].x");

    // Step 2: invalid (occluded) — must hold prev unchanged
    for (int f = 0; f < 2; ++f) {
        auto ts = make_trackers({999.0f, 999.0f, 999.0f}, /*valid=*/false);
        apply_pos_smoothing(ts, prev, alpha);
    }
    check_close(prev[0][0], 0.875f, "dropout.step2.prev held");
    check_close(prev[0][1], 0.0f,   "dropout.step2.prev y held");

    // Step 3: recover at (1.5,0,0) — convergence starts from 0.875, NOT 0
    auto ts = make_trackers({1.5f, 0.0f, 0.0f}, /*valid=*/true);
    apply_pos_smoothing(ts, prev, alpha);
    // After 1 frame with alpha=0.5: prev = 0.875 + 0.5*(1.5 - 0.875) = 1.1875.
    // If the cache had snapped back to 0, we'd see 0.75 instead — the test
    // distinguishes "held" from "reset to 0".
    check_close(prev[0][0], 1.1875f, "dropout.phase3.prev recovers from held");
}

// ---------- ctx-aware overload (hip-relative hold + velocity gate) -------

// pose-3d/locomotion-stability M1:
// Seed prev_pos to a known foot position 0.5 m in front of and below the hip.
// Move the hip 1 m to the right while marking the tracker invalid. The held
// position must follow the hip (i.e. tracker.pos[0] increases by ~1 m),
// not stay at the original world coordinates.
void test_hip_relative_hold_on_invalid() {
    std::array<cv::Vec3f, kTrackerCount> prev{};
    PosSmoothingContext ctx;

    // Seed: hip at (0,0,1), tracker idx 8 (LeftFoot) at (0.1, 0.3, 0.05).
    ctx.current_hip_pos = cv::Vec3f{0.0f, 0.0f, 1.0f};
    ctx.hip_valid       = true;
    {
        // First call with a valid tracker so prev_pos[8] gets initialized.
        auto ts = make_trackers({0.1f, 0.3f, 0.05f}, /*valid=*/true);
        apply_pos_smoothing(ts, prev, ctx, /*base_alpha=*/1.0f);
    }
    check_close(prev[8][0], 0.1f, "hip-rel.seed.prev[8].x");
    check(ctx.prev_hip_valid, "hip-rel.seed.prev_hip_valid");
    check_close(ctx.prev_hip_pos[2], 1.0f, "hip-rel.seed.prev_hip.z");

    // Second call: hip moves to (1, 0, 1), tracker is invalid.
    ctx.current_hip_pos = cv::Vec3f{1.0f, 0.0f, 1.0f};
    ctx.hip_valid       = true;
    {
        auto ts = make_trackers({999.0f, 999.0f, 999.0f}, /*valid=*/false);
        apply_pos_smoothing(ts, prev, ctx, /*base_alpha=*/0.5f);
        // offset = (0.1, 0.3, 0.05) - (0, 0, 1) = (0.1, 0.3, -0.95)
        // new world = (1, 0, 1) + (0.1, 0.3, -0.95) = (1.1, 0.3, 0.05)
        check_vec3(ts[8].pos, {1.1f, 0.3f, 0.05f}, "hip-rel.hold.curr[8]");
    }
    check_vec3(prev[8], {1.1f, 0.3f, 0.05f}, "hip-rel.hold.prev[8]");
}

// Regression: a hip dropout must clear prev_hip_valid so the next valid-hip
// frame does NOT re-anchor a held tracker across the gap using a stale
// prev_hip_pos. The hip-relative hold's invariant is "previous frame's hip
// was also valid"; without resetting prev_hip_valid every tick it stays stuck
// true and a held limb teleports by the across-gap hip displacement.
void test_hip_dropout_clears_prev_hip_valid() {
    std::array<cv::Vec3f, kTrackerCount> prev{};
    PosSmoothingContext ctx;

    // Frame 1: hip at (0,0,1), tracker 8 valid at (0.1, 0.3, 0.05).
    ctx.current_hip_pos = cv::Vec3f{0.0f, 0.0f, 1.0f};
    ctx.hip_valid       = true;
    {
        auto ts = make_trackers({0.1f, 0.3f, 0.05f}, /*valid=*/true);
        apply_pos_smoothing(ts, prev, ctx, /*base_alpha=*/1.0f);
    }
    check(ctx.prev_hip_valid, "hip-dropout.frame1.prev_hip_valid set");

    // Frame 2: hip drops out (and tracker too). prev_hip_valid must clear.
    ctx.current_hip_pos = cv::Vec3f{0.0f, 0.0f, 1.0f};
    ctx.hip_valid       = false;
    {
        auto ts = make_trackers({999.0f, 999.0f, 999.0f}, /*valid=*/false);
        apply_pos_smoothing(ts, prev, ctx, /*base_alpha=*/0.5f);
    }
    check(!ctx.prev_hip_valid, "hip-dropout.frame2.prev_hip_valid cleared");

    // Frame 3: hip reappears 2 m away; tracker still invalid (held). Because
    // the previous frame's hip was invalid, the held tracker must NOT be
    // re-anchored — it holds its prev position. (With the bug it would
    // teleport to ~(2.1, 0.3, 0.05) via the stale prev_hip_pos.)
    ctx.current_hip_pos = cv::Vec3f{2.0f, 0.0f, 1.0f};
    ctx.hip_valid       = true;
    {
        auto ts = make_trackers({999.0f, 999.0f, 999.0f}, /*valid=*/false);
        apply_pos_smoothing(ts, prev, ctx, /*base_alpha=*/0.5f);
        check_vec3(ts[8].pos, {0.1f, 0.3f, 0.05f},
                   "hip-dropout.frame3.no re-anchor across gap");
    }
    // And prev_hip is now armed again for the next (both-valid) frame.
    check(ctx.prev_hip_valid, "hip-dropout.frame3.prev_hip_valid re-armed");
}

// First-frame-after-init must NOT trigger the velocity gate. prev starts at
// (0,0,0); a real first measurement at (0.3, 0.5, 1.0) is ~70 m/s in a 16 ms
// tick, well above the gate threshold. The initialized[] flag should
// suppress the gate so prev converges normally.
void test_first_valid_frame_skips_velocity_gate() {
    std::array<cv::Vec3f, kTrackerCount> prev{};
    PosSmoothingContext ctx;
    ctx.dt_s = 1.0f / 60.0f;

    auto ts = make_trackers({0.3f, 0.5f, 1.0f}, /*valid=*/true);
    apply_pos_smoothing(ts, prev, ctx, /*base_alpha=*/0.5f);

    // alpha=0.5 from (0,0,0) toward (0.3, 0.5, 1.0) → prev = (0.15, 0.25, 0.5)
    check_close(prev[0][0], 0.15f, "first-frame.prev.x");
    check_close(prev[0][1], 0.25f, "first-frame.prev.y");
    check_close(prev[0][2], 0.50f, "first-frame.prev.z");
    check(ctx.has_last_raw[0], "first-frame.has_last_raw");
}

// After a few valid frames have initialized the per-tracker state, an
// extreme jump must collapse the alpha. With prev settled around (1, 0, 0)
// and curr at (6, 0, 0) in one 16 ms tick (~300 m/s), prev must barely move.
void test_velocity_gate_attenuates_jump() {
    std::array<cv::Vec3f, kTrackerCount> prev{};
    PosSmoothingContext ctx;
    ctx.dt_s = 1.0f / 60.0f;

    // Converge prev to (1, 0, 0).
    for (int f = 0; f < 6; ++f) {
        auto ts = make_trackers({1.0f, 0.0f, 0.0f}, /*valid=*/true);
        apply_pos_smoothing(ts, prev, ctx, /*base_alpha=*/0.5f);
    }
    const float pre_x = prev[0][0];
    check(pre_x > 0.95f, "vel-gate.pre.converged");

    // 5 m jump in one tick → ~300 m/s, far above the 16 m/s gate ceiling.
    auto ts = make_trackers({6.0f, 0.0f, 0.0f}, /*valid=*/true);
    apply_pos_smoothing(ts, prev, ctx, /*base_alpha=*/0.5f);

    // prev should barely move (gate ~ 1, effective alpha ~ 0).
    const float delta = prev[0][0] - pre_x;
    check(delta < 0.05f,
          "vel-gate.attenuates: delta=" + std::to_string(delta) +
              " (want < 0.05 m for ~300 m/s spike)");
}

// Regression for codex review: after an N-frame dropout, the recovery
// frame's prev→curr displacement is divided by (1 + N) · dt_s, not by a
// single tick. Without this, a 0.5 m recovery after a 30-frame occlusion
// would compute v = 0.5 / (1/60) = 30 m/s and gate alpha to 0 — the
// tracker would stay stuck on its held position instead of converging
// toward the freshly observed location.
void test_velocity_gate_handles_multi_frame_dropout_recovery() {
    std::array<cv::Vec3f, kTrackerCount> prev{};
    PosSmoothingContext ctx;
    ctx.dt_s = 1.0f / 60.0f;

    // Converge prev to (1, 0, 0) so last_raw_pos[0] = (1, 0, 0).
    for (int f = 0; f < 6; ++f) {
        auto ts = make_trackers({1.0f, 0.0f, 0.0f}, /*valid=*/true);
        apply_pos_smoothing(ts, prev, ctx, /*base_alpha=*/0.5f);
    }
    check_close(ctx.last_raw_pos[0][0], 1.0f, "dropout-rec.seed.last_raw.x");

    // 30 invalid frames (subject occluded for 0.5 s at 60 Hz).
    for (int f = 0; f < 30; ++f) {
        auto ts = make_trackers({0.0f, 0.0f, 0.0f}, /*valid=*/false);
        apply_pos_smoothing(ts, prev, ctx, /*base_alpha=*/0.5f);
    }
    check(ctx.invalid_ticks_since_last_raw[0] == 30u,
          "dropout-rec.counter advanced");

    // Recovery: tracker reappears at (1.5, 0, 0) — a 0.5 m delta over
    // 31 ticks ≈ 0.97 m/s, well below the 8 m/s gate start. Without
    // the elapsed-tick correction, dist/dt = 0.5/(1/60) = 30 m/s →
    // gate would clamp alpha to 0.
    const float pre_x = prev[0][0];
    auto ts = make_trackers({1.5f, 0.0f, 0.0f}, /*valid=*/true);
    apply_pos_smoothing(ts, prev, ctx, /*base_alpha=*/0.5f);

    // alpha should be ~0.5 (no attenuation), so prev moves about halfway
    // toward 1.5 from its held position. Without the fix prev would barely
    // move.
    const float delta = prev[0][0] - pre_x;
    check(delta > 0.10f,
          "dropout-rec.gate.recover: delta=" + std::to_string(delta) +
              " (want > 0.10 m at alpha ≈ 0.5)");
    // And the invalid-tick counter must reset on the valid frame.
    check(ctx.invalid_ticks_since_last_raw[0] == 0u,
          "dropout-rec.counter reset on valid");
}

// Plausible 5 m/s motion (vigorous walking / jogging) must pass through the
// velocity gate without attenuation. With prev at (1,0,0) and curr at
// (1.083, 0, 0) in 16 ms ≈ 5 m/s, the EMA should converge normally.
void test_velocity_gate_passes_plausible_motion() {
    std::array<cv::Vec3f, kTrackerCount> prev{};
    PosSmoothingContext ctx;
    ctx.dt_s = 1.0f / 60.0f;

    // Settle prev.
    for (int f = 0; f < 6; ++f) {
        auto ts = make_trackers({1.0f, 0.0f, 0.0f}, /*valid=*/true);
        apply_pos_smoothing(ts, prev, ctx, /*base_alpha=*/0.5f);
    }

    // 5 m/s motion = 5 / 60 ≈ 0.083 m per tick. Should not be gated.
    auto ts = make_trackers({1.083f, 0.0f, 0.0f}, /*valid=*/true);
    apply_pos_smoothing(ts, prev, ctx, /*base_alpha=*/0.5f);
    // alpha=0.5 → prev = pre + 0.5 * (curr - pre)
    const float want = prev[0][0];  // capture; should be near halfway
    check(want > 1.03f && want < 1.05f,
          "vel-gate.passes.normal: prev=" + std::to_string(want));
}

void test_rate_independent_smoothing() {
    // Frame-rate independence: smoothing toward a constant target over the same
    // wall-clock time must land at the same place regardless of how many steps
    // it is split into. For the EMA this holds exactly — two dt=nominal/2 steps
    // equal one dt=nominal step (remaining factor (1-a)^0.5 squared = (1-a)).
    const cv::Vec3f target{4.0f, 0.0f, 0.0f};
    const float a = 0.5f, nom = 1.0f / 60.0f;

    // (a) one full-nominal step.
    std::array<cv::Vec3f, kTrackerCount> prev_full{};
    {
        auto ts = make_trackers(target, true);
        apply_pos_smoothing(ts, prev_full, a, nom, nom);
    }
    // (b) two half-nominal steps (same total wall time).
    std::array<cv::Vec3f, kTrackerCount> prev_half{};
    for (int s = 0; s < 2; ++s) {
        auto ts = make_trackers(target, true);
        apply_pos_smoothing(ts, prev_half, a, nom * 0.5f, nom);
    }
    for (std::size_t i = 0; i < kTrackerCount; ++i)
        check_close(prev_half[i][0], prev_full[i][0],
                    "rate-indep.half==full[" + std::to_string(i) + "]");

    // (c) dt == nominal must equal the legacy (dt defaulted) path exactly, so
    // the fixed-rate behavior is unchanged by the dt-aware generalization.
    std::array<cv::Vec3f, kTrackerCount> prev_dt{}, prev_legacy{};
    for (int f = 0; f < 4; ++f) {
        auto t1 = make_trackers(target, true);
        apply_pos_smoothing(t1, prev_dt, a, nom, nom);
        auto t2 = make_trackers(target, true);
        apply_pos_smoothing(t2, prev_legacy, a);  // default dt -> base_alpha
    }
    for (std::size_t i = 0; i < kTrackerCount; ++i)
        check_close(prev_dt[i][0], prev_legacy[i][0],
                    "rate-indep.dt==legacy[" + std::to_string(i) + "]");

    // (d) a higher rate (smaller dt) must take a smaller per-step alpha: after
    // ONE dt=nominal/2 step we should be less converged than one full step.
    std::array<cv::Vec3f, kTrackerCount> prev_one_half{};
    {
        auto ts = make_trackers(target, true);
        apply_pos_smoothing(ts, prev_one_half, a, nom * 0.5f, nom);
    }
    check(prev_one_half[0][0] < prev_full[0][0],
          "rate-indep: single half-dt step should under-shoot the full step");
}

}  // namespace

int main() {
    try {
        test_no_smooth_passthrough();
        test_smooth_convergence();
        test_invalid_holds_prev();
        test_dropout_recovery();
        test_hip_relative_hold_on_invalid();
        test_hip_dropout_clears_prev_hip_valid();
        test_first_valid_frame_skips_velocity_gate();
        test_velocity_gate_attenuates_jump();
        test_velocity_gate_handles_multi_frame_dropout_recovery();
        test_velocity_gate_passes_plausible_motion();
        test_rate_independent_smoothing();
        std::puts("test_tracker_extract_pos ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_tracker_extract_pos failed: %s\n", e.what());
        return 1;
    }
}
