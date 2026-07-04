// test_st_filter — spatiotemporal filter core (M-C2).
//
// Pins the pure regime primitives of docs/design/pose-3d-spatiotemporal-filter.md
// so the tuning in M-C4 and the wiring in M-C3 can't silently change the shape:
//   1. st_alpha_d      — deadband → ramp → normal, continuous at both knots
//   2. st_vel_gate     — 1 (trust) → 0 (reject), smoothstep between
//   3. deadband step   — STRONG FILTER with re-centring (not a freeze / no
//                        permanent offset), single step uses alpha_rest
//   4. lag cap         — fast trusted motion keeps the output within cap of target
//   5. outlier hold    — v > v_reject freezes the output (gate → 0)
//   6. rate adjust     — same wall-clock response at 1x and 2x frame rate
//   7. st_twist_angle  — signed twist about +Z, swing rejected
//   8. st_twist_alpha  — roll_confidence / velocity gating of the twist weight
//   9. default config  — seeded per-group table is well-formed

#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

#include "slimevr/st_filter.hpp"

namespace {

constexpr float kEps = 1.0e-4f;

using fitra::slimevr::SlimeTracker;
using fitra::slimevr::StFilterConfig;
using fitra::slimevr::StGroup;
using fitra::slimevr::StPosParams;
using fitra::slimevr::StPosState;
using fitra::slimevr::StRegime;
using fitra::slimevr::StTwistState;
using fitra::slimevr::TrackerRole;
using fitra::slimevr::apply_pos_st_filter;
using fitra::slimevr::apply_quat_smoothing;
using fitra::slimevr::default_st_config;
using fitra::slimevr::fill_st_twist_overrides;
using fitra::slimevr::kStGroupCount;
using fitra::slimevr::kTrackerCount;
using fitra::slimevr::st_alpha_d;
using fitra::slimevr::st_group_for;
using fitra::slimevr::st_has_roll;
using fitra::slimevr::st_pos_params;
using fitra::slimevr::st_pos_step;
using fitra::slimevr::st_twist_alpha;
using fitra::slimevr::st_twist_angle;
using fitra::slimevr::st_vel_gate;

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

float vnorm(const cv::Vec3f& v) {
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void check_vec3(const cv::Vec3f& got, const cv::Vec3f& want, const std::string& label,
                float eps = kEps) {
    for (int i = 0; i < 3; ++i) {
        check_close(got[i], want[i], label + "[" + std::to_string(i) + "]", eps);
    }
}

// Test regime: 1 cm deadband, 3 cm ramp end; rest 0.2, normal 0.6; vel gate 4→8.
StRegime test_regime() { return StRegime{0.01f, 0.03f, 0.2f, 0.6f, 4.0f, 8.0f}; }

cv::Vec4f q_ztwist(float theta) {
    return cv::Vec4f{std::cos(theta * 0.5f), 0.0f, 0.0f, std::sin(theta * 0.5f)};
}

// --------------------------------------------------------------------------- #

void test_alpha_d_shape() {
    const StRegime r = test_regime();
    check_close(st_alpha_d(0.005f, r), 0.2f, "alpha_d.below_core");
    check_close(st_alpha_d(0.05f, r),  0.6f, "alpha_d.above_full");
    // Midpoint of [d_core, d_full]: smoothstep(0.5) = 0.5 → rest + 0.5·(0.4).
    check_close(st_alpha_d(0.02f, r),  0.4f, "alpha_d.mid");
    // Continuity at the knots (approach from inside the ramp).
    check_close(st_alpha_d(0.0101f, r), 0.2f, "alpha_d.core_continuity", 2.0e-3f);
    check_close(st_alpha_d(0.0299f, r), 0.6f, "alpha_d.full_continuity", 2.0e-3f);
    // Monotone non-decreasing across the ramp.
    float prev = st_alpha_d(0.01f, r);
    for (float d = 0.011f; d <= 0.030f; d += 0.001f) {
        float a = st_alpha_d(d, r);
        check(a >= prev - 1.0e-6f, "alpha_d not monotone");
        prev = a;
    }
}

void test_vel_gate_shape() {
    const StRegime r = test_regime();
    check_close(st_vel_gate(2.0f, r),  1.0f, "gate.trust");
    check_close(st_vel_gate(10.0f, r), 0.0f, "gate.reject");
    check_close(st_vel_gate(6.0f, r),  0.5f, "gate.mid");  // smoothstep(0.5)
}

void test_deadband_recenters() {
    // Held sits 5 mm off a static target (inside the 1 cm deadband). It must
    // keep moving toward the target (strong filter, not a freeze) and leave no
    // permanent offset.
    const StPosParams p{test_regime(), 0.10f};
    const cv::Vec3f target{0.0f, 0.0f, 0.0f};
    const cv::Vec3f last_raw = target;  // static → v = 0, gate = 1
    const float dt = 1.0f / 60.0f;

    cv::Vec3f held{0.005f, 0.0f, 0.0f};
    // Single step: moves by alpha_rest (0.2) of the offset — NOT frozen, NOT a snap.
    cv::Vec3f after1 = st_pos_step(held, target, last_raw, dt, dt, p);
    check_close(after1[0], 0.004f, "deadband.single_step");

    for (int i = 0; i < 120; ++i) held = st_pos_step(held, target, last_raw, dt, dt, p);
    check(vnorm(held) < 1.0e-4f, "deadband did not re-centre (permanent offset)");
}

void test_lag_cap() {
    // Output far behind a trusted target: the raw stream is momentarily steady
    // (last_raw == target → v = 0 → gate = 1) but held lags by 0.5 m. The step's
    // EMA alone would leave it 0.2 m behind; the cap pulls it to within lag_cap.
    const StPosParams p{test_regime(), 0.10f};
    const cv::Vec3f held{0.0f, 0.0f, 0.0f};
    const cv::Vec3f target{0.5f, 0.0f, 0.0f};
    const cv::Vec3f last_raw = target;
    const float dt = 1.0f / 60.0f;

    cv::Vec3f out = st_pos_step(held, target, last_raw, dt, dt, p);
    check_close(vnorm(target - out), 0.10f, "lag_cap not enforced");
    check_close(out[0], 0.40f, "lag_cap.position");
}

void test_lag_cap_partial_gate() {
    // v inside (v_high, v_reject): the measurement is only partially trusted, so
    // the cap's pull must scale with the gate instead of snapping the full cap
    // distance toward a suspected glitch (and the output must stay continuous
    // as v crosses v_reject into the full hold).
    const StPosParams p{test_regime(), 0.10f};  // v_high 4, v_reject 8
    const cv::Vec3f held{0.0f, 0.0f, 0.0f};
    const cv::Vec3f target{0.5f, 0.0f, 0.0f};
    const float dt = 1.0f / 60.0f;

    // v = 6 m/s → gate = 0.5. EMA: alpha_normal·gate = 0.3 → 0.15; cap point is
    // 0.40; half-authority pull lands at 0.15 + 0.5·(0.40 − 0.15) = 0.275.
    {
        const cv::Vec3f last_raw{0.5f - 6.0f * dt, 0.0f, 0.0f};
        cv::Vec3f out = st_pos_step(held, target, last_raw, dt, dt, p);
        check_close(out[0], 0.275f, "partial gate: cap authority not proportional");
        check(out[0] < 0.40f - kEps, "partial gate: cap snapped the full distance");
    }
    // v = 7.9 m/s (just under v_reject) → gate ≈ 0.002: output must be ≈ the
    // hold (continuous with the v ≥ v_reject rejection), not a 0.4 m snap.
    {
        const cv::Vec3f last_raw{0.5f - 7.9f * dt, 0.0f, 0.0f};
        cv::Vec3f out = st_pos_step(held, target, last_raw, dt, dt, p);
        check(out[0] < 0.01f, "near-reject: output not continuous with the hold");
    }
}

void test_outlier_hold() {
    // A 1 m jump between consecutive raw frames = 60 m/s ≫ v_reject → gate 0 →
    // the output holds (no movement, and the cap does not drag it toward).
    const StPosParams p{test_regime(), 0.10f};
    const cv::Vec3f held{0.0f, 0.0f, 0.0f};
    const cv::Vec3f target{1.0f, 0.0f, 0.0f};
    const cv::Vec3f last_raw{0.0f, 0.0f, 0.0f};
    const float dt = 1.0f / 60.0f;

    cv::Vec3f out = st_pos_step(held, target, last_raw, dt, dt, p);
    check(vnorm(out - held) < kEps, "outlier not held");
}

void test_rate_adjust_independence() {
    // Same wall-clock step response at 1x and 2x frame rate. Disable the cap
    // (huge lag_cap) and keep d > d_full throughout so alpha_d ≡ alpha_normal,
    // isolating the rate adjust.
    const StPosParams p{test_regime(), 1.0e6f};
    const cv::Vec3f target{1.0f, 0.0f, 0.0f};
    const cv::Vec3f last_raw = target;  // v = 0, gate = 1
    const float nominal = 1.0f / 60.0f;

    cv::Vec3f a{0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; ++i) a = st_pos_step(a, target, last_raw, nominal, nominal, p);

    cv::Vec3f b{0.0f, 0.0f, 0.0f};
    const float half = nominal * 0.5f;
    for (int i = 0; i < 6; ++i) b = st_pos_step(b, target, last_raw, half, nominal, p);

    check_close(a[0], b[0], "rate adjust not frame-rate independent", 5.0e-3f);
}

void test_twist_angle() {
    // Pure +Z twist round-trip.
    check_close(st_twist_angle(q_ztwist(0.1f), q_ztwist(0.4f)), 0.3f, "twist.recover");
    // A pure swing about +X carries no twist about +Z.
    cv::Vec4f swing_x{std::cos(0.25f), std::sin(0.25f), 0.0f, 0.0f};
    check_close(st_twist_angle(cv::Vec4f{1, 0, 0, 0}, swing_x), 0.0f, "twist.swing_rejected");
}

void test_twist_alpha() {
    const StRegime r{0.05f, 0.2f, 0.1f, 0.5f, 4.0f, 12.0f};
    const float dt = 1.0f / 60.0f;
    // Degenerate roll (confidence 0) → held (weight 0).
    check_close(st_twist_alpha(0.3f, 0.0f, 0.0f, dt, dt, r), 0.0f, "twist_alpha.conf0");
    // Fast roll (> v_reject) → rejected regardless of distance / confidence.
    check_close(st_twist_alpha(0.3f, 20.0f, 1.0f, dt, dt, r), 0.0f, "twist_alpha.reject");
    // Small deadband roll, trusted → alpha_rest.
    check_close(st_twist_alpha(0.02f, 0.0f, 1.0f, dt, dt, r), 0.1f, "twist_alpha.rest");
    // Large roll, trusted → alpha_normal.
    check_close(st_twist_alpha(0.3f, 0.0f, 1.0f, dt, dt, r), 0.5f, "twist_alpha.normal");
}

void test_default_config() {
    const auto& cfg = default_st_config();
    // Group mapping.
    check(st_group_for(TrackerRole::LeftUpperArm)  == StGroup::UpperArm, "grp.arm");
    check(st_group_for(TrackerRole::RightUpperLeg) == StGroup::UpperLeg, "grp.leg");
    check(st_group_for(TrackerRole::Waist)         == StGroup::Waist,    "grp.waist");
    check(st_group_for(TrackerRole::LeftFoot)      == StGroup::Foot,     "grp.foot");
    // Roll regime is ARM-ONLY (M-C4/M-C5: legs get no twist benefit).
    check(st_has_roll(cfg, TrackerRole::LeftUpperArm),  "roll.arm on");
    check(st_has_roll(cfg, TrackerRole::RightUpperArm), "roll.arm on");
    check(!st_has_roll(cfg, TrackerRole::LeftUpperLeg),  "roll.upper_leg off (arm-only)");
    check(!st_has_roll(cfg, TrackerRole::RightUpperLeg), "roll.upper_leg off (arm-only)");
    check(!st_has_roll(cfg, TrackerRole::LeftLowerLeg),  "roll.lower_leg off (arm-only)");
    check(!st_has_roll(cfg, TrackerRole::Chest), "roll.chest off");
    check(!st_has_roll(cfg, TrackerRole::Waist), "roll.waist off");
    check(!st_has_roll(cfg, TrackerRole::RightFoot), "roll.foot off");
    // Every group's knobs are well-formed.
    for (std::size_t g = 0; g < kStGroupCount; ++g) {
        const auto& pp = cfg.pos[g];
        check(pp.regime.d_core < pp.regime.d_full, "cfg pos d_core<d_full");
        check(pp.regime.v_high < pp.regime.v_reject, "cfg pos v_high<v_reject");
        check(pp.regime.alpha_rest <= pp.regime.alpha_normal, "cfg pos a_rest<=a_norm");
        check(pp.regime.alpha_rest >= 0.0f && pp.regime.alpha_normal <= 1.0f, "cfg pos alpha range");
        check(pp.lag_cap_m > 0.0f, "cfg lag_cap>0");
        const auto& rr = cfg.roll[g];
        check(rr.d_core < rr.d_full, "cfg roll d_core<d_full");
        check(rr.v_high < rr.v_reject, "cfg roll v_high<v_reject");
    }
    // Waist is the world reference: it should be the slowest to move
    // (smallest normal alpha ≤ every other group's).
    const float waist_norm = st_pos_params(cfg, TrackerRole::Waist).regime.alpha_normal;
    check(waist_norm <= st_pos_params(cfg, TrackerRole::LeftUpperArm).regime.alpha_normal,
          "waist should not be faster than the arm");
}

// --------------------------------------------------------------------------- #
// M-C3 wiring: apply_pos_st_filter / fill_st_twist_overrides / byte-identity
// --------------------------------------------------------------------------- #

constexpr std::size_t kWaistIdx = static_cast<std::size_t>(TrackerRole::Waist);
constexpr std::size_t kArmIdx   = static_cast<std::size_t>(TrackerRole::LeftUpperArm);

// A tracker array with everything invalid; caller sets the few it needs.
std::array<SlimeTracker, kTrackerCount> blank_trackers() {
    std::array<SlimeTracker, kTrackerCount> t{};
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        t[i].role  = static_cast<TrackerRole>(i);
        t[i].valid = false;
        t[i].quat_wxyz = cv::Vec4f{1, 0, 0, 0};
        t[i].roll_confidence = 1.0f;
        t[i].swing_confidence = 1.0f;
    }
    return t;
}

void set_pos(std::array<SlimeTracker, kTrackerCount>& t, std::size_t i,
             const cv::Vec3f& p) {
    t[i].pos = p;
    t[i].valid = true;
}

void test_st_pos_whole_body_translation_no_limb_lag() {
    // A rigid waist+arm translated fast: the arm's absolute position must track
    // its true position (the translation is absorbed by the waist reference),
    // while the waist itself carries the single filter's lag. This is the
    // waist-relative payoff — the limb does NOT inherit the translation lag.
    const StFilterConfig cfg = default_st_config();
    StPosState st{};
    const float dt = 1.0f / 60.0f;
    const cv::Vec3f arm_off{0.2f, 0.0f, 0.4f};
    const float dv = 0.03f;  // m/frame ≈ 1.8 m/s (trusted, normal regime)

    cv::Vec3f waist_true{0.0f, 0.0f, 1.0f};
    cv::Vec3f arm_out, waist_out, arm_true;
    for (int k = 0; k <= 20; ++k) {
        auto t = blank_trackers();
        waist_true = cv::Vec3f{dv * static_cast<float>(k), 0.0f, 1.0f};
        arm_true   = waist_true + arm_off;
        set_pos(t, kWaistIdx, waist_true);
        set_pos(t, kArmIdx,   arm_true);
        apply_pos_st_filter(t, st, cfg, dt, dt);
        waist_out = t[kWaistIdx].pos;
        arm_out   = t[kArmIdx].pos;
    }
    const float arm_err   = vnorm(arm_out - arm_true);
    const float waist_err = vnorm(waist_out - waist_true);
    check(arm_err < 0.005f, "translation: arm should not lag (abs err < 5mm)");
    check(arm_err < waist_err, "translation: arm should track better than the waist");
    check(waist_err > 0.015f, "translation: the waist should carry the filter lag");
}

void test_st_pos_static_jitter_suppressed() {
    // Stationary waist; arm jitters ±5 mm (inside the deadband). The relative
    // frame + strong rest filter should crush the output swing.
    const StFilterConfig cfg = default_st_config();
    StPosState st{};
    const float dt = 1.0f / 60.0f;
    const cv::Vec3f waist{0.0f, 0.0f, 1.0f};
    const cv::Vec3f arm0{0.2f, 0.0f, 1.4f};
    const float amp = 0.005f;  // < upper_arm d_core (0.012)

    float lo = 1e9f, hi = -1e9f;
    for (int k = 0; k <= 80; ++k) {
        auto t = blank_trackers();
        set_pos(t, kWaistIdx, waist);
        const float j = (k % 2 == 0) ? amp : -amp;  // deterministic square jitter
        set_pos(t, kArmIdx, arm0 + cv::Vec3f{j, 0.0f, 0.0f});
        apply_pos_st_filter(t, st, cfg, dt, dt);
        if (k >= 20) {  // after warmup
            lo = std::min(lo, t[kArmIdx].pos[0]);
            hi = std::max(hi, t[kArmIdx].pos[0]);
        }
    }
    // Input swing is 2·amp = 10 mm; the deadband filter should cut it well below.
    check((hi - lo) < 0.003f, "static jitter not suppressed (output swing >= 3mm)");
}

void test_st_pos_invalid_limb_drags_with_waist() {
    // Seed waist+arm, then move the waist while the arm goes invalid: the arm
    // must ride the waist (relative offset held), not freeze in world.
    const StFilterConfig cfg = default_st_config();
    StPosState st{};
    const float dt = 1.0f / 60.0f;
    const cv::Vec3f arm_off{0.2f, 0.0f, 0.4f};

    // Frame 0: snap both at rest.
    {
        auto t = blank_trackers();
        set_pos(t, kWaistIdx, {0.0f, 0.0f, 1.0f});
        set_pos(t, kArmIdx,   cv::Vec3f{0.0f, 0.0f, 1.0f} + arm_off);
        apply_pos_st_filter(t, st, cfg, dt, dt);
    }
    // Frames 1..6: waist moves to +X, arm INVALID (held).
    cv::Vec3f arm_out, waist_out;
    for (int k = 1; k <= 6; ++k) {
        auto t = blank_trackers();
        set_pos(t, kWaistIdx, {0.1f, 0.0f, 1.0f});
        // arm left invalid
        apply_pos_st_filter(t, st, cfg, dt, dt);
        waist_out = t[kWaistIdx].pos;
        arm_out   = t[kArmIdx].pos;
    }
    // The held relative offset is preserved: arm rides the waist exactly.
    check_close((arm_out - waist_out)[0], arm_off[0], "invalid arm x offset held");
    check_close((arm_out - waist_out)[2], arm_off[2], "invalid arm z offset held");
    // And it actually moved with the waist (not frozen at the world seed 0.2).
    check(arm_out[0] > 0.2f + 0.02f, "invalid arm did not drag with the waist");
}

void test_st_pos_recovery_snaps() {
    // After a dropout, a returning limb at a far-away position must SNAP to the
    // measurement (not be rejected as an outlier or lag-capped toward it).
    const StFilterConfig cfg = default_st_config();
    StPosState st{};
    const float dt = 1.0f / 60.0f;

    {   // seed
        auto t = blank_trackers();
        set_pos(t, kWaistIdx, {0.0f, 0.0f, 1.0f});
        set_pos(t, kArmIdx,   {0.2f, 0.0f, 1.4f});
        apply_pos_st_filter(t, st, cfg, dt, dt);
    }
    {   // arm dropout
        auto t = blank_trackers();
        set_pos(t, kWaistIdx, {0.0f, 0.0f, 1.0f});
        apply_pos_st_filter(t, st, cfg, dt, dt);
    }
    {   // arm returns far away → snap
        auto t = blank_trackers();
        set_pos(t, kWaistIdx, {0.0f, 0.0f, 1.0f});
        set_pos(t, kArmIdx,   {0.7f, 0.0f, 1.4f});  // 0.5 m jump
        apply_pos_st_filter(t, st, cfg, dt, dt);
        check_vec3(t[kArmIdx].pos, {0.7f, 0.0f, 1.4f}, "recovery did not snap", 1.0e-4f);
    }
}

void test_st_pos_waist_recovery_limb_continuity() {
    // The waist drops out and recovers DISPLACED: the reference origin jump is
    // the waist's motion, not the limbs' — a steady limb whose own measurement
    // never moved must keep a continuous world output (no teleport by the
    // waist's dropout displacement, no velocity-gate hold of the frame shift).
    const StFilterConfig cfg = default_st_config();
    StPosState st{};
    const float dt = 1.0f / 60.0f;
    const cv::Vec3f arm_world{0.2f, 0.0f, 1.4f};

    {   // seed both
        auto t = blank_trackers();
        set_pos(t, kWaistIdx, {0.0f, 0.0f, 1.0f});
        set_pos(t, kArmIdx,   arm_world);
        apply_pos_st_filter(t, st, cfg, dt, dt);
    }
    for (int k = 0; k < 5; ++k) {  // waist dropout; arm still measured, static
        auto t = blank_trackers();
        set_pos(t, kArmIdx, arm_world);
        apply_pos_st_filter(t, st, cfg, dt, dt);
    }
    {   // waist recovers 0.5 m away; the arm's own measurement is unchanged
        auto t = blank_trackers();
        set_pos(t, kWaistIdx, {0.5f, 0.0f, 1.0f});
        set_pos(t, kArmIdx,   arm_world);
        apply_pos_st_filter(t, st, cfg, dt, dt);
        check_vec3(t[kWaistIdx].pos, {0.5f, 0.0f, 1.0f}, "waist recovery snap");
        check_vec3(t[kArmIdx].pos, arm_world, "steady limb teleported on waist recovery", 1.0e-3f);
    }
}

void test_st_twist_overrides_scope() {
    const StFilterConfig cfg = default_st_config();
    StTwistState st{};
    const float dt = 1.0f / 60.0f;
    std::array<cv::Vec4f, kTrackerCount> prev_quat{};
    for (auto& q : prev_quat) q = cv::Vec4f{1, 0, 0, 0};

    auto curr = blank_trackers();
    curr[kArmIdx].valid = true;                              // has_roll
    curr[kArmIdx].quat_wxyz = q_ztwist(0.3f);
    curr[kArmIdx].roll_confidence = 1.0f;
    const std::size_t chest = static_cast<std::size_t>(TrackerRole::Chest);
    curr[chest].valid = true;                                // NOT has_roll
    curr[chest].quat_wxyz = q_ztwist(0.3f);

    std::array<float, kTrackerCount> ov{};
    // First call: has_roll bone is not yet steady → sentinel (existing path snaps).
    fill_st_twist_overrides(curr, prev_quat, st, cfg, dt, dt, ov);
    check(ov[kArmIdx] < 0.0f, "first-frame has_roll should be sentinel");
    check(ov[chest] < 0.0f,   "non-has_roll must always be sentinel");

    // Second call: now steady → regime-driven weight in [0, 1].
    fill_st_twist_overrides(curr, prev_quat, st, cfg, dt, dt, ov);
    check(ov[kArmIdx] >= 0.0f && ov[kArmIdx] <= 1.0f, "steady has_roll should be a regime weight");
    check(ov[chest] < 0.0f, "non-has_roll still sentinel");

    // Invalid has_roll bone → sentinel + steady reset.
    curr[kArmIdx].valid = false;
    fill_st_twist_overrides(curr, prev_quat, st, cfg, dt, dt, ov);
    check(ov[kArmIdx] < 0.0f, "invalid has_roll should be sentinel");
}

void test_quat_override_nullptr_byte_identical() {
    // apply_quat_smoothing with a null override (default) must be bit-identical
    // to the pre-M-C3 call, and to passing an all-sentinel override. This pins
    // the st_filter OFF path.
    const float dt = 1.0f / 60.0f;
    auto make = []() {
        auto t = blank_trackers();
        for (std::size_t i = 0; i < kTrackerCount; ++i) {
            t[i].valid = true;
            t[i].quat_wxyz = q_ztwist(0.2f + 0.03f * static_cast<float>(i));
            t[i].roll_confidence  = 0.5f;   // exercise the split (sa != ta) branch
            t[i].swing_confidence = 1.0f;
        }
        return t;
    };
    std::array<cv::Vec4f, kTrackerCount> pa{}, pb{};
    for (auto& q : pa) q = cv::Vec4f{1, 0, 0, 0};
    pb = pa;

    auto ta = make();
    auto tb = make();
    apply_quat_smoothing(ta, pa, 0.5f, dt, dt);            // no override arg (default null)
    std::array<float, kTrackerCount> sentinel;
    sentinel.fill(-1.0f);
    apply_quat_smoothing(tb, pb, 0.5f, dt, dt, &sentinel); // explicit all-sentinel

    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        for (int c = 0; c < 4; ++c) {
            check_close(ta[i].quat_wxyz[c], tb[i].quat_wxyz[c],
                        "override sentinel != nullptr at tracker " + std::to_string(i));
        }
    }
}

}  // namespace

int main() {
    try {
        test_alpha_d_shape();
        test_vel_gate_shape();
        test_deadband_recenters();
        test_lag_cap();
        test_lag_cap_partial_gate();
        test_outlier_hold();
        test_rate_adjust_independence();
        test_twist_angle();
        test_twist_alpha();
        test_default_config();
        test_st_pos_whole_body_translation_no_limb_lag();
        test_st_pos_static_jitter_suppressed();
        test_st_pos_invalid_limb_drags_with_waist();
        test_st_pos_recovery_snaps();
        test_st_pos_waist_recovery_limb_continuity();
        test_st_twist_overrides_scope();
        test_quat_override_nullptr_byte_identical();
        std::puts("test_st_filter ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_st_filter failed: %s\n", e.what());
        return 1;
    }
}
