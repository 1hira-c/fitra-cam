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

using fitra::slimevr::StGroup;
using fitra::slimevr::StPosParams;
using fitra::slimevr::StRegime;
using fitra::slimevr::TrackerRole;
using fitra::slimevr::default_st_config;
using fitra::slimevr::kStGroupCount;
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
    // Roll is inferred (regime-driven) only for the limb groups.
    check(st_has_roll(cfg, TrackerRole::LeftUpperArm), "roll.arm on");
    check(st_has_roll(cfg, TrackerRole::RightUpperLeg), "roll.leg on");
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

}  // namespace

int main() {
    try {
        test_alpha_d_shape();
        test_vel_gate_shape();
        test_deadband_recenters();
        test_lag_cap();
        test_outlier_hold();
        test_rate_adjust_independence();
        test_twist_angle();
        test_twist_alpha();
        test_default_config();
        std::puts("test_st_filter ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_st_filter failed: %s\n", e.what());
        return 1;
    }
}
