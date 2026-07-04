// test_tracker_extract_roll — arm/thigh roll gate-raise hysteresis (#2).
//
// extract_trackers(..., roll_hysteresis=true) raises the effective roll trust
// gate for the inferred-roll bones (upper arm / thigh) with an acquire/release
// hysteresis on sin θ(up, forward), so a straightening limb HOLDS its last
// confident roll (confidence → 0) through the noisy near-degenerate band
// instead of following the amplified mid-band roll noise. The emitted
// roll_confidence is the observable, so we assert it directly.
//
//   1. flag OFF (or null ctx) is byte-identical to the legacy path
//   2. arm hysteresis: lock at acquire, hold through release band, no re-lock
//      until acquire again — vs OFF which follows the mid-band
//   3. thigh hysteresis (same, tracker idx 4)
//   4. fully-degenerate / first-frame: no lock, valid + finite quat, no NaN
//   5. reset (ExtractContext{}) clears the latch
//   6. dropout lifecycle: a missing measurement (wrist KP / shoulder+elbow) is
//      latch-neutral for short gaps and clears the latch after a long occlusion

#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

#include "infer/types.hpp"
#include "lift/keypoint_format.hpp"
#include "slimevr/tracker_extract.hpp"

namespace {

using fitra::infer::Skeleton3D;
using fitra::slimevr::ExtractContext;
using fitra::slimevr::FootPosMode;
using fitra::slimevr::SlimeTracker;
using fitra::slimevr::extract_trackers;
using fitra::slimevr::kTrackerCount;

constexpr std::size_t kLArm = 0;   // LeftUpperArm out idx
constexpr std::size_t kLLeg = 4;   // LeftUpperLeg  out idx

void check(bool cond, const std::string& msg) {
    if (!cond) throw std::runtime_error(msg);
}

void set_joint(Skeleton3D& s, std::size_t j, float x, float y, float z) {
    s.joints[j].x = x; s.joints[j].y = y; s.joints[j].z = z;
    s.joints[j].score = 0.9f; s.joints[j].valid = true;
}

// Arm with a controlled sin θ between (wrist-elbow) and (elbow-shoulder):
// shoulder@origin, elbow arm-down (forward -Z), wrist bent by asin(sinth).
// Geometry gives sin θ == sinth exactly (see test comment / derivation).
Skeleton3D arm_pose(float sinth) {
    Skeleton3D s; s.kp_count = 26;
    const float c = std::sqrt(std::max(0.0f, 1.0f - sinth * sinth));
    set_joint(s, 5, 0.0f, 0.0f, 0.0f);                       // l_shoulder
    set_joint(s, 7, 0.0f, 0.0f, -0.3f);                      // l_elbow (fwd = -Z)
    set_joint(s, 9, 0.25f * sinth, 0.0f, -0.3f - 0.25f * c); // l_wrist
    return s;
}

// Thigh version: hip@origin, knee down, ankle bent by asin(sinth).
Skeleton3D leg_pose(float sinth) {
    Skeleton3D s; s.kp_count = 26;
    const float c = std::sqrt(std::max(0.0f, 1.0f - sinth * sinth));
    set_joint(s, 11, 0.0f, 0.0f, 0.0f);                       // l_hip
    set_joint(s, 13, 0.0f, 0.0f, -0.4f);                      // l_knee (fwd = -Z)
    set_joint(s, 15, 0.4f * sinth, 0.0f, -0.4f - 0.4f * c);   // l_ankle
    return s;
}

float roll_conf(const Skeleton3D& s, ExtractContext* ctx, bool hyst, std::size_t idx) {
    auto t = extract_trackers(s, ctx, FootPosMode::Midpoint, 0.5f, 0.0f, hyst);
    check(t[idx].valid, "tracker should be valid for the pose");
    return t[idx].roll_confidence;
}

// ---- tests --------------------------------------------------------------- #

void test_flag_off_byte_identical() {
    // Legacy call (null ctx, no hysteresis) vs &ctx with roll_hysteresis=false
    // must match bit-for-bit across a battery of arm/thigh poses.
    for (float sinth : {0.6f, 0.35f, 0.22f, 0.05f}) {
        for (int leg = 0; leg < 2; ++leg) {
            Skeleton3D s = leg ? leg_pose(sinth) : arm_pose(sinth);
            auto a = extract_trackers(s);                                   // legacy defaults
            ExtractContext ctx{};
            auto b = extract_trackers(s, &ctx, FootPosMode::Midpoint, 0.5f, 0.0f, false);
            for (std::size_t i = 0; i < kTrackerCount; ++i) {
                check(a[i].valid == b[i].valid, "off: valid mismatch");
                check(a[i].roll_confidence == b[i].roll_confidence, "off: roll_conf mismatch");
                check(a[i].swing_confidence == b[i].swing_confidence, "off: swing_conf mismatch");
                for (int c = 0; c < 4; ++c)
                    check(a[i].quat_wxyz[c] == b[i].quat_wxyz[c], "off: quat mismatch");
            }
        }
    }
}

void test_arm_hysteresis_sequence() {
    // OFF vs ON at the SAME mid-band angle, from a fresh (never-locked) latch:
    //  sin 0.35 is above the legacy ceiling (0.30) so OFF fully trusts it (1.0),
    //  but below acquire (0.42) so ON has never locked → holds (0.0).
    check(roll_conf(arm_pose(0.35f), nullptr, false, kLArm) == 1.0f,
          "OFF trusts mid-band 0.35 (>=legacy ceiling)");
    { ExtractContext ctx{};
      check(roll_conf(arm_pose(0.35f), &ctx, true, kLArm) == 0.0f,
            "ON never-locked at 0.35 holds (below acquire)"); }

    // Hysteresis walk with ONE persistent latch:
    ExtractContext ctx{};
    check(roll_conf(arm_pose(0.60f), &ctx, true, kLArm) > 0.0f,  "lock at acquire (0.60)");
    check(roll_conf(arm_pose(0.35f), &ctx, true, kLArm) > 0.0f,  "stay locked in release band (0.35)");
    check(roll_conf(arm_pose(0.20f), &ctx, true, kLArm) == 0.0f, "unlock below release (0.20) -> hold");
    check(roll_conf(arm_pose(0.35f), &ctx, true, kLArm) == 0.0f, "no re-lock at 0.35 (hysteresis)");
    check(roll_conf(arm_pose(0.60f), &ctx, true, kLArm) > 0.0f,  "re-lock at acquire (0.60)");
    // OFF at 0.20 still follows the noisy band (non-zero), ON held it at 0.
    check(roll_conf(arm_pose(0.20f), nullptr, false, kLArm) > 0.0f,
          "OFF follows noisy 0.20 (legacy smoothstep > 0)");
}

void test_thigh_hysteresis_sequence() {
    ExtractContext ctx{};
    check(roll_conf(leg_pose(0.60f), &ctx, true, kLLeg) > 0.0f,  "thigh lock (0.60)");
    check(roll_conf(leg_pose(0.20f), &ctx, true, kLLeg) == 0.0f, "thigh unlock/hold (0.20)");
    check(roll_conf(leg_pose(0.35f), &ctx, true, kLLeg) == 0.0f, "thigh no re-lock at 0.35");
    // OFF follows the same 0.35 fully.
    check(roll_conf(leg_pose(0.35f), nullptr, false, kLLeg) == 1.0f, "thigh OFF trusts 0.35");
}

void test_degenerate_no_nan() {
    // Fully extended (sin θ ≈ 0): never locks, roll held (conf 0), but the bone
    // direction is still emitted valid with a finite unit quat.
    ExtractContext ctx{};
    auto t = extract_trackers(arm_pose(0.0f), &ctx, FootPosMode::Midpoint, 0.5f, 0.0f, true);
    check(t[kLArm].valid, "degenerate arm still valid (forward-only)");
    check(t[kLArm].roll_confidence == 0.0f, "degenerate arm holds roll (conf 0)");
    float n2 = 0.0f;
    for (int c = 0; c < 4; ++c) {
        check(std::isfinite(t[kLArm].quat_wxyz[c]), "degenerate quat finite");
        n2 += t[kLArm].quat_wxyz[c] * t[kLArm].quat_wxyz[c];
    }
    check(std::abs(std::sqrt(n2) - 1.0f) < 1.0e-3f, "degenerate quat unit norm");
}

void test_reset_clears_latch() {
    ExtractContext ctx{};
    check(roll_conf(arm_pose(0.60f), &ctx, true, kLArm) > 0.0f, "lock before reset");
    ctx = ExtractContext{};  // reset_smoothing() equivalent
    // After reset the latch is cleared, so a mid-band angle below acquire holds.
    check(roll_conf(arm_pose(0.35f), &ctx, true, kLArm) == 0.0f, "reset cleared the latch");
}

void test_occlusion_latch_lifecycle() {
    // A missing measurement is NOT "limb measured straight": a zero up vector
    // (dropped wrist KP) or missing shoulder/elbow must not release the latch
    // on a short flicker (that would freeze a mid-band roll until re-acquire),
    // while a LONG occlusion must clear it (the limb may have straightened
    // while unseen — resume needs a fresh acquire).
    Skeleton3D dropout; dropout.kp_count = 26;  // shoulder/elbow invalid

    // (a) 1-frame wrist dropout: roll held for the frame, latch kept.
    {
        ExtractContext ctx{};
        check(roll_conf(arm_pose(0.60f), &ctx, true, kLArm) > 0.0f, "lock (0.60)");
        Skeleton3D no_wrist = arm_pose(0.35f);
        no_wrist.joints[9].valid = false;   // sin θ unobservable this frame
        check(roll_conf(no_wrist, &ctx, true, kLArm) == 0.0f,
              "missing wrist holds roll for the frame");
        check(roll_conf(arm_pose(0.35f), &ctx, true, kLArm) > 0.0f,
              "latch survives a 1-frame wrist flicker (mid-band resumes)");
    }
    // (b) short shoulder/elbow occlusion: latch kept.
    {
        ExtractContext ctx{};
        check(roll_conf(arm_pose(0.60f), &ctx, true, kLArm) > 0.0f, "lock (0.60)");
        for (int k = 0; k < 3; ++k)
            extract_trackers(dropout, &ctx, FootPosMode::Midpoint, 0.5f, 0.0f, true);
        check(roll_conf(arm_pose(0.35f), &ctx, true, kLArm) > 0.0f,
              "latch survives a short occlusion");
    }
    // (c) long occlusion: latch cleared, mid-band resume must re-acquire.
    {
        ExtractContext ctx{};
        check(roll_conf(arm_pose(0.60f), &ctx, true, kLArm) > 0.0f, "lock (0.60)");
        for (int k = 0; k < 40; ++k)
            extract_trackers(dropout, &ctx, FootPosMode::Midpoint, 0.5f, 0.0f, true);
        check(roll_conf(arm_pose(0.35f), &ctx, true, kLArm) == 0.0f,
              "long occlusion forces re-acquire (mid-band held)");
        check(roll_conf(arm_pose(0.60f), &ctx, true, kLArm) > 0.0f, "re-acquire at 0.60");
    }
}

}  // namespace

int main() {
    try {
        fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
        test_flag_off_byte_identical();
        test_arm_hysteresis_sequence();
        test_thigh_hysteresis_sequence();
        test_degenerate_no_nan();
        test_reset_clears_latch();
        test_occlusion_latch_lifecycle();
        std::puts("test_tracker_extract_roll ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_tracker_extract_roll failed: %s\n", e.what());
        return 1;
    }
}
