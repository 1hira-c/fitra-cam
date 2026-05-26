// test_tracker_extract — exercise Halpe26 → 10 SlimeVR tracker extraction +
// quaternion smoothing. Output is in the WORLD frame (Z-up); the publisher
// applies the Y-up conversion when serializing.
//
// Synthetic T-pose verifies:
//   - all 10 trackers (upper arms / chest / waist / upper legs / lower legs /
//     feet) produce a valid orientation
//   - position outputs are in the expected world coordinates
//   - quaternion-from-forward-up via Shoemake's matrix-to-quat
//   - degeneracy handling (parallel forward/up, missing joints)
//   - slerp smoothing direction handling (q · -q double-cover)
//   - keypoint-format assertion (Halpe26 only)

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
#include "slimevr/firmware_protocol.hpp"

namespace {

constexpr float kEps = 1.0e-4f;

void check(bool cond, const std::string& msg) {
    if (!cond) throw std::runtime_error(msg);
}

void check_vec3_close(const cv::Vec3f& got, const cv::Vec3f& want, const std::string& label,
                      float eps = kEps) {
    for (int i = 0; i < 3; ++i) {
        float d = std::abs(got[i] - want[i]);
        if (d > eps) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                "%s: vec3[%d] got=%.6f want=%.6f diff=%.6f",
                label.c_str(), i, got[i], want[i], d);
            throw std::runtime_error(buf);
        }
    }
}

void set_joint(fitra::infer::Skeleton3D& s, std::size_t i,
               float x, float y, float z, bool valid = true) {
    s.joints[i].x = x; s.joints[i].y = y; s.joints[i].z = z;
    s.joints[i].score = 1.0f;
    s.joints[i].valid = valid;
}

cv::Vec3f vec_normalize(const cv::Vec3f& v) {
    float n = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (n < 1.0e-9f) return cv::Vec3f{0, 0, 0};
    return cv::Vec3f{v[0]/n, v[1]/n, v[2]/n};
}

// Reconstruct the rotation-matrix columns from a wxyz unit quaternion, matching
// the column convention used in detail::quat_from_forward_up (R = [right|up|fwd]).
void quat_to_basis(const cv::Vec4f& q, cv::Vec3f& right, cv::Vec3f& up, cv::Vec3f& fwd) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    right = cv::Vec3f{1 - 2*(y*y + z*z), 2*(x*y + w*z),     2*(x*z - w*y)};
    up    = cv::Vec3f{2*(x*y - w*z),     1 - 2*(x*x + z*z), 2*(y*z + w*x)};
    fwd   = cv::Vec3f{2*(x*z + w*y),     2*(y*z - w*x),     1 - 2*(x*x + y*y)};
}

// Synthetic Halpe26 T-pose: subject standing at origin facing +Y, arms
// outstretched, knees slightly forward (so the up/forward hints on knee /
// elbow trackers are linearly independent — a perfectly straight limb makes
// the rotation about the limb axis genuinely ambiguous, both for SlimeVR's
// solver and for any vector-based rotation reconstruction).
fitra::infer::Skeleton3D make_t_pose() {
    fitra::infer::Skeleton3D s;
    s.kp_count = 26;
    set_joint(s, 19, 0,    0,     0.9f);   // hip_center
    set_joint(s, 11, 0.1f, 0,     0.9f);   // l_hip
    set_joint(s, 12, -0.1f, 0,    0.9f);   // r_hip
    set_joint(s, 18, 0,    0,     1.45f);  // neck
    set_joint(s,  5, 0.18f,0,     1.42f);  // l_shoulder
    set_joint(s,  6, -0.18f,0,    1.42f);  // r_shoulder
    set_joint(s,  7, 0.45f, 0.02f, 1.42f); // l_elbow (~3 cm forward)
    // Wrists dropped 15 cm in Z (forearm hangs slightly below the
    // upper-arm axis). The previous strictly-horizontal arm had
    // wrist - elbow ∥ elbow - shoulder (both pointing +X with only ~3 cm Y
    // offset), so primary up was sin θ ≈ 0.01 from fwd → falls into the
    // degeneracy gate (kRollSinLow = 0.15). With wrist dropped, primary up
    // gains a substantial -Z component → sin θ ≈ 0.48 → full confidence,
    // upper-arm trackers stay valid in the T-pose. Anatomically: arms
    // extended laterally with forearms relaxed downward 12°.
    set_joint(s,  9, 0.72f, 0.05f, 1.27f); // l_wrist
    set_joint(s,  8, -0.45f,0.02f, 1.42f); // r_elbow
    set_joint(s, 10, -0.72f,0.05f, 1.27f); // r_wrist
    // T-pose has anatomically realistic mild knee flexion (knee 10 cm
    // forward of the hip-ankle line ≈ 12°). The previous 1 cm offset was a
    // visually-T-shaped figure but anatomically degenerate — both thigh fwd
    // (knee-hip) and shin up (hip-knee) became near-parallel to the leg axis,
    // and the new sin θ-based degeneracy gate in quat_from_forward_up rejects
    // any quat built from such inputs (rightly: roll is unobservable). With
    // the 10 cm knee bend, sin θ ≈ 0.34 for both shin and thigh, comfortably
    // above kRollSinHigh, so all 10 trackers stay valid in the T-pose.
    set_joint(s, 13, 0.1f,  0.10f, 0.45f); // l_knee
    set_joint(s, 14, -0.1f, 0.10f, 0.45f); // r_knee
    set_joint(s, 15, 0.1f,  0.05f, 0.05f); // l_ankle
    set_joint(s, 16, -0.1f, 0.05f, 0.05f); // r_ankle
    set_joint(s, 24, 0.1f,  0.02f, 0.0f);  // l_heel
    set_joint(s, 20, 0.1f,  0.17f, 0.0f);  // l_big_toe
    set_joint(s, 25, -0.1f, 0.02f, 0.0f);  // r_heel
    set_joint(s, 21, -0.1f, 0.17f, 0.0f);  // r_big_toe
    set_joint(s,  0, 0, 0, 1.65f);         // nose (not used)
    set_joint(s, 17, 0, 0, 1.72f);         // head_top (not used)
    return s;
}

void test_quat_from_forward_up_degenerate() {
    cv::Vec4f q;
    bool ok = fitra::slimevr::detail::quat_from_forward_up(
        cv::Vec3f{0, 0, 1}, cv::Vec3f{0, 0, 1}, q);
    check(!ok, "quat_from_forward_up parallel inputs should be invalid");
    check_vec3_close(cv::Vec3f{q[0], q[1], q[2]}, cv::Vec3f{1, 0, 0},
                     "degenerate quat should be identity wxyz=(1,0,0,0)");
    check(std::abs(q[3]) < kEps, "degenerate quat z component");

    ok = fitra::slimevr::detail::quat_from_forward_up(
        cv::Vec3f{0, 0, 0}, cv::Vec3f{0, 0, 1}, q);
    check(!ok, "quat_from_forward_up zero forward should be invalid");
}

void test_t_pose_extracts_all_ten() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_t_pose();
    auto trackers = fitra::slimevr::extract_trackers(skel);

    check(trackers.size() == 10, "must produce exactly 10 trackers");

    // All 10 trackers should be valid for a complete T-pose.
    for (std::size_t i = 0; i < trackers.size(); ++i) {
        if (!trackers[i].valid) {
            throw std::runtime_error(
                "tracker " + std::to_string(i) + " unexpectedly invalid");
        }
    }

    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };

    // Spot-check positions in WORLD frame (Z-up, X-right, Y-forward).
    // LeftUpperArm pos = midpoint(l_shoulder(0.18,0,1.42), l_elbow(0.45,0.02,1.42))
    //                  = (0.315, 0.01, 1.42)
    check_vec3_close(trackers[idx(R::LeftUpperArm)].pos,
                     cv::Vec3f{0.315f, 0.01f, 1.42f},
                     "LeftUpperArm pos (world)");
    // Chest pos = midpoint(neck(0,0,1.45), hip_center(0,0,0.9)) = (0, 0, 1.175)
    check_vec3_close(trackers[idx(R::Chest)].pos,
                     cv::Vec3f{0, 0, 1.175f},
                     "Chest pos (world)");
    // Waist pos = hip_center = (0, 0, 0.9)
    check_vec3_close(trackers[idx(R::Waist)].pos,
                     cv::Vec3f{0, 0, 0.9f},
                     "Waist pos (world)");
    // LeftUpperLeg pos = midpoint(l_hip(0.1,0,0.9), l_knee(0.1,0.10,0.45))
    //                  = (0.1, 0.05, 0.675)
    check_vec3_close(trackers[idx(R::LeftUpperLeg)].pos,
                     cv::Vec3f{0.1f, 0.05f, 0.675f},
                     "LeftUpperLeg pos (world)");
    // LeftLowerLeg pos = midpoint(l_knee(0.1,0.10,0.45), l_ankle(0.1,0.05,0.05))
    //                  = (0.1, 0.075, 0.25)
    check_vec3_close(trackers[idx(R::LeftLowerLeg)].pos,
                     cv::Vec3f{0.1f, 0.075f, 0.25f},
                     "LeftLowerLeg pos (world)");
    // LeftFoot pos = midpoint(l_ankle(0.1,0.05,0.05), l_big_toe(0.1,0.17,0))
    //              = (0.1, 0.11, 0.025). Foot no longer uses heel — heel KP is
    //              too noisy in the 2D→3D pipeline.
    check_vec3_close(trackers[idx(R::LeftFoot)].pos,
                     cv::Vec3f{0.1f, 0.11f, 0.025f},
                     "LeftFoot pos (world)");

    // All quats unit length within ε.
    for (const auto& t : trackers) {
        float n2 = t.quat_wxyz[0]*t.quat_wxyz[0] + t.quat_wxyz[1]*t.quat_wxyz[1]
                 + t.quat_wxyz[2]*t.quat_wxyz[2] + t.quat_wxyz[3]*t.quat_wxyz[3];
        check(std::abs(n2 - 1.0f) < 1.0e-3f, "quat not unit length");
    }
}

void test_role_to_position_mapping() {
    using fitra::slimevr::position_for;
    using R = fitra::slimevr::TrackerRole;
    using P = fitra::slimevr::TrackerPosition;
    check(position_for(R::LeftUpperArm)  == P::LeftUpperArm,  "LeftUpperArm");
    check(position_for(R::RightUpperArm) == P::RightUpperArm, "RightUpperArm");
    check(position_for(R::Chest)         == P::Chest,         "Chest");
    check(position_for(R::Waist)         == P::Hip,           "Waist→Hip");
    check(position_for(R::LeftUpperLeg)  == P::LeftUpperLeg,  "LeftUpperLeg");
    check(position_for(R::RightUpperLeg) == P::RightUpperLeg, "RightUpperLeg");
    check(position_for(R::LeftLowerLeg)  == P::LeftLowerLeg,  "LeftLowerLeg");
    check(position_for(R::RightLowerLeg) == P::RightLowerLeg, "RightLowerLeg");
    check(position_for(R::LeftFoot)      == P::LeftFoot,      "LeftFoot");
    check(position_for(R::RightFoot)     == P::RightFoot,     "RightFoot");
    // Sensor IDs are the enum ordinals 0..9.
    check(fitra::slimevr::sensor_id_for(R::LeftUpperArm) == 0, "sensor id LUA");
    check(fitra::slimevr::sensor_id_for(R::RightFoot)    == 9, "sensor id RF");
}

void test_missing_joints_yield_invalid() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };

    {
        // Knock out l_elbow → only LeftUpperArm should fail.
        auto skel = make_t_pose();
        skel.joints[7].valid = false;
        auto trackers = fitra::slimevr::extract_trackers(skel);
        check(!trackers[idx(R::LeftUpperArm)].valid,
              "LeftUpperArm invalid without l_elbow");
        check( trackers[idx(R::RightUpperArm)].valid,
              "RightUpperArm still valid");
        check( trackers[idx(R::Chest)].valid, "Chest still valid");
        check( trackers[idx(R::Waist)].valid, "Waist still valid");
    }
    {
        // Knock out hip_center → Chest and Waist go invalid (still require it).
        // UpperLegs no longer depend on hip_center (the secondary lateral pin
        // was removed); they stay valid as long as primary `ankle - knee` is
        // non-degenerate, which it is in the T-pose.
        auto skel = make_t_pose();
        skel.joints[19].valid = false;
        auto trackers = fitra::slimevr::extract_trackers(skel);
        check(!trackers[idx(R::Chest)].valid,         "Chest invalid w/o hip_center");
        check(!trackers[idx(R::Waist)].valid,         "Waist invalid w/o hip_center");
        check( trackers[idx(R::LeftUpperLeg)].valid,  "LUL still valid w/o hip_center");
        check( trackers[idx(R::RightUpperLeg)].valid, "RUL still valid w/o hip_center");
        // LowerLegs and feet do not require hip_center; they should still be ok.
        check( trackers[idx(R::LeftLowerLeg)].valid,  "LLL still valid w/o hip_center");
        check( trackers[idx(R::LeftFoot)].valid,      "LFoot still valid w/o hip_center");
    }
    {
        // l_heel no longer participates in foot tracker (tibia-aligned up;
        // see foot_tracker in tracker_extract.cpp). Knocking it out must NOT
        // invalidate LeftFoot. The required joints are now knee/ankle/toe.
        auto skel = make_t_pose();
        skel.joints[24].valid = false;
        auto trackers = fitra::slimevr::extract_trackers(skel);
        check( trackers[idx(R::LeftFoot)].valid, "LFoot still valid w/o l_heel");
        check( trackers[idx(R::RightFoot)].valid, "RFoot still valid");
    }
    {
        // Knock out l_big_toe → LeftFoot invalid (toe is required for fwd).
        auto skel = make_t_pose();
        skel.joints[20].valid = false;
        auto trackers = fitra::slimevr::extract_trackers(skel);
        check(!trackers[idx(R::LeftFoot)].valid, "LFoot invalid w/o l_big_toe");
        check( trackers[idx(R::RightFoot)].valid, "RFoot still valid");
    }
}

void test_smoothing_double_cover() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    std::array<fitra::slimevr::SlimeTracker, fitra::slimevr::kTrackerCount> curr{};
    std::array<cv::Vec4f, fitra::slimevr::kTrackerCount> prev{};
    // Set up a single valid tracker with prev = identity and curr = -identity
    // (same rotation under double-cover). Smoothing should flip curr and
    // produce identity-ish.
    curr[0].valid = true;
    curr[0].quat_wxyz = cv::Vec4f{-1, 0, 0, 0};
    prev[0] = cv::Vec4f{1, 0, 0, 0};
    fitra::slimevr::apply_quat_smoothing(curr, prev, 0.5f);
    cv::Vec4f r = curr[0].quat_wxyz;
    float n = std::sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2] + r[3]*r[3]);
    check(std::abs(n - 1.0f) < 1.0e-3f, "smoothed quat not unit");
    check(std::abs(std::abs(r[0]) - 1.0f) < 1.0e-3f, "smoothed quat not ±identity");
    check(std::abs(r[1]) < 1.0e-3f, "smoothed quat x ≠ 0");
    check(std::abs(r[2]) < 1.0e-3f, "smoothed quat y ≠ 0");
    check(std::abs(r[3]) < 1.0e-3f, "smoothed quat z ≠ 0");
}

// Behavior change (was test_smoothing_invalid_resets_prev): on invalid input,
// prev_quat must be HELD (not overwritten with the raw curr quat), and curr's
// quat is replaced by prev so the publisher can keep the last good orientation.
// This avoids the identity-snap that the previous reset logic caused whenever
// a tracker briefly dropped visibility.
void test_smoothing_invalid_holds_prev() {
    std::array<fitra::slimevr::SlimeTracker, fitra::slimevr::kTrackerCount> curr{};
    std::array<cv::Vec4f, fitra::slimevr::kTrackerCount> prev{};
    curr[0].valid = false;
    curr[0].quat_wxyz = cv::Vec4f{1, 0, 0, 0};   // raw extract output on invalid
    prev[0] = cv::Vec4f{0, 1, 0, 0};             // last known good
    fitra::slimevr::apply_quat_smoothing(curr, prev, 0.5f);
    // prev is unchanged.
    check_vec3_close(cv::Vec3f{prev[0][0], prev[0][1], prev[0][2]},
                     cv::Vec3f{0, 1, 0}, "invalid: prev_quat unchanged");
    check(std::abs(prev[0][3]) < kEps, "invalid: prev_quat[3] unchanged");
    // curr is replaced by prev.
    check_vec3_close(cv::Vec3f{curr[0].quat_wxyz[0], curr[0].quat_wxyz[1], curr[0].quat_wxyz[2]},
                     cv::Vec3f{0, 1, 0}, "invalid: curr follows prev for publisher continuity");
}

// Verify the tracker's local +Z axis (the bone forward) matches the expected
// world-frame bone direction within `eps`. Uses dot product to be sign-aware.
void check_tracker_forward(const fitra::slimevr::SlimeTracker& t,
                            const cv::Vec3f& expected_fwd_unit,
                            const std::string& label,
                            float eps = 1e-3f) {
    cv::Vec3f right, up, fwd;
    quat_to_basis(t.quat_wxyz, right, up, fwd);
    float dot = fwd[0]*expected_fwd_unit[0] + fwd[1]*expected_fwd_unit[1]
              + fwd[2]*expected_fwd_unit[2];
    if (std::abs(dot - 1.0f) > eps) {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
            "%s: forward axis dot=%.6f (want 1.0); fwd=(%.4f,%.4f,%.4f) expected=(%.4f,%.4f,%.4f)",
            label.c_str(), dot, fwd[0], fwd[1], fwd[2],
            expected_fwd_unit[0], expected_fwd_unit[1], expected_fwd_unit[2]);
        throw std::runtime_error(buf);
    }
}

// Verify the tracker's local +Y axis (the up hint after orthogonalization) lies
// in the half-space pointed to by `expected_up_dir`. We don't require exact
// equality because the up is projected onto fwd⊥ — but its sign and dominant
// component must match expectation for the chosen up source.
void check_tracker_up_direction(const fitra::slimevr::SlimeTracker& t,
                                 const cv::Vec3f& expected_up_dir,
                                 const std::string& label,
                                 float min_dot = 0.3f) {
    cv::Vec3f right, up, fwd;
    quat_to_basis(t.quat_wxyz, right, up, fwd);
    cv::Vec3f dir = vec_normalize(expected_up_dir);
    float dot = up[0]*dir[0] + up[1]*dir[1] + up[2]*dir[2];
    if (dot < min_dot) {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
            "%s: up axis dot=%.6f < %.4f; up=(%.4f,%.4f,%.4f) expected_dir=(%.4f,%.4f,%.4f)",
            label.c_str(), dot, min_dot, up[0], up[1], up[2], dir[0], dir[1], dir[2]);
        throw std::runtime_error(buf);
    }
}

// Build a minimal pose-specific skeleton starting from the T-pose and applying
// the caller's mutations. Useful when only a few joints differ from rest.
fitra::infer::Skeleton3D make_modified_t_pose(
    void (*mutate)(fitra::infer::Skeleton3D&)) {
    auto s = make_t_pose();
    mutate(s);
    return s;
}

// === Upper arm: arm raised straight forward (前方挙上), elbow slightly bent ===
// The forearm bends downward, so wrist - elbow has a strong vertical component
// orthogonal to the (horizontal) upper-arm axis. Primary up should be used and
// the tracker's local up should point upward (positive Z component).
void test_upper_arm_forward_raised() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_modified_t_pose([](fitra::infer::Skeleton3D& s) {
        // Raise left arm forward to +Y, elbow ~150° (30° flex), wrist drops slightly.
        set_joint(s, 5, 0.18f, 0.00f, 1.42f);   // l_shoulder
        set_joint(s, 7, 0.18f, 0.30f, 1.42f);   // l_elbow (~30cm forward, same height)
        set_joint(s, 9, 0.18f, 0.55f, 1.35f);   // l_wrist (forward + slightly down)
    });
    auto trackers = fitra::slimevr::extract_trackers(skel);
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };
    auto& t = trackers[idx(R::LeftUpperArm)];
    check(t.valid, "forward-raised upper arm must be valid");
    check_tracker_forward(t, cv::Vec3f{0, 1, 0}, "LUA forward raise: fwd axis", 1e-3f);
    // Wrist drops below the elbow → primary up has +Z component (orth to fwd).
    check_tracker_up_direction(t, cv::Vec3f{0, 0, -1},
                                "LUA forward raise: up tilts toward wrist drop");
}

// === Upper arm: arm raised overhead with elbow bent (頭上挙上) ===
// fwd ≈ +Z; wrist - elbow ≈ +X (elbow bent forward/laterally). The wrist primary
// should activate and the tracker's local up should reflect the wrist direction.
void test_upper_arm_overhead_bent_elbow() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_modified_t_pose([](fitra::infer::Skeleton3D& s) {
        set_joint(s, 5, 0.18f, 0.0f, 1.42f);    // l_shoulder
        set_joint(s, 7, 0.18f, 0.0f, 1.85f);    // l_elbow directly above shoulder
        set_joint(s, 9, 0.42f, 0.0f, 1.85f);    // l_wrist lateral 24cm (elbow flexed)
    });
    auto trackers = fitra::slimevr::extract_trackers(skel);
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };
    auto& t = trackers[idx(R::LeftUpperArm)];
    check(t.valid, "overhead-bent upper arm must be valid");
    check_tracker_forward(t, cv::Vec3f{0, 0, 1}, "LUA overhead: fwd axis");
    // wrist is to +X of elbow → tracker up should point toward +X.
    check_tracker_up_direction(t, cv::Vec3f{1, 0, 0},
                                "LUA overhead: up follows wrist direction");
}

// Confirms roll_confidence on the foot tracker is the fixed low-pass
// smoothing weight (kFootSmoothingWeight = 0.3), not 1.0 like the rigid
// anatomically-pinned bones.
void check_foot_smoothing_weight(const fitra::slimevr::SlimeTracker& t,
                                  const std::string& label) {
    constexpr float kExpected = 0.3f;  // kFootSmoothingWeight in tracker_extract.cpp
    if (std::abs(t.roll_confidence - kExpected) > 1.0e-3f) {
        char buf[192];
        std::snprintf(buf, sizeof(buf),
            "%s: roll_confidence got=%.4f want %.2f (kFootSmoothingWeight)",
            label.c_str(), t.roll_confidence, kExpected);
        throw std::runtime_error(buf);
    }
}

// === Foot: toe stance (つま先立ち) — toes on ground, ankle pitched forward ===
// New tibia-aligned design: fwd = ankle→toe, up = knee→ankle (tibia axis).
// heel is no longer consumed. Toe-stance pitches the ankle forward and up; the
// fwd vector tilts downward in Y (ankle forward of toe? no — ankle pitched
// forward AND raised, toe stays at ground). up follows the (knee - ankle)
// tibia axis which remains dominantly +Z.
void test_foot_toe_stance() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_modified_t_pose([](fitra::infer::Skeleton3D& s) {
        // Toe-stance: toe stays at ground, ankle raised 18cm and pitched
        // forward to sit above the ball of the foot. (heel value left as
        // T-pose default but no longer participates.)
        set_joint(s, 20, 0.1f,  0.17f, 0.0f);   // l_big_toe on ground
        set_joint(s, 15, 0.1f,  0.07f, 0.18f);  // l_ankle pitched forward of knee
        // l_knee stays at T-pose (0.1, 0.01, 0.45).
    });
    auto trackers = fitra::slimevr::extract_trackers(skel);
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };
    auto& t = trackers[idx(R::LeftFoot)];
    check(t.valid, "toe-stance foot must be valid");
    // fwd = (toe - ankle) = (0, 0.10, -0.18) — diving forward and down.
    cv::Vec3f expected_fwd = vec_normalize(cv::Vec3f{0, 0.10f, -0.18f});
    check_tracker_forward(t, expected_fwd, "Foot toe-stance: fwd is ankle→toe");
    // up = (knee - ankle) = (0, -0.06, 0.27) — tibia axis, dominantly +Z even
    // after orthogonalization vs the pitched fwd.
    check_tracker_up_direction(t, cv::Vec3f{0, 0, 1},
                                "Foot toe-stance: up follows tibia (+Z)");
    check_foot_smoothing_weight(t, "Foot toe-stance");
}

// === Foot: heel stance (かかと立ち) — heel on ground, toes raised ===
// fwd tilts upward in Z (ankle below toe), up still follows the tibia.
void test_foot_heel_stance() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_modified_t_pose([](fitra::infer::Skeleton3D& s) {
        set_joint(s, 20, 0.1f,  0.17f, 0.08f);  // l_big_toe raised
        set_joint(s, 15, 0.1f,  0.04f, 0.15f);  // l_ankle above (would-be) heel
    });
    auto trackers = fitra::slimevr::extract_trackers(skel);
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };
    auto& t = trackers[idx(R::LeftFoot)];
    check(t.valid, "heel-stance foot must be valid");
    // fwd = (toe - ankle) = (0, 0.13, -0.07).
    cv::Vec3f expected_fwd = vec_normalize(cv::Vec3f{0, 0.13f, -0.07f});
    check_tracker_forward(t, expected_fwd, "Foot heel-stance: fwd is ankle→toe");
    // up = (knee - ankle) = (0, -0.03, 0.30) → dominantly +Z.
    check_tracker_up_direction(t, cv::Vec3f{0, 0, 1},
                                "Foot heel-stance: up follows tibia (+Z)");
    check_foot_smoothing_weight(t, "Foot heel-stance");
}

// === Foot: roll is unobservable from these joints (足内反) ===
// Pre-fix design exposed foot inversion as up tilting toward +X when the ankle
// shifted laterally. The new tibia-aligned up (knee→ankle) makes foot roll
// genuinely unobservable: the same lateral ankle shift now changes both fwd
// (ankle→toe) and up (knee→ankle) in tandem, with no residual roll component.
// This test pins that contrast — the up direction must NOT mirror the old
// "ankle-foot_mid" pattern that signalled an inversion roll.
void test_foot_inversion_unobservable() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_modified_t_pose([](fitra::infer::Skeleton3D& s) {
        set_joint(s, 20, 0.1f,  0.17f, 0.0f);   // l_big_toe (flat on ground)
        // ankle shifted +X 8cm (lateral offset). The old design would have
        // turned the foot up into ankle - foot_mid ≈ (+0.08, *, +0.10) →
        // dot-with-(+X-leaning) > 0.7. The new design pulls up from knee →
        // ankle = (-0.08, -0.03, 0.35), perpendicular to the old +X direction.
        set_joint(s, 15, 0.18f, 0.04f, 0.10f);
    });
    auto trackers = fitra::slimevr::extract_trackers(skel);
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };
    auto& t = trackers[idx(R::LeftFoot)];
    check(t.valid, "lateral-ankle foot must still be valid");
    // up must be dominantly +Z (tibia axis); definitely not the pre-fix +X-
    // leaning ankle - foot_mid direction.
    check_tracker_up_direction(t, cv::Vec3f{0, 0, 1},
                                "Foot lat-ankle: up follows tibia, no +X roll");
    // Sanity: assert up does NOT align with the old expectation.
    cv::Vec3f right, up, fwd;
    quat_to_basis(t.quat_wxyz, right, up, fwd);
    cv::Vec3f old_expected = vec_normalize(cv::Vec3f{0.08f, -0.055f, 0.10f});
    float dot_with_old = up[0]*old_expected[0] + up[1]*old_expected[1]
                       + up[2]*old_expected[2];
    if (dot_with_old > 0.85f) {
        char buf[192];
        std::snprintf(buf, sizeof(buf),
            "Foot lat-ankle: up still matches pre-fix ankle-foot_mid pattern "
            "(dot=%.4f); foot roll should be unobservable",
            dot_with_old);
        throw std::runtime_error(buf);
    }
    check_foot_smoothing_weight(t, "Foot lat-ankle");
}

// === Thigh: knee bent 90° (seated, knee toward camera) ===
// fwd = knee - hip (forward, ~horizontal). ankle below knee → ankle - knee ≈ -Z.
// Primary up activates; tracker up should be dominantly -Z (since ankle hangs).
void test_thigh_seated_knee_bent() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_modified_t_pose([](fitra::infer::Skeleton3D& s) {
        set_joint(s, 11, 0.1f,  0.0f,  0.5f);   // l_hip (seated, lower)
        set_joint(s, 13, 0.1f,  0.45f, 0.5f);   // l_knee 45cm forward
        set_joint(s, 15, 0.1f,  0.45f, 0.05f);  // l_ankle directly below knee
    });
    auto trackers = fitra::slimevr::extract_trackers(skel);
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };
    auto& t = trackers[idx(R::LeftUpperLeg)];
    check(t.valid, "seated thigh must be valid");
    check_tracker_forward(t, cv::Vec3f{0, 1, 0}, "Thigh seated: fwd axis");
    // ankle - knee = (0, 0, -0.45); orth to fwd=+Y is itself. up should be -Z.
    check_tracker_up_direction(t, cv::Vec3f{0, 0, -1},
                                "Thigh seated: up follows ankle-knee (down)");
}

// === Thigh: walking single-leg lift (knee bent ~60°) — primary up active ===
void test_thigh_walking_knee_60() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_modified_t_pose([](fitra::infer::Skeleton3D& s) {
        // hip standard; knee lifted forward & up; shin swings forward by 60°
        // (mid-swing phase of walking gait — anatomically ~60-70° knee flexion).
        // The degeneracy gate is sin 0.15; 60° gives sin ≈ 0.60, comfortably
        // above the full-confidence ceiling (0.30).
        set_joint(s, 11, 0.1f,  0.0f,  0.9f);    // l_hip
        set_joint(s, 13, 0.1f,  0.15f, 0.55f);   // l_knee
        // Thigh axis = (0, 0.15, -0.35) ≈ forward-down. Shin pivots 60° forward
        // from the thigh axis (= shin direction is 60° behind world -Z):
        //   ankle = knee + R(-60° around X) * (0, 0, -0.45)
        //         = knee + (0, 0.45*sin60°, -0.45*cos60°)
        //         = (0.1, 0.15+0.390, 0.55-0.225) = (0.1, 0.54, 0.325)
        set_joint(s, 15, 0.1f,  0.54f, 0.325f);
    });
    auto trackers = fitra::slimevr::extract_trackers(skel);
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };
    auto& t = trackers[idx(R::LeftUpperLeg)];
    check(t.valid, "walking-knee-60 thigh must be valid");
    cv::Vec3f expected_fwd = vec_normalize(cv::Vec3f{0, 0.15f, -0.35f});
    check_tracker_forward(t, expected_fwd, "Thigh walk60: fwd axis");
}

// === Thigh: standing (knee straight) — primary degenerate, lateral pin removed ===
// With the secondary lateral pin (hip - hip_center) gone, primary-degenerate
// standing falls through to tertiary world Z. The femur axis is vertical (-Z)
// so world Z is itself parallel to fwd → quat_from_forward_up rejects the
// basis → valid=false, roll_confidence=0. apply_quat_smoothing then holds the
// previous quat, which decouples thigh roll from waist yaw (the symptom the
// lateral pin was causing during walking / shallow bends).
void test_thigh_standing_knee_straight() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_modified_t_pose([](fitra::infer::Skeleton3D& s) {
        // hip and knee vertically aligned, ankle continues vertically.
        set_joint(s, 11, 0.1f, 0.0f, 0.9f);     // l_hip
        set_joint(s, 13, 0.1f, 0.0f, 0.45f);    // l_knee
        set_joint(s, 15, 0.1f, 0.0f, 0.05f);    // l_ankle (perfectly straight)
        set_joint(s, 19, 0.0f, 0.0f, 0.9f);     // hip_center
    });
    auto trackers = fitra::slimevr::extract_trackers(skel);
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };
    auto& t = trackers[idx(R::LeftUpperLeg)];
    check(!t.valid, "standing-straight thigh must be invalid (no observable roll)");
    if (t.roll_confidence > 1.0e-3f) {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "standing-straight confidence got=%.4f want 0", t.roll_confidence);
        throw std::runtime_error(buf);
    }
}

// === Thigh: ankle laterally offset → primary activates, no pelvis-rigid pin ===
// Pre-fix (lateral pin): both strict-standing and laterally-offset-ankle were
// invariably valid and shared the pelvis lateral axis; femur axial rotation was
// silently glued to waist yaw whenever primary went degenerate.
// Post-fix (2-stage): strict-standing → valid=false + confidence=0 (covered by
// test_thigh_standing_knee_straight). A lateral ankle offset breaks primary
// degeneracy → primary activates → tracker is valid and its roll comes from
// the ankle direction alone (no waist coupling). This test pins that contrast.
void test_thigh_lateral_ankle_uses_primary() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel_lateral = make_modified_t_pose([](fitra::infer::Skeleton3D& s) {
        set_joint(s, 11, 0.1f,  0.0f, 0.9f);    // l_hip
        set_joint(s, 13, 0.1f,  0.0f, 0.45f);   // l_knee (femur vertical)
        // ankle laterally offset 15 cm outboard of knee. primary up
        // (ankle - knee) = (0.15, 0, -0.40), |up|=0.427, sin θ = 0.15/0.427 ≈
        // 0.351 → > kRollSinHigh (0.30) → full confidence. 15 cm puts the
        // lateral cue safely above the kRollSinLow (0.15) gate.
        set_joint(s, 15, 0.25f, 0.0f, 0.05f);
    });
    auto trackers = fitra::slimevr::extract_trackers(skel_lateral);
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };
    auto& t = trackers[idx(R::LeftUpperLeg)];
    check(t.valid, "lateral-ankle thigh must be valid (primary active)");
    check_tracker_forward(t, cv::Vec3f{0, 0, -1}, "Thigh lat-ankle: fwd is -Z");
    // primary up = (0.15, 0, -0.40); after orthogonalization vs fwd=-Z the up
    // component is dominantly +X. Crucially this does NOT match the previous
    // lateral pin (which would also have been +X but rigidly tied to the
    // pelvis); here it follows the ankle, fully independent of hip_center.
    check_tracker_up_direction(t, cv::Vec3f{1, 0, 0},
                                "Thigh lat-ankle: up follows ankle, not pelvis");
    if (t.roll_confidence < 0.1f) {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "lat-ankle confidence got=%.4f want > 0.1 (primary active)",
            t.roll_confidence);
        throw std::runtime_error(buf);
    }
}

// === Thigh: 直座り (seated, legs extended forward, knee straight) — primary
//             degenerate, MUST freeze instead of falling to world Z ============
// Subject is sitting on the floor with the leg extended straight forward
// (a very common indoor sitting style in Japan: 直座り / 長座 / あぐらからの
// 脚伸ばし). Thigh axis is horizontal (+Y), and with the knee fully straight
// the shin axis is also horizontal (+Y), so (ankle - knee) is colinear with
// the thigh axis → primary is degenerate.
//
// Pre-fix behavior (bug): tertiary was world Z, and pick_up_multistage
// accepted world Z unconditionally at i==2 with confidence = sin(worldZ, fwd)
// = sin 90° = 1.0. That writes a fabricated "knee faces ceiling" thigh roll
// with full confidence — the avatar's thigh would lock to a world-Z-aligned
// roll for the entire duration of the seated pose.
//
// Post-fix behavior: tertiary is the zero sentinel → pick_up_multistage
// returns confidence=0 and zero up → quat_from_forward_up valid=false →
// apply_quat_smoothing holds the previous thigh quat. Same freeze semantics
// as test_thigh_standing_knee_straight (which has fwd ∥ worldZ; the freeze
// there was incidentally correct because cross(worldZ, fwd) = 0). This test
// covers the non-vertical-thigh case the standing test couldn't.
void test_thigh_seated_extended_straight_knee() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_modified_t_pose([](fitra::infer::Skeleton3D& s) {
        // Sitting on the floor, hip ~10 cm above the ground, leg extended
        // forward (+Y) with thigh and shin both horizontal and colinear.
        set_joint(s, 11, 0.1f, 0.0f, 0.1f);   // l_hip   (floor-level)
        set_joint(s, 13, 0.1f, 0.4f, 0.1f);   // l_knee  (40 cm forward, same z)
        set_joint(s, 15, 0.1f, 0.8f, 0.1f);   // l_ankle (40 cm further, same z)
        set_joint(s, 19, 0.0f, 0.0f, 0.1f);   // hip_center
    });
    auto trackers = fitra::slimevr::extract_trackers(skel);
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };
    auto& t = trackers[idx(R::LeftUpperLeg)];
    check(!t.valid,
          "直座り thigh must be invalid (primary degenerate, world-Z must not rescue)");
    if (t.roll_confidence > 1.0e-3f) {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
            "直座り confidence got=%.4f want 0 (world-Z fallback must not fire)",
            t.roll_confidence);
        throw std::runtime_error(buf);
    }
}

// === Confidence A: 90° elbow bend → primary up perpendicular to fwd → confidence ≈ 1 ===
void test_upper_arm_confidence_full_at_90deg_bend() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_modified_t_pose([](fitra::infer::Skeleton3D& s) {
        set_joint(s, 5, 0.18f, 0.0f, 1.42f);    // l_shoulder
        set_joint(s, 7, 0.18f, 0.0f, 1.85f);    // l_elbow directly above
        set_joint(s, 9, 0.42f, 0.0f, 1.85f);    // l_wrist lateral (90° flex)
    });
    auto trackers = fitra::slimevr::extract_trackers(skel);
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };
    auto& t = trackers[idx(R::LeftUpperArm)];
    check(t.valid, "90° elbow tracker must be valid");
    if (t.roll_confidence < 0.99f) {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "confidence at 90° bend got=%.4f want >= 0.99", t.roll_confidence);
        throw std::runtime_error(buf);
    }
}

// === Confidence B: every up parallel to fwd → confidence = 0 (frozen update) ===
void test_upper_arm_confidence_zero_all_degenerate() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_modified_t_pose([](fitra::infer::Skeleton3D& s) {
        // Shoulder directly below neck; arm fully extended straight up so wrist,
        // elbow, shoulder and neck are all colinear along Z.
        set_joint(s, 18, 0.0f, 0.0f, 1.55f);    // neck
        set_joint(s,  5, 0.0f, 0.0f, 1.50f);    // l_shoulder colinear
        set_joint(s,  7, 0.0f, 0.0f, 1.85f);    // l_elbow
        set_joint(s,  9, 0.0f, 0.0f, 2.10f);    // l_wrist
    });
    auto trackers = fitra::slimevr::extract_trackers(skel);
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };
    auto& t = trackers[idx(R::LeftUpperArm)];
    // All three up candidates ∥ fwd → quat_from_forward_up rejects → valid=false.
    check(!t.valid, "fully colinear arm must be invalid");
    if (t.roll_confidence > 1.0e-3f) {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "confidence with all-degenerate ups got=%.4f want 0", t.roll_confidence);
        throw std::runtime_error(buf);
    }
}

// === Confidence C: sin θ = 0.125 (midrange) → smoothstep maps to ≈ 0.5 ===
void test_upper_arm_confidence_smoothstep_midrange() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    // Pose: arm overhead (fwd ≈ +Z), wrist offset from elbow at θ such that
    // sin θ = 0.225 (= midpoint of the smoothstep band [0.15, 0.30]).
    // With forearm length 0.25 m and elbow at (0, 0, 1.85),
    // wrist = elbow + (sin θ · 0.25, 0, cos θ · 0.25) ≈ (0.05625, 0, 2.0936).
    auto skel = make_modified_t_pose([](fitra::infer::Skeleton3D& s) {
        set_joint(s,  5, 0.0f,     0.0f, 1.42f);   // l_shoulder
        set_joint(s,  7, 0.0f,     0.0f, 1.85f);   // l_elbow
        set_joint(s,  9, 0.05625f, 0.0f, 2.0936f); // l_wrist 13.0° off-axis
        set_joint(s, 18, 0.0f,     0.0f, 1.55f);   // neck colinear → secondary degenerate
    });
    auto trackers = fitra::slimevr::extract_trackers(skel);
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };
    auto& t = trackers[idx(R::LeftUpperArm)];
    check(t.valid, "midrange-sin tracker must be valid");
    // smoothstep((0.225-0.15)/(0.30-0.15)) = smoothstep(0.5) = 0.5
    float c = t.roll_confidence;
    if (std::abs(c - 0.5f) > 0.05f) {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
            "midrange confidence got=%.4f want ≈ 0.5 (smoothstep(0.5))", c);
        throw std::runtime_error(buf);
    }
}

// === Confidence D: low confidence freezes smoothing across frames ===
// Set up a tracker with roll_confidence=0; SLERP must be a no-op and the curr
// quat should track the prev across many frames regardless of curr's raw quat.
void test_smoothing_freezes_under_low_confidence() {
    std::array<fitra::slimevr::SlimeTracker, fitra::slimevr::kTrackerCount> curr{};
    std::array<cv::Vec4f, fitra::slimevr::kTrackerCount> prev{};
    // Tracker 0: valid but zero confidence; raw quat is wildly different from prev.
    curr[0].valid = true;
    curr[0].roll_confidence = 0.0f;
    curr[0].quat_wxyz = cv::Vec4f{0, 1, 0, 0};   // 180° about +X
    prev[0] = cv::Vec4f{1, 0, 0, 0};             // identity, last known good
    for (int i = 0; i < 5; ++i) {
        fitra::slimevr::apply_quat_smoothing(curr, prev, 0.5f);
        // prev unchanged.
        check(std::abs(prev[0][0] - 1.0f) < kEps,
              "freeze: prev[0] held over multiple frames");
        check(std::abs(prev[0][1]) < kEps && std::abs(prev[0][2]) < kEps
              && std::abs(prev[0][3]) < kEps,
              "freeze: prev other components held");
        // curr replaced by prev so publisher sees stable orientation.
        check(std::abs(curr[0].quat_wxyz[0] - 1.0f) < kEps,
              "freeze: curr follows prev");
        // Reset curr to the noisy raw quat for the next iteration (simulating a
        // frame where the deterministic pick swung the raw measurement).
        curr[0].quat_wxyz = cv::Vec4f{0, 1, 0, 0};
    }
}

void test_keypoint_format_assert() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Coco17);
    auto skel = make_t_pose();
    bool threw = false;
    try {
        (void)fitra::slimevr::extract_trackers(skel);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    check(threw, "extract_trackers should refuse COCO17");
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
}

}  // namespace

int main() {
    try {
        test_quat_from_forward_up_degenerate();   std::printf("[ok] quat_from_forward_up degeneracy\n");
        test_t_pose_extracts_all_ten();           std::printf("[ok] T-pose extracts all 10 trackers\n");
        test_role_to_position_mapping();          std::printf("[ok] role → TrackerPosition / sensor_id\n");
        test_missing_joints_yield_invalid();      std::printf("[ok] missing joints → invalid trackers\n");
        test_smoothing_double_cover();            std::printf("[ok] slerp double-cover handling\n");
        test_smoothing_invalid_holds_prev();      std::printf("[ok] invalid tracker holds previous orientation\n");
        test_upper_arm_confidence_full_at_90deg_bend(); std::printf("[ok] confidence: 90° elbow bend → 1.0\n");
        test_upper_arm_confidence_zero_all_degenerate(); std::printf("[ok] confidence: all-degenerate → 0\n");
        test_upper_arm_confidence_smoothstep_midrange(); std::printf("[ok] confidence: smoothstep midrange ≈ 0.5\n");
        test_smoothing_freezes_under_low_confidence();   std::printf("[ok] smoothing: low confidence freezes update\n");
        test_upper_arm_forward_raised();          std::printf("[ok] upper arm: forward-raised (前方挙上)\n");
        test_upper_arm_overhead_bent_elbow();     std::printf("[ok] upper arm: overhead + bent elbow (頭上挙上)\n");
        test_foot_toe_stance();                   std::printf("[ok] foot: toe stance (つま先立ち)\n");
        test_foot_heel_stance();                  std::printf("[ok] foot: heel stance (かかと立ち)\n");
        test_foot_inversion_unobservable();       std::printf("[ok] foot: lateral ankle, roll unobservable (横ずれ)\n");
        test_thigh_seated_knee_bent();            std::printf("[ok] thigh: seated knee bent 90° (着座)\n");
        test_thigh_walking_knee_60();             std::printf("[ok] thigh: walking knee bent ~60° (歩行片足)\n");
        test_thigh_standing_knee_straight();      std::printf("[ok] thigh: standing knee straight (立位)\n");
        test_thigh_lateral_ankle_uses_primary();  std::printf("[ok] thigh: lateral ankle activates primary (足首横ずれ)\n");
        test_thigh_seated_extended_straight_knee(); std::printf("[ok] thigh: 直座り — primary degenerate freezes (no world-Z rescue)\n");
        test_keypoint_format_assert();            std::printf("[ok] Halpe26 keypoint-format assertion\n");
        std::puts("test_tracker_extract ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_tracker_extract failed: %s\n", e.what());
        return 1;
    }
}
