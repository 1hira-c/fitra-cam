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
#include "slimevr/tracker_extractor.hpp"
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

// Chest / Waist height fracs slide the tracker POSITION up the spine without
// touching orientation. T-pose: neck(0,0,1.45), hip_center(0,0,0.9), so
// spine = (0,0,0.55) and pos.z = 0.9 + frac·0.55.
void test_torso_tracker_height_frac() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_t_pose();
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };

    // Function defaults (0.5 / 0.0) reproduce the historical placement.
    auto base = fitra::slimevr::extract_trackers(skel);
    check_vec3_close(base[idx(R::Chest)].pos, cv::Vec3f{0, 0, 1.175f},
                     "Chest default frac 0.5 == midpoint");
    check_vec3_close(base[idx(R::Waist)].pos, cv::Vec3f{0, 0, 0.9f},
                     "Waist default frac 0.0 == hip_center");

    // Product defaults (raised): chest 0.65, waist 0.15.
    auto raised = fitra::slimevr::extract_trackers(
        skel, nullptr, fitra::slimevr::FootPosMode::Ankle, 0.65f, 0.15f);
    check_vec3_close(raised[idx(R::Chest)].pos, cv::Vec3f{0, 0, 1.2575f},
                     "Chest frac 0.65 slides up the spine");
    check_vec3_close(raised[idx(R::Waist)].pos, cv::Vec3f{0, 0, 0.9825f},
                     "Waist frac 0.15 slides up the spine");
    check(raised[idx(R::Chest)].pos[2] > base[idx(R::Chest)].pos[2],
          "raised chest sits higher than default");
    check(raised[idx(R::Waist)].pos[2] > base[idx(R::Waist)].pos[2],
          "raised waist sits higher than default");

    // Orientation is independent of the height frac (position-only knob).
    auto qclose = [](const cv::Vec4f& a, const cv::Vec4f& b) {
        return std::abs(a[0]-b[0]) < 1e-4f && std::abs(a[1]-b[1]) < 1e-4f
            && std::abs(a[2]-b[2]) < 1e-4f && std::abs(a[3]-b[3]) < 1e-4f;
    };
    check(qclose(base[idx(R::Chest)].quat_wxyz, raised[idx(R::Chest)].quat_wxyz),
          "chest orientation unchanged by height frac");
    check(qclose(base[idx(R::Waist)].quat_wxyz, raised[idx(R::Waist)].quat_wxyz),
          "waist orientation unchanged by height frac");
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

// === Thigh: standing (knee straight) — roll degenerate, swing still tracks ===
// With the leg fully extended the up hint (ankle - knee) is colinear with the
// femur axis, so roll is unobservable. Post swing/twist split: the tracker is
// VALID with roll_confidence=0 and a forward-only orientation. The bone
// direction (-Z) is still emitted so apply_quat_smoothing tracks pitch/yaw,
// while the roll holds at the previous frame (no waist-yaw coupling).
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
    check(t.valid, "standing-straight thigh stays valid (forward-only orientation)");
    // forward = knee - hip = (0,0,-0.45) → -Z; swing tracks the bone direction.
    check_tracker_forward(t, cv::Vec3f{0, 0, -1}, "Thigh standing: fwd is -Z");
    if (t.roll_confidence > 1.0e-3f) {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "standing-straight roll_confidence got=%.4f want 0 (roll held)",
            t.roll_confidence);
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

// === Thigh: 直座り (seated, legs extended forward, knee straight) — roll
//             degenerate, MUST hold roll without a world-Z fallback ============
// Subject is sitting on the floor with the leg extended straight forward
// (a very common indoor sitting style in Japan: 直座り / 長座 / あぐらからの
// 脚伸ばし). Thigh axis is horizontal (+Y), and with the knee fully straight
// the shin axis is also horizontal (+Y), so (ankle - knee) is colinear with
// the thigh axis → roll is unobservable.
//
// Pre-fix bug: tertiary was world Z, accepted unconditionally with confidence
// = sin(worldZ, fwd) = sin 90° = 1.0 → fabricated "knee faces ceiling" roll
// locked in with full confidence.
//
// Post-fix (swing/twist split): the up hint degenerates → build_tracker emits a
// forward-only orientation (fwd = +Y) with roll_confidence=0. The tracker stays
// VALID so the thigh direction keeps tracking, but the (unobservable) roll is
// held by apply_quat_smoothing — no fabricated world-Z roll.
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
    check(t.valid,
          "直座り thigh stays valid (forward-only orientation, roll held)");
    // forward = knee - hip = (0,0.4,0) → +Y; swing tracks, no world-Z roll.
    check_tracker_forward(t, cv::Vec3f{0, 1, 0}, "Thigh 直座り: fwd is +Y");
    if (t.roll_confidence > 1.0e-3f) {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
            "直座り roll_confidence got=%.4f want 0 (world-Z fallback must not fire)",
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

// === Confidence B: every up parallel to fwd → roll held, swing still tracks ===
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
    // up ∥ fwd → roll unobservable. Forward (elbow - shoulder = +Z) is still
    // valid, so the tracker emits a forward-only orientation with
    // roll_confidence=0: swing tracks, roll held.
    check(t.valid, "fully colinear arm stays valid (forward-only orientation)");
    check_tracker_forward(t, cv::Vec3f{0, 0, 1}, "Arm colinear: fwd is +Z");
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

// === Confidence D: both gates zero freezes smoothing across frames ===
// Set up a tracker with swing_confidence=roll_confidence=0; the smoothing must
// be a no-op and the curr quat should track the prev across many frames
// regardless of curr's raw quat. (roll_confidence alone no longer freezes the
// whole orientation — swing tracks unless swing_confidence is also 0; see
// test_roll_hold_keeps_swing for the roll-only-hold contract.)
void test_smoothing_freezes_under_low_confidence() {
    std::array<fitra::slimevr::SlimeTracker, fitra::slimevr::kTrackerCount> curr{};
    std::array<cv::Vec4f, fitra::slimevr::kTrackerCount> prev{};
    // Tracker 0: valid but zero confidence; raw quat is wildly different from prev.
    curr[0].valid = true;
    curr[0].roll_confidence = 0.0f;
    curr[0].swing_confidence = 0.0f;
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

// pose-3d/locomotion-stability M1:
// FK fallback for foot trackers. With ctx and an anchor seeded from a good
// frame, dropping ankle (and/or toe) on the next frame must still produce a
// valid foot tracker — synthesized via knee + dir·length. Without ctx (or
// with no anchor) the same drop yields invalid (preserves old behavior).
void test_foot_fk_fallback_uses_last_anchor() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    fitra::slimevr::ExtractContext ctx{};
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };

    // Frame 1: fully measured T-pose seeds the anchor.
    auto skel1 = make_t_pose();
    auto t1 = fitra::slimevr::extract_trackers(skel1, &ctx);
    check(t1[idx(R::LeftFoot)].valid,  "fk-fallback.seed.left.valid");
    check(t1[idx(R::RightFoot)].valid, "fk-fallback.seed.right.valid");
    check(ctx.foot_anchors[0].valid,   "fk-fallback.seed.anchor[0]");
    check(ctx.foot_anchors[0].tibia_len_m > 1.0e-4f, "fk-fallback.seed.tibia_len > 0");
    check(ctx.foot_anchors[0].foot_len_m  > 1.0e-4f, "fk-fallback.seed.foot_len > 0");

    const cv::Vec3f anchor_pos_before = ctx.foot_anchors[0].knee_to_ankle_dir;
    const float     tibia_before      = ctx.foot_anchors[0].tibia_len_m;

    // Frame 2: hip + knee + toe valid, but ankle dropped on left. With
    // ctx the foot must still produce a valid tracker via FK; without ctx
    // it would early-return.
    auto skel2 = make_t_pose();
    skel2.joints[15].valid = false;  // l_ankle dropped
    auto t2 = fitra::slimevr::extract_trackers(skel2, &ctx);
    check(t2[idx(R::LeftFoot)].valid,
          "fk-fallback.left foot valid via FK when ankle dropped");
    // Confidence weight drops to the FK-mode value (0.15) when synthesized.
    if (t2[idx(R::LeftFoot)].roll_confidence > 0.20f) {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
            "fk-fallback.left foot weight got=%.3f want ≤ 0.20 (FK mode)",
            t2[idx(R::LeftFoot)].roll_confidence);
        throw std::runtime_error(buf);
    }
    // Anchor must NOT be updated from a synthesized frame.
    check_vec3_close(ctx.foot_anchors[0].knee_to_ankle_dir,
                     anchor_pos_before,
                     "fk-fallback.anchor.dir unchanged on synth frame");
    check(std::abs(ctx.foot_anchors[0].tibia_len_m - tibia_before) < 1.0e-6f,
          "fk-fallback.anchor.tibia_len unchanged on synth frame");

    // Without ctx the same drop yields invalid (old behavior preserved).
    auto t2_noctx = fitra::slimevr::extract_trackers(skel2);
    check(!t2_noctx[idx(R::LeftFoot)].valid,
          "fk-fallback.no-ctx: ankle drop must produce invalid foot");
}

// Without a seeded anchor (first frame, ankle already invalid), FK fallback
// has no data to draw on → foot tracker invalid, matching the old behavior.
void test_foot_fk_fallback_needs_seed() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    fitra::slimevr::ExtractContext ctx{};
    using R = fitra::slimevr::TrackerRole;
    auto idx = [](R r) { return static_cast<std::size_t>(r); };

    auto skel = make_t_pose();
    skel.joints[15].valid = false;  // l_ankle dropped on the very first frame
    auto t = fitra::slimevr::extract_trackers(skel, &ctx);
    check(!t[idx(R::LeftFoot)].valid,
          "fk-fallback.unseeded: must not invent an anchor from nothing");
    check(!ctx.foot_anchors[0].valid,
          "fk-fallback.unseeded: anchor stays invalid");
}

// pose-3d/locomotion-stability M3: roll-only hold via swing/twist split.
// With roll_confidence=0 but swing_confidence=1, the swing (pitch/yaw = bone
// forward) must fully track the new measurement while the twist (roll) is held,
// independent of whatever arbitrary roll the raw curr quat carries. This is the
// fix that keeps an extended limb's bone direction moving instead of freezing
// the whole orientation when roll becomes unobservable.
void test_roll_hold_keeps_swing() {
    namespace sv = fitra::slimevr;
    using sv::SlimeTracker;
    using sv::kTrackerCount;

    // prev orientation: bone pointing +Z, canonical up +Y.
    cv::Vec4f P;
    check(sv::detail::quat_from_forward_up(cv::Vec3f{0, 0, 1}, cv::Vec3f{0, 1, 0}, P),
          "roll-hold: build prev");

    // New forward tilted toward +Y. Two curr quats SHARE this forward but carry
    // different rolls (different up hints) — i.e. they differ only by a rotation
    // about the bone axis, exactly what the twist gate should discard.
    cv::Vec3f f = vec_normalize(cv::Vec3f{0, 0.5f, 1.0f});
    cv::Vec4f Qa, Qb;
    check(sv::detail::quat_from_forward_up(f, cv::Vec3f{1, 0, 0}, Qa), "roll-hold: curr_a");
    check(sv::detail::quat_from_forward_up(f, cv::Vec3f{0, 1, 0.2f}, Qb), "roll-hold: curr_b");

    auto run = [&](const cv::Vec4f& Q, float roll_conf) {
        std::array<SlimeTracker, kTrackerCount> curr{};
        std::array<cv::Vec4f, kTrackerCount> prev{};
        prev[0] = P;
        curr[0].valid = true;
        curr[0].quat_wxyz = Q;
        curr[0].roll_confidence  = roll_conf;
        curr[0].swing_confidence = 1.0f;
        sv::apply_quat_smoothing(curr, prev, 1.0f);  // base_alpha=1 → swing fully applied
        return curr[0];
    };
    auto abs_dot = [](const cv::Vec4f& a, const cv::Vec4f& b) {
        return std::abs(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]);
    };

    // Roll held (roll_confidence=0): swing tracks forward, roll discarded.
    SlimeTracker ra = run(Qa, 0.0f);
    SlimeTracker rb = run(Qb, 0.0f);
    check_tracker_forward(ra, f, "roll-hold: swing tracks new forward (a)", 1e-3f);
    check_tracker_forward(rb, f, "roll-hold: swing tracks new forward (b)", 1e-3f);
    float dot_held = abs_dot(ra.quat_wxyz, rb.quat_wxyz);
    if (dot_held < 0.999f) {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
            "roll-hold: results differ across curr rolls (|dot|=%.5f want ≈1) — roll not held",
            dot_held);
        throw std::runtime_error(buf);
    }

    // Contrast: with full roll confidence the roll IS tracked, so the two curr
    // rolls produce visibly different orientations.
    float dot_tracked = abs_dot(run(Qa, 1.0f).quat_wxyz, run(Qb, 1.0f).quat_wxyz);
    check(dot_tracked < 0.999f,
          "roll-hold contrast: full roll confidence must track roll (results differ)");
}

// pose-3d/locomotion-stability M5: pelvis-yaw transport for a held roll.
// A standing extended leg has a near-vertical forward, so a body yaw is a
// rotation about the bone's own forward axis — pure roll, which is held
// (roll_confidence=0) and would otherwise freeze the limb's facing when the
// subject turns sideways. With the waist tracker yawing between frames, the
// held roll must ride along; without an observable waist it must stay put.
void test_pelvis_yaw_transport_held_roll() {
    namespace sv = fitra::slimevr;
    using sv::SlimeTracker;
    using sv::kTrackerCount;
    constexpr std::size_t kWaist = static_cast<std::size_t>(sv::TrackerRole::Waist);
    constexpr std::size_t kLeg   = static_cast<std::size_t>(sv::TrackerRole::LeftUpperLeg);
    const float pi = 3.14159265358979f;
    const float th = 30.0f * pi / 180.0f;  // pelvis yaw between frames

    // Vertical bone forward (-Z). prev leg basis: up = +Y.
    cv::Vec4f Pleg;
    check(sv::detail::quat_from_forward_up(cv::Vec3f{0, 0, -1}, cv::Vec3f{0, 1, 0}, Pleg),
          "pelvis-yaw: build prev leg");

    auto run = [&](bool waist_valid) {
        std::array<SlimeTracker, kTrackerCount> curr{};
        std::array<cv::Vec4f, kTrackerCount> prev{};
        prev[kLeg]   = Pleg;
        prev[kWaist] = cv::Vec4f{1, 0, 0, 0};  // waist faces world-forward last frame
        // Leg: same vertical forward, arbitrary roll (up hint +X), roll held.
        cv::Vec4f Qleg;
        check(sv::detail::quat_from_forward_up(cv::Vec3f{0, 0, -1}, cv::Vec3f{1, 0, 0}, Qleg),
              "pelvis-yaw: build curr leg");
        curr[kLeg].valid           = true;
        curr[kLeg].quat_wxyz       = Qleg;
        curr[kLeg].roll_confidence  = 0.0f;   // roll unobservable → held
        curr[kLeg].swing_confidence = 1.0f;
        // Waist yaws +th about world Z this frame (rigid: fast path, untouched).
        curr[kWaist].valid           = waist_valid;
        curr[kWaist].quat_wxyz       = cv::Vec4f{std::cos(th * 0.5f), 0, 0, std::sin(th * 0.5f)};
        curr[kWaist].roll_confidence  = 1.0f;
        curr[kWaist].swing_confidence = 1.0f;
        sv::apply_quat_smoothing(curr, prev, 1.0f);  // base_alpha=1, default dt → gate open
        return curr[kLeg];
    };

    // Waist valid → held roll rides the +th pelvis yaw: prev up (+Y) rotates
    // about Z to (-sin th, cos th, 0). Forward stays vertical (Z unaffected).
    SlimeTracker tr = run(/*waist_valid=*/true);
    check_tracker_forward(tr, cv::Vec3f{0, 0, -1},
                          "pelvis-yaw: forward still vertical", 1e-3f);
    check_tracker_up_direction(tr, cv::Vec3f{-std::sin(th), std::cos(th), 0.0f},
                               "pelvis-yaw: up rides pelvis yaw", 0.99f);

    // Control: waist invalid → no transport → held roll stays at prev up (+Y).
    SlimeTracker tr0 = run(/*waist_valid=*/false);
    check_tracker_up_direction(tr0, cv::Vec3f{0, 1, 0},
                               "pelvis-yaw: no transport without waist (held)", 0.99f);
}

// pose-3d/locomotion-stability M5: arms ride the chest, legs ride the waist.
// The pelvis can yaw independently of the chest (spine twist), so the parent
// reference must be per-limb. Yaw the chest and waist in OPPOSITE directions in
// one frame and confirm a held-roll arm follows the chest while a held-roll leg
// follows the waist.
void test_arm_chest_leg_waist_transport() {
    namespace sv = fitra::slimevr;
    using sv::SlimeTracker;
    using sv::kTrackerCount;
    constexpr std::size_t kChest = static_cast<std::size_t>(sv::TrackerRole::Chest);
    constexpr std::size_t kWaist = static_cast<std::size_t>(sv::TrackerRole::Waist);
    constexpr std::size_t kArm   = static_cast<std::size_t>(sv::TrackerRole::LeftUpperArm);
    constexpr std::size_t kLeg   = static_cast<std::size_t>(sv::TrackerRole::LeftUpperLeg);
    const float pi = 3.14159265358979f;
    const float tc =  30.0f * pi / 180.0f;  // chest yaws +30°
    const float tw = -40.0f * pi / 180.0f;  // waist yaws -40°

    // Vertical bone forward (-Z), prev up = +Y, for both limbs.
    cv::Vec4f Plimb;
    check(sv::detail::quat_from_forward_up(cv::Vec3f{0, 0, -1}, cv::Vec3f{0, 1, 0}, Plimb),
          "arm/leg: build prev limb");
    cv::Vec4f Qlimb;  // same vertical forward, arbitrary roll (up +X) → roll held
    check(sv::detail::quat_from_forward_up(cv::Vec3f{0, 0, -1}, cv::Vec3f{1, 0, 0}, Qlimb),
          "arm/leg: build curr limb");

    std::array<SlimeTracker, kTrackerCount> curr{};
    std::array<cv::Vec4f, kTrackerCount> prev{};
    auto set_held_limb = [&](std::size_t i) {
        prev[i] = Plimb;
        curr[i].valid           = true;
        curr[i].quat_wxyz       = Qlimb;
        curr[i].roll_confidence  = 0.0f;
        curr[i].swing_confidence = 1.0f;
    };
    set_held_limb(kArm);
    set_held_limb(kLeg);
    auto set_parent = [&](std::size_t i, float th) {
        prev[i] = cv::Vec4f{1, 0, 0, 0};
        curr[i].valid           = true;
        curr[i].quat_wxyz       = cv::Vec4f{std::cos(th * 0.5f), 0, 0, std::sin(th * 0.5f)};
        curr[i].roll_confidence  = 1.0f;
        curr[i].swing_confidence = 1.0f;
    };
    set_parent(kChest, tc);
    set_parent(kWaist, tw);

    sv::apply_quat_smoothing(curr, prev, 1.0f);

    // Arm rides the chest (+30°): up → (-sin tc, cos tc, 0).
    check_tracker_up_direction(curr[kArm], cv::Vec3f{-std::sin(tc), std::cos(tc), 0.0f},
                               "arm rides chest yaw (+30°)", 0.99f);
    // Leg rides the waist (-40°): up → (-sin tw, cos tw, 0).
    check_tracker_up_direction(curr[kLeg], cv::Vec3f{-std::sin(tw), std::cos(tw), 0.0f},
                               "leg rides waist yaw (-40°)", 0.99f);
}

// pose-3d/locomotion-stability M5: the held-roll transport must converge to
// the parent's yaw, not overshoot it. The transport delta is the FULL parent
// change (prev smoothed → curr raw), but with base_alpha < 1 the parent only
// moves alpha per step; riding the full delta every frame re-adds the still-
// open gap and the held limb converges to Θ/alpha (a 2× overshoot at alpha=0.5)
// instead of Θ. Drive a constant +Θ waist yaw at alpha=0.5 for many frames and
// confirm the held leg lands at Θ (the M5 single-frame tests run at alpha=1, so
// they cannot catch this).
void test_pelvis_yaw_transport_no_overshoot() {
    namespace sv = fitra::slimevr;
    using sv::SlimeTracker;
    using sv::kTrackerCount;
    constexpr std::size_t kWaist = static_cast<std::size_t>(sv::TrackerRole::Waist);
    constexpr std::size_t kLeg   = static_cast<std::size_t>(sv::TrackerRole::LeftUpperLeg);
    const float pi = 3.14159265358979f;
    const float th = 30.0f * pi / 180.0f;  // sustained pelvis yaw target

    cv::Vec4f Pleg;  // prev leg: vertical forward (-Z), up = +Y
    check(sv::detail::quat_from_forward_up(cv::Vec3f{0, 0, -1}, cv::Vec3f{0, 1, 0}, Pleg),
          "no-overshoot: build prev leg");
    cv::Vec4f Qleg;  // curr leg: same vertical forward, arbitrary roll → held
    check(sv::detail::quat_from_forward_up(cv::Vec3f{0, 0, -1}, cv::Vec3f{1, 0, 0}, Qleg),
          "no-overshoot: build curr leg");

    std::array<SlimeTracker, kTrackerCount> curr{};
    std::array<cv::Vec4f, kTrackerCount> prev{};
    prev[kLeg]   = Pleg;
    prev[kWaist] = cv::Vec4f{1, 0, 0, 0};  // waist starts facing world-forward

    // Waist holds a constant +th yaw (raw measurement) every frame; the leg
    // keeps its held vertical forward with unobservable roll.
    for (int f = 0; f < 24; ++f) {
        curr[kLeg].valid            = true;
        curr[kLeg].quat_wxyz        = Qleg;
        curr[kLeg].roll_confidence  = 0.0f;
        curr[kLeg].swing_confidence = 1.0f;
        curr[kWaist].valid            = true;
        curr[kWaist].quat_wxyz        = cv::Vec4f{std::cos(th * 0.5f), 0, 0, std::sin(th * 0.5f)};
        curr[kWaist].roll_confidence  = 1.0f;
        curr[kWaist].swing_confidence = 1.0f;
        sv::apply_quat_smoothing(curr, prev, 0.5f);
    }

    // Leg up must converge to the +th yaw: (-sin th, cos th, 0). A min_dot of
    // 0.999 (≈ 2.6° tolerance) excludes the buggy 2·th = 60° overshoot, whose
    // dot with the th target is cos(30°) ≈ 0.866.
    check_tracker_forward(curr[kLeg], cv::Vec3f{0, 0, -1},
                          "no-overshoot: forward still vertical", 1e-3f);
    check_tracker_up_direction(curr[kLeg], cv::Vec3f{-std::sin(th), std::cos(th), 0.0f},
                               "no-overshoot: up converges to waist yaw (not 2×)", 0.999f);
}

void set_left_arm_flex(fitra::infer::Skeleton3D& s, float flex_deg) {
    constexpr float kPi = 3.14159265358979323846f;
    const float a = flex_deg * kPi / 180.0f;
    const cv::Vec3f shoulder{0.18f, 0.0f, 1.42f};
    const cv::Vec3f elbow = shoulder + cv::Vec3f{0.0f, 0.30f, 0.0f};
    const cv::Vec3f wrist = elbow + cv::Vec3f{0.28f * std::sin(a),
                                              0.28f * std::cos(a), 0.0f};
    set_joint(s, 5, shoulder[0], shoulder[1], shoulder[2]);
    set_joint(s, 7, elbow[0], elbow[1], elbow[2]);
    set_joint(s, 9, wrist[0], wrist[1], wrist[2]);
}

void set_left_leg_flex(fitra::infer::Skeleton3D& s, float flex_deg,
                       const cv::Vec3f& toe_direction = cv::Vec3f{0.0f, 0.20f, 0.0f}) {
    constexpr float kPi = 3.14159265358979323846f;
    const float a = flex_deg * kPi / 180.0f;
    const cv::Vec3f hip{0.10f, 0.0f, 0.90f};
    const cv::Vec3f knee = hip + cv::Vec3f{0.0f, 0.0f, -0.45f};
    const cv::Vec3f ankle = knee + cv::Vec3f{0.0f, -0.40f * std::sin(a),
                                             -0.40f * std::cos(a)};
    const cv::Vec3f toe = ankle + toe_direction;
    set_joint(s, 11, hip[0], hip[1], hip[2]);
    set_joint(s, 13, knee[0], knee[1], knee[2]);
    set_joint(s, 15, ankle[0], ankle[1], ankle[2]);
    set_joint(s, 20, toe[0], toe[1], toe[2]);
    // These descendants are not orientation inputs, but keeping them near the
    // measured foot makes the fixture realistic and exercises rigid translation.
    set_joint(s, 22, toe[0] - 0.03f, toe[1], toe[2]);
    set_joint(s, 24, ankle[0], ankle[1] - 0.05f, ankle[2]);
}

void test_limb_extension_product_and_low_level_defaults() {
    namespace sv = fitra::slimevr;
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);

    const sv::TrackerExtractorOptions product_defaults;
    check(product_defaults.limb_extension.snap,
          "TrackerExtractor product default enables extension snap");
    check(product_defaults.limb_extension.toe_direction,
          "TrackerExtractor product default enables toe direction");

    auto skel = make_t_pose();
    set_left_arm_flex(skel, 8.0f);
    set_left_leg_flex(skel, 8.0f);

    sv::ExtractContext legacy_ctx, explicit_off_ctx;
    const auto legacy = sv::extract_trackers(skel, &legacy_ctx);
    sv::LimbExtensionOptions off;
    off.enter_flex_deg = 30.0f;
    off.exit_flex_deg = 5.0f;
    const auto explicit_off = sv::extract_trackers(
        skel, &explicit_off_ctx, sv::FootPosMode::Midpoint, 0.5f, 0.0f, off);

    for (std::size_t i = 0; i < sv::kTrackerCount; ++i) {
        check(legacy[i].role == explicit_off[i].role &&
              legacy[i].valid == explicit_off[i].valid &&
              legacy[i].roll_confidence == explicit_off[i].roll_confidence &&
              legacy[i].swing_confidence == explicit_off[i].swing_confidence,
              "extension OFF: tracker scalar fields must be exact");
        for (int k = 0; k < 3; ++k) {
            check(legacy[i].pos[k] == explicit_off[i].pos[k],
                  "extension OFF: tracker position must be exact");
        }
        for (int k = 0; k < 4; ++k) {
            check(legacy[i].quat_wxyz[k] == explicit_off[i].quat_wxyz[k],
                  "extension OFF: tracker quaternion must be exact");
        }
    }
    for (std::size_t i = 0; i < explicit_off_ctx.extension_latched.size(); ++i) {
        check(!explicit_off_ctx.extension_latched[i],
              "extension OFF: hysteresis latch must remain untouched");
        check(explicit_off_ctx.extension_flex_extreme_deg[i] < 0.0f,
              "extension OFF: direction extremum must remain untouched");
        check(explicit_off_ctx.extension_prev_flex_deg[i] < 0.0f,
              "extension OFF: previous flexion must remain untouched");
        check(explicit_off_ctx.extension_transition_frames[i] == 0,
              "extension OFF: transition confirmation must remain untouched");
    }
}

// Near-extension snapping reconstructs only the tracker-facing copy: both
// segment lengths are preserved, upper/lower leg forwards become one axis,
// and the input Skeleton3D remains byte-for-byte at its measured positions.
void test_limb_extension_snap_reconstructs_private_chain() {
    namespace sv = fitra::slimevr;
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_t_pose();
    set_left_arm_flex(skel, 8.0f);
    set_left_leg_flex(skel, 8.0f);

    const cv::Vec3f shoulder = cv::Vec3f{skel.joints[5].x, skel.joints[5].y, skel.joints[5].z};
    const cv::Vec3f elbow = cv::Vec3f{skel.joints[7].x, skel.joints[7].y, skel.joints[7].z};
    const cv::Vec3f wrist = cv::Vec3f{skel.joints[9].x, skel.joints[9].y, skel.joints[9].z};
    const cv::Vec3f hip = cv::Vec3f{skel.joints[11].x, skel.joints[11].y, skel.joints[11].z};
    const cv::Vec3f knee = cv::Vec3f{skel.joints[13].x, skel.joints[13].y, skel.joints[13].z};
    const cv::Vec3f ankle = cv::Vec3f{skel.joints[15].x, skel.joints[15].y, skel.joints[15].z};

    sv::LimbExtensionOptions opts;
    opts.snap = true;
    sv::ExtractContext ctx;
    auto trackers = sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle,
                                         0.5f, 0.0f, opts);
    check(ctx.extension_latched[0], "left arm should enter extension snap");
    check(ctx.extension_latched[2], "left leg should enter extension snap");

    const cv::Vec3f arm_axis = vec_normalize(wrist - shoulder);
    const cv::Vec3f snapped_elbow = shoulder + arm_axis * cv::norm(elbow - shoulder);
    check_tracker_forward(trackers[static_cast<std::size_t>(sv::TrackerRole::LeftUpperArm)],
                          arm_axis, "extension snap: arm uses shoulder->wrist axis");
    check_vec3_close(trackers[static_cast<std::size_t>(sv::TrackerRole::LeftUpperArm)].pos,
                     (shoulder + snapped_elbow) * 0.5f,
                     "extension snap: arm tracker position");

    const cv::Vec3f leg_axis = vec_normalize(ankle - hip);
    const cv::Vec3f snapped_knee = hip + leg_axis * cv::norm(knee - hip);
    const cv::Vec3f snapped_ankle = snapped_knee + leg_axis * cv::norm(ankle - knee);
    const auto upper_idx = static_cast<std::size_t>(sv::TrackerRole::LeftUpperLeg);
    const auto lower_idx = static_cast<std::size_t>(sv::TrackerRole::LeftLowerLeg);
    const auto foot_idx = static_cast<std::size_t>(sv::TrackerRole::LeftFoot);
    check_tracker_forward(trackers[upper_idx], leg_axis,
                          "extension snap: thigh uses hip->ankle axis");
    check_tracker_forward(trackers[lower_idx], leg_axis,
                          "extension snap: shin shares thigh axis");
    check_vec3_close(trackers[upper_idx].pos, (hip + snapped_knee) * 0.5f,
                     "extension snap: thigh tracker position");
    check_vec3_close(trackers[lower_idx].pos, (snapped_knee + snapped_ankle) * 0.5f,
                     "extension snap: shin tracker position");
    check_vec3_close(trackers[foot_idx].pos, snapped_ankle,
                     "extension snap: ankle foot position follows straight chain");
    check_vec3_close(ctx.foot_anchors[0].knee_to_ankle_dir,
                     vec_normalize(ankle - knee),
                     "extension snap: FK anchor keeps measured tibia direction");

    // The source bus payload is a const input and must not be rewritten.
    check_vec3_close(cv::Vec3f{skel.joints[7].x, skel.joints[7].y, skel.joints[7].z},
                     elbow, "extension snap: source elbow unchanged", 0.0f);
    check_vec3_close(cv::Vec3f{skel.joints[15].x, skel.joints[15].y, skel.joints[15].z},
                     ankle, "extension snap: source ankle unchanged", 0.0f);
}

void test_limb_extension_hysteresis() {
    namespace sv = fitra::slimevr;
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    sv::LimbExtensionOptions opts;
    opts.snap = true;
    opts.enter_flex_deg = 20.0f;
    opts.exit_flex_deg = 12.0f;
    sv::ExtractContext ctx;

    auto skel = make_t_pose();
    set_left_leg_flex(skel, 30.0f);
    (void)sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle, 0.5f, 0.0f, opts);
    check(!ctx.extension_latched[2], "reverse hysteresis starts free while bent");

    // Enter early while extending. One valid sample is not enough, and an
    // intervening missing sample must break the confirmation sequence.
    set_left_leg_flex(skel, 19.0f);
    (void)sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle, 0.5f, 0.0f, opts);
    check(!ctx.extension_latched[2],
          "reverse hysteresis requires two enter-confirmation frames");
    skel.joints[13].valid = false;
    (void)sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle, 0.5f, 0.0f, opts);
    skel.joints[13].valid = true;
    set_left_leg_flex(skel, 18.0f);
    (void)sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle, 0.5f, 0.0f, opts);
    check(!ctx.extension_latched[2],
          "missing sample resets enter confirmation");
    set_left_leg_flex(skel, 17.0f);
    (void)sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle, 0.5f, 0.0f, opts);
    check(!ctx.extension_latched[2],
          "first post-missing direction sample only starts confirmation");
    set_left_leg_flex(skel, 16.0f);
    auto held = sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle,
                                    0.5f, 0.0f, opts);
    check(ctx.extension_latched[2],
          "reverse hysteresis enters near 20 deg while extending");
    const cv::Vec3f hip{skel.joints[11].x, skel.joints[11].y, skel.joints[11].z};
    const cv::Vec3f ankle{skel.joints[15].x, skel.joints[15].y, skel.joints[15].z};
    check_tracker_forward(held[static_cast<std::size_t>(sv::TrackerRole::LeftUpperLeg)],
                          vec_normalize(ankle - hip),
                          "reverse hysteresis: extending mid-band is snapped");

    // A fresh mid-band sample has no motion direction and must not guess.
    sv::ExtractContext fresh_ctx;
    (void)sv::extract_trackers(skel, &fresh_ctx, sv::FootPosMode::Ankle,
                               0.5f, 0.0f, opts);
    check(!fresh_ctx.extension_latched[2],
          "reverse hysteresis does not acquire from a fresh mid-band frame");

    // Reach full extension, then begin flexing. A one-frame crossing of the
    // tighter exit threshold is ignored; two deliberate samples release early.
    set_left_leg_flex(skel, 8.0f);
    (void)sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle, 0.5f, 0.0f, opts);
    set_left_leg_flex(skel, 13.0f);
    (void)sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle, 0.5f, 0.0f, opts);
    check(ctx.extension_latched[2],
          "one-frame exit-threshold crossing must not release");
    set_left_leg_flex(skel, 12.5f);
    (void)sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle, 0.5f, 0.0f, opts);
    check(ctx.extension_latched[2],
          "opposite motion inside the overlap resets exit confirmation");
    set_left_leg_flex(skel, 13.0f);
    (void)sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle, 0.5f, 0.0f, opts);
    set_left_leg_flex(skel, 14.0f);
    (void)sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle, 0.5f, 0.0f, opts);
    check(!ctx.extension_latched[2],
          "reverse hysteresis exits near 12 deg while flexing");

    // Once released, continued flexion cannot immediately reacquire merely
    // because it remains below the wider enter threshold. A confirmed motion
    // reversal can reacquire without first having to bend past 20 degrees.
    set_left_leg_flex(skel, 17.0f);
    (void)sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle, 0.5f, 0.0f, opts);
    set_left_leg_flex(skel, 14.0f);
    (void)sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle, 0.5f, 0.0f, opts);
    check(!ctx.extension_latched[2],
          "single extending sample does not reacquire");
    set_left_leg_flex(skel, 15.0f);
    (void)sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle, 0.5f, 0.0f, opts);
    check(!ctx.extension_latched[2],
          "opposite motion inside the overlap resets enter confirmation");
    set_left_leg_flex(skel, 14.0f);
    (void)sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle, 0.5f, 0.0f, opts);
    set_left_leg_flex(skel, 13.0f);
    (void)sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle, 0.5f, 0.0f, opts);
    check(ctx.extension_latched[2],
          "confirmed mid-band motion reversal reacquires snap");
}

void test_extended_leg_toe_direction_drives_thigh_and_shin_twist() {
    namespace sv = fitra::slimevr;
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_t_pose();
    set_left_leg_flex(skel, 4.0f);

    sv::LimbExtensionOptions opts;
    opts.toe_direction = true;
    sv::ExtractContext ctx;
    auto trackers = sv::extract_trackers(skel, &ctx, sv::FootPosMode::Ankle,
                                         0.5f, 0.0f, opts);
    check(ctx.extension_latched[2], "toe direction activates in extension regime");

    const auto upper_idx = static_cast<std::size_t>(sv::TrackerRole::LeftUpperLeg);
    const auto lower_idx = static_cast<std::size_t>(sv::TrackerRole::LeftLowerLeg);
    check_tracker_up_direction(trackers[upper_idx], cv::Vec3f{0, -1, 0},
                               "toe direction: thigh twist follows toe", 0.99f);
    check_tracker_up_direction(trackers[lower_idx], cv::Vec3f{0, -1, 0},
                               "toe direction: shin twist follows toe", 0.99f);
    check(std::abs(trackers[upper_idx].roll_confidence - 0.3f) < 1.0e-4f,
          "toe direction: thigh uses foot jitter smoothing weight");
    check(std::abs(trackers[lower_idx].roll_confidence - 0.3f) < 1.0e-4f,
          "toe direction: shin uses foot jitter smoothing weight");

    // A missing toe must not invent a world-axis roll. The chain stays valid
    // through forward-only orientation, but roll returns to the held gate.
    skel.joints[20].valid = false;
    sv::ExtractContext missing_ctx;
    auto missing = sv::extract_trackers(skel, &missing_ctx, sv::FootPosMode::Ankle,
                                        0.5f, 0.0f, opts);
    check(missing[upper_idx].valid && missing[lower_idx].valid,
          "toe direction missing: leg swing remains valid");
    check(missing[upper_idx].roll_confidence == 0.0f &&
          missing[lower_idx].roll_confidence == 0.0f,
          "toe direction missing: fall back to held roll");
}

void test_extended_leg_toe_direction_ignores_bent_leg() {
    namespace sv = fitra::slimevr;
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_t_pose();
    set_left_leg_flex(skel, 35.0f, cv::Vec3f{0.20f, 0.0f, 0.0f});

    sv::ExtractContext base_ctx, toe_ctx;
    const auto base = sv::extract_trackers(skel, &base_ctx);
    sv::LimbExtensionOptions opts;
    opts.toe_direction = true;
    const auto with_toe = sv::extract_trackers(skel, &toe_ctx,
                                               sv::FootPosMode::Midpoint,
                                               0.5f, 0.0f, opts);
    check(!toe_ctx.extension_latched[2], "toe direction stays off for bent leg");
    for (const auto role : {sv::TrackerRole::LeftUpperLeg,
                            sv::TrackerRole::LeftLowerLeg}) {
        const auto i = static_cast<std::size_t>(role);
        float qdot = 0.0f;
        for (int k = 0; k < 4; ++k) qdot += base[i].quat_wxyz[k] * with_toe[i].quat_wxyz[k];
        check(std::abs(std::abs(qdot) - 1.0f) < 1.0e-6f,
              "toe direction: bent-leg orientation must be unchanged");
        check(base[i].roll_confidence == with_toe[i].roll_confidence,
              "toe direction: bent-leg confidence must be unchanged");
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

// ---------- One Euro (speed-adaptive) rotation overload ------------------

// Yaw quaternion (rotation about world/local +Z): wxyz = {cos(a/2),0,0,sin(a/2)}.
cv::Vec4f quat_yaw(float angle_rad) {
    return cv::Vec4f{std::cos(angle_rad * 0.5f), 0.0f, 0.0f, std::sin(angle_rad * 0.5f)};
}

// Geodesic angle (rad) between two unit quaternions, sign-agnostic.
float quat_angle(const cv::Vec4f& a, const cv::Vec4f& b) {
    float dot = std::abs(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]);
    return 2.0f * std::acos(std::min(1.0f, dot));
}

// First valid frame snaps (prev <- raw), marks ctx.initialized, zeroes the
// angular-speed estimate.
void test_one_euro_quat_first_frame_snaps() {
    using namespace fitra::slimevr;
    std::array<SlimeTracker, kTrackerCount> curr{};
    std::array<cv::Vec4f, kTrackerCount> prev{};
    QuatSmoothingContext ctx;
    const OneEuroParams p{1.0f, 0.3f, 1.0f};

    const cv::Vec4f raw = quat_yaw(0.7f);
    curr[0].valid = true;
    curr[0].quat_wxyz = raw;
    apply_quat_smoothing(curr, prev, ctx, p, 1.0f / 60.0f, 1.0f / 60.0f);

    check(quat_angle(curr[0].quat_wxyz, raw) < 1.0e-4f, "oe-quat.first.curr snaps to raw");
    check(quat_angle(prev[0], raw) < 1.0e-4f, "oe-quat.first.prev snaps to raw");
    check(ctx.initialized[0], "oe-quat.first.initialized set");
    check(std::abs(ctx.ang_vel_hat[0]) < 1.0e-6f, "oe-quat.first.ang_vel_hat zeroed");
}

// Invalid input holds prev and leaves ctx untouched (curr follows prev).
void test_one_euro_quat_invalid_holds_prev() {
    using namespace fitra::slimevr;
    std::array<SlimeTracker, kTrackerCount> curr{};
    std::array<cv::Vec4f, kTrackerCount> prev{};
    QuatSmoothingContext ctx;
    ctx.initialized[0] = true;
    ctx.ang_vel_hat[0] = 1.23f;
    prev[0] = quat_yaw(0.5f);
    curr[0].valid = false;
    curr[0].quat_wxyz = quat_yaw(2.0f);  // raw garbage on invalid
    const OneEuroParams p{1.0f, 0.3f, 1.0f};

    apply_quat_smoothing(curr, prev, ctx, p, 1.0f / 60.0f, 1.0f / 60.0f);

    check(quat_angle(prev[0], quat_yaw(0.5f)) < 1.0e-4f, "oe-quat.invalid.prev held");
    check(quat_angle(curr[0].quat_wxyz, quat_yaw(0.5f)) < 1.0e-4f, "oe-quat.invalid.curr follows prev");
    check(std::abs(ctx.ang_vel_hat[0] - 1.23f) < 1.0e-6f, "oe-quat.invalid.ctx untouched");
}

// At rest, a small rotational jitter moves the output far less than the
// fixed-alpha EMA (base_alpha=0.5). Default swing/twist confidences are 1.0,
// so both paths reduce to a single slerp gated by the per-step alpha.
void test_one_euro_quat_static_below_ema() {
    using namespace fitra::slimevr;
    const OneEuroParams p{1.0f, 0.3f, 1.0f};
    const float dt = 1.0f / 60.0f;
    const cv::Vec4f settled = quat_yaw(0.0f);     // identity
    const cv::Vec4f jitter  = quat_yaw(0.01f);    // 0.01 rad yaw jitter

    // One Euro: snap to identity, settle (ang_vel_hat -> 0), then jitter once.
    std::array<SlimeTracker, kTrackerCount> curr_oe{};
    std::array<cv::Vec4f, kTrackerCount> prev_oe{};
    QuatSmoothingContext ctx;
    for (int f = 0; f < 50; ++f) {
        curr_oe[0].valid = true; curr_oe[0].quat_wxyz = settled;
        apply_quat_smoothing(curr_oe, prev_oe, ctx, p, dt, dt);
    }
    curr_oe[0].valid = true; curr_oe[0].quat_wxyz = jitter;
    apply_quat_smoothing(curr_oe, prev_oe, ctx, p, dt, dt);
    const float moved_oe = quat_angle(prev_oe[0], settled);

    // Fixed-alpha EMA on the same step: slerp by 0.5 → 0.005 rad.
    std::array<SlimeTracker, kTrackerCount> curr_ema{};
    std::array<cv::Vec4f, kTrackerCount> prev_ema{};
    prev_ema[0] = settled;
    curr_ema[0].valid = true; curr_ema[0].quat_wxyz = jitter;
    apply_quat_smoothing(curr_ema, prev_ema, /*base_alpha=*/0.5f);
    const float moved_ema = quat_angle(prev_ema[0], settled);

    check(moved_oe < 0.5f * moved_ema,
          "oe-quat.static: One Euro response (" + std::to_string(moved_oe) +
          ") must be < half the EMA response (" + std::to_string(moved_ema) + ")");
}

}  // namespace

int main() {
    try {
        test_quat_from_forward_up_degenerate();   std::printf("[ok] quat_from_forward_up degeneracy\n");
        test_t_pose_extracts_all_ten();           std::printf("[ok] T-pose extracts all 10 trackers\n");
        test_torso_tracker_height_frac();         std::printf("[ok] chest/waist height frac slides position up the spine\n");
        test_role_to_position_mapping();          std::printf("[ok] role → TrackerPosition / sensor_id\n");
        test_missing_joints_yield_invalid();      std::printf("[ok] missing joints → invalid trackers\n");
        test_smoothing_double_cover();            std::printf("[ok] slerp double-cover handling\n");
        test_smoothing_invalid_holds_prev();      std::printf("[ok] invalid tracker holds previous orientation\n");
        test_upper_arm_confidence_full_at_90deg_bend(); std::printf("[ok] confidence: 90° elbow bend → 1.0\n");
        test_upper_arm_confidence_zero_all_degenerate(); std::printf("[ok] confidence: all-degenerate → 0\n");
        test_upper_arm_confidence_smoothstep_midrange(); std::printf("[ok] confidence: smoothstep midrange ≈ 0.5\n");
        test_smoothing_freezes_under_low_confidence();   std::printf("[ok] smoothing: both gates zero freezes update\n");
        test_roll_hold_keeps_swing();             std::printf("[ok] smoothing: roll-only hold keeps swing (M3)\n");
        test_pelvis_yaw_transport_held_roll();    std::printf("[ok] smoothing: pelvis-yaw transport rides held roll (M5)\n");
        test_arm_chest_leg_waist_transport();     std::printf("[ok] smoothing: arm→chest / leg→waist transport (M5)\n");
        test_pelvis_yaw_transport_no_overshoot(); std::printf("[ok] smoothing: pelvis-yaw transport converges, no overshoot (M5)\n");
        test_upper_arm_forward_raised();          std::printf("[ok] upper arm: forward-raised (前方挙上)\n");
        test_upper_arm_overhead_bent_elbow();     std::printf("[ok] upper arm: overhead + bent elbow (頭上挙上)\n");
        test_foot_toe_stance();                   std::printf("[ok] foot: toe stance (つま先立ち)\n");
        test_foot_heel_stance();                  std::printf("[ok] foot: heel stance (かかと立ち)\n");
        test_foot_inversion_unobservable();       std::printf("[ok] foot: lateral ankle, roll unobservable (横ずれ)\n");
        test_thigh_seated_knee_bent();            std::printf("[ok] thigh: seated knee bent 90° (着座)\n");
        test_thigh_walking_knee_60();             std::printf("[ok] thigh: walking knee bent ~60° (歩行片足)\n");
        test_thigh_standing_knee_straight();      std::printf("[ok] thigh: standing knee straight (立位)\n");
        test_thigh_lateral_ankle_uses_primary();  std::printf("[ok] thigh: lateral ankle activates primary (足首横ずれ)\n");
        test_thigh_seated_extended_straight_knee(); std::printf("[ok] thigh: 直座り — roll held, no world-Z rescue (swing tracks)\n");
        test_foot_fk_fallback_uses_last_anchor(); std::printf("[ok] foot: FK fallback synthesizes ankle/toe from last anchor\n");
        test_foot_fk_fallback_needs_seed();       std::printf("[ok] foot: FK fallback requires a seeded anchor\n");
        test_limb_extension_product_and_low_level_defaults(); std::printf("[ok] extension features: product ON / low-level reference OFF\n");
        test_limb_extension_snap_reconstructs_private_chain(); std::printf("[ok] extension snap: private arm/leg chain reconstruction\n");
        test_limb_extension_hysteresis();         std::printf("[ok] extension snap: enter/exit hysteresis\n");
        test_extended_leg_toe_direction_drives_thigh_and_shin_twist(); std::printf("[ok] extension toe direction: thigh/shin twist + missing fallback\n");
        test_extended_leg_toe_direction_ignores_bent_leg(); std::printf("[ok] extension toe direction: bent leg unchanged\n");
        test_keypoint_format_assert();            std::printf("[ok] Halpe26 keypoint-format assertion\n");
        test_one_euro_quat_first_frame_snaps();   std::printf("[ok] One Euro rotation: first frame snaps\n");
        test_one_euro_quat_invalid_holds_prev();  std::printf("[ok] One Euro rotation: invalid holds prev, ctx untouched\n");
        test_one_euro_quat_static_below_ema();    std::printf("[ok] One Euro rotation: at-rest jitter < EMA response\n");
        std::puts("test_tracker_extract ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_tracker_extract failed: %s\n", e.what());
        return 1;
    }
}
