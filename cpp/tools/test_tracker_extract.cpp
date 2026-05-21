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
    set_joint(s,  9, 0.72f, 0.05f, 1.42f); // l_wrist
    set_joint(s,  8, -0.45f,0.02f, 1.42f); // r_elbow
    set_joint(s, 10, -0.72f,0.05f, 1.42f); // r_wrist
    set_joint(s, 13, 0.1f,  0.01f, 0.45f); // l_knee
    set_joint(s, 14, -0.1f, 0.01f, 0.45f); // r_knee
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
    // LeftUpperLeg pos = midpoint(l_hip(0.1,0,0.9), l_knee(0.1,0.01,0.45))
    //                  = (0.1, 0.005, 0.675)
    check_vec3_close(trackers[idx(R::LeftUpperLeg)].pos,
                     cv::Vec3f{0.1f, 0.005f, 0.675f},
                     "LeftUpperLeg pos (world)");
    // LeftLowerLeg pos = midpoint(l_knee(0.1,0.01,0.45), l_ankle(0.1,0.05,0.05))
    //                  = (0.1, 0.03, 0.25)
    check_vec3_close(trackers[idx(R::LeftLowerLeg)].pos,
                     cv::Vec3f{0.1f, 0.03f, 0.25f},
                     "LeftLowerLeg pos (world)");
    // LeftFoot pos = midpoint(l_heel(0.1,0.02,0), l_big_toe(0.1,0.17,0))
    //              = (0.1, 0.095, 0)
    check_vec3_close(trackers[idx(R::LeftFoot)].pos,
                     cv::Vec3f{0.1f, 0.095f, 0.0f},
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
    check(position_for(R::Waist)         == P::Waist,         "Waist");
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
        // Knock out hip_center → Chest, Waist, both UpperLegs go invalid.
        auto skel = make_t_pose();
        skel.joints[19].valid = false;
        auto trackers = fitra::slimevr::extract_trackers(skel);
        check(!trackers[idx(R::Chest)].valid,         "Chest invalid w/o hip_center");
        check(!trackers[idx(R::Waist)].valid,         "Waist invalid w/o hip_center");
        check(!trackers[idx(R::LeftUpperLeg)].valid,  "LUL invalid w/o hip_center");
        check(!trackers[idx(R::RightUpperLeg)].valid, "RUL invalid w/o hip_center");
        // LowerLegs and feet do not require hip_center; they should still be ok.
        check( trackers[idx(R::LeftLowerLeg)].valid,  "LLL still valid w/o hip_center");
        check( trackers[idx(R::LeftFoot)].valid,      "LFoot still valid w/o hip_center");
    }
    {
        // Knock out l_heel → LeftFoot invalid, others fine.
        auto skel = make_t_pose();
        skel.joints[24].valid = false;
        auto trackers = fitra::slimevr::extract_trackers(skel);
        check(!trackers[idx(R::LeftFoot)].valid, "LFoot invalid w/o l_heel");
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

void test_smoothing_invalid_resets_prev() {
    std::array<fitra::slimevr::SlimeTracker, fitra::slimevr::kTrackerCount> curr{};
    std::array<cv::Vec4f, fitra::slimevr::kTrackerCount> prev{};
    curr[0].valid = false;
    curr[0].quat_wxyz = cv::Vec4f{1, 0, 0, 0};
    prev[0] = cv::Vec4f{0, 1, 0, 0};
    fitra::slimevr::apply_quat_smoothing(curr, prev, 0.5f);
    check_vec3_close(cv::Vec3f{prev[0][0], prev[0][1], prev[0][2]},
                     cv::Vec3f{1, 0, 0}, "invalid tracker resets prev to curr");
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
        test_smoothing_invalid_resets_prev();     std::printf("[ok] invalid tracker resets smoothing state\n");
        test_keypoint_format_assert();            std::printf("[ok] Halpe26 keypoint-format assertion\n");
        std::puts("test_tracker_extract ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_tracker_extract failed: %s\n", e.what());
        return 1;
    }
}
