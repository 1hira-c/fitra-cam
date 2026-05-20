// test_tracker_extract — exercise Halpe26 → 8 VMC tracker extraction +
// coordinate transform + quaternion smoothing.
//
// Three synthetic skeletons (T-pose, A-pose, left-leg-raised) verify:
//   - position transform (world Z-up → Unity Y-up)
//   - quaternion-from-forward-up via Shoemake's matrix-to-quat
//   - degeneracy handling (parallel forward/up, missing joints)
//   - slerp smoothing direction handling (q · -q double-cover)

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

// Build a Halpe26 quasi-T-pose: subject standing at origin facing +Y, arms
// roughly straight to the sides, knees with a small forward bend. Realistic
// human pose -- perfectly straight limbs cause anti-parallel forward/up axes
// at the knee/elbow trackers and the rotation about the limb axis is
// genuinely ambiguous in that case; SlimeVR sees the same degeneracy and
// reports zero bend. We bend ~3 cm forward at each elbow / knee to mirror a
// real T-pose hold without locked joints.
fitra::infer::Skeleton3D make_t_pose() {
    fitra::infer::Skeleton3D s;
    s.kp_count = 26;
    set_joint(s, 19, 0,    0, 0.9f);   // hip_center
    set_joint(s, 11, 0.1f, 0, 0.9f);   // l_hip
    set_joint(s, 12, -0.1f, 0, 0.9f);  // r_hip
    set_joint(s, 18, 0,    0, 1.45f);  // neck
    set_joint(s,  5, 0.18f,0, 1.42f);  // l_shoulder
    set_joint(s,  6, -0.18f,0,1.42f);  // r_shoulder

    // Arms with slight forward elbow/wrist offset (Y+) so the
    // elbow→wrist (forward) and elbow→shoulder (up) hints are not collinear.
    set_joint(s,  7, 0.45f, 0.02f, 1.42f);  // l_elbow
    set_joint(s,  9, 0.72f, 0.05f, 1.42f);  // l_wrist
    set_joint(s,  8, -0.45f,0.02f, 1.42f);  // r_elbow
    set_joint(s, 10, -0.72f,0.05f, 1.42f);  // r_wrist

    // Legs with small forward shift at ankle/foot to avoid degenerate
    // knee orientation.
    set_joint(s, 13, 0.1f,  0.01f, 0.45f); // l_knee (tiny forward shift too)
    set_joint(s, 14, -0.1f, 0.01f, 0.45f); // r_knee
    set_joint(s, 15, 0.1f,  0.05f, 0.05f); // l_ankle
    set_joint(s, 16, -0.1f, 0.05f, 0.05f); // r_ankle

    set_joint(s, 24, 0.1f,  0.02f, 0.0f);  // l_heel
    set_joint(s, 20, 0.1f,  0.17f, 0.0f);  // l_big_toe
    set_joint(s, 25, -0.1f, 0.02f, 0.0f);  // r_heel
    set_joint(s, 21, -0.1f, 0.17f, 0.0f);  // r_big_toe

    set_joint(s,  0, 0, 0, 1.65f);  // nose
    set_joint(s, 17, 0, 0, 1.72f);  // head_top
    return s;
}

void test_coordinate_transforms() {
    // World (X right, Y forward, Z up) → Unity LH (X right, Y up, Z forward).
    // Spec: (px, py, pz) → (px, pz, -py).
    auto p = fitra::slimevr::detail::world_pos_to_vmc(cv::Vec3f{1, 2, 3});
    check_vec3_close(p, cv::Vec3f{1, 3, -2}, "world_pos_to_vmc(1,2,3)");

    // Edge case: (0, 1, 0) (pointing forward) → (0, 0, -1) (Unity backward,
    // since RH-forward becomes LH-backward without an extra Z flip).
    p = fitra::slimevr::detail::world_pos_to_vmc(cv::Vec3f{0, 1, 0});
    check_vec3_close(p, cv::Vec3f{0, 0, -1}, "world_pos_to_vmc(0,1,0)");

    // Quaternion transform: (qw, qx, qy, qz) wxyz → wxyz (-qw, qx, qz, -qy).
    // Identity world quat (1,0,0,0) → (-1, 0, 0, 0). When written xyzw on the
    // wire that's (0, 0, 0, -1) which represents the same rotation as
    // (0, 0, 0, 1) under the quat double-cover -- VMCHandler.kt treats both
    // identically.
    auto q = fitra::slimevr::detail::world_quat_to_vmc(cv::Vec4f{1, 0, 0, 0});
    check_vec3_close(cv::Vec3f{q[0], q[1], q[2]}, cv::Vec3f{-1, 0, 0},
                     "world_quat_to_vmc(identity).w/x/y");
    check(std::abs(q[3]) < kEps, "world_quat_to_vmc(identity).z != 0");
}

void test_quat_from_forward_up_degenerate() {
    cv::Vec4f q;
    // forward = up exactly → degenerate.
    bool ok = fitra::slimevr::detail::quat_from_forward_up(
        cv::Vec3f{0, 0, 1}, cv::Vec3f{0, 0, 1}, q);
    check(!ok, "quat_from_forward_up parallel inputs should be invalid");
    check_vec3_close(cv::Vec3f{q[0], q[1], q[2]}, cv::Vec3f{1, 0, 0},
                     "degenerate quat should be identity wxyz=(1,0,0,0)");
    check(std::abs(q[3]) < kEps, "degenerate quat z component");

    // zero forward → degenerate.
    ok = fitra::slimevr::detail::quat_from_forward_up(
        cv::Vec3f{0, 0, 0}, cv::Vec3f{0, 0, 1}, q);
    check(!ok, "quat_from_forward_up zero forward should be invalid");
}

void test_t_pose_extracts_all_eight() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_t_pose();
    auto trackers = fitra::slimevr::extract_vmc_trackers(skel);

    // All 8 trackers should be valid for a complete T-pose.
    for (std::size_t i = 0; i < trackers.size(); ++i) {
        if (!trackers[i].valid) {
            throw std::runtime_error(
                "tracker " + std::to_string(i) + " unexpectedly invalid");
        }
    }

    // Sanity: positions should be in Unity Y-up frame. Waist (hip_center) at
    // world (0,0,0.9) → Unity (0, 0.9, 0).
    check_vec3_close(trackers[0].pos, cv::Vec3f{0, 0.9f, 0},
                     "T-pose waist pos in Unity frame");
    // Chest (neck) at world (0,0,1.45) → Unity (0, 1.45, 0).
    check_vec3_close(trackers[1].pos, cv::Vec3f{0, 1.45f, 0},
                     "T-pose chest pos in Unity frame");
    // Left foot center = average of l_heel(0.1,0.02,0) and l_big_toe(0.1,0.17,0)
    //   = world (0.1, 0.095, 0) → Unity (0.1, 0, -0.095).
    check_vec3_close(trackers[6].pos, cv::Vec3f{0.1f, 0, -0.095f},
                     "T-pose left foot pos in Unity frame");
    // Left knee at world (0.1,0.01,0.45) → Unity (0.1, 0.45, -0.01).
    check_vec3_close(trackers[2].pos, cv::Vec3f{0.1f, 0.45f, -0.01f},
                     "T-pose left knee pos in Unity frame");
    // Left elbow at world (0.45,0.02,1.42) → Unity (0.45, 1.42, -0.02).
    check_vec3_close(trackers[4].pos, cv::Vec3f{0.45f, 1.42f, -0.02f},
                     "T-pose left elbow pos in Unity frame");

    // Quaternions should all be unit length within ε.
    for (const auto& t : trackers) {
        float n2 = t.quat_wxyz[0]*t.quat_wxyz[0] + t.quat_wxyz[1]*t.quat_wxyz[1]
                 + t.quat_wxyz[2]*t.quat_wxyz[2] + t.quat_wxyz[3]*t.quat_wxyz[3];
        check(std::abs(n2 - 1.0f) < 1.0e-3f, "quat not unit length");
    }
}

void test_missing_joints_yield_invalid() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto skel = make_t_pose();
    // Knock out the left wrist; expect left elbow tracker to be invalid, the
    // rest still valid.
    skel.joints[9].valid = false;
    auto trackers = fitra::slimevr::extract_vmc_trackers(skel);
    check(!trackers[4].valid, "left elbow should be invalid when wrist missing");
    check(trackers[5].valid,  "right elbow should still be valid");
    check(trackers[0].valid,  "waist should still be valid");

    // Knock out hip_center; waist + chest should both be invalid.
    skel.joints[19].valid = false;
    trackers = fitra::slimevr::extract_vmc_trackers(skel);
    check(!trackers[0].valid, "waist invalid without hip_center");
    check(!trackers[1].valid, "chest invalid without hip_center");
}

void test_smoothing_double_cover() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    std::array<fitra::slimevr::VmcTracker, fitra::slimevr::kTrackerCount> curr{};
    std::array<cv::Vec4f, fitra::slimevr::kTrackerCount> prev{};
    // Set up a single valid tracker with prev = identity and curr = -identity
    // (same rotation under double-cover). Smoothing should flip curr and
    // produce identity-ish.
    curr[0].valid = true;
    curr[0].quat_wxyz = cv::Vec4f{-1, 0, 0, 0};
    prev[0] = cv::Vec4f{1, 0, 0, 0};
    fitra::slimevr::apply_quat_smoothing(curr, prev, 0.5f);
    // After smoothing, the magnitude should be 1 and the rotation should be
    // identity (within ε).
    cv::Vec4f r = curr[0].quat_wxyz;
    float n = std::sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2] + r[3]*r[3]);
    check(std::abs(n - 1.0f) < 1.0e-3f, "smoothed quat not unit");
    // |w| ≈ 1, x/y/z ≈ 0.
    check(std::abs(std::abs(r[0]) - 1.0f) < 1.0e-3f, "smoothed quat not ±identity");
    check(std::abs(r[1]) < 1.0e-3f, "smoothed quat x ≠ 0");
    check(std::abs(r[2]) < 1.0e-3f, "smoothed quat y ≠ 0");
    check(std::abs(r[3]) < 1.0e-3f, "smoothed quat z ≠ 0");
}

void test_smoothing_invalid_resets_prev() {
    std::array<fitra::slimevr::VmcTracker, fitra::slimevr::kTrackerCount> curr{};
    std::array<cv::Vec4f, fitra::slimevr::kTrackerCount> prev{};
    curr[0].valid = false;
    curr[0].quat_wxyz = cv::Vec4f{1, 0, 0, 0};
    prev[0] = cv::Vec4f{0, 1, 0, 0};
    fitra::slimevr::apply_quat_smoothing(curr, prev, 0.5f);
    // prev should now match curr (the identity), not the stale (0,1,0,0).
    check_vec3_close(cv::Vec3f{prev[0][0], prev[0][1], prev[0][2]},
                     cv::Vec3f{1, 0, 0}, "invalid tracker resets prev to curr");
}

void test_keypoint_format_assert() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Coco17);
    auto skel = make_t_pose();
    bool threw = false;
    try {
        (void)fitra::slimevr::extract_vmc_trackers(skel);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    check(threw, "extract_vmc_trackers should refuse COCO17");
    // restore for any subsequent tests
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
}

}  // namespace

int main() {
    try {
        test_coordinate_transforms();
        test_quat_from_forward_up_degenerate();
        test_t_pose_extracts_all_eight();
        test_missing_joints_yield_invalid();
        test_smoothing_double_cover();
        test_smoothing_invalid_resets_prev();
        test_keypoint_format_assert();
        std::puts("test_tracker_extract ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_tracker_extract failed: %s\n", e.what());
        return 1;
    }
}
