#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "infer/types.hpp"
#include "lift/ik.hpp"
#include "lift/keypoint_format.hpp"
#include "lift/pose_recognizer.hpp"

namespace {

void check(bool cond, const std::string& msg) {
    if (!cond) throw std::runtime_error(msg);
}

void set_joint(fitra::infer::Skeleton3D& s, std::size_t i,
               float x, float y, float z) {
    s.joints[i].x = x;
    s.joints[i].y = y;
    s.joints[i].z = z;
    s.joints[i].score = 1.0f;
    s.joints[i].valid = true;
}

fitra::infer::Skeleton3D make_extended_t_pose() {
    fitra::infer::Skeleton3D s;
    s.kp_count = 26;

    // Torso midpoints used by compute_pose_angles under Halpe26.
    set_joint(s, 18, 0.0f, 0.0f, 1.45f);  // neck
    set_joint(s, 19, 0.0f, 0.0f, 0.95f);  // hip_center

    set_joint(s, 5, -0.20f, 0.0f, 1.40f);  // l_shoulder
    set_joint(s, 7, -0.50f, 0.0f, 1.40f);  // l_elbow
    set_joint(s, 9, -0.80f, 0.0f, 1.40f);  // l_wrist
    set_joint(s, 6,  0.20f, 0.0f, 1.40f);  // r_shoulder
    set_joint(s, 8,  0.50f, 0.0f, 1.40f);  // r_elbow
    set_joint(s, 10, 0.80f, 0.0f, 1.40f);  // r_wrist

    set_joint(s, 11, -0.12f, 0.0f, 0.95f);  // l_hip
    set_joint(s, 13, -0.12f, 0.0f, 0.55f);  // l_knee
    set_joint(s, 15, -0.12f, 0.0f, 0.15f);  // l_ankle
    set_joint(s, 12,  0.12f, 0.0f, 0.95f);  // r_hip
    set_joint(s, 14,  0.12f, 0.0f, 0.55f);  // r_knee
    set_joint(s, 16,  0.12f, 0.0f, 0.15f);  // r_ankle
    return s;
}

void test_extended_elbow_angles_are_measured_pre_ik() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    const auto measured = make_extended_t_pose();
    const auto raw_angles = fitra::lift::compute_pose_angles(measured);
    check(raw_angles.valid, "raw pose angles must be valid");
    check(raw_angles.left_elbow_flex < 1.0,
          "raw left elbow flex should stay near 0 deg");
    check(raw_angles.right_elbow_flex < 1.0,
          "raw right elbow flex should stay near 0 deg");

    fitra::lift::IkSolver::Options opts;
    opts.subject_height_m = 1.70;
    opts.max_hinge_deg = 150.0;  // artificial 30 deg extension clamp.
    fitra::lift::IkSolver ik{opts};
    const auto post_ik = ik.update(measured);
    const auto ik_angles = fitra::lift::compute_pose_angles(post_ik);
    check(ik_angles.valid, "post-IK pose angles must be valid");
    check(ik_angles.left_elbow_flex > 20.0,
          "post-IK left elbow flex shows hinge-clamp bias");
    check(ik_angles.right_elbow_flex > 20.0,
          "post-IK right elbow flex shows hinge-clamp bias");
}

}  // namespace

int main() {
    try {
        test_extended_elbow_angles_are_measured_pre_ik();
        std::printf("test_pose_recognizer: OK\n");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_pose_recognizer: FAIL: %s\n", e.what());
        return 1;
    }
}
