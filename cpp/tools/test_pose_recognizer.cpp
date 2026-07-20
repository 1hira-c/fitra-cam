#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "infer/types.hpp"
#include "lift/head_direction.hpp"
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

// The pipeline feeds the recognizer the measured (pre-IK) skeleton for angles
// but must pair it with the *post-IK* bone_drift_pct. The post-IK skeleton is
// clamped to the model so its drift is ~0; raw pre-IK triangulation drifts well
// past max_bone_drift_pct (~10%). This locks that a perfectly valid T-pose is
// accepted with low drift but rejected on the bone_drift axis with high drift,
// so feeding the raw pre-IK drift (as a past regression did) would make pose
// hold impossible.
void test_drift_gate_decoupled_from_angles() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    const auto t_pose = make_extended_t_pose();

    fitra::lift::PoseRecognizer rec(30.0);
    rec.set_target(fitra::lift::TargetPose::kTPose);

    const auto lo = rec.update(t_pose, /*bone_drift_pct=*/2.0, 1.0 / 30.0);
    check(lo.angles_valid, "T-pose angles must be valid");
    check(lo.in_band, "valid T-pose with low (post-IK) drift must be in band");

    rec.reset();
    const auto hi = rec.update(t_pose, /*bone_drift_pct=*/25.0, 1.0 / 30.0);
    check(!hi.in_band, "high (raw pre-IK) drift must reject the pose");
    check(hi.failing_axis == "bone_drift",
          "rejection must be on the bone_drift axis, not an angle");
}

void test_halpe_face_joints_are_ignored_by_ik() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto measured = make_extended_t_pose();
    set_joint(measured, 17, 0.0f, 0.0f, 1.72f);  // head_top, retained
    set_joint(measured, 0, 0.0f, 0.10f, 1.62f);  // nose
    set_joint(measured, 1, -0.03f, 0.10f, 1.65f);
    set_joint(measured, 2, 0.03f, 0.10f, 1.65f);
    set_joint(measured, 3, -0.07f, 0.06f, 1.64f);
    set_joint(measured, 4, 0.07f, 0.06f, 1.64f);

    fitra::lift::IkSolver::Options opts;
    opts.bone_calib_frames = 1;
    opts.iterations = 1;
    fitra::lift::IkSolver ik{opts};
    (void)ik.update(measured);  // observe and lock the body lengths

    auto bad_face = measured;
    bad_face.joints[0].y = 10.0f;
    bad_face.joints[1].x = -8.0f;
    const auto face_out = ik.update(bad_face);
    check(face_out.joints[0].valid,
          "IK output must retain the direction-only nose endpoint");
    const double ndx = static_cast<double>(face_out.joints[0].x)
                     - face_out.joints[17].x;
    const double ndy = static_cast<double>(face_out.joints[0].y)
                     - face_out.joints[17].y;
    const double ndz = static_cast<double>(face_out.joints[0].z)
                     - face_out.joints[17].z;
    const double nose_len = std::sqrt(ndx * ndx + ndy * ndy + ndz * ndz);
    check(std::abs(nose_len - fitra::lift::kHeadDirectionLengthM) < 1.0e-4,
          "IK output nose must be fixed-length direction only");
    check(!face_out.joints[1].valid,
          "IK must drop Halpe26 eye observations");
    check(ik.bone_drift_pct(face_out) < 0.01,
          "facial outliers must not inflate subject-calibration bone drift");

    auto bad_head_top = measured;
    bad_head_top.joints[17].z = 2.50f;
    const auto head_out = ik.update(bad_head_top);
    const double dz = static_cast<double>(head_out.joints[17].z)
                    - head_out.joints[18].z;
    check(std::abs(std::abs(dz) - 0.27) < 1.0e-4,
          "head_top-to-neck length must remain constrained");
}

}  // namespace

int main() {
    try {
        test_extended_elbow_angles_are_measured_pre_ik();
        test_drift_gate_decoupled_from_angles();
        test_halpe_face_joints_are_ignored_by_ik();
        std::printf("test_pose_recognizer: OK\n");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_pose_recognizer: FAIL: %s\n", e.what());
        return 1;
    }
}
