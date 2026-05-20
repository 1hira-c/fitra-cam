#include "lift/pose_recognizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <opencv2/core.hpp>

#include "lift/keypoint_format.hpp"

namespace fitra::lift {

namespace {

cv::Vec3d to_vec(const infer::Joint3D& j) {
    return {j.x, j.y, j.z};
}

double norm(const cv::Vec3d& v) { return std::sqrt(v.dot(v)); }

double angle_between_deg(const cv::Vec3d& a, const cv::Vec3d& b) {
    double na = norm(a);
    double nb = norm(b);
    if (na < 1.0e-6 || nb < 1.0e-6) return -1.0;
    double dot = std::clamp(a.dot(b) / (na * nb), -1.0, 1.0);
    return std::acos(dot) * 180.0 / CV_PI;
}

double joint_angle_deg(const infer::Joint3D& a,
                       const infer::Joint3D& v,
                       const infer::Joint3D& c) {
    if (!a.valid || !v.valid || !c.valid) return -1.0;
    cv::Vec3d va = to_vec(a) - to_vec(v);
    cv::Vec3d vc = to_vec(c) - to_vec(v);
    return angle_between_deg(va, vc);
}

double shoulder_abduction_deg(const infer::Joint3D& hip,
                              const infer::Joint3D& shoulder,
                              const infer::Joint3D& elbow) {
    if (!hip.valid || !shoulder.valid || !elbow.valid) return -1.0;
    // Both vectors point "down" when the arm is at rest, so the angle is 0
    // for a standing arm-at-side pose and ~90 for a T-pose horizontal arm.
    cv::Vec3d torso_down = to_vec(hip) - to_vec(shoulder);
    cv::Vec3d arm = to_vec(elbow) - to_vec(shoulder);
    return angle_between_deg(torso_down, arm);
}

double flex_from_joint_angle(double joint_angle_deg) {
    if (joint_angle_deg < 0.0) return -1.0;
    // 180 - inner angle so that 0 = fully extended, 90 = right-angle bend.
    return 180.0 - joint_angle_deg;
}

// Templates ordered to match TargetPose enum.
constexpr std::array<PoseTemplate, 4> kPoseTemplates{{
    // standing
    {TargetPose::kStanding,
     {0.0, 30.0}, {0.0, 30.0},
     {0.0, 30.0}, {0.0, 30.0},
     {0.0, 30.0}, {0.0, 30.0},
     {0.0, 15.0},
     10.0},
    // t_pose
    {TargetPose::kTPose,
     {0.0, 35.0}, {0.0, 35.0},
     {60.0, 120.0}, {60.0, 120.0},
     {0.0, 30.0}, {0.0, 30.0},
     {0.0, 15.0},
     10.0},
    // elbow_flex (arms held out, elbows bent ~90 deg)
    {TargetPose::kElbowFlex,
     {60.0, 130.0}, {60.0, 130.0},
     {30.0, 110.0}, {30.0, 110.0},
     {0.0, 30.0}, {0.0, 30.0},
     {0.0, 20.0},
     10.0},
    // knee_flex (squat-like, knees bent)
    {TargetPose::kKneeFlex,
     {0.0, 60.0}, {0.0, 60.0},
     {0.0, 70.0}, {0.0, 70.0},
     {40.0, 130.0}, {40.0, 130.0},
     {0.0, 30.0},
     10.0},
}};

bool in_band(double v, const AngleBand& b) {
    return v >= 0.0 && v >= b.min_deg && v <= b.max_deg;
}

}  // namespace

const char* target_pose_name(TargetPose p) {
    switch (p) {
        case TargetPose::kStanding:  return "standing";
        case TargetPose::kTPose:     return "t_pose";
        case TargetPose::kElbowFlex: return "elbow_flex";
        case TargetPose::kKneeFlex:  return "knee_flex";
        default:                     return "unknown";
    }
}

TargetPose target_pose_from_name(const std::string& name) {
    if (name == "standing")    return TargetPose::kStanding;
    if (name == "t_pose")      return TargetPose::kTPose;
    if (name == "elbow_flex")  return TargetPose::kElbowFlex;
    if (name == "knee_flex")   return TargetPose::kKneeFlex;
    return TargetPose::kCount;
}

PoseAngles compute_pose_angles(const infer::Skeleton3D& s) {
    PoseAngles a;
    a.left_elbow_flex  = flex_from_joint_angle(
        joint_angle_deg(s.joints[5], s.joints[7], s.joints[9]));
    a.right_elbow_flex = flex_from_joint_angle(
        joint_angle_deg(s.joints[6], s.joints[8], s.joints[10]));
    a.left_shoulder_abduction  =
        shoulder_abduction_deg(s.joints[11], s.joints[5], s.joints[7]);
    a.right_shoulder_abduction =
        shoulder_abduction_deg(s.joints[12], s.joints[6], s.joints[8]);
    a.left_knee_flex  = flex_from_joint_angle(
        joint_angle_deg(s.joints[11], s.joints[13], s.joints[15]));
    a.right_knee_flex = flex_from_joint_angle(
        joint_angle_deg(s.joints[12], s.joints[14], s.joints[16]));

    // Under Halpe26 the model produces explicit shoulder-midpoint (neck = 18)
    // and hip-center (19) joints; use them directly so we don't reintroduce
    // the small bias that a (L+R)/2 average has when the shoulders or hips
    // are foreshortened. Fall back to the COCO17 midpoint formula whenever
    // any of those Halpe26 joints is missing or we're running under COCO17.
    const auto& ls = s.joints[5];
    const auto& rs = s.joints[6];
    const auto& lh = s.joints[11];
    const auto& rh = s.joints[12];
    bool have_midpoints = false;
    cv::Vec3d mid_sh{0.0, 0.0, 0.0};
    cv::Vec3d mid_hp{0.0, 0.0, 0.0};
    if (active_keypoint_format() == KeypointFormat::Halpe26) {
        const auto& neck = s.joints[18];
        const auto& hipc = s.joints[19];
        if (neck.valid && hipc.valid) {
            mid_sh = cv::Vec3d{neck.x, neck.y, neck.z};
            mid_hp = cv::Vec3d{hipc.x, hipc.y, hipc.z};
            have_midpoints = true;
        }
    }
    if (!have_midpoints && ls.valid && rs.valid && lh.valid && rh.valid) {
        mid_sh = cv::Vec3d{(ls.x + rs.x) * 0.5,
                           (ls.y + rs.y) * 0.5,
                           (ls.z + rs.z) * 0.5};
        mid_hp = cv::Vec3d{(lh.x + rh.x) * 0.5,
                           (lh.y + rh.y) * 0.5,
                           (lh.z + rh.z) * 0.5};
        have_midpoints = true;
    }
    if (have_midpoints) {
        cv::Vec3d torso = mid_sh - mid_hp;
        cv::Vec3d up{0.0, 0.0, 1.0};
        a.torso_tilt = angle_between_deg(torso, up);
    }

    a.valid = a.left_elbow_flex >= 0.0
           && a.right_elbow_flex >= 0.0
           && a.left_shoulder_abduction >= 0.0
           && a.right_shoulder_abduction >= 0.0
           && a.left_knee_flex >= 0.0
           && a.right_knee_flex >= 0.0
           && a.torso_tilt >= 0.0;
    return a;
}

PoseRecognizer::PoseRecognizer(double fps_hint) : fps_hint_(std::max(1.0, fps_hint)) {}

void PoseRecognizer::set_target(TargetPose target) {
    target_ = target;
    consecutive_ok_ = 0;
    hold_elapsed_sec_ = 0.0;
}

void PoseRecognizer::set_required_hold_sec(double sec) {
    required_hold_sec_ = std::max(0.1, sec);
}

void PoseRecognizer::set_fps_hint(double fps) {
    fps_hint_ = std::max(1.0, fps);
}

void PoseRecognizer::reset() {
    consecutive_ok_ = 0;
    hold_elapsed_sec_ = 0.0;
}

const PoseTemplate& PoseRecognizer::templ_for(TargetPose pose) {
    auto idx = static_cast<std::size_t>(pose);
    if (idx >= kPoseTemplates.size()) idx = 0;
    return kPoseTemplates[idx];
}

PoseDetectionState PoseRecognizer::update(const infer::Skeleton3D& skel,
                                           double bone_drift_pct,
                                           double dt_sec) {
    PoseDetectionState st;
    st.target = target_;
    st.angles = compute_pose_angles(skel);
    st.angles_valid = st.angles.valid;
    st.bone_drift_pct = bone_drift_pct;
    st.required_hold_sec = required_hold_sec_;
    st.consecutive_required = std::max(
        1, static_cast<int>(std::lround(required_hold_sec_ * fps_hint_)));

    const PoseTemplate& t = templ_for(target_);

    if (!st.angles_valid) {
        st.failing_axis = "joints_invalid";
    } else if (bone_drift_pct > t.max_bone_drift_pct) {
        st.failing_axis = "bone_drift";
    } else if (!in_band(st.angles.left_elbow_flex, t.left_elbow_flex)) {
        st.failing_axis = "left_elbow_flex";
    } else if (!in_band(st.angles.right_elbow_flex, t.right_elbow_flex)) {
        st.failing_axis = "right_elbow_flex";
    } else if (!in_band(st.angles.left_shoulder_abduction, t.left_shoulder_abduction)) {
        st.failing_axis = "left_shoulder_abduction";
    } else if (!in_band(st.angles.right_shoulder_abduction, t.right_shoulder_abduction)) {
        st.failing_axis = "right_shoulder_abduction";
    } else if (!in_band(st.angles.left_knee_flex, t.left_knee_flex)) {
        st.failing_axis = "left_knee_flex";
    } else if (!in_band(st.angles.right_knee_flex, t.right_knee_flex)) {
        st.failing_axis = "right_knee_flex";
    } else if (!in_band(st.angles.torso_tilt, t.torso_tilt)) {
        st.failing_axis = "torso_tilt";
    }

    st.in_band = st.failing_axis.empty();
    double step_sec = dt_sec > 0.0 ? dt_sec : (1.0 / fps_hint_);
    if (!std::isfinite(step_sec) || step_sec < 0.0) step_sec = 0.0;
    step_sec = std::min(step_sec, 0.5);
    if (st.in_band) {
        ++consecutive_ok_;
        hold_elapsed_sec_ += step_sec;
    } else {
        consecutive_ok_ = 0;
        // Decay 2x faster than fill so a sustained miss empties the gauge in
        // ~half the hold window, but a 1-2 frame angle wobble barely moves it.
        hold_elapsed_sec_ -= step_sec * 2.0;
        if (hold_elapsed_sec_ < 0.0) hold_elapsed_sec_ = 0.0;
    }
    st.consecutive_ok = consecutive_ok_;
    st.hold_elapsed_sec = hold_elapsed_sec_;
    st.hold_progress = std::min(
        1.0,
        hold_elapsed_sec_ / std::max(0.001, required_hold_sec_));
    return st;
}

}  // namespace fitra::lift
