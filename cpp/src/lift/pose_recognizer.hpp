#pragma once
//
// 3D-angle based pose classifier for the subject-calibration wizard.
// Operates on a measured pre-IK Skeleton3D plus the current IK bone_drift_pct,
// and reports whether the subject is holding the requested target pose for
// long enough to start recording.
//
// Joint angles are computed in 3D world space, so judgment is independent of
// the cameras' azimuth/elevation. The wizard still requires a subject height
// up front so bone_drift_pct can reject geometry that is too far from the
// height prior, but angle classification intentionally avoids post-IK clamps.

#include <array>
#include <string>
#include <utility>

#include "infer/types.hpp"

namespace fitra::lift {

enum class TargetPose {
    kStanding = 0,
    kTPose = 1,
    kElbowFlex = 2,
    kKneeFlex = 3,
    kCount = 4,
};

const char* target_pose_name(TargetPose p);
TargetPose target_pose_from_name(const std::string& name);

struct AngleBand {
    double min_deg;
    double max_deg;
};

struct PoseTemplate {
    TargetPose pose;
    AngleBand left_elbow_flex;
    AngleBand right_elbow_flex;
    AngleBand left_shoulder_abduction;
    AngleBand right_shoulder_abduction;
    AngleBand left_knee_flex;
    AngleBand right_knee_flex;
    AngleBand torso_tilt;
    double max_bone_drift_pct;
};

struct PoseAngles {
    double left_elbow_flex = -1.0;
    double right_elbow_flex = -1.0;
    double left_shoulder_abduction = -1.0;
    double right_shoulder_abduction = -1.0;
    double left_knee_flex = -1.0;
    double right_knee_flex = -1.0;
    double torso_tilt = -1.0;
    bool valid = false;
};

PoseAngles compute_pose_angles(const infer::Skeleton3D& skel);

struct PoseDetectionState {
    TargetPose target = TargetPose::kStanding;
    bool in_band = false;
    bool angles_valid = false;
    int consecutive_ok = 0;
    int consecutive_required = 0;
    double hold_elapsed_sec = 0.0;
    double required_hold_sec = 0.0;
    double hold_progress = 0.0;
    double bone_drift_pct = 0.0;
    PoseAngles angles;
    std::string failing_axis;
};

class PoseRecognizer {
public:
    explicit PoseRecognizer(double fps_hint = 30.0);

    void set_target(TargetPose target);
    TargetPose target() const { return target_; }

    void set_required_hold_sec(double sec);
    double required_hold_sec() const { return required_hold_sec_; }

    void set_fps_hint(double fps);
    double fps_hint() const { return fps_hint_; }

    void reset();
    PoseDetectionState update(const infer::Skeleton3D& skel,
                              double bone_drift_pct,
                              double dt_sec = -1.0);

    static const PoseTemplate& templ_for(TargetPose pose);

private:
    TargetPose target_ = TargetPose::kStanding;
    double fps_hint_ = 30.0;
    double required_hold_sec_ = 1.5;
    int consecutive_ok_ = 0;
    double hold_elapsed_sec_ = 0.0;
};

}  // namespace fitra::lift
