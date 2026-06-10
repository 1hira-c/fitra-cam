#pragma once
//
// Controller-marker multi-camera extrinsic solver (hand-eye / AX=ZB).
//
// See docs/design/pose-3d-controller-marker-extrinsic.md. A multi-face marker
// is rigidly mounted to a VR controller whose 6DoF pose is known in the VR
// world frame. Each static camera observes the marker at different times; we
// never need two cameras to see it simultaneously. The rigid chain per sample
// is
//
//     T_cam←marker(i) = T_cam←world · T_world←controller(i) · T_controller←marker
//
// Rearranged to A_i·X = Z·B_i with
//
//     A_i = T_cam←marker(i)        (per-camera PnP, the marker as seen by cam)
//     B_i = T_world←controller(i)  (VR controller pose)
//     X   = T_marker←controller    (shared rigid mount offset = Y_face⁻¹)
//     Z   = T_cam←world            (the extrinsic we want, per camera)
//
// which is exactly cv::calibrateRobotWorldHandEye's model (AX = ZB):
//   A ↔ ᶜT_w (R/t_world2cam), B ↔ ᵍT_b (R/t_base2gripper),
//   X ↔ ʷT_b (base2world),    Z ↔ ᶜT_g (gripper2cam).
//
// Each marker face is an independent "world" frame, so we solve once per
// (camera, face) group. A camera's per-face T_cam←world estimates should
// agree; their spread is a free calibration-quality metric (the doc's
// "Y_face ばらつき = 校正品質指標"). The final per-camera extrinsic is the
// aggregate across faces.

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "geom/frames.hpp"

namespace fitra::lift {

// One accepted observation: a single marker face seen by one camera at the
// same instant the controller pose was sampled (motion-gated upstream).
//
// The controller poses come from the VR runtime in the VMT/SteamVR world frame
// (Y-up), so the world the solver recovers extrinsics in is VmtWorld — see the
// T_cam_world fields below and the basis change applied when persisting.
struct ExtrinsicSample {
    int                       cam_index = 0;
    int                       face_id   = 0;
    geom::T_cam_marker        T_cam_marker{};        // A_i = T_cam←marker (solvePnP)
    geom::T_world_controller  T_world_controller{};  // B_i = T_world←controller (VR)
};

struct ExtrinsicSolverOptions {
    // OpenCV requires ≥3 per group; the doc wants real rotation diversity, so
    // default higher. Groups below this are skipped (reported, not solved).
    int min_samples_per_group = 6;
    // CALIB_ROBOT_WORLD_HAND_EYE_SHAH (0) or _LI (1).
    int method = 0;
};

// Per (camera, face) hand-eye solution + self-consistency residual.
struct FaceSolution {
    int                       cam_index = 0;
    int                       face_id   = 0;
    int                       n_samples = 0;
    // Z (per-face extrinsic estimate). In VmtWorld — the controller-pose world;
    // the basis change to fitra Z-up is applied only when persisting the YAML.
    geom::T_cam_vmtworld      T_cam_world{};
    geom::T_marker_controller T_marker_controller{};  // X (= Y_face⁻¹)
    // Spread of the controller-pose rotations feeding this group — low values
    // mean poor observability (the doc's "回転多様性" warning).
    double rotation_span_deg = 0.0;
    // RMS of measured A_i vs predicted Z·B_i·X⁻¹ over the group's samples.
    double residual_trans_rms_m   = 0.0;
    double residual_rot_rms_deg   = 0.0;
    bool   solved = false;
};

// Aggregated extrinsic for one camera across all its faces.
struct CameraExtrinsic {
    int                  cam_index = 0;
    // world → camera, in VmtWorld (the controller-pose frame). The persisted
    // calib_io Extrinsics.T_cw is this re-expressed into fitra Z-up via
    // geom::fitra_to_vmt_basis() — they match only after that basis change.
    geom::T_cam_vmtworld T_cam_world{};
    int                  n_faces   = 0;
    int         n_samples = 0;
    // Cross-face disagreement of the T_cam←world estimates (quality metric).
    double face_spread_trans_m   = 0.0;
    double face_spread_rot_deg   = 0.0;
};

struct ExtrinsicSolution {
    std::vector<FaceSolution>    faces;
    std::vector<CameraExtrinsic> cameras;
    bool        ok = false;
    std::string message;
};

// Solve all (camera, face) groups and aggregate per camera. Never throws on
// degenerate input — returns ok=false with a message and whatever partial
// per-face solutions succeeded.
ExtrinsicSolution solve_extrinsics(const std::vector<ExtrinsicSample>& samples,
                                   const ExtrinsicSolverOptions& opts = {});

// --- small SE(3) helpers (shared with the collector / tests) ---------------

// Build T_world←controller (or any pose) from a position + xyzw quaternion.
// The quaternion is normalized internally.
cv::Matx44d pose_from_pos_quat(double x, double y, double z,
                               double qx, double qy, double qz, double qw);

// Geodesic angle (degrees) between the rotation parts of two SE(3) poses.
double rotation_angle_deg(const cv::Matx44d& a, const cv::Matx44d& b);

// Average a set of SE(3) poses: arithmetic mean of translations + chordal
// (sign-aligned quaternion) mean of rotations. Used for burst averaging of
// motion-gated observations. `poses` must be non-empty.
cv::Matx44d average_poses(const std::vector<cv::Matx44d>& poses);

}  // namespace fitra::lift
