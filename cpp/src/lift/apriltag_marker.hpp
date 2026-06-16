#pragma once
//
// Multi-face AprilTag marker detection + per-face PnP, producing the
// T_cam←face observations the extrinsic hand-eye solver consumes.
//
// See docs/design/pose-3d-controller-marker-extrinsic.md. A rigid mount on the
// VR controller carries several AprilTag 36h11 faces (each a distinct ID).
// Held near a camera, at least one face faces it; we decode that face and run
// solvePnP against the known square-tag geometry to get T_cam←face. The
// per-face mount offset relative to the controller is NOT measured here — it is
// recovered as the hand-eye X term (see extrinsic_solver.hpp).
//
// AprilTag is preferred over plain ArUco for its low-resolution / long-range
// decode robustness; near-field use here makes decoding comfortable.

#include <array>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include "geom/frames.hpp"

namespace fitra::lift {

// One configured marker face: an AprilTag 36h11 ID and the physical side length
// of its black square (the PnP scale). Faces may differ in size.
struct MarkerFace {
    int    face_id    = 0;
    double tag_size_m = 0.10;
};

struct MarkerBoardConfig {
    // cv::aruco predefined-dictionary id. Default DICT_APRILTAG_36h11.
    int dictionary = -1;            // -1 → resolved to DICT_APRILTAG_36h11 in ctor
    std::vector<MarkerFace> faces;

    // Optional CLAHE (contrast-limited adaptive histogram equalisation) applied
    // to the grayscale image before detection. The floor-AprilTag feasibility
    // test (2026-06-13, docs/research/floor-apriltag-sfm-map.md) found soft-focus
    // fisheye lenses lose low-contrast tags that CLAHE(2.0,(8,8)) recovers; it
    // also helps the controller-marker path. Off by default to preserve the
    // exact prior decode behaviour.
    bool   use_clahe  = false;
    double clahe_clip = 2.0;
    int    clahe_grid = 8;          // (clahe_grid × clahe_grid) tiles

    const MarkerFace* find(int id) const;
};

// A decoded face with its recovered pose.
struct TagDetection {
    int                        face_id = 0;
    std::array<cv::Point2f, 4> corners{};      // image px, aruco order (TL,TR,BR,BL)
    geom::T_cam_marker         T_cam_face{};    // object(face) → camera (from PnP)
    double                     reproj_rms_px = 0.0;
    bool                       pose_ok = false;
};

class AprilTagDetector {
public:
    explicit AprilTagDetector(MarkerBoardConfig cfg);

    // Detect configured faces in `image` (BGR or grayscale) using the given
    // intrinsics (K 3x3, dist 1xN, both CV_64F). Returns one entry per decoded
    // face whose ID is in the config; faces not in the config are ignored.
    //
    // Not thread-safe: the internal cv::aruco::ArucoDetector carries state and
    // must not be called from multiple threads concurrently. Call sites that
    // hand frames in serially (the calibration session's frame tap) are fine.
    //
    // `fisheye` selects the per-face PnP distortion model and MUST match the
    // (K,dist) pair: it is per-camera (not a board-wide setting), so a mixed
    // pinhole+fisheye rig passes the model of the camera that produced `image`.
    std::vector<TagDetection> detect(const cv::Mat& image,
                                     const cv::Mat& K,
                                     const cv::Mat& dist,
                                     bool fisheye = false);

    const MarkerBoardConfig& config() const { return cfg_; }

private:
    MarkerBoardConfig         cfg_;
    cv::aruco::ArucoDetector  detector_;  // built once in the ctor
};

// Pure PnP for a single square tag. `corners` are the four image-space corners
// in aruco order (TL, TR, BR, BL); `tag_size_m` is the black-square side. On
// success fills `T_cam_face` (object→camera) and the reprojection RMS (px).
// Uses SOLVEPNP_IPPE_SQUARE (planar-square specialised). Returns false if PnP
// fails. Pure function — unit-tested without any image.
bool solve_tag_pose(const std::array<cv::Point2f, 4>& corners,
                    double tag_size_m,
                    const cv::Mat& K,
                    const cv::Mat& dist,
                    geom::T_cam_marker& T_cam_face,
                    double& reproj_rms_px,
                    bool fisheye = false);

// Object-space corners of a square tag (side `s`) centred at the face origin,
// Z=0 plane, matching aruco corner order: TL(-,+), TR(+,+), BR(+,-), BL(-,-).
std::array<cv::Point3f, 4> tag_object_corners(double s);

}  // namespace fitra::lift
