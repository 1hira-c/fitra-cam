#pragma once
//
// Floor-AprilTag extrinsic solver: localise each fixed camera against a known
// FloorTagMap by multi-tag PnP. This is the shared "localise against the map"
// core — see docs/design/pose-3d-floor-apriltag-extrinsic.md.
//
// Per camera we gather every visible tag's (2D image corners ↔ known 3D world
// corners) into one correspondence set and run a single solvePnP, yielding
// T_cam←world directly in the fitra Z-up world (no VMT basis change, unlike the
// controller-marker hand-eye path in extrinsic_solver.hpp). Cameras are solved
// independently: they never need to co-observe a tag.
//
// Distinct from ExtrinsicSolution by design — there is no controller, no per-
// face hand-eye "X", and the world is FitraWorld not VmtWorld. Only the final
// T_cam←world → calib_io persistence step is shared.
//
// The map's provenance is irrelevant here: a hand-measured layout (mode a) or a
// future smartphone-SfM reconstruction (mode b) both present the same
// FloorTagMap; this solver localises against whichever.

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "geom/frames.hpp"
#include "lift/floor_tag_map.hpp"

namespace fitra::lift {

// One tag as seen by one camera (image-space corners, aruco order TL,TR,BR,BL),
// already burst-averaged upstream by the collection session.
struct FloorTagObservation {
    int                        id = 0;
    std::array<cv::Point2f, 4> corners{};
};

// All tags one camera saw this session, plus its intrinsics. K/dist are the
// CALIBRATION-resolution intrinsics used for PnP; the recovered T_cam←world is
// resolution-independent (the physical camera pose) and is reused at the
// runtime resolution.
struct FloorCameraInput {
    int                              cam_index = 0;
    cv::Mat                          K;        // 3x3 CV_64F
    cv::Mat                          dist;     // 1xN CV_64F
    bool                             fisheye = false;
    std::vector<FloorTagObservation> obs;
};

struct FloorCameraSolution {
    int               cam_index = 0;
    geom::T_cam_world T_cam_world{};   // world → camera, fitra Z-up (T_cw)
    int               n_tags    = 0;
    int               n_points  = 0;   // = 4 * n_tags
    double            reproj_rms_px = 0.0;
    // True when the observed tag corners are (near-)coplanar — a high camera
    // viewing a flat floor-only layout has poor height/tilt observability.
    // Add off-plane stand tags to resolve.
    bool              planar_degenerate = false;
    double            plane_thickness_m = 0.0;  // std along the min principal axis
    bool              solved = false;
};

struct FloorExtrinsicSolution {
    std::vector<FloorCameraSolution> cameras;
    bool        ok = false;
    std::string message;
};

struct FloorSolverOptions {
    int    min_tags  = 1;     // 1 tag can solve (IPPE) but is fragile
    int    min_points = 8;    // ≥2 tags recommended for a stable pose
    double max_reproj_px = 3.0;             // above this a camera is flagged unsolved
    double planar_warn_thickness_m = 0.02;  // below this → planar_degenerate
};

// Localise every camera against `map`. Never throws on degenerate input —
// returns ok=false with a message and whatever per-camera solutions succeeded.
// ok is true only if every camera solved within max_reproj_px.
FloorExtrinsicSolution solve_floor_extrinsics(
    const std::vector<FloorCameraInput>& cams,
    const FloorTagMap& map,
    const FloorSolverOptions& opts = {});

}  // namespace fitra::lift
