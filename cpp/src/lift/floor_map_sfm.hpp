#pragma once
//
// Floor-AprilTag map reconstruction from multi-frame tag observations — the
// "mode (b) smartphone-SfM" producer named in floor_tag_map.hpp. A single
// moving camera (e.g. a phone) walks the floor; per frame we run per-tag PnP
// (the known tag size fixes the metric scale) and collect each tag's
// T_cam←tag. This module chains those observations across frames into one
// consistent FloorTagMap WITHOUT any tape-measure layout: relative poses
// between co-visible tags accumulate over frames, an anchor tag defines the
// world origin, and the connected co-visibility graph is walked to place every
// tag. The result is the same FloorTagMap the floor extrinsic solver localises
// against — its provenance (hand-measured mode (a) or this) is irrelevant there.
//
// See docs/design/pose-3d-smartphone-sfm-marker-map.md. Pure geometry, no image
// I/O — the AprilTag detection front-end lives in tools/sfm_floor_map.cpp; this
// is unit-tested against synthetic observations.

#include <array>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "geom/frames.hpp"
#include "lift/floor_tag_map.hpp"

namespace fitra::lift {

// One tag decoded in one frame: its id, the per-tag PnP pose (tag→camera), the
// PnP reprojection error (used to gate noisy observations), and the raw image
// corners (kept so the same observation list can drive holdout validation
// through solve_floor_extrinsics without re-detecting).
struct SfmTagObs {
    int                        id            = 0;
    geom::T_cam_marker         T_cam_tag{};
    double                     reproj_rms_px = 0.0;
    std::array<cv::Point2f, 4> corners{};
};

// All tags decoded in a single video frame.
struct SfmFrame {
    std::vector<SfmTagObs> tags;
};

struct SfmMapOptions {
    double tag_size_m      = 0.1145;  // physical black-square side of every tag
    int    anchor_tag_id   = -1;      // -1 → smallest observed id = world origin
    int    min_pair_views  = 1;       // min co-views for a trusted graph edge
    double max_pose_rms_px = 3.0;     // drop per-tag obs above this before graphing
    bool   fit_floor_plane = true;    // re-gauge to FitraWorld (floor z=0, z-up)
    bool   rotation_refine = true;    // extra pose-averaging relaxation pass
};

struct SfmMapReport {
    bool             connected = false;  // every observed tag reachable from anchor
    int              anchor_id = -1;
    int              n_tags    = 0;      // tags placed in the map
    int              n_edges   = 0;      // co-visibility edges with ≥min_pair_views
    std::vector<int> unreached_ids;      // tags seen but not connected to the anchor
    double           floor_plane_rms_m = 0.0;  // std of tag centres along plane normal
    std::string      message;
};

// Build a FloorTagMap from multi-frame per-tag observations by chaining relative
// poses across the co-visibility graph (anchor tag = world origin). Never throws;
// returns true iff every observed tag was placed (report.connected). On a split
// graph it still fills `out_map` with the reachable component and lists the rest
// in report.unreached_ids.
bool build_floor_map_sfm(const std::vector<SfmFrame>& frames,
                         const SfmMapOptions& opts,
                         FloorTagMap& out_map,
                         SfmMapReport& report);

}  // namespace fitra::lift
