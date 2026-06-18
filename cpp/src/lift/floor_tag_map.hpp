#pragma once
//
// Floor-AprilTag fiducial map for the marker-less (VR-free) extrinsic path.
//
// See docs/design/pose-3d-floor-apriltag-extrinsic.md (promoted from
// docs/research/floor-apriltag-sfm-map.md). Several AprilTag 36h11 faces are
// placed at KNOWN poses to form a single static map; the map IS the world
// frame. Each fixed camera localises against the map independently (multi-tag
// solvePnP, see floor_extrinsic_solver.hpp) — cameras never need to co-observe.
//
// The map is expressed in the fitra Z-up world (X-right, Y-forward, Z-up) with
// the floor at Z=0, so the recovered T_cam←world is fitra Z-up directly: unlike
// the controller-marker path there is NO VMT basis change when persisting.
//
// World-frame convention (authoring contract): the origin tag's centre is the
// world origin, its in-plane axes the world X/Y, and the floor normal +Z up.
// Floor tags lie on Z=0 (R=I); off-plane stand tags carry an arbitrary 6DoF
// pose to break planar degeneracy (see the solver's planar_degenerate check).
//
// The map is a CONTRACT, not a method: mode (a) authors `T_world_tag` from a
// tape-measure layout (floor_tag_map_load of a hand-written YAML); a future
// mode (b) smartphone-SfM step would emit the same FloorTagMap from corner
// correspondences (tag size_m fixing the metric scale). The localise core
// (floor_extrinsic_solver) does not know which produced the map.

#include <array>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "geom/frames.hpp"

namespace fitra::lift {

// One placed tag: a 36h11 ID, the side length of its black square (the PnP /
// SfM scale), and its pose in the fitra Z-up world.
struct FloorTag {
    int                  id          = 0;
    double               size_m      = 0.0;
    geom::T_world_marker T_world_tag{};
};

struct FloorTagMap {
    std::vector<FloorTag> tags;
    // Informational: which tag defined the origin (default -1 = unspecified /
    // smallest id by convention). Not used by the solver — the poses are
    // already absolute — but recorded so the authoring intent round-trips.
    int origin_tag_id = -1;

    const FloorTag* find(int id) const;

    // The tag's four black-square corners in world (fitra Z-up) coordinates,
    // in aruco order (TL, TR, BR, BL) to match AprilTagDetector::corners. Uses
    // tag_object_corners(size_m) transformed by T_world_tag.
    std::array<geom::Point3<geom::frame::FitraWorld>, 4> world_corners(
        const FloorTag& tag) const;
};

// Load / write the map in OpenCV FileStorage YAML (same family as calib_io).
// Each tag is a map { id, size_m, R(3x3) | rpy_deg(roll,pitch,yaw), t(3) }; the
// writer emits R, the reader accepts either R or rpy_deg. Throws on open
// failure or a malformed entry (missing id/size/translation).
FloorTagMap floor_tag_map_load(const std::string& path);
void        floor_tag_map_write(const std::string& path, const FloorTagMap& map);

// Build a regular grid of coplanar floor tags (all on Z=0, R=I), row-major from
// the origin, IDs first_id, first_id+1, ... Convenience for the common
// "measure a pitch, lay a grid" layout; off-plane stand tags are appended by
// the caller. The grid centre is the world origin.
FloorTagMap floor_tag_grid(int rows, int cols, double pitch_m,
                           int first_id, double size_m);

}  // namespace fitra::lift
