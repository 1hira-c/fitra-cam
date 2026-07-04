#pragma once
//
// Floor-contact grounding for the lifted 3D skeleton (spatial-filtering M-D).
// See docs/design/pose-3d-floor-grounding.md.
//
// The rig's world frame is fitra Z-up with the floor at Z = 0 (the floor
// AprilTag extrinsic convention, floor_tag_map.hpp; the VMT publisher's
// disable_below_floor also treats pos.z < 0 as below the floor). So the floor
// plane is a *constant* Z = floor_z_m in the same coordinates the triangulator
// already produces — no extrinsic needs to be threaded into the lift stage;
// `joint.z` is directly the height above the floor.
//
// Two grounding actions on the foot SOLE points (Halpe26 big-toe / small-toe /
// heel — COCO17 lacks these, so it is a no-op there):
//   (1) below-floor clamp (always, stateless): z < floor_z -> floor_z. Removes
//       the visible floor penetration (heel-sink) that is a 2D RTMPose foot
//       accuracy artifact and cannot be fixed at the 2D stage
//       (project-heel-sink-2d-limitation). A sole point below the floor is
//       physically impossible, so this is unconditionally correct.
//   (2) stance snap (needs prev frame): a near-floor, low-speed sole point
//       (floor_z <= z < floor_z + snap_band_m AND speed < stance_vel_mps) is
//       planted exactly on the floor (Z only) to kill stance foot jitter /
//       micro-bounce. XY is left free (freezing XY would cause visible slip).
//
// Runs as the LAST 3D stage — after Kalman + IK — so no downstream smoother can
// re-sink the foot and the (tiny) post-IK length adjustment is not fed back
// into the Kalman/IK state (output-only, no drift accumulation). The ankle
// joint is intentionally NOT grounded: it is the leg joint (~8 cm above the
// floor in stance) whose length IK maintains; only the sole contact points are.

#include <array>

#include <opencv2/core.hpp>

#include "infer/types.hpp"

namespace fitra::lift {

struct FloorGroundingOptions {
    double floor_z_m       = 0.0;    // world floor plane (fitra Z-up, floor = Z = 0)
    double stance_vel_mps  = 0.15;   // below this raw foot speed a near-floor point is "planted"
    double snap_band_m     = 0.03;   // a point in [floor_z, floor_z + band] & slow snaps to floor_z
};

// Inter-frame state: the previous RAW (pre-grounding) sole-point positions, for
// the stance-speed estimate. Default-constructed is the fresh state; reset() on
// idle/standby resume (mirrors SkeletonKalman::reset()).
struct FloorGroundingState {
    std::array<cv::Vec3f, infer::kMaxKeypoints> prev_pos{};
    std::array<bool,      infer::kMaxKeypoints> has_prev{};
    void reset() { has_prev.fill(false); }
};

// Ground the foot sole points of `skel` to the floor plane in place (see file
// header). `dt_s` is the 3D-update period (for the stance-speed estimate).
// Returns true if any joint was modified. COCO17 (no toe/heel slots) is a
// no-op. Invalid sole joints are skipped and drop their prev anchor.
bool apply_floor_grounding(infer::Skeleton3D& skel,
                           FloorGroundingState& state,
                           double dt_s,
                           const FloorGroundingOptions& opts);

}  // namespace fitra::lift
