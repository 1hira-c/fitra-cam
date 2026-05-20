#pragma once
//
// Phase 11: extract 8 SlimeVR full-body trackers from a Halpe26 3D skeleton
// and convert to VMC's Unity-left-handed Y-up coordinate frame.
//
// 8 tracker layout (matches SteamVR FBT and VRChat OSC trackers ≤ 8):
//   WAIST, CHEST, LEFT_KNEE, RIGHT_KNEE, LEFT_ELBOW, RIGHT_ELBOW,
//   LEFT_FOOT, RIGHT_FOOT.
// HEAD is intentionally omitted: SlimeVR expects the HMD to provide it.
//
// Halpe26-only: this module asserts the active keypoint format. COCO17 lacks
// the neck (18), hip-center (19), heel (24/25), and big-toe (20/21) joints
// needed for stable WAIST / CHEST positions and per-foot yaw.

#include <array>
#include <cstdint>
#include <string_view>

#include <opencv2/core.hpp>

#include "infer/types.hpp"

namespace fitra::slimevr {

enum class TrackerRole : std::uint8_t {
    Waist = 0,
    Chest,
    LeftKnee,
    RightKnee,
    LeftElbow,
    RightElbow,
    LeftFoot,
    RightFoot,
    Count
};

inline constexpr std::size_t kTrackerCount = static_cast<std::size_t>(TrackerRole::Count);

// VMC OSC role names. The strings are sent verbatim as the first argument of
// `/VMC/Ext/Tra/Pos`. SlimeVR's VMCHandler.kt matches these against Unity's
// HumanBodyBones camelCase names (`leftFoot`, `rightFoot`, ...). Case matters.
// Keep in TrackerRole enum order.
inline constexpr std::array<std::string_view, kTrackerCount> kRoleNames{{
    "waist",      "chest",
    "leftKnee",   "rightKnee",
    "leftElbow",  "rightElbow",
    "leftFoot",   "rightFoot",
}};

struct VmcTracker {
    TrackerRole       role  = TrackerRole::Waist;
    cv::Vec3f         pos   = {0.0f, 0.0f, 0.0f};   // already in VMC frame (Y-up)
    // wxyz storage for stable arithmetic; the publisher writes xyzw on the wire.
    cv::Vec4f         quat_wxyz = {1.0f, 0.0f, 0.0f, 0.0f};
    bool              valid = false;
};

// Extract 8 trackers from a Halpe26 3D skeleton. Joints come from our world
// frame (Z-up, X-right, Y-forward, meters, floor=Z0). Output `pos` and
// `quat_wxyz` are converted to VMC = Unity left-handed (Y-up, Z-forward):
//
//   pos_vmc  = ( px, pz, -py )
//   quat_vmc = ( qx_unity, qy_unity, qz_unity, qw_unity )
//            = ( qx, qz, -qy, -qw )
//
// Degenerate joints (invalid landmarks or near-parallel forward/up hints)
// produce `valid=false` + identity quaternion; the publisher skips those.
//
// `kp_format` must be Halpe26 — the caller asserts this before invoking.
std::array<VmcTracker, kTrackerCount>
extract_vmc_trackers(const infer::Skeleton3D& skel);

// Per-tracker quaternion exponential smoothing via slerp.
//   curr_i ← slerp(prev_i, curr_i, alpha)
// alpha ∈ [0, 1]. 0 = use prev, 1 = use curr. Updates `prev_quat` in place
// with the smoothed values. Invalid trackers reset their prev slot to the
// raw (typically identity) quat so a tracker recovering visibility starts
// clean rather than blending against a stale orientation.
void apply_quat_smoothing(std::array<VmcTracker, kTrackerCount>& curr,
                          std::array<cv::Vec4f, kTrackerCount>& prev_quat,
                          float alpha);

// Coordinate transforms exposed for unit testing.
namespace detail {
cv::Vec3f world_pos_to_vmc(const cv::Vec3f& world_xyz);
cv::Vec4f world_quat_to_vmc(const cv::Vec4f& world_quat_wxyz);
// Build a (right, up, forward) → wxyz quaternion via Shoemake's matrix-to-quat.
// Inputs need not be unit; routine orthonormalizes. Returns identity + false
// on degenerate input (forward ≈ up).
bool quat_from_forward_up(const cv::Vec3f& forward,
                          const cv::Vec3f& up,
                          cv::Vec4f& out_wxyz);
}  // namespace detail

}  // namespace fitra::slimevr
