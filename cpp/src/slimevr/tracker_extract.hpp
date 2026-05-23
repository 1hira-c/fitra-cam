#pragma once
//
// Phase 11: extract 10 SlimeVR full-body trackers from a Halpe26 3D skeleton.
// Output is in the WORLD frame (Z-up, X-right, Y-forward, meters); the
// publisher applies the SlimeVR Y-up coordinate transform when serializing
// (see firmware_protocol::world_quat_to_slime).
//
// 10 tracker layout (matches the SlimeVR TrackerPosition enum exactly):
//   LEFT_UPPER_ARM, RIGHT_UPPER_ARM,
//   CHEST, WAIST,
//   LEFT_UPPER_LEG, RIGHT_UPPER_LEG,        (thigh)
//   LEFT_LOWER_LEG, RIGHT_LOWER_LEG,        (shin / 脛)
//   LEFT_FOOT, RIGHT_FOOT.
// HEAD is intentionally omitted (HMD provides it). Forearms/hands are
// omitted (lower-body FBT is SlimeVR's primary use case; controllers cover
// the arms).
//
// Halpe26-only: this module asserts the active keypoint format. COCO17 lacks
// the neck (18), hip-center (19), heel (24/25), and big-toe (20/21) joints
// needed for stable WAIST/CHEST positions and per-foot yaw.

#include <array>
#include <cstdint>

#include <opencv2/core.hpp>

#include "infer/types.hpp"
#include "slimevr/firmware_protocol.hpp"   // TrackerPosition enum

namespace fitra::slimevr {

enum class TrackerRole : std::uint8_t {
    LeftUpperArm = 0,
    RightUpperArm,
    Chest,
    Waist,
    LeftUpperLeg,
    RightUpperLeg,
    LeftLowerLeg,
    RightLowerLeg,
    LeftFoot,
    RightFoot,
    Count
};

inline constexpr std::size_t kTrackerCount = static_cast<std::size_t>(TrackerRole::Count);

// Stable sensor id 0..9 for the native UDP SensorInfo packet. The enum order
// of TrackerRole IS the sensor id (cast directly).
inline constexpr std::uint8_t sensor_id_for(TrackerRole r) {
    return static_cast<std::uint8_t>(r);
}

// Map TrackerRole → SlimeVR firmware-protocol TrackerPosition. The publisher
// calls this once per sensor when sending SensorInfo packets.
TrackerPosition position_for(TrackerRole role);

struct SlimeTracker {
    TrackerRole role  = TrackerRole::LeftUpperArm;
    // World frame: Z-up, X-right, Y-forward, meters. Position is informational
    // only — Firmware UDP does not transmit per-tracker positions; SlimeVR's
    // skeleton solver reconstructs positions from rotations and avatar bones.
    cv::Vec3f   pos   = {0.0f, 0.0f, 0.0f};
    // wxyz storage; the publisher converts to SlimeVR's xyzw Y-up wire frame.
    cv::Vec4f   quat_wxyz = {1.0f, 0.0f, 0.0f, 0.0f};
    bool        valid = false;
    // [0, 1] per-tracker smoothing weight used by apply_quat_smoothing as
    // effective_alpha = base_alpha · roll_confidence.
    //   * chest / waist / shin: rigid anatomical pin, stays at 1.0.
    //   * upper arm / thigh: dynamic via smoothstep on the up-vector's sin θ
    //     to forward — full at sin θ ≥ kRollSinHigh, zero at sin θ ≤
    //     kRollSinLow (so a degenerate roll measurement holds the previous
    //     quat instead of injecting noise).
    //   * foot: fixed low value kFootSmoothingWeight, not because roll is
    //     uncertain (foot has no roll observation by design — tibia-aligned
    //     up only resolves yaw) but as a strong low-pass against ankle/toe
    //     KP jitter.
    float       roll_confidence = 1.0f;
};

// Extract 10 trackers from a Halpe26 3D skeleton. Degenerate joints (invalid
// landmarks or near-parallel forward/up hints) produce `valid=false` +
// identity quaternion; the publisher skips those.
//
// `kp_format` must be Halpe26 — call sites assert this before invoking.
std::array<SlimeTracker, kTrackerCount>
extract_trackers(const infer::Skeleton3D& skel);

// Per-tracker quaternion exponential smoothing via slerp.
//   effective_alpha_i = base_alpha · curr_i.roll_confidence
//   curr_i ← slerp(prev_i, curr_i, effective_alpha_i)
// base_alpha ∈ [0, 1]. 0 = use prev, 1 = use curr. Confidence < 1 throttles the
// update, so a low-confidence roll measurement decays toward the previous
// orientation instead of injecting noise. Invalid trackers keep prev unchanged
// (curr is replaced by prev so the publisher can still see a stable quat).
// Updates `prev_quat` in place with the smoothed values.
void apply_quat_smoothing(std::array<SlimeTracker, kTrackerCount>& curr,
                          std::array<cv::Vec4f, kTrackerCount>& prev_quat,
                          float base_alpha);

namespace detail {
// Build a (right, up, forward) → wxyz quaternion via Shoemake's matrix-to-quat.
// Inputs need not be unit; routine orthonormalizes. Returns identity + false
// on degenerate input (forward ≈ up).
bool quat_from_forward_up(const cv::Vec3f& forward,
                          const cv::Vec3f& up,
                          cv::Vec4f& out_wxyz);
}  // namespace detail

}  // namespace fitra::slimevr
