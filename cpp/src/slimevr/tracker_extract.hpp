#pragma once
//
// Extract 10 SlimeVR full-body trackers from a Halpe26 3D skeleton.
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

// Per-foot anchor used by extract_trackers to reconstruct ankle / big_toe via
// short-chain FK when the keypoint goes invalid for a frame. The anchor is
// learned from successful (ankle/toe both valid) frames and held across
// invalid ones — bone length + last-known direction; the FK reconstruction is
// knee + dir · length and ankle + dir · length respectively. Without this,
// foot trackers `valid=false` for the whole frame and the position freeze
// kicks in (which used to be world-absolute and looked like "foot stuck on
// the ground while the body slides past").
struct FootAnchor {
    cv::Vec3f knee_to_ankle_dir{0.0f, 0.0f, -1.0f};
    float     tibia_len_m = 0.0f;
    cv::Vec3f ankle_to_toe_dir{0.0f, 1.0f, 0.0f};
    float     foot_len_m = 0.0f;
    bool      valid = false;
};

// Carries inter-frame state for extract_trackers. Optional: passing nullptr
// disables FK fallback (preserves the old early-return behavior).
struct ExtractContext {
    // [0] = left foot, [1] = right foot.
    std::array<FootAnchor, 2> foot_anchors{};
};

// Extract 10 trackers from a Halpe26 3D skeleton. Degenerate joints (invalid
// landmarks or near-parallel forward/up hints) produce `valid=false` +
// identity quaternion; the publisher skips those.
//
// `kp_format` must be Halpe26 — call sites assert this before invoking.
//
// When `ctx` is non-null, foot trackers attempt a short-chain FK fallback
// using the per-foot anchors before giving up. Successful (ankle+toe both
// real) frames update the anchor; FK-synthesized frames leave it unchanged
// so the next valid frame re-references the last real measurement.
std::array<SlimeTracker, kTrackerCount>
extract_trackers(const infer::Skeleton3D& skel,
                 ExtractContext* ctx = nullptr);

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

// Inter-frame state for the position smoothing path. Pass-by-ref so the
// extractor can hold one of these per loop; default-constructed value is
// safe (the first call falls back to world-absolute hold).
//
// hip_valid + prev_hip_valid together gate the hip-relative hold for
// `valid=false` trackers. When both are true and the tracker held a real
// position last frame, the world-absolute prev is reanchored via
//     offset = prev_pos[i] - prev_hip_pos
//     new world = current_hip_pos + offset
// so a hip translation drags the held tracker along. Without this, the
// extended-leg locomotion case in pose-3d/locomotion-stability sees the
// foot freeze in world coordinates while the body slides past.
//
// dt_s is the publish period (seconds); used by the velocity gate so a
// jitter spike >> human-plausible speed attenuates the EMA alpha instead
// of latching the smoothed history to the outlier.
struct PosSmoothingContext {
    cv::Vec3f current_hip_pos{0.0f, 0.0f, 0.0f};
    bool      hip_valid       = false;
    cv::Vec3f prev_hip_pos{0.0f, 0.0f, 0.0f};
    bool      prev_hip_valid  = false;
    // Per-tracker "last raw curr.pos that we saw valid" + a paired flag.
    // The velocity gate consults this rather than prev_pos so an EMA that
    // has not yet converged (prev still lagging the real position) doesn't
    // read as inflated motion. The flag also doubles as "tracker has been
    // valid at least once" — gates the hip-relative hold on the first
    // invalid frame after init, and prevents the very first valid frame
    // from being misread as a 70 m/s outlier off the (0,0,0) sentinel.
    //
    // `invalid_ticks_since_last_raw` counts publish ticks where the tracker
    // came in invalid (last_raw_pos held). On the recovery frame the
    // velocity gate divides the prev→curr distance by (1 + invalid_ticks)
    // · dt_s so a several-frame dropout doesn't read as a single-tick spike
    // and erroneously gate the recovery into freeze-at-stale-position.
    std::array<cv::Vec3f,    kTrackerCount> last_raw_pos{};
    std::array<bool,         kTrackerCount> has_last_raw{};
    std::array<std::uint32_t, kTrackerCount> invalid_ticks_since_last_raw{};
    float     dt_s = 1.0f / 60.0f;
};

// Per-tracker position exponential moving average.
//   curr_i.pos ← prev_pos_i + α · (curr_i.pos − prev_pos_i)
//   α = base_alpha · (1 − smoothstep(velocity, 8 m/s, 16 m/s))
// base_alpha ∈ [0, 1]. 0 = freeze (prev forever), 1 = no smoothing.
//
// Unlike apply_quat_smoothing, this does NOT modulate the alpha by
// roll_confidence — pos noise (camera triangulation reprojection error /
// keypoint score) is independent of roll observability. Instead the alpha
// is attenuated when the prev→curr displacement exceeds human-plausible
// velocity, so a single bad triangulation frame (5 m jump in one tick)
// does not snap the history to the outlier.
//
// Invalid trackers fall into the hold branch: with a valid hip reference
// (ctx.hip_valid && ctx.prev_hip_valid) the held position re-anchors to
// the current hip (`hip-relative hold`); otherwise it falls back to
// world-absolute hold. The waist tracker has prev_pos ≡ hip_center by
// construction, so hip-relative collapses to "follow hip" — no special
// case needed.
//
// The 1-arg overload preserves the original world-absolute-hold behavior
// for tests and any callers that don't track hip context yet.
//
// Wired into TrackerExtractor::run_loop directly after apply_quat_smoothing
// so the VMT publisher (which sends pos on the wire) and the WebUI viz
// (which renders pos via AxesHelper) both see one shared smoothed history —
// same architectural invariant as the quat path.
void apply_pos_smoothing(std::array<SlimeTracker, kTrackerCount>& curr,
                         std::array<cv::Vec3f, kTrackerCount>& prev_pos,
                         PosSmoothingContext& ctx,
                         float base_alpha);

// Legacy 1-arg form: world-absolute hold, no velocity gate, no hip
// re-anchor. Kept for existing tests that don't pass a context.
void apply_pos_smoothing(std::array<SlimeTracker, kTrackerCount>& curr,
                         std::array<cv::Vec3f, kTrackerCount>& prev_pos,
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
