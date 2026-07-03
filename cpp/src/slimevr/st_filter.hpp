#pragma once
//
// Spatiotemporal filter core (M-C2) — regime-adaptive smoothing primitives for
// the SlimeVR tracker stage. See docs/design/pose-3d-spatiotemporal-filter.md.
//
// This header is the *core* of the spatiotemporal ("時空") filter: pure,
// self-contained, unit-tested math. It is NOT wired into the pipeline here —
// TrackerExtractor consumes these primitives in M-C3 (position path replaces
// the One Euro EMA; the twist path drives apply_quat_smoothing's twist_alpha).
//
// The design is a two-axis regime switch — "space × time":
//
//   d = |raw target − held output|   (spatial distance): sets the follow
//       strength (strong-filter deadband → normal), and the lag cap.
//   v = |raw curr − raw prev| / dt   (raw speed, m/s or rad/s): sets outlier
//       rejection only. "fast but real (follow)" vs "too fast = glitch
//       (reject)" is decided by v; how strongly to follow is decided by d.
//
// Position step, per tracker, in its working frame (world for the waist,
// waist-relative for limbs — the framing is the caller's M-C3 concern):
//
//   d       = |target − held|
//   v       = |target − last_raw| / dt
//   gate    = 1 − smoothstep(v, v_high, v_reject)          // 1=trust, 0=reject
//   alpha_d = (d<d_core) ? alpha_rest                       // strong + re-center
//           : (d<d_full) ? smoothstep(d,d_core,d_full)·(a_norm−a_rest)+a_rest
//           :              alpha_normal
//   alpha   = rate_adjust(alpha_d, dt, nominal) · gate
//   out     = held + alpha·(target − held)
//   if gate>eps and |target − out| > cap:                  // trusted only
//       out = target − cap · unit(target − held)           // within cap of target
//
// The deadband is a STRONG FILTER with re-centering (small alpha_rest), NOT a
// freeze: micro-jitter is suppressed but the output still converges to the true
// centre, so there is no permanent offset and no snap at the boundary.
//
// Distances are in metres, speeds in m/s, angles in radians — all frame-rate
// independent by construction; alpha is rate-adjusted for wall-clock
// consistency the same way apply_pos_smoothing / apply_quat_smoothing are.

#include <array>
#include <cstdint>

#include <opencv2/core.hpp>

#include "slimevr/tracker_extract.hpp"   // TrackerRole / kTrackerCount

namespace fitra::slimevr {

// One regime axis's knobs. Units are metres + m/s for a position axis, radians
// + rad/s for the twist axis. Invariants (enforced by the ctest, assumed by the
// math): d_core < d_full, v_high < v_reject, 0 ≤ alpha_rest ≤ alpha_normal ≤ 1.
struct StRegime {
    float d_core       = 0.010f;  // deadband inner radius: strong filter below
    float d_full       = 0.030f;  // ramp reaches alpha_normal at this distance
    float alpha_rest   = 0.15f;   // per-step weight inside the deadband (re-centring)
    float alpha_normal = 0.50f;   // per-step weight in the normal regime
    float v_high       = 6.0f;    // outlier gate: fully trusted at/below this raw speed
    float v_reject     = 12.0f;   // outlier gate: fully rejected (held) at/above this
};

// Position parameters for one body-part group: the regime plus the spatial lag
// cap that bounds how far behind a fast target the output is allowed to trail.
struct StPosParams {
    StRegime regime{};
    float    lag_cap_m = 0.10f;   // fast motion: output kept within this of the target
};

// Body-part groups (L/R symmetric). The waist is the world-frame reference
// (slow); the foot is the fastest with the strongest damping. See design
// "部位別パラメータ". Group index is stable (array layout).
enum class StGroup : std::uint8_t {
    Waist = 0,
    Chest,
    UpperArm,
    UpperLeg,
    LowerLeg,
    Foot,
    Count
};
inline constexpr std::size_t kStGroupCount = static_cast<std::size_t>(StGroup::Count);

// Map a tracker role to its parameter group.
StGroup st_group_for(TrackerRole role);

// Per-group parameter table. `roll` / `has_roll` only matter for groups whose
// roll is *inferred* from a limb bend (upper arm / thigh, and the degenerate
// shin) — the design's roll scope. For groups with has_roll=false the twist is
// left on the existing (pinned) path in M-C3.
struct StFilterConfig {
    std::array<StPosParams, kStGroupCount> pos{};
    std::array<StRegime,    kStGroupCount> roll{};
    std::array<bool,        kStGroupCount> has_roll{};
};

// Seeded code defaults (M-C2). These are starting points derived from the
// M-infra baseline and the ~3 cm "discard fine motion" target; the offline
// harness (M-C1) sweeps them into the tuned defaults in M-C4.
const StFilterConfig& default_st_config();

// Convenience accessors (group lookup + table index).
const StPosParams& st_pos_params(const StFilterConfig& cfg, TrackerRole role);
const StRegime&    st_roll_params(const StFilterConfig& cfg, TrackerRole role);
bool               st_has_roll(const StFilterConfig& cfg, TrackerRole role);

// ---- pure scalar core ----------------------------------------------------- #

// Distance→alpha ramp: alpha_rest inside the deadband (d ≤ d_core), smoothstep
// ramp to alpha_normal across [d_core, d_full], alpha_normal beyond. Distance
// domain only (frame-rate independent); the caller applies the rate adjust and
// the velocity gate. C¹-continuous at both knots.
float st_alpha_d(float d, const StRegime& r);

// Velocity gate: 1 (trust) at/below v_high, 0 (reject → hold) at/above
// v_reject, smoothstep between. Same shape as the existing kPosVelGate.
float st_vel_gate(float v, const StRegime& r);

// Frame-rate-independent per-step weight (mirrors tracker_extract's
// rate_adjust_alpha): 1 − (1−base)^(dt/nominal). dt==nominal (or either ≤ 0)
// returns base unchanged.
float st_rate_adjust_alpha(float base_alpha, float dt_s, float nominal_dt_s);

// ---- position step -------------------------------------------------------- #

// One regime step for a 3D point in its working frame. `held` is the previous
// output, `target` the raw measurement, `last_raw` the previous raw (for v).
// Returns the new output. Pure — the caller owns held / last_raw state.
cv::Vec3f st_pos_step(const cv::Vec3f& held,
                      const cv::Vec3f& target,
                      const cv::Vec3f& last_raw,
                      float dt_s, float nominal_dt_s,
                      const StPosParams& p);

// ---- twist (inferred roll) ------------------------------------------------ #

// Signed twist angle (rad, about the local bone-forward +Z) of the relative
// rotation prev⁻¹·curr — the same swing/twist decomposition apply_quat_smoothing
// uses. Positive = right-handed about +Z. Used to form d_roll (held→target) and
// v_roll (raw consecutive / dt) for the twist regime.
float st_twist_angle(const cv::Vec4f& prev_wxyz, const cv::Vec4f& curr_wxyz);

// Regime-driven twist weight for apply_quat_smoothing: the design's
//   twist_alpha = alpha_d(d_roll) · roll_confidence · gate(v_roll)
// with the rate adjust folded in. roll_confidence keeps the existing degenerate
// gate (0 → roll held). Returns a weight in [0, 1]. d_roll / v_roll are
// magnitudes (the sign is irrelevant to the alpha).
float st_twist_alpha(float d_roll, float v_roll, float roll_confidence,
                     float dt_s, float nominal_dt_s, const StRegime& r);

}  // namespace fitra::slimevr
