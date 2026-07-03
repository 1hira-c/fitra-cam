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

// Where to place the foot tracker's POSITION (rotation is unchanged either
// way: forward = ankle→toe, so the foot's yaw/pitch always tracks the toe).
//   Ankle    : pos = ankle joint. Matches how a SteamVR/VRChat "foot" tracker
//              is perceived (the foot bone sits at the ankle), so VRChat FBT
//              calibration binds it cleanly. Product default.
//   Midpoint : pos = midpoint(ankle, toe). The historical behavior; kept for
//              A/B comparison and as the golden-test default of extract_trackers.
// Only consumers of tracker POSITION are affected (VMT publish + WebUI viz).
// The SlimeVR Firmware UDP path sends rotation only, so it is identical.
enum class FootPosMode : std::uint8_t { Ankle, Midpoint };

struct SlimeTracker {
    TrackerRole role  = TrackerRole::LeftUpperArm;
    // World frame: Z-up, X-right, Y-forward, meters. Position is informational
    // only — Firmware UDP does not transmit per-tracker positions; SlimeVR's
    // skeleton solver reconstructs positions from rotations and avatar bones.
    cv::Vec3f   pos   = {0.0f, 0.0f, 0.0f};
    // wxyz storage; the publisher converts to SlimeVR's xyzw Y-up wire frame.
    cv::Vec4f   quat_wxyz = {1.0f, 0.0f, 0.0f, 0.0f};
    bool        valid = false;
    // [0, 1] per-tracker gate for the TWIST (roll, rotation about the bone's
    // forward axis) component of apply_quat_smoothing: twist_alpha =
    // base_alpha · roll_confidence. The SWING (pitch/yaw, where the bone
    // points) is gated separately by swing_confidence, so a degenerate roll
    // measurement holds only the previous roll while the bone direction keeps
    // tracking.
    //   * chest / waist / shin (non-degenerate): rigid anatomical pin, 1.0.
    //   * upper arm / thigh: dynamic via smoothstep on the up-vector's sin θ
    //     to forward — full at sin θ ≥ kRollSinHigh, zero at sin θ ≤
    //     kRollSinLow. At 0 the roll is held (twist_alpha = 0) but swing still
    //     follows the new forward.
    //   * any roll-bearing bone whose up hint degenerates (e.g. shin with the
    //     leg fully extended): build_tracker forces roll_confidence = 0 with a
    //     forward-only orientation, so roll holds and swing tracks.
    //   * foot: fixed low value kFootSmoothingWeight as a strong low-pass
    //     against ankle/toe KP jitter (matched by swing_confidence so the foot
    //     gets the same overall damping it had before the swing/twist split).
    float       roll_confidence = 1.0f;
    // [0, 1] per-tracker gate for the SWING (pitch/yaw) component:
    // swing_alpha = base_alpha · swing_confidence. 1.0 for every bone except
    // the foot (kFootSmoothingWeight). When swing_confidence == roll_confidence
    // apply_quat_smoothing takes a single-slerp fast path identical to the
    // pre-split behavior (zero regression for rigid bones and the foot).
    float       swing_confidence = 1.0f;
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

// Extract 10 trackers from a Halpe26 3D skeleton. A missing forward hint
// (invalid landmarks) produces `valid=false` + identity quaternion; the
// publisher skips those. A degenerate roll (near-parallel forward/up, e.g. an
// extended limb) instead stays `valid=true` with roll_confidence=0 and a
// forward-only orientation, so smoothing holds the roll but tracks the swing.
//
// `kp_format` must be Halpe26 — call sites assert this before invoking.
//
// When `ctx` is non-null, foot trackers attempt a short-chain FK fallback
// using the per-foot anchors before giving up. Successful (ankle+toe both
// real) frames update the anchor; FK-synthesized frames leave it unchanged
// so the next valid frame re-references the last real measurement.
//
// `foot_pos_mode` selects the foot tracker position (see FootPosMode). The
// function default is Midpoint to preserve existing golden tests; the runtime
// product default (TrackerExtractorOptions::foot_pos_mode) is Ankle.
//
// `chest_height_frac` / `waist_height_frac` slide the Chest and Waist (Hip)
// tracker POSITIONS up the spine: pos = hip_center + frac · (neck − hip_center),
// frac ∈ [0, 1] (0 = hip_center, 1 = neck). Sliding along the spine (not world
// up) keeps the tracker on the torso when the subject leans. Only POSITION is
// affected — orientation (forward/up) is unchanged — so this is a VMT-publish +
// WebUI-viz concern only; the rotation-only SlimeVR Firmware UDP path is
// identical. The function defaults reproduce the historical placement
// (chest = spine midpoint 0.5, waist = hip_center 0.0) to preserve golden
// tests; the runtime product defaults (TrackerExtractorOptions) sit higher so
// the trackers land nearer the sternum / belt line for VRChat FBT.
std::array<SlimeTracker, kTrackerCount>
extract_trackers(const infer::Skeleton3D& skel,
                 ExtractContext* ctx = nullptr,
                 FootPosMode foot_pos_mode = FootPosMode::Midpoint,
                 float chest_height_frac = 0.5f,
                 float waist_height_frac = 0.0f);

// ---- One Euro filter (speed-adaptive low-pass) ----------------------------
//
// The fixed-alpha EMA overloads below cannot be both jitter-free at rest and
// lag-free in motion: a single alpha sets one wall-clock time constant. The
// One Euro filter (Casiez et al. 2012) makes the low-pass cutoff a function of
// the signal speed — low cutoff (strong smoothing) when still, high cutoff
// (responsive) when moving — which is exactly the "still → smooth, moving →
// snappy" behavior we want. See docs/design/vr-output-one-euro-filter.md.
//
//   te      = dt (real step, seconds)
//   dx      = (x − x_prev_filtered) / te
//   dx_hat  = lowpass(dx, alpha(dcutoff, te))         // smooth the speed
//   cutoff  = mincutoff + beta·|dx_hat|               // speed-adaptive cutoff
//   x_hat   = lowpass(x, alpha(cutoff, te))           // the output
//
// `mincutoff` (Hz) sets the at-rest smoothness (lower = smoother / more lag),
// `beta` the motion responsiveness (higher = less lag when moving; beta = 0
// degenerates to a fixed-cutoff, frame-rate-independent EMA — the regression
// fallback), `dcutoff` (Hz) the speed-estimate low-pass (1.0 is the usual
// default). dt is used directly, so the filter is inherently frame-rate
// independent (same role as rate_adjust_alpha for the EMA path).
struct OneEuroParams {
    float mincutoff = 1.0f;  // Hz, cutoff at zero speed (at-rest smoothness)
    float beta      = 0.0f;  // speed coefficient (0 = fixed-cutoff EMA)
    float dcutoff   = 1.0f;  // Hz, cutoff for the speed (derivative) low-pass
};

// First-order low-pass smoothing factor for cutoff `cutoff_hz` at step `dt_s`:
//   tau = 1/(2π·cutoff); alpha = dt / (dt + tau) ∈ [0, 1].
// cutoff_hz <= 0 → 0 (freeze on prev) takes precedence; otherwise dt_s <= 0 → 1
// (degenerate step, trust curr).
float one_euro_alpha(float cutoff_hz, float dt_s);

// Inter-frame state for the rotation One Euro path. ang_vel_hat holds the
// low-passed geodesic angular speed (rad/s) per tracker; `initialized` is false
// until the first valid frame is seen (so the tracker snaps to the first
// measurement rather than slerping up from the identity quaternion).
struct QuatSmoothingContext {
    std::array<float, kTrackerCount> ang_vel_hat{};
    std::array<bool,  kTrackerCount> initialized{};
};

// Per-tracker quaternion exponential smoothing with independent swing/twist
// gates and frame-rate-independent step weighting. First the per-step weight is
// rate-adjusted:
//   alpha_rate = 1 - (1-base_alpha)^(dt_s/nominal_dt_s)   (frame-rate indep.)
// then the relative rotation prev⁻¹·curr is decomposed about the bone's local
// forward axis (+Z) into swing (pitch/yaw) and twist (roll):
//   swing_alpha_i = alpha_rate · curr_i.swing_confidence
//   twist_alpha_i = alpha_rate · curr_i.roll_confidence
//   curr_i ← prev_i · slerp(I, swing, swing_alpha) · slerp(I, twist, twist_alpha)
// When swing_confidence == roll_confidence this collapses to the single slerp
// slerp(prev, curr, alpha_rate·conf) (fast path, bit-identical to the previous
// behavior). roll_confidence = 0 holds the previous roll while swing keeps
// tracking the new forward — this is how a fully-extended limb keeps moving
// without its (unobservable) roll snapping around. base_alpha ∈ [0, 1] is the
// per-step weight tuned at the nominal cadence (nominal_dt_s = 1/extract_rate_hz);
// the dt_s/nominal_dt_s exponent makes the wall-clock time constant independent
// of the actual frame rate, so the event-driven extractor (variable source rate)
// neither over- nor under-smooths vs the fixed-rate path. When dt_s == nominal_dt_s
// (or either is <= 0, the default) alpha_rate == base_alpha, i.e. the fixed-rate
// behavior is unchanged. Invalid trackers keep prev unchanged (curr is replaced
// by prev so the publisher can still see a stable quat). Updates `prev_quat` in
// place with the smoothed values.
// `twist_alpha_override` (optional): when non-null, entry [i] >= 0 replaces the
// per-tracker TWIST weight (ta) for tracker i — the swing weight and everything
// else are unchanged. Entry < 0 (or a null pointer) keeps the built-in ta
// (alpha_rate·roll_confidence). Used by the spatiotemporal filter (M-C3) to
// drive the inferred-roll twist by its regime for the arm/leg groups only;
// null (the default) is byte-identical to the pre-M-C3 behavior.
void apply_quat_smoothing(std::array<SlimeTracker, kTrackerCount>& curr,
                          std::array<cv::Vec4f, kTrackerCount>& prev_quat,
                          float base_alpha,
                          float dt_s = 0.0f, float nominal_dt_s = 0.0f,
                          const std::array<float, kTrackerCount>* twist_alpha_override = nullptr);

// One Euro overload: identical swing/twist + parent-yaw-transport machinery as
// the fixed-alpha form, but the per-step base weight is the speed-adaptive
// One Euro alpha (derived from each tracker's geodesic angular speed) instead
// of a single rate-adjusted base_alpha. `ctx` carries the per-tracker speed
// estimate across frames. dt_s is the real step; nominal_dt_s is still used by
// the (alpha-independent) parent-yaw transport gate. Held (invalid) trackers
// keep prev and their ctx state untouched, same as the fixed-alpha form.
void apply_quat_smoothing(std::array<SlimeTracker, kTrackerCount>& curr,
                          std::array<cv::Vec4f, kTrackerCount>& prev_quat,
                          QuatSmoothingContext& ctx,
                          const OneEuroParams& params,
                          float dt_s, float nominal_dt_s,
                          const std::array<float, kTrackerCount>* twist_alpha_override = nullptr);

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
    // Per-tracker, per-axis low-passed velocity (m/s) for the One Euro path
    // (apply_pos_smoothing's OneEuroParams overload). Holds the filtered
    // derivative `dx_hat` whose magnitude opens the per-axis cutoff. Unused by
    // the fixed-alpha EMA overload. Reset to (0,0,0) on the first valid frame
    // for each tracker (snap, no convergence-from-origin transient).
    std::array<cv::Vec3f,    kTrackerCount> pos_dx_hat{};
};

// Per-tracker position exponential moving average.
//   alpha_rate = 1 - (1-base_alpha)^(dt_s/nominal_dt_s)   (frame-rate independent)
//   α = alpha_rate · (1 − smoothstep(velocity, 8 m/s, 16 m/s))
//   curr_i.pos ← prev_pos_i + α · (curr_i.pos − prev_pos_i)
// base_alpha ∈ [0, 1]. 0 = freeze (prev forever), 1 = no smoothing. dt_s /
// nominal_dt_s make the time constant rate-independent (see apply_quat_smoothing);
// the ctx overload reads dt_s from ctx.dt_s and takes nominal_dt_s as an arg,
// while defaults (<=0) reduce to plain base_alpha for the fixed-rate path.
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
                         float base_alpha,
                         float nominal_dt_s = 0.0f);

// One Euro overload: same hip-relative hold and outlier velocity gate as the
// fixed-alpha ctx form, but the per-axis EMA weight is the speed-adaptive
// One Euro alpha (each of x/y/z filtered independently) instead of a single
// rate-adjusted base_alpha. The outlier gate still multiplies the result so a
// triangulation glitch (>16 m/s) freezes rather than being chased by the
// speed-opened cutoff. ctx.pos_dx_hat carries the per-axis speed estimate;
// the first valid frame per tracker snaps (prev ← curr, dx_hat ← 0). nominal
// is accepted for signature parity but unused (One Euro reads ctx.dt_s).
void apply_pos_smoothing(std::array<SlimeTracker, kTrackerCount>& curr,
                         std::array<cv::Vec3f, kTrackerCount>& prev_pos,
                         PosSmoothingContext& ctx,
                         const OneEuroParams& params,
                         float nominal_dt_s = 0.0f);

// World-absolute hold form: no hip re-anchor, no velocity gate. Frame-rate
// independent via dt_s/nominal_dt_s (defaults <=0 reduce to plain base_alpha).
// Kept for existing tests and callers that don't track hip context.
void apply_pos_smoothing(std::array<SlimeTracker, kTrackerCount>& curr,
                         std::array<cv::Vec3f, kTrackerCount>& prev_pos,
                         float base_alpha,
                         float dt_s = 0.0f, float nominal_dt_s = 0.0f);

namespace detail {
// Build a (right, up, forward) → wxyz quaternion via Shoemake's matrix-to-quat.
// Inputs need not be unit; routine orthonormalizes. Returns identity + false
// on degenerate input (forward ≈ up).
bool quat_from_forward_up(const cv::Vec3f& forward,
                          const cv::Vec3f& up,
                          cv::Vec4f& out_wxyz);
}  // namespace detail

}  // namespace fitra::slimevr
