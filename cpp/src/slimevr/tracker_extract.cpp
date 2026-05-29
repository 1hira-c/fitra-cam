#include "slimevr/tracker_extract.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "lift/keypoint_format.hpp"

namespace fitra::slimevr {

namespace {

inline cv::Vec3f to_vec3f(const infer::Joint3D& j) {
    return cv::Vec3f{j.x, j.y, j.z};
}

inline float norm(const cv::Vec3f& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

inline cv::Vec3f cross(const cv::Vec3f& a, const cv::Vec3f& b) {
    return cv::Vec3f{
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };
}

inline cv::Vec3f normalize_or_zero(const cv::Vec3f& v) {
    float n = norm(v);
    if (n < 1.0e-6f) return cv::Vec3f{0, 0, 0};
    return cv::Vec3f{v[0] / n, v[1] / n, v[2] / n};
}

// Halpe26 index symbols used by all role builders.
constexpr std::size_t kLShoulder = 5,  kRShoulder = 6;
constexpr std::size_t kLElbow    = 7,  kRElbow    = 8;
constexpr std::size_t kLWrist    = 9,  kRWrist    = 10;
constexpr std::size_t kLHip      = 11, kRHip      = 12;
constexpr std::size_t kLKnee     = 13, kRKnee     = 14;
constexpr std::size_t kLAnkle    = 15, kRAnkle    = 16;
constexpr std::size_t kNeck      = 18;
constexpr std::size_t kHipCenter = 19;
constexpr std::size_t kLBigToe   = 20, kRBigToe   = 21;

// Confidence smoothstep bounds on sin(angle(up, fwd)) for upper arm / thigh
// roll. Below kRollSinLow the chosen up is treated as degenerate (confidence 0,
// previous roll held). Above kRollSinHigh the measurement is fully trusted
// (confidence 1). Between, smoothstep gradually opens the gate so the update
// rate scales with measurement reliability. See docs/phase12-slimevr-bridge-relay.md.
//
// しきい:
//   * sin 0.15 ≈ 8.6° bend: degenerate gate。歩行 (>30°) / しゃがみ (>60°) /
//     着座 (~90°) は確実に超え、立位伸展 (0-5°) は確実に下回る
//   * sin 0.30 ≈ 17.5° bend: full-confidence ceiling。中間域 (8.6°..17.5°)
//     だけが smoothstep の漸進更新ゾーンになり、実用ポーズ域からは離れる
// `quat_from_forward_up` の degeneracy 判定も同じ kRollSinLow を sin θ gate
// として使うので、pick_up_multistage を経由しない rigid tracker
// (shin / foot / chest / waist) も同等に守られる。
constexpr float kRollSinLow  = 0.15f;  // sin 8.6°: degenerate gate
constexpr float kRollSinHigh = 0.30f;  // sin 17.5°: full-confidence ceiling

// Smoothing throttle for foot trackers. The foot is treated as a rigid
// extension of the shin (up = tibia axis; see foot_tracker below) because
// the heel KP is too noisy in our 2D→3D pipeline. Even after dropping heel
// the ankle/toe pair retains residual jitter, so the per-tracker smoothing
// weight is set low to act as a strong low-pass via apply_quat_smoothing's
// effective_alpha = base_alpha · roll_confidence.
//   base_alpha 0.5 × 0.3 = 0.15 → τ ≈ 6 frames at 60 Hz
constexpr float kFootSmoothingWeight = 0.3f;

// When the foot tracker falls back to FK (ankle or toe synthesized from the
// anchor instead of measured), drop the smoothing weight further so the
// quaternion EMA leans even more on prev_quat. The synthesized direction is
// only as accurate as the last real frame's anchor, so we want to bias
// strongly toward continuity until a real KP returns.
constexpr float kFootFkSmoothingWeight = 0.15f;

// Position-EMA velocity gate (m/s). Inputs are the magnitude of the prev →
// curr displacement divided by dt_s. The gate widens via smoothstep so a
// 5 m jitter spike across a 16 ms tick (≈ 300 m/s, far above sprint top
// speed ~12 m/s) collapses the effective alpha to 0; nominal motion
// (walking ~1.5 m/s, vigorous gestures ~5 m/s) passes through unattenuated.
constexpr float kPosVelGateLow_mps  = 8.0f;
constexpr float kPosVelGateHigh_mps = 16.0f;

// Pelvis-yaw transport gate (rad/s). apply_quat_smoothing rides a held roll
// (extended limb, roll_confidence → 0) along with the *change* in the waist
// tracker's orientation so a body yaw still turns the limb. A wild pelvis delta
// — triangulation glitch, or hip_axis collapsing onto the camera depth axis
// when the subject is side-on — would inject bogus roll, so the transport is
// attenuated toward identity as the per-second pelvis turn rate climbs. 8 rad/s
// ≈ 458°/s is past a brisk human turn; beyond 16 rad/s it is certainly a glitch
// and the transport is dropped (held roll falls back to world-absolute hold).
constexpr float kPelvisYawGateLow_rps  = 8.0f;
constexpr float kPelvisYawGateHigh_rps = 16.0f;

bool joints_valid(const infer::Skeleton3D& s, std::initializer_list<std::size_t> idxs) {
    for (auto i : idxs) {
        if (!s.joints[i].valid) return false;
    }
    return true;
}

// Hermite smoothstep: 0 at x ≤ low, 1 at x ≥ high, C¹-continuous in between.
inline float smoothstep01(float x, float low, float high) {
    if (x <= low)  return 0.0f;
    if (x >= high) return 1.0f;
    float t = (x - low) / (high - low);
    return t * t * (3.0f - 2.0f * t);
}

// ---- Quaternion (wxyz) helpers for apply_quat_smoothing -------------------
inline cv::Vec4f quat_normalize(const cv::Vec4f& q) {
    float n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1.0e-9f) return cv::Vec4f{1, 0, 0, 0};
    return cv::Vec4f{q[0]/n, q[1]/n, q[2]/n, q[3]/n};
}

inline cv::Vec4f quat_conj(const cv::Vec4f& q) {
    return cv::Vec4f{q[0], -q[1], -q[2], -q[3]};
}

// Hamilton product a·b (wxyz).
inline cv::Vec4f quat_mul(const cv::Vec4f& a, const cv::Vec4f& b) {
    return cv::Vec4f{
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]};
}

// slerp from p to q with parameter t ∈ [0, 1]. Handles the q · -q double cover
// (shorter arc) and falls back to normalized lerp for near-aligned inputs.
cv::Vec4f quat_slerp(const cv::Vec4f& p, cv::Vec4f q, float t) {
    float d = p[0]*q[0] + p[1]*q[1] + p[2]*q[2] + p[3]*q[3];
    if (d < 0.0f) { q = cv::Vec4f{-q[0], -q[1], -q[2], -q[3]}; d = -d; }
    cv::Vec4f r;
    if (d > 0.9995f) {
        r = cv::Vec4f{p[0] + t * (q[0] - p[0]),
                      p[1] + t * (q[1] - p[1]),
                      p[2] + t * (q[2] - p[2]),
                      p[3] + t * (q[3] - p[3])};
    } else {
        float theta = std::acos(std::clamp(d, -1.0f, 1.0f));
        float sin_t = std::sin(theta);
        float wa = std::sin((1.0f - t) * theta) / sin_t;
        float wb = std::sin(t * theta) / sin_t;
        r = cv::Vec4f{wa*p[0] + wb*q[0], wa*p[1] + wb*q[1],
                      wa*p[2] + wb*q[2], wa*p[3] + wb*q[3]};
    }
    return quat_normalize(r);
}

inline cv::Vec4f quat_identity() { return cv::Vec4f{1, 0, 0, 0}; }

// Swing/twist decomposition of `delta` about the local +Z axis (the bone
// forward in quat_from_forward_up's [right|up|fwd] basis): delta = swing·twist
// where twist is a rotation about Z (roll) and swing carries Z elsewhere
// (pitch/yaw). Caller passes a w ≥ 0 (canonicalized) delta. Degenerates to
// twist = identity for a ~180° swing about an axis in the XY plane.
void swing_twist_about_z(const cv::Vec4f& delta, cv::Vec4f& swing, cv::Vec4f& twist) {
    cv::Vec4f t_raw{delta[0], 0.0f, 0.0f, delta[3]};  // project onto z-axis
    float n = std::sqrt(t_raw[0]*t_raw[0] + t_raw[3]*t_raw[3]);
    if (n < 1.0e-6f) {
        twist = quat_identity();
        swing = delta;
        return;
    }
    twist = cv::Vec4f{t_raw[0]/n, 0.0f, 0.0f, t_raw[3]/n};
    swing = quat_mul(delta, quat_conj(twist));  // swing = delta · twist⁻¹
}

// Pick the first up hint whose sin θ to forward is above kRollSinLow (i.e. not
// near-degenerate); compute confidence as smoothstep(sin θ, kRollSinLow,
// kRollSinHigh). Zero-vector ups (from invalid joints, or a deliberately
// passed sentinel) skip immediately. The returned up vector still needs
// orthogonalization by quat_from_forward_up.
// `which` is 0/1/2 for the stage that won (primary/secondary/tertiary).
//
// Tertiary semantics: when reached as a fallthrough with a non-zero vector,
// it is ACCEPTED unconditionally (no kRollSinLow gate) and confidence comes
// from its own sin θ. This works well for upper_arm where world Z is a sane
// best-effort up. For trackers where world Z would write a physically wrong
// roll at high confidence (e.g. thigh — see upper_leg), callers must pass
// the zero vector as tertiary: the for-loop skips it and the function falls
// through to the bottom, returning a zero up with out_confidence=0, which
// drives quat_from_forward_up to valid=false and freezes smoothing.
cv::Vec3f pick_up_multistage(const cv::Vec3f& fwd,
                              const cv::Vec3f& up_primary,
                              const cv::Vec3f& up_secondary,
                              const cv::Vec3f& up_tertiary,
                              float& out_confidence,
                              int& which) {
    float f_n = norm(fwd);
    if (f_n < 1.0e-6f) {
        out_confidence = 0.0f;
        which = 2;
        return up_tertiary;
    }
    const cv::Vec3f* candidates[3] = {&up_primary, &up_secondary, &up_tertiary};
    for (int i = 0; i < 3; ++i) {
        const cv::Vec3f& u = *candidates[i];
        float u_n = norm(u);
        if (u_n < 1.0e-6f) continue;  // zero vector → invalid joint, skip
        float sin_theta = norm(cross(u, fwd)) / (u_n * f_n);
        // Tertiary (world Z) is always accepted as last resort but its
        // confidence is still computed from its own sin θ (0 when fwd ∥ Z).
        if (i == 2 || sin_theta >= kRollSinLow) {
            out_confidence = smoothstep01(sin_theta, kRollSinLow, kRollSinHigh);
            which = i;
            return u;
        }
    }
    out_confidence = 0.0f;
    which = 2;
    return up_tertiary;
}

}  // namespace

TrackerPosition position_for(TrackerRole role) {
    switch (role) {
        case TrackerRole::LeftUpperArm:  return TrackerPosition::LeftUpperArm;
        case TrackerRole::RightUpperArm: return TrackerPosition::RightUpperArm;
        case TrackerRole::Chest:         return TrackerPosition::Chest;
        // The pelvis tracker (pos = hip_center) is anatomically a Hip, not a
        // Waist. SlimeVR Server auto-assigns it correctly only as Hip in the
        // 10-tracker firmware UDP configuration.
        case TrackerRole::Waist:         return TrackerPosition::Hip;
        case TrackerRole::LeftUpperLeg:  return TrackerPosition::LeftUpperLeg;
        case TrackerRole::RightUpperLeg: return TrackerPosition::RightUpperLeg;
        case TrackerRole::LeftLowerLeg:  return TrackerPosition::LeftLowerLeg;
        case TrackerRole::RightLowerLeg: return TrackerPosition::RightLowerLeg;
        case TrackerRole::LeftFoot:      return TrackerPosition::LeftFoot;
        case TrackerRole::RightFoot:     return TrackerPosition::RightFoot;
        case TrackerRole::Count:         break;
    }
    return TrackerPosition::None;
}

namespace detail {

bool quat_from_forward_up(const cv::Vec3f& forward_raw,
                          const cv::Vec3f& up_raw,
                          cv::Vec4f& out_wxyz) {
    // Build a right-handed orthonormal basis (right, up, forward) from
    // forward/up hints, then convert to a wxyz quaternion via Shoemake.
    cv::Vec3f fwd = normalize_or_zero(forward_raw);
    if (norm(fwd) < 0.5f) {           // forward hint degenerate
        out_wxyz = cv::Vec4f{1, 0, 0, 0};
        return false;
    }
    cv::Vec3f right_raw = cross(up_raw, fwd);
    // sin θ-based degeneracy gate so rigid trackers (shin, foot, chest,
    // waist) that don't go through pick_up_multistage still get the same
    // near-parallel protection as the thigh / upper-arm path.
    //   sin θ = |cross(up, fwd)| / (|up| · |fwd|)
    // fwd is already unit-normalized above (norm > 0.5 + normalize_or_zero
    // gave it length ~1.0). So we just need |up|.
    float up_norm    = norm(up_raw);
    float right_norm = norm(right_raw);
    if (up_norm < 1.0e-6f || right_norm < kRollSinLow * up_norm) {
        // Zero up_raw (invalid joint / sentinel) OR up is within
        // sin⁻¹(kRollSinLow) of fwd: declare degenerate and let the caller
        // (apply_quat_smoothing) hold the previous quat.
        out_wxyz = cv::Vec4f{1, 0, 0, 0};
        return false;
    }
    cv::Vec3f right = normalize_or_zero(right_raw);
    cv::Vec3f up    = normalize_or_zero(cross(fwd, right));

    // Column vectors of the rotation matrix: [right | up | fwd].
    float m00 = right[0], m01 = up[0], m02 = fwd[0];
    float m10 = right[1], m11 = up[1], m12 = fwd[1];
    float m20 = right[2], m21 = up[2], m22 = fwd[2];

    float trace = m00 + m11 + m22;
    float qw, qx, qy, qz;
    if (trace > 0.0f) {
        float s = std::sqrt(trace + 1.0f) * 2.0f;
        qw = 0.25f * s;
        qx = (m21 - m12) / s;
        qy = (m02 - m20) / s;
        qz = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        qw = (m21 - m12) / s;
        qx = 0.25f * s;
        qy = (m01 + m10) / s;
        qz = (m02 + m20) / s;
    } else if (m11 > m22) {
        float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        qw = (m02 - m20) / s;
        qx = (m01 + m10) / s;
        qy = 0.25f * s;
        qz = (m12 + m21) / s;
    } else {
        float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        qw = (m10 - m01) / s;
        qx = (m02 + m20) / s;
        qy = (m12 + m21) / s;
        qz = 0.25f * s;
    }
    float n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (n < 1.0e-9f) { out_wxyz = cv::Vec4f{1, 0, 0, 0}; return false; }
    out_wxyz = cv::Vec4f{qw / n, qx / n, qy / n, qz / n};
    return true;
}

}  // namespace detail

namespace {

// Pick a world axis that is safely non-parallel to `fwd`, to seed a
// forward-only orientation when the real up hint degenerates. The resulting
// roll is arbitrary, but apply_quat_smoothing's twist gate (roll_confidence=0)
// discards it and holds the previous roll, so only the axis choice's validity
// matters — never its roll value.
cv::Vec3f fallback_up_for(const cv::Vec3f& fwd) {
    cv::Vec3f f = normalize_or_zero(fwd);
    // World Z unless fwd is too close to it; then world Y.
    if (std::abs(f[2]) < 0.9f) return cv::Vec3f{0, 0, 1};
    return cv::Vec3f{0, 1, 0};
}

// Try to build a SlimeTracker for one role from a position joint, a forward
// hint and an up hint. Updates `out` in place.
//   * forward degenerate (no bone direction)      → valid=false (publisher skips)
//   * forward valid, up valid                      → valid=true, roll_confidence as passed
//   * forward valid, up degenerate (extended limb) → valid=true, roll_confidence=0,
//        forward-only orientation (roll held by smoothing, swing keeps tracking)
// `roll_confidence` defaults to 1.0 for rigid-pin bones (chest/waist/shin);
// upper arm / thigh pass their smoothstep-derived value. `swing_confidence`
// defaults to 1.0; the foot passes kFootSmoothingWeight for an overall low-pass.
bool build_tracker(TrackerRole role,
                   const cv::Vec3f& world_pos,
                   const cv::Vec3f& forward,
                   const cv::Vec3f& up,
                   SlimeTracker& out,
                   float roll_confidence = 1.0f,
                   float swing_confidence = 1.0f) {
    out.role = role;
    out.pos  = world_pos;
    out.swing_confidence = swing_confidence;
    cv::Vec4f q_wxyz;
    if (detail::quat_from_forward_up(forward, up, q_wxyz)) {
        out.quat_wxyz = q_wxyz;
        out.roll_confidence = roll_confidence;
        out.valid = true;
        return true;
    }
    // quat_from_forward_up failed. Distinguish a missing forward (true dropout)
    // from a degenerate roll (forward fine, up ∥ forward): the latter should
    // still emit the bone direction with the roll held.
    if (norm(forward) >= 1.0e-6f) {
        cv::Vec4f q_fwd;
        if (detail::quat_from_forward_up(forward, fallback_up_for(forward), q_fwd)) {
            out.quat_wxyz = q_fwd;
            out.roll_confidence = 0.0f;  // hold roll; swing still tracks
            out.valid = true;
            return true;
        }
    }
    out.quat_wxyz = q_wxyz;  // identity from the failed call
    out.roll_confidence = roll_confidence;
    out.valid = false;
    return false;
}

}  // namespace

std::array<SlimeTracker, kTrackerCount>
extract_trackers(const infer::Skeleton3D& skel, ExtractContext* ctx) {
    if (lift::active_keypoint_format() != lift::KeypointFormat::Halpe26) {
        throw std::runtime_error(
            "extract_trackers requires --keypoint-format=halpe26");
    }
    std::array<SlimeTracker, kTrackerCount> out{};
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        out[i].role = static_cast<TrackerRole>(i);
    }

    // ---- Upper arms (shoulder → elbow) ------------------------------------
    // forward = elbow - shoulder. Single physical roll handle (no fallback):
    //   1. wrist - elbow      (elbow hinge fixes humerus roll when bent — only
    //                          physically valid handle from 3D KP alone)
    //
    // Tertiary is intentionally a zero vector. World Z is NOT usable as a
    // fallback for upper-arm roll: at a horizontal arm (fwd ⊥ Z), world Z is
    // assigned confidence ~1 by pick_up_multistage and writes "elbow points
    // toward the ceiling" as an arbitrary roll. Secondary `(neck - shoulder)`
    // is the chest's lateral axis; when primary degenerates with the arm
    // straight, secondary rigidly couples upper-arm roll to torso yaw — the
    // same "rigid pelvis pin" anti-pattern we removed from upper_leg.
    // With both fallbacks zeroed, pick_up_multistage hits its confidence=0
    // sentinel and quat_from_forward_up returns valid=false →
    // apply_quat_smoothing holds the previous humerus quat.
    //
    // Active regime: knee/elbow-style hinge with wrist offset > sin 0.15 from
    // the upper-arm axis (≈ 8.6° elbow flexion). Below that, freeze — twist
    // is unobservable from 3D KP alone in that configuration.
    auto upper_arm = [&](TrackerRole role, std::size_t shoulder, std::size_t elbow,
                          std::size_t wrist, std::size_t out_idx) {
        if (!joints_valid(skel, {shoulder, elbow})) return;
        cv::Vec3f sp = to_vec3f(skel.joints[shoulder]);
        cv::Vec3f ep = to_vec3f(skel.joints[elbow]);
        cv::Vec3f pos = (sp + ep) * 0.5f;
        cv::Vec3f fwd = ep - sp;
        cv::Vec3f up_primary = skel.joints[wrist].valid
                                ? (to_vec3f(skel.joints[wrist]) - ep)
                                : cv::Vec3f{0, 0, 0};
        cv::Vec3f up_tertiary = cv::Vec3f{0, 0, 0};
        float confidence = 0.0f;
        int which = -1;
        // 1-stage selection (same shape as upper_leg): primary slot holds
        // (wrist - elbow); secondary duplicates primary so secondary's
        // degeneracy test collapses; tertiary is the zero sentinel.
        cv::Vec3f up = pick_up_multistage(fwd, up_primary, up_primary, up_tertiary,
                                           confidence, which);
        build_tracker(role, pos, fwd, up, out[out_idx], confidence);
    };
    upper_arm(TrackerRole::LeftUpperArm,  kLShoulder, kLElbow, kLWrist, 0);
    upper_arm(TrackerRole::RightUpperArm, kRShoulder, kRElbow, kRWrist, 1);

    // ---- Chest (mid-torso) ------------------------------------------------
    // pos = midpoint(neck, hip_center); up = neck - hip_center (spine);
    // forward = ⊥(shoulder_axis × spine) so the chest faces forward.
    if (joints_valid(skel, {kNeck, kHipCenter, kLShoulder, kRShoulder})) {
        cv::Vec3f neck = to_vec3f(skel.joints[kNeck]);
        cv::Vec3f hc   = to_vec3f(skel.joints[kHipCenter]);
        cv::Vec3f spine = neck - hc;
        cv::Vec3f shoulder_axis = to_vec3f(skel.joints[kLShoulder]) - to_vec3f(skel.joints[kRShoulder]);
        cv::Vec3f fwd = cross(shoulder_axis, spine);
        cv::Vec3f pos = (neck + hc) * 0.5f;
        build_tracker(TrackerRole::Chest, pos, fwd, spine, out[2]);
    }

    // ---- Waist (pelvis) ---------------------------------------------------
    // pos = hip_center; up = spine; forward = ⊥(hip_axis × spine).
    if (joints_valid(skel, {kHipCenter, kNeck, kLHip, kRHip})) {
        cv::Vec3f hc = to_vec3f(skel.joints[kHipCenter]);
        cv::Vec3f spine = to_vec3f(skel.joints[kNeck]) - hc;
        cv::Vec3f hip_axis = to_vec3f(skel.joints[kLHip]) - to_vec3f(skel.joints[kRHip]);
        cv::Vec3f fwd = cross(hip_axis, spine);
        build_tracker(TrackerRole::Waist, hc, fwd, spine, out[3]);
    }

    // ---- Upper legs (thigh: hip → knee) -----------------------------------
    // forward = knee - hip. Single physical roll handle (no world-Z fallback):
    //   1. ankle - knee       (knee hinge fixes femur roll when bent — only
    //                          physically valid handle from 3D KP alone)
    //
    // Tertiary is intentionally a zero vector. World Z is NOT usable as a
    // fallback for thigh roll: it pins "knee faces ceiling" which is
    // geometrically valid but physically arbitrary, and pick_up_multistage's
    // sin θ-based confidence assigns it 1.0 whenever the thigh is non-vertical
    // (sin(worldZ, fwd) ≈ 1 for horizontal thighs). That writes a fabricated
    // roll with full confidence in routine poses where the primary is
    // degenerate — e.g. 直座り / 長座 / あぐら-からの-脚伸ばし (a very common
    // indoor sitting style in Japan: legs extended forward on the floor with
    // knees fully straight, so (ankle - knee) is colinear with the thigh
    // axis). With the zero tertiary, pick_up_multistage hits its
    // confidence=0 sentinel and quat_from_forward_up returns valid=false →
    // apply_quat_smoothing holds the previous thigh quat.
    //
    // The earlier lateral pin (hip - hip_center) was removed for a different
    // reason: it equals the pelvis lateral axis, which rigidly couples thigh
    // roll to waist yaw and makes them visually inseparable during walking /
    // shallow bends when primary's sin θ briefly dips below kRollSinLow.
    auto upper_leg = [&](TrackerRole role, std::size_t hip, std::size_t knee,
                         std::size_t ankle, std::size_t out_idx) {
        if (!joints_valid(skel, {hip, knee})) return;
        cv::Vec3f hp = to_vec3f(skel.joints[hip]);
        cv::Vec3f kp = to_vec3f(skel.joints[knee]);
        cv::Vec3f pos = (hp + kp) * 0.5f;
        cv::Vec3f fwd = kp - hp;
        cv::Vec3f up_primary = skel.joints[ankle].valid
                                ? (to_vec3f(skel.joints[ankle]) - kp)
                                : cv::Vec3f{0, 0, 0};
        cv::Vec3f up_tertiary = cv::Vec3f{0, 0, 0};
        float confidence = 0.0f;
        int which = -1;
        // 1-stage selection: primary slot holds (ankle - knee); secondary
        // duplicates primary so the secondary degeneracy test collapses;
        // tertiary is the zero sentinel above (forces confidence=0 freeze).
        // Signature is shared with upper_arm which uses a real 3-stage chain
        // ending in world Z.
        cv::Vec3f up = pick_up_multistage(fwd, up_primary, up_primary, up_tertiary,
                                           confidence, which);
        build_tracker(role, pos, fwd, up, out[out_idx], confidence);
    };
    upper_leg(TrackerRole::LeftUpperLeg,  kLHip, kLKnee, kLAnkle, 4);
    upper_leg(TrackerRole::RightUpperLeg, kRHip, kRKnee, kRAnkle, 5);

    // ---- Lower legs (shin: knee → ankle) ----------------------------------
    // pos = midpoint(knee, ankle); forward = ankle - knee; up = hip - knee
    // (back-up the chain to pin shin yaw).
    auto lower_leg = [&](TrackerRole role, std::size_t hip, std::size_t knee, std::size_t ankle,
                         std::size_t out_idx) {
        if (!joints_valid(skel, {hip, knee, ankle})) return;
        cv::Vec3f hp = to_vec3f(skel.joints[hip]);
        cv::Vec3f kp = to_vec3f(skel.joints[knee]);
        cv::Vec3f ap = to_vec3f(skel.joints[ankle]);
        cv::Vec3f pos = (kp + ap) * 0.5f;
        cv::Vec3f fwd = ap - kp;
        cv::Vec3f up  = hp - kp;
        build_tracker(role, pos, fwd, up, out[out_idx]);
    };
    lower_leg(TrackerRole::LeftLowerLeg,  kLHip, kLKnee, kLAnkle, 6);
    lower_leg(TrackerRole::RightLowerLeg, kRHip, kRKnee, kRAnkle, 7);

    // ---- Feet (yaw only; tibia-aligned up) --------------------------------
    // heel KP precision is poor in the 2D→3D pipeline, so the foot is treated
    // as a rigid extension of the shin: up follows the tibia (knee → ankle)
    // and fwd is ankle → toe. This forfeits foot roll (inversion/eversion)
    // and isolates pitch (toe/heel stance) into the fwd tilt, but yaw — the
    // only quantity needed for in-place foot direction in SlimeVR IK — stays
    // free of heel noise. A strong smoothing throttle (kFootSmoothingWeight)
    // further damps residual ankle/toe jitter via apply_quat_smoothing.
    //
    // FK fallback: when ctx is non-null and the anchor has been seeded by a
    // previous good frame, an invalid ankle/toe is reconstructed via
    //   ankle = knee + dir · tibia_len  /  toe = ankle + dir · foot_len
    // This is the workaround for the extended-leg locomotion freeze: under
    // motion blur or self-occlusion the ankle/toe KPs often drop together
    // exactly when the body is sliding past, so without FK the foot tracker
    // gives up for the whole frame and the position freeze (now hip-relative
    // via apply_pos_smoothing) drags it along but the rotation stays stale.
    // With FK, we get a plausible orientation as well so the foot keeps
    // pointing the right way while it follows the hip.
    auto foot_tracker = [&](TrackerRole role, std::size_t knee, std::size_t ankle,
                            std::size_t toe, std::size_t out_idx,
                            std::size_t anchor_idx) {
        if (!skel.joints[knee].valid) return;
        cv::Vec3f kp = to_vec3f(skel.joints[knee]);

        // Resolve ankle: real KP when valid, else FK from anchor.
        cv::Vec3f ap;
        bool ap_synth = false;
        if (skel.joints[ankle].valid) {
            ap = to_vec3f(skel.joints[ankle]);
        } else if (ctx != nullptr && ctx->foot_anchors[anchor_idx].valid) {
            const auto& a = ctx->foot_anchors[anchor_idx];
            ap = cv::Vec3f{kp[0] + a.knee_to_ankle_dir[0] * a.tibia_len_m,
                           kp[1] + a.knee_to_ankle_dir[1] * a.tibia_len_m,
                           kp[2] + a.knee_to_ankle_dir[2] * a.tibia_len_m};
            ap_synth = true;
        } else {
            return;
        }

        // Resolve toe: real KP when valid, else FK from anchor (ankle + dir·len).
        cv::Vec3f tp;
        bool tp_synth = false;
        if (skel.joints[toe].valid) {
            tp = to_vec3f(skel.joints[toe]);
        } else if (ctx != nullptr && ctx->foot_anchors[anchor_idx].valid) {
            const auto& a = ctx->foot_anchors[anchor_idx];
            tp = cv::Vec3f{ap[0] + a.ankle_to_toe_dir[0] * a.foot_len_m,
                           ap[1] + a.ankle_to_toe_dir[1] * a.foot_len_m,
                           ap[2] + a.ankle_to_toe_dir[2] * a.foot_len_m};
            tp_synth = true;
        } else {
            return;
        }

        cv::Vec3f pos = (ap + tp) * 0.5f;
        cv::Vec3f fwd = tp - ap;
        cv::Vec3f up  = kp - ap;            // tibia axis (ankle → knee)
        const float weight = (ap_synth || tp_synth)
                                 ? kFootFkSmoothingWeight
                                 : kFootSmoothingWeight;
        // Foot has no roll observation by design (tibia-aligned up resolves
        // yaw only); the low weight is an overall low-pass, so apply it to BOTH
        // swing and twist. Equal confidences keep the foot on the single-slerp
        // fast path, identical to the pre-split behavior.
        build_tracker(role, pos, fwd, up, out[out_idx], weight, weight);

        // Only re-anchor from a fully measured frame; an FK-synthesized
        // direction would otherwise drift on itself if a real KP never returns.
        if (ctx != nullptr && !ap_synth && !tp_synth) {
            auto& a = ctx->foot_anchors[anchor_idx];
            cv::Vec3f ka{ap[0] - kp[0], ap[1] - kp[1], ap[2] - kp[2]};
            float ka_len = norm(ka);
            cv::Vec3f at{tp[0] - ap[0], tp[1] - ap[1], tp[2] - ap[2]};
            float at_len = norm(at);
            if (ka_len > 1.0e-4f && at_len > 1.0e-4f) {
                a.knee_to_ankle_dir = cv::Vec3f{ka[0] / ka_len, ka[1] / ka_len, ka[2] / ka_len};
                a.tibia_len_m       = ka_len;
                a.ankle_to_toe_dir  = cv::Vec3f{at[0] / at_len, at[1] / at_len, at[2] / at_len};
                a.foot_len_m        = at_len;
                a.valid             = true;
            }
        }
    };
    foot_tracker(TrackerRole::LeftFoot,  kLKnee, kLAnkle, kLBigToe, 8, 0);
    foot_tracker(TrackerRole::RightFoot, kRKnee, kRAnkle, kRBigToe, 9, 1);

    return out;
}

namespace {
// Frame-rate-independent EMA alpha. `base_alpha` is the per-step weight tuned at
// `nominal_dt_s` (= 1/extract_rate_hz). For an actual step of `dt_s` the
// effective weight is 1 - (1-base_alpha)^(dt_s/nominal_dt_s), so the wall-clock
// response (time constant) is the same regardless of how fast frames arrive.
//   * dt_s == nominal_dt_s  -> returns base_alpha exactly (fixed-rate path is
//     byte-for-byte unchanged).
//   * higher rate (dt_s < nominal) -> smaller per-step alpha (more, gentler
//     steps) so the event-driven path no longer over-smooths at high fps.
//   * a long post-idle gap (dt_s >> nominal) -> alpha -> 1 (snap to current,
//     don't keep trusting a stale prev).
float rate_adjust_alpha(float base_alpha, float dt_s, float nominal_dt_s) {
    base_alpha = std::clamp(base_alpha, 0.0f, 1.0f);
    if (base_alpha <= 0.0f) return 0.0f;
    if (base_alpha >= 1.0f) return 1.0f;
    if (!(nominal_dt_s > 0.0f) || !(dt_s > 0.0f)) return base_alpha;
    // Fixed-rate path (run_loop passes dt_s = nominal_dt_s): return base_alpha
    // exactly, skipping std::pow so the behavior is byte-for-byte unchanged
    // (std::pow(x, 1) is not guaranteed bit-exact) and the hot path pays nothing.
    if (dt_s == nominal_dt_s) return base_alpha;
    const float ratio = dt_s / nominal_dt_s;
    return 1.0f - std::pow(1.0f - base_alpha, ratio);
}
}  // namespace

void apply_quat_smoothing(std::array<SlimeTracker, kTrackerCount>& curr,
                          std::array<cv::Vec4f, kTrackerCount>& prev_quat,
                          float base_alpha, float dt_s, float nominal_dt_s) {
    const float alpha_rate = rate_adjust_alpha(base_alpha, dt_s, nominal_dt_s);

    // ---- Pelvis-yaw transport for held-roll bones -------------------------
    // An extended limb's roll is unobservable (roll_confidence → 0), so it is
    // held frame-to-frame. For a near-vertical limb (standing, legs/arms down)
    // a body yaw is a rotation about the bone's own forward axis — i.e. pure
    // roll — so a held roll freezes the limb's facing when the subject turns
    // sideways. A parent tracker carries an observable, reliable orientation;
    // ride the held roll along with the *change* in that orientation so a yaw
    // still turns the limb. swing reconciles the observed forward afterward, so
    // only the forward-axis component (the yaw, for a vertical bone) survives as
    // roll; a parent pitch/roll is absorbed by swing. This is the rotational
    // analogue of the M1 hip-relative position hold: a delta coupling that
    // preserves the relative roll offset, NOT the rejected absolute parent pin.
    //
    // Reference per limb: arms ride the CHEST (shoulder girdle), legs ride the
    // WAIST (pelvis). The pelvis can yaw independently of the chest via spine
    // twist, so using the pelvis for arms would mis-track a torso rotation; the
    // chest is the anatomically correct parent for the humerus. Each delta is
    // computed once up front from the PREVIOUS parent quat (before the loop
    // overwrites prev_quat) and this frame's raw parent measurement.
    auto make_transport_delta = [&](std::size_t ref, cv::Vec4f& out) -> bool {
        if (!curr[ref].valid) return false;
        const cv::Vec4f rp = quat_normalize(prev_quat[ref]);
        const cv::Vec4f rc = quat_normalize(curr[ref].quat_wxyz);
        cv::Vec4f d = quat_mul(rc, quat_conj(rp));  // world-frame: maps rp → rc
        if (d[0] < 0.0f) d = cv::Vec4f{-d[0], -d[1], -d[2], -d[3]};  // shorter arc
        const float ang  = 2.0f * std::acos(std::clamp(d[0], -1.0f, 1.0f));  // [0, π]
        const float step = (dt_s > 0.0f) ? dt_s : nominal_dt_s;
        const float rate = (step > 0.0f) ? ang / step : 0.0f;
        const float scale =
            1.0f - smoothstep01(rate, kPelvisYawGateLow_rps, kPelvisYawGateHigh_rps);
        out = quat_slerp(quat_identity(), d, scale);
        return true;
    };
    constexpr std::size_t kWaistIdx = static_cast<std::size_t>(TrackerRole::Waist);
    constexpr std::size_t kChestIdx = static_cast<std::size_t>(TrackerRole::Chest);
    cv::Vec4f waist_delta = quat_identity(), chest_delta = quat_identity();
    const bool have_waist_delta = make_transport_delta(kWaistIdx, waist_delta);
    const bool have_chest_delta = make_transport_delta(kChestIdx, chest_delta);

    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        if (!curr[i].valid) {
            // Hold previous orientation: publisher will see prev via curr, and
            // prev_quat itself is left untouched so a tracker can recover from
            // a dropped frame without snapping back through identity.
            curr[i].quat_wxyz = prev_quat[i];
            continue;
        }
        // Rate-adjusted per-step weight, then split by swing/twist confidence.
        const float sa = std::clamp(alpha_rate * curr[i].swing_confidence, 0.0f, 1.0f);
        const float ta = std::clamp(alpha_rate * curr[i].roll_confidence,  0.0f, 1.0f);
        cv::Vec4f p = quat_normalize(prev_quat[i]);
        const cv::Vec4f q = quat_normalize(curr[i].quat_wxyz);

        // Transport the held-roll reference by the parent delta. Gated to the
        // split branch (sa != ta) so rigid fast-path bones, the foot, and the
        // chest/waist references themselves stay bit-identical to the
        // pre-transport behavior. Arms ride the chest; legs ride the waist
        // (arms fall back to the waist only if the chest is unobservable, since
        // a frozen arm is worse than a slightly-off one). `carry`
        // complementary-blends the parent-yaw prior against the bone's own roll
        // measurement: a fully held roll (ta = 0) rides the full parent delta
        // (carry = 1, the prior is all we have); as the roll becomes observable
        // (ta → sa) carry → 0 and the twist slerp toward q takes over, so a limb
        // twisting relative to its parent is not forced to follow it.
        const bool is_arm = (i == static_cast<std::size_t>(TrackerRole::LeftUpperArm) ||
                             i == static_cast<std::size_t>(TrackerRole::RightUpperArm));
        const bool  have_ref  = is_arm ? (have_chest_delta || have_waist_delta)
                                       : have_waist_delta;
        const cv::Vec4f& ref_delta =
            is_arm ? (have_chest_delta ? chest_delta : waist_delta) : waist_delta;
        if (have_ref && std::abs(sa - ta) >= 1.0e-6f) {
            const float carry = std::clamp(1.0f - (ta / std::max(sa, 1.0e-6f)), 0.0f, 1.0f);
            const cv::Vec4f d = quat_slerp(quat_identity(), ref_delta, carry);
            p = quat_normalize(quat_mul(d, p));
        }

        cv::Vec4f r;
        if (std::abs(sa - ta) < 1.0e-6f) {
            // Fast path: equal gates → single slerp, bit-identical to the
            // pre-split behavior (rigid bones at 1.0, foot at its weight). Also
            // covers sa == ta == 0 (fully frozen).
            r = (sa <= 0.0f) ? p : quat_slerp(p, q, sa);
        } else {
            // Swing/twist split: decompose the relative rotation about the
            // bone forward (+Z) and interpolate roll and pitch/yaw with
            // independent gates. ta = 0 holds the roll while sa keeps the bone
            // direction tracking — the extended-limb case the publisher needs.
            cv::Vec4f delta = quat_mul(quat_conj(p), q);
            if (delta[0] < 0.0f) {  // canonicalize to w ≥ 0 (shorter arc)
                delta = cv::Vec4f{-delta[0], -delta[1], -delta[2], -delta[3]};
            }
            cv::Vec4f swing, twist;
            swing_twist_about_z(delta, swing, twist);
            cv::Vec4f swing_step = quat_slerp(quat_identity(), swing, sa);
            cv::Vec4f twist_step = quat_slerp(quat_identity(), twist, ta);
            r = quat_normalize(quat_mul(quat_mul(p, swing_step), twist_step));
        }
        curr[i].quat_wxyz = r;
        prev_quat[i] = r;
    }
}

void apply_pos_smoothing(std::array<SlimeTracker, kTrackerCount>& curr,
                         std::array<cv::Vec3f, kTrackerCount>& prev_pos,
                         PosSmoothingContext& ctx,
                         float base_alpha, float nominal_dt_s) {
    // Frame-rate-independent per-step weight; the velocity gate scales this.
    const float alpha_rate = rate_adjust_alpha(base_alpha, ctx.dt_s, nominal_dt_s);
    const float dt = std::max(1.0e-3f, ctx.dt_s);
    const bool  can_hip_relative = ctx.hip_valid && ctx.prev_hip_valid;

    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        cv::Vec3f& p = prev_pos[i];

        if (!curr[i].valid) {
            // Hold branch. With a valid hip reference (both current and
            // previous), re-anchor the held position so it travels with
            // the hip. The waist tracker has prev_pos ≡ hip_center by
            // construction, so offset ≈ 0 and the hip-relative hold
            // collapses to "snap to current hip" — exactly what we want.
            if (can_hip_relative && ctx.has_last_raw[i]) {
                cv::Vec3f offset{p[0] - ctx.prev_hip_pos[0],
                                 p[1] - ctx.prev_hip_pos[1],
                                 p[2] - ctx.prev_hip_pos[2]};
                cv::Vec3f world{ctx.current_hip_pos[0] + offset[0],
                                ctx.current_hip_pos[1] + offset[1],
                                ctx.current_hip_pos[2] + offset[2]};
                curr[i].pos = world;
                p = world;
            } else {
                curr[i].pos = p;
            }
            // Saturate at uint32 so a tracker that's been invalid since
            // process start doesn't wrap; once large, the gate effectively
            // collapses to "no gate" anyway because elapsed → ∞.
            if (ctx.invalid_ticks_since_last_raw[i] < 0xFFFFFFFFu) {
                ctx.invalid_ticks_since_last_raw[i] += 1;
            }
            continue;
        }

        const cv::Vec3f q = curr[i].pos;

        // Velocity gate: consecutive raw (pre-smoothing) measurement deltas
        // are the right outlier signal — prev_pos lags due to EMA, so
        // prev→curr distance can look inflated during convergence even
        // when the actual measurement stream is steady. With last_raw_pos,
        // a single triangulation glitch (curr jumps several meters relative
        // to the previous valid frame) collapses the alpha; nominal motion
        // (walking, gestures) passes through.
        //
        // After an N-frame dropout, the real elapsed time between
        // last_raw_pos and q is (1 + N) · dt_s, not a single tick — without
        // this correction a recovery frame would divide a normal
        // displacement by a single tick and collapse the alpha, leaving
        // the smoother stuck on the held position.
        float alpha = alpha_rate;
        if (ctx.has_last_raw[i]) {
            float dx = q[0] - ctx.last_raw_pos[i][0];
            float dy = q[1] - ctx.last_raw_pos[i][1];
            float dz = q[2] - ctx.last_raw_pos[i][2];
            float dist     = std::sqrt(dx*dx + dy*dy + dz*dz);
            float elapsed  = dt * static_cast<float>(
                                       1u + ctx.invalid_ticks_since_last_raw[i]);
            float v_mps    = dist / elapsed;
            float gate     = smoothstep01(v_mps, kPosVelGateLow_mps, kPosVelGateHigh_mps);
            alpha = alpha_rate * (1.0f - gate);
        }

        p[0] += alpha * (q[0] - p[0]);
        p[1] += alpha * (q[1] - p[1]);
        p[2] += alpha * (q[2] - p[2]);
        curr[i].pos = p;
        ctx.last_raw_pos[i] = q;
        ctx.has_last_raw[i] = true;
        ctx.invalid_ticks_since_last_raw[i] = 0;
    }

    // Cache the hip position for the next call's hip-relative hold.
    if (ctx.hip_valid) {
        ctx.prev_hip_pos    = ctx.current_hip_pos;
        ctx.prev_hip_valid  = true;
    }
}

// World-absolute hold form: no hip re-anchor, no velocity gate. Frame-rate
// independent via dt_s/nominal_dt_s. Kept for tests and context-less callers.
void apply_pos_smoothing(std::array<SlimeTracker, kTrackerCount>& curr,
                         std::array<cv::Vec3f, kTrackerCount>& prev_pos,
                         float base_alpha, float dt_s, float nominal_dt_s) {
    const float alpha = rate_adjust_alpha(base_alpha, dt_s, nominal_dt_s);
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        if (!curr[i].valid) {
            curr[i].pos = prev_pos[i];
            continue;
        }
        cv::Vec3f& p = prev_pos[i];
        const cv::Vec3f q = curr[i].pos;
        p[0] += alpha * (q[0] - p[0]);
        p[1] += alpha * (q[1] - p[1]);
        p[2] += alpha * (q[2] - p[2]);
        curr[i].pos = p;
    }
}

}  // namespace fitra::slimevr
