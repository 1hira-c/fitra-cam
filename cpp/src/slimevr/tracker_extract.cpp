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
// Phase 13 (2026-05-25): raised from 0.05 / 0.20 → 0.15 / 0.30 after the
// 直立で脚を伸ばし切ったときに大腿が一気に 90° roll する現象の実機ログ
// (ang vel p95 = 1.8 rad/s, conf_avg = 0.16, leakage_pct = 100% on the
// thigh during slow extension) で smoothstep leakage 仮説が確定したため。
// 新しきい:
//   * sin 0.15 ≈ 8.6° bend: degenerate gate。歩行 (>30°) / しゃがみ (>60°)
//     /着座 (~90°) は確実に超え、立位伸展 (0-5°) は確実に下回る
//   * sin 0.30 ≈ 17.5° bend: full-confidence ceiling。中間域 (8.6°..17.5°)
//     だけが smoothstep の漸進更新ゾーンになり、実用ポーズ域からは離れる
// `quat_from_forward_up` の degeneracy 判定も同じ kRollSinLow を sin θ
// gate として使うので、pick_up_multistage を経由しない rigid tracker
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
    // Phase 13: sin θ-based degeneracy gate so rigid trackers (shin, foot,
    // chest, waist) that don't go through pick_up_multistage still get the
    // same near-parallel protection as the thigh / upper-arm path.
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

// Try to build a SlimeTracker for one role from a position joint, a forward
// hint and an up hint. Updates `out` in place; valid=false on degeneracy.
// `roll_confidence` defaults to 1.0 for rigid-pin bones (chest/waist/shin/foot);
// upper arm / thigh pass their smoothstep-derived value.
bool build_tracker(TrackerRole role,
                   const cv::Vec3f& world_pos,
                   const cv::Vec3f& forward,
                   const cv::Vec3f& up,
                   SlimeTracker& out,
                   float roll_confidence = 1.0f) {
    out.role = role;
    out.pos  = world_pos;
    out.roll_confidence = roll_confidence;
    cv::Vec4f q_wxyz;
    bool ok = detail::quat_from_forward_up(forward, up, q_wxyz);
    out.quat_wxyz = q_wxyz;
    out.valid = ok;
    return ok;
}

}  // namespace

std::array<SlimeTracker, kTrackerCount>
extract_trackers(const infer::Skeleton3D& skel) {
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
    // same "rigid pelvis pin" anti-pattern we removed from upper_leg in
    // Phase 12 M1. With both fallbacks zeroed, pick_up_multistage hits its
    // confidence=0 sentinel and quat_from_forward_up returns valid=false →
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
    auto foot_tracker = [&](TrackerRole role, std::size_t knee, std::size_t ankle,
                            std::size_t toe, std::size_t out_idx) {
        if (!joints_valid(skel, {knee, ankle, toe})) return;
        cv::Vec3f kp = to_vec3f(skel.joints[knee]);
        cv::Vec3f ap = to_vec3f(skel.joints[ankle]);
        cv::Vec3f tp = to_vec3f(skel.joints[toe]);
        cv::Vec3f pos = (ap + tp) * 0.5f;
        cv::Vec3f fwd = tp - ap;
        cv::Vec3f up  = kp - ap;            // tibia axis (ankle → knee)
        build_tracker(role, pos, fwd, up, out[out_idx], kFootSmoothingWeight);
    };
    foot_tracker(TrackerRole::LeftFoot,  kLKnee, kLAnkle, kLBigToe, 8);
    foot_tracker(TrackerRole::RightFoot, kRKnee, kRAnkle, kRBigToe, 9);

    return out;
}

void apply_quat_smoothing(std::array<SlimeTracker, kTrackerCount>& curr,
                          std::array<cv::Vec4f, kTrackerCount>& prev_quat,
                          float base_alpha) {
    base_alpha = std::clamp(base_alpha, 0.0f, 1.0f);
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        if (!curr[i].valid) {
            // Hold previous orientation: publisher will see prev via curr, and
            // prev_quat itself is left untouched so a tracker can recover from
            // a dropped frame without snapping back through identity.
            curr[i].quat_wxyz = prev_quat[i];
            continue;
        }
        float effective_alpha = std::clamp(
            base_alpha * curr[i].roll_confidence, 0.0f, 1.0f);
        if (effective_alpha <= 0.0f) {
            // Fully frozen: keep prev. Don't touch prev_quat.
            curr[i].quat_wxyz = prev_quat[i];
            continue;
        }
        const cv::Vec4f& p = prev_quat[i];
        cv::Vec4f q = curr[i].quat_wxyz;
        // Shorter arc via flip if p·q < 0.
        float d = p[0]*q[0] + p[1]*q[1] + p[2]*q[2] + p[3]*q[3];
        if (d < 0.0f) { q = cv::Vec4f{-q[0], -q[1], -q[2], -q[3]}; d = -d; }
        cv::Vec4f r;
        if (d > 0.9995f) {
            // nlerp fallback for small angles.
            r = cv::Vec4f{p[0] + effective_alpha * (q[0] - p[0]),
                          p[1] + effective_alpha * (q[1] - p[1]),
                          p[2] + effective_alpha * (q[2] - p[2]),
                          p[3] + effective_alpha * (q[3] - p[3])};
        } else {
            float theta = std::acos(std::clamp(d, -1.0f, 1.0f));
            float sin_t = std::sin(theta);
            float wa = std::sin((1.0f - effective_alpha) * theta) / sin_t;
            float wb = std::sin(effective_alpha * theta) / sin_t;
            r = cv::Vec4f{wa * p[0] + wb * q[0],
                          wa * p[1] + wb * q[1],
                          wa * p[2] + wb * q[2],
                          wa * p[3] + wb * q[3]};
        }
        float n = std::sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2] + r[3]*r[3]);
        if (n > 1.0e-9f) {
            r = cv::Vec4f{r[0]/n, r[1]/n, r[2]/n, r[3]/n};
        } else {
            r = cv::Vec4f{1, 0, 0, 0};
        }
        curr[i].quat_wxyz = r;
        prev_quat[i] = r;
    }
}

void apply_pos_smoothing(std::array<SlimeTracker, kTrackerCount>& curr,
                         std::array<cv::Vec3f, kTrackerCount>& prev_pos,
                         float base_alpha) {
    base_alpha = std::clamp(base_alpha, 0.0f, 1.0f);
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        if (!curr[i].valid) {
            // Hold previous position: publisher sees prev via curr, and
            // prev_pos itself is left untouched so a tracker can recover
            // from a dropped frame without snapping back through (0,0,0).
            curr[i].pos = prev_pos[i];
            continue;
        }
        cv::Vec3f& p = prev_pos[i];
        const cv::Vec3f q = curr[i].pos;
        p[0] += base_alpha * (q[0] - p[0]);
        p[1] += base_alpha * (q[1] - p[1]);
        p[2] += base_alpha * (q[2] - p[2]);
        curr[i].pos = p;
    }
}

}  // namespace fitra::slimevr
