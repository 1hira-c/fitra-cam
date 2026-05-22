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
constexpr std::size_t kLHeel     = 24, kRHeel     = 25;

// Below this sin(angle(up, fwd)) the up hint is treated as degenerate and the
// fallback up is used. 0.2 ≈ sin(11.5°); see docs/phase12-slimevr-bridge-relay.md.
constexpr float kUpperArmRollSwitch = 0.2f;
constexpr float kThighRollSwitch    = 0.2f;

bool joints_valid(const infer::Skeleton3D& s, std::initializer_list<std::size_t> idxs) {
    for (auto i : idxs) {
        if (!s.joints[i].valid) return false;
    }
    return true;
}

// Pick the first up hint whose perpendicular component to fwd has magnitude
// > threshold * |up|. Zero-vector ups (from invalid joints) skip immediately.
// `forward` is assumed non-zero (caller checks); ups are inspected in primary,
// secondary, tertiary order. Returns the chosen up; sets `which` to 0/1/2 for
// the stage that won, or 2 if all degenerated (in which case the tertiary is
// returned even though it may also fail downstream).
cv::Vec3f pick_up_multistage(const cv::Vec3f& fwd,
                              const cv::Vec3f& up_primary,
                              const cv::Vec3f& up_secondary,
                              const cv::Vec3f& up_tertiary,
                              float threshold,
                              int& which) {
    float f_n = norm(fwd);
    const cv::Vec3f* candidates[3] = {&up_primary, &up_secondary, &up_tertiary};
    for (int i = 0; i < 3; ++i) {
        const cv::Vec3f& u = *candidates[i];
        float u_n = norm(u);
        if (u_n < 1.0e-6f) continue;  // zero vector → invalid joint, skip
        // sin θ = |u × fwd| / (|u| |fwd|). Tertiary always accepted.
        if (i == 2 || norm(cross(u, fwd)) / (u_n * f_n) >= threshold) {
            which = i;
            return u;
        }
    }
    which = 2;
    return up_tertiary;
}

}  // namespace

TrackerPosition position_for(TrackerRole role) {
    switch (role) {
        case TrackerRole::LeftUpperArm:  return TrackerPosition::LeftUpperArm;
        case TrackerRole::RightUpperArm: return TrackerPosition::RightUpperArm;
        case TrackerRole::Chest:         return TrackerPosition::Chest;
        case TrackerRole::Waist:         return TrackerPosition::Waist;
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
    if (norm(right_raw) < 1.0e-6f) {  // up is parallel to forward
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
bool build_tracker(TrackerRole role,
                   const cv::Vec3f& world_pos,
                   const cv::Vec3f& forward,
                   const cv::Vec3f& up,
                   SlimeTracker& out) {
    out.role = role;
    out.pos  = world_pos;
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
    // forward = elbow - shoulder. Roll is pinned by a three-stage up choice:
    //   1. wrist - elbow      (elbow hinge gives strongest physical handle)
    //   2. neck  - shoulder   (lateral pin; degenerate when arm raised)
    //   3. world Z            (last resort)
    auto upper_arm = [&](TrackerRole role, std::size_t shoulder, std::size_t elbow,
                          std::size_t wrist, std::size_t out_idx) {
        if (!joints_valid(skel, {kNeck, shoulder, elbow})) return;
        cv::Vec3f sp = to_vec3f(skel.joints[shoulder]);
        cv::Vec3f ep = to_vec3f(skel.joints[elbow]);
        cv::Vec3f np = to_vec3f(skel.joints[kNeck]);
        cv::Vec3f pos = (sp + ep) * 0.5f;
        cv::Vec3f fwd = ep - sp;
        cv::Vec3f up_primary = skel.joints[wrist].valid
                                ? (to_vec3f(skel.joints[wrist]) - ep)
                                : cv::Vec3f{0, 0, 0};
        cv::Vec3f up_secondary = np - sp;
        cv::Vec3f up_tertiary  = cv::Vec3f{0, 0, 1};
        int which = -1;
        cv::Vec3f up = pick_up_multistage(fwd, up_primary, up_secondary, up_tertiary,
                                           kUpperArmRollSwitch, which);
        build_tracker(role, pos, fwd, up, out[out_idx]);
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
    // forward = knee - hip. Three-stage up choice mirroring the upper arm:
    //   1. ankle - knee       (knee hinge fixes femur roll when bent)
    //   2. hip   - hip_center (lateral pin; ok in standing pose)
    //   3. world Z
    auto upper_leg = [&](TrackerRole role, std::size_t hip, std::size_t knee,
                         std::size_t ankle, std::size_t out_idx) {
        if (!joints_valid(skel, {kHipCenter, hip, knee})) return;
        cv::Vec3f hp = to_vec3f(skel.joints[hip]);
        cv::Vec3f kp = to_vec3f(skel.joints[knee]);
        cv::Vec3f hc = to_vec3f(skel.joints[kHipCenter]);
        cv::Vec3f pos = (hp + kp) * 0.5f;
        cv::Vec3f fwd = kp - hp;
        cv::Vec3f up_primary = skel.joints[ankle].valid
                                ? (to_vec3f(skel.joints[ankle]) - kp)
                                : cv::Vec3f{0, 0, 0};
        cv::Vec3f up_secondary = hp - hc;
        cv::Vec3f up_tertiary  = cv::Vec3f{0, 0, 1};
        int which = -1;
        cv::Vec3f up = pick_up_multistage(fwd, up_primary, up_secondary, up_tertiary,
                                           kThighRollSwitch, which);
        build_tracker(role, pos, fwd, up, out[out_idx]);
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

    // ---- Feet -------------------------------------------------------------
    // forward = big_toe - heel. Up = ankle - foot_mid: this is the foot-plane
    // normal (ankle sits above the heel-toe midpoint), so toe-up / heel-up /
    // inversion all tilt this vector and produce the correct foot roll/pitch.
    // World Z fallback only if ankle is missing.
    auto foot_tracker = [&](TrackerRole role, std::size_t heel, std::size_t toe,
                            std::size_t ankle, std::size_t out_idx) {
        if (!joints_valid(skel, {heel, toe})) return;
        cv::Vec3f heel_p = to_vec3f(skel.joints[heel]);
        cv::Vec3f toe_p  = to_vec3f(skel.joints[toe]);
        cv::Vec3f foot_mid = (heel_p + toe_p) * 0.5f;
        cv::Vec3f pos = foot_mid;
        cv::Vec3f fwd = toe_p - heel_p;
        cv::Vec3f up = skel.joints[ankle].valid
                        ? (to_vec3f(skel.joints[ankle]) - foot_mid)
                        : cv::Vec3f{0, 0, 1};
        build_tracker(role, pos, fwd, up, out[out_idx]);
    };
    foot_tracker(TrackerRole::LeftFoot,  kLHeel, kLBigToe, kLAnkle, 8);
    foot_tracker(TrackerRole::RightFoot, kRHeel, kRBigToe, kRAnkle, 9);

    return out;
}

void apply_quat_smoothing(std::array<SlimeTracker, kTrackerCount>& curr,
                          std::array<cv::Vec4f, kTrackerCount>& prev_quat,
                          float alpha) {
    alpha = std::clamp(alpha, 0.0f, 1.0f);
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        if (!curr[i].valid) {
            prev_quat[i] = curr[i].quat_wxyz;
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
            r = cv::Vec4f{p[0] + alpha * (q[0] - p[0]),
                          p[1] + alpha * (q[1] - p[1]),
                          p[2] + alpha * (q[2] - p[2]),
                          p[3] + alpha * (q[3] - p[3])};
        } else {
            float theta = std::acos(std::clamp(d, -1.0f, 1.0f));
            float sin_t = std::sin(theta);
            float wa = std::sin((1.0f - alpha) * theta) / sin_t;
            float wb = std::sin(alpha * theta) / sin_t;
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

}  // namespace fitra::slimevr
