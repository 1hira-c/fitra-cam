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

inline float dot(const cv::Vec3f& a, const cv::Vec3f& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline cv::Vec3f normalize_or_zero(const cv::Vec3f& v) {
    float n = norm(v);
    if (n < 1.0e-6f) return cv::Vec3f{0, 0, 0};
    return cv::Vec3f{v[0] / n, v[1] / n, v[2] / n};
}

// Halpe26 index symbols. Kept local because tracker_extract is the only call
// site that needs all of them by name.
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

}  // namespace

namespace detail {

cv::Vec3f world_pos_to_vmc(const cv::Vec3f& w) {
    // (X, Y, Z) world (Z-up, X-right, Y-forward, m) -> (X, Z, -Y) Unity LH Y-up.
    return cv::Vec3f{w[0], w[2], -w[1]};
}

cv::Vec4f world_quat_to_vmc(const cv::Vec4f& q_wxyz) {
    // Unity left-handed Y-up quaternion (qx_u, qy_u, qz_u, qw_u) =
    //   ( qx, qz, -qy, -qw )
    // The conjugation here matches SlimeVR-Server's VMCHandler.kt where the
    // receiver re-flips Z and W back into its internal frame.
    float qw = q_wxyz[0], qx = q_wxyz[1], qy = q_wxyz[2], qz = q_wxyz[3];
    return cv::Vec4f{-qw, qx, qz, -qy};   // wxyz storage: w x y z = -qw qx qz -qy
}

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
    // Shoemake's matrix-to-quaternion: pick the largest of (1+trace, 1+m00-m11-m22,
    // 1+m11-m00-m22, 1+m22-m00-m11) for numerical stability, then derive the
    // other three components.
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
    // Normalize defensively; fp drift can yield |q| slightly off 1.
    float n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (n < 1.0e-9f) { out_wxyz = cv::Vec4f{1, 0, 0, 0}; return false; }
    out_wxyz = cv::Vec4f{qw / n, qx / n, qy / n, qz / n};
    return true;
}

}  // namespace detail

namespace {

bool joints_valid(const infer::Skeleton3D& s, std::initializer_list<std::size_t> idxs) {
    for (auto i : idxs) {
        if (!s.joints[i].valid) return false;
    }
    return true;
}

// Try to build a VmcTracker for one role. Returns false on degeneracy.
bool build_tracker(TrackerRole role,
                   const cv::Vec3f& world_pos,
                   const cv::Vec3f& forward,
                   const cv::Vec3f& up,
                   VmcTracker& out) {
    out.role = role;
    cv::Vec4f q_wxyz;
    bool ok = detail::quat_from_forward_up(forward, up, q_wxyz);
    out.pos = detail::world_pos_to_vmc(world_pos);
    out.quat_wxyz = detail::world_quat_to_vmc(q_wxyz);
    out.valid = ok;
    return ok;
}

}  // namespace

std::array<VmcTracker, kTrackerCount>
extract_vmc_trackers(const infer::Skeleton3D& skel) {
    if (lift::active_keypoint_format() != lift::KeypointFormat::Halpe26) {
        throw std::runtime_error(
            "extract_vmc_trackers requires --keypoint-format=halpe26");
    }
    std::array<VmcTracker, kTrackerCount> out{};
    // Initialize roles even for invalid trackers so the publisher can skip
    // them by `valid` without checking the role enum.
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        out[i].role = static_cast<TrackerRole>(i);
    }

    // WAIST = hip-center. Forward (z+) = neck above hip-center, up = same
    // spine direction since waist orientation primarily encodes torso yaw.
    // We use the right-hip→left-hip axis to pin yaw.
    if (joints_valid(skel, {kHipCenter, kNeck, kLHip, kRHip})) {
        cv::Vec3f pos = to_vec3f(skel.joints[kHipCenter]);
        cv::Vec3f spine = to_vec3f(skel.joints[kNeck]) - pos;
        cv::Vec3f hip_axis = to_vec3f(skel.joints[kLHip]) - to_vec3f(skel.joints[kRHip]);
        // forward (pelvis facing) ≈ hip_axis × spine (right-handed)
        cv::Vec3f fwd = cross(hip_axis, spine);
        build_tracker(TrackerRole::Waist, pos, fwd, spine, out[0]);
    }

    // CHEST = neck. Same scheme using shoulder line.
    if (joints_valid(skel, {kNeck, kHipCenter, kLShoulder, kRShoulder})) {
        cv::Vec3f pos = to_vec3f(skel.joints[kNeck]);
        cv::Vec3f spine = pos - to_vec3f(skel.joints[kHipCenter]);
        cv::Vec3f shoulder_axis = to_vec3f(skel.joints[kLShoulder]) - to_vec3f(skel.joints[kRShoulder]);
        cv::Vec3f fwd = cross(shoulder_axis, spine);
        build_tracker(TrackerRole::Chest, pos, fwd, spine, out[1]);
    }

    // KNEE (each side): pos = knee, forward = knee→ankle (lower leg axis),
    // up = knee→hip (thigh axis, sign flipped so a vertical leg yields up=+spine).
    auto knee_tracker = [&](TrackerRole role, std::size_t hip, std::size_t knee, std::size_t ankle,
                            std::size_t out_idx) {
        if (!joints_valid(skel, {hip, knee, ankle})) return;
        cv::Vec3f pos  = to_vec3f(skel.joints[knee]);
        cv::Vec3f fwd  = to_vec3f(skel.joints[ankle]) - pos;
        cv::Vec3f up   = to_vec3f(skel.joints[hip]) - pos;
        build_tracker(role, pos, fwd, up, out[out_idx]);
    };
    knee_tracker(TrackerRole::LeftKnee,  kLHip, kLKnee, kLAnkle, 2);
    knee_tracker(TrackerRole::RightKnee, kRHip, kRKnee, kRAnkle, 3);

    // ELBOW (each side): pos = elbow, forward = elbow→wrist, up = elbow→shoulder.
    auto elbow_tracker = [&](TrackerRole role, std::size_t shoulder, std::size_t elbow,
                             std::size_t wrist, std::size_t out_idx) {
        if (!joints_valid(skel, {shoulder, elbow, wrist})) return;
        cv::Vec3f pos = to_vec3f(skel.joints[elbow]);
        cv::Vec3f fwd = to_vec3f(skel.joints[wrist]) - pos;
        cv::Vec3f up  = to_vec3f(skel.joints[shoulder]) - pos;
        build_tracker(role, pos, fwd, up, out[out_idx]);
    };
    elbow_tracker(TrackerRole::LeftElbow,  kLShoulder, kLElbow, kLWrist,  4);
    elbow_tracker(TrackerRole::RightElbow, kRShoulder, kRElbow, kRWrist, 5);

    // FOOT (each side): pos = (heel + big_toe) / 2 = foot center.
    // forward = heel→big_toe (toe direction), up = world Z (yaw-only).
    auto foot_tracker = [&](TrackerRole role, std::size_t heel, std::size_t toe,
                            std::size_t out_idx) {
        if (!joints_valid(skel, {heel, toe})) return;
        cv::Vec3f heel_p = to_vec3f(skel.joints[heel]);
        cv::Vec3f toe_p  = to_vec3f(skel.joints[toe]);
        cv::Vec3f pos = (heel_p + toe_p) * 0.5f;
        cv::Vec3f fwd = toe_p - heel_p;
        cv::Vec3f up{0.0f, 0.0f, 1.0f};  // world Z-up; yaw-only is the meaningful DoF
        build_tracker(role, pos, fwd, up, out[out_idx]);
    };
    foot_tracker(TrackerRole::LeftFoot,  kLHeel, kLBigToe, 6);
    foot_tracker(TrackerRole::RightFoot, kRHeel, kRBigToe, 7);

    return out;
}

void apply_quat_smoothing(std::array<VmcTracker, kTrackerCount>& curr,
                          std::array<cv::Vec4f, kTrackerCount>& prev_quat,
                          float alpha) {
    alpha = std::clamp(alpha, 0.0f, 1.0f);
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        if (!curr[i].valid) {
            // Reset prev so a tracker recovering visibility doesn't blend
            // against stale orientation. Leave the (identity) curr quat alone.
            prev_quat[i] = curr[i].quat_wxyz;
            continue;
        }
        const cv::Vec4f& p = prev_quat[i];
        cv::Vec4f q = curr[i].quat_wxyz;
        // Take the shorter arc by flipping q if p·q < 0.
        float d = p[0]*q[0] + p[1]*q[1] + p[2]*q[2] + p[3]*q[3];
        if (d < 0.0f) { q = cv::Vec4f{-q[0], -q[1], -q[2], -q[3]}; d = -d; }
        // Slerp; fall back to nlerp when angles are tiny to avoid div-by-zero.
        cv::Vec4f r;
        if (d > 0.9995f) {
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
