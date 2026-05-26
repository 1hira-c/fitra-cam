#include "vmt/auto_alignment.hpp"

#include <opencv2/core.hpp>

#include <cmath>
#include <cstddef>

namespace fitra::vmt {

namespace {

constexpr float kPi = 3.14159265358979323846f;

inline float wrap_pi(float x) {
    while (x >  kPi) x -= 2.0f * kPi;
    while (x < -kPi) x += 2.0f * kPi;
    return x;
}

inline float rad_to_deg(float r) { return r * (180.0f / kPi); }

}  // namespace

const char* status_name(AutoAlignmentStatus s) {
    switch (s) {
        case AutoAlignmentStatus::Ok:               return "ok";
        case AutoAlignmentStatus::NoHmd:            return "no_hmd";
        case AutoAlignmentStatus::StaleHmd:         return "stale_hmd";
        case AutoAlignmentStatus::NotEnoughSamples: return "not_enough_samples";
        case AutoAlignmentStatus::Degenerate:       return "degenerate";
    }
    return "unknown";
}

float yaw_from_vmt_quat(const VmtQuat& q) {
    // Standard yaw extraction for a Y-up quaternion in xyzw order:
    //   yaw = atan2(2(w·y + x·z), 1 − 2(y² + z²))
    const float sin_y_cos_p = 2.0f * (q.w * q.y + q.x * q.z);
    const float cos_y_cos_p = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    return std::atan2(sin_y_cos_p, cos_y_cos_p);
}

AutoAlignmentResult solve_tpose(const HmdPose& hmd,
                                const VmtPos&  chest_vmt_pos,
                                const VmtQuat& chest_vmt_quat_xyzw) {
    AutoAlignmentResult r;

    if (!hmd.valid) {
        r.status = AutoAlignmentStatus::NoHmd;
        r.err    = "hmd valid=false";
        return r;
    }

    const float chest_yaw = yaw_from_vmt_quat(chest_vmt_quat_xyzw);
    const float hmd_yaw   = yaw_from_vmt_quat(VmtQuat{hmd.qx, hmd.qy, hmd.qz, hmd.qw});
    const float yaw_rad   = wrap_pi(hmd_yaw - chest_yaw);

    // Apply yaw to the chest xz, then derive xz offset.
    // R_y(yaw) in apply_vmt_alignment is [[c, s], [-s, c]] (see vmt_protocol.cpp:39).
    const float c = std::cos(yaw_rad);
    const float s = std::sin(yaw_rad);
    const float cx = chest_vmt_pos.x;
    const float cz = chest_vmt_pos.z;
    const float rotated_x =  c * cx + s * cz;
    const float rotated_z = -s * cx + c * cz;

    r.alignment.x       = hmd.x - rotated_x;
    r.alignment.y       = 0.0f;  // intentionally untouched; see header docs
    r.alignment.z       = hmd.z - rotated_z;
    r.alignment.yaw_deg = rad_to_deg(yaw_rad);
    r.n_samples         = 1;
    r.residual_m        = 0.0f;
    r.status            = AutoAlignmentStatus::Ok;
    return r;
}

AutoAlignmentResult solve_motion(const std::vector<MotionSample>& samples,
                                 int min_samples) {
    AutoAlignmentResult r;
    const int N = static_cast<int>(samples.size());
    if (N < min_samples) {
        r.status = AutoAlignmentStatus::NotEnoughSamples;
        r.err    = "need >= " + std::to_string(min_samples)
                   + " samples, got " + std::to_string(N);
        r.n_samples = N;
        return r;
    }

    // Centroids.
    double mu_hx = 0.0, mu_hz = 0.0, mu_cx = 0.0, mu_cz = 0.0;
    for (const auto& s : samples) {
        mu_hx += s.hmd_x;   mu_hz += s.hmd_z;
        mu_cx += s.chest_x; mu_cz += s.chest_z;
    }
    mu_hx /= N; mu_hz /= N; mu_cx /= N; mu_cz /= N;

    // Cross-covariance H = Σ (p_c − μ_c)(p_h − μ_h)^T  ∈ R^{2×2}.
    // (p_c as 2×1 column, p_h^T as 1×2 row → 2×2 outer product.)
    double H00 = 0.0, H01 = 0.0, H10 = 0.0, H11 = 0.0;
    for (const auto& s : samples) {
        const double dcx = s.chest_x - mu_cx;
        const double dcz = s.chest_z - mu_cz;
        const double dhx = s.hmd_x   - mu_hx;
        const double dhz = s.hmd_z   - mu_hz;
        H00 += dcx * dhx;
        H01 += dcx * dhz;
        H10 += dcz * dhx;
        H11 += dcz * dhz;
    }

    cv::Mat H = (cv::Mat_<double>(2, 2) << H00, H01, H10, H11);
    cv::Mat S_vec, U, Vt;
    cv::SVD::compute(H, S_vec, U, Vt, cv::SVD::FULL_UV);

    const double s0 = S_vec.at<double>(0);
    const double s1 = S_vec.at<double>(1);
    if (s0 <= 0.0 || s1 < 1e-3 * s0) {
        r.status = AutoAlignmentStatus::Degenerate;
        r.err    = "motion trajectory is too collinear (σ1/σ0 < 1e-3)";
        r.n_samples = N;
        return r;
    }

    cv::Mat V = Vt.t();
    cv::Mat VU = V * U.t();
    const double det = cv::determinant(VU);
    cv::Mat D = (cv::Mat_<double>(2, 2) << 1.0, 0.0, 0.0, det >= 0.0 ? 1.0 : -1.0);
    cv::Mat R = V * D * U.t();

    const double r00 = R.at<double>(0, 0);
    const double r01 = R.at<double>(0, 1);
    const double r10 = R.at<double>(1, 0);
    // yaw extraction: R has the form [[c, s], [-s, c]] (VMT convention,
    // see apply_vmt_alignment). Either r01 or -r10 recovers sin(yaw).
    const double yaw_rad = std::atan2(r01, r00);

    // Translation: t = μ_h − R · μ_c.
    const double tx = mu_hx - (r00 * mu_cx + r01 * mu_cz);
    const double tz = mu_hz - (r10 * mu_cx + r00 * mu_cz);  // r11 == r00

    // Mean residual ‖R·p_c + t − p_h‖.
    double sum_resid = 0.0;
    for (const auto& s : samples) {
        const double pred_x = r00 * s.chest_x + r01 * s.chest_z + tx;
        const double pred_z = r10 * s.chest_x + r00 * s.chest_z + tz;
        const double dx = pred_x - s.hmd_x;
        const double dz = pred_z - s.hmd_z;
        sum_resid += std::sqrt(dx * dx + dz * dz);
    }

    r.alignment.x       = static_cast<float>(tx);
    r.alignment.y       = 0.0f;
    r.alignment.z       = static_cast<float>(tz);
    r.alignment.yaw_deg = rad_to_deg(static_cast<float>(yaw_rad));
    r.residual_m        = static_cast<float>(sum_resid / N);
    r.n_samples         = N;
    r.status            = AutoAlignmentStatus::Ok;
    return r;
}

}  // namespace fitra::vmt
