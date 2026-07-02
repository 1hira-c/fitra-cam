#include "lift/rigid_fit.hpp"

#include <algorithm>
#include <cmath>

namespace fitra::lift {

RigidTemplate RigidTemplate::from_distances(double d01, double d02, double d12,
                                            double min_height_m) {
    RigidTemplate t;
    if (d01 <= 1.0e-6 || d02 <= 1.0e-6 || d12 <= 1.0e-6) return t;
    // Triangle inequality (strict, with a small margin so a barely-degenerate
    // triangle is rejected rather than producing a near-collinear template).
    if (d01 + d02 <= d12 || d01 + d12 <= d02 || d02 + d12 <= d01) return t;

    // p0 at origin, p1 on +x. p2 from the law of cosines at p0.
    const double cos0 = (d01 * d01 + d02 * d02 - d12 * d12) / (2.0 * d01 * d02);
    const double c = std::clamp(cos0, -1.0, 1.0);
    const double s = std::sqrt(std::max(0.0, 1.0 - c * c));
    t.pts[0] = {0.0, 0.0, 0.0};
    t.pts[1] = {d01, 0.0, 0.0};
    t.pts[2] = {d02 * c, d02 * s, 0.0};

    // Perpendicular height of p0 from the p1-p2 base line = 2·area / base. Below
    // the threshold the three points are effectively collinear and the fit
    // cannot pin the rotation about the long axis.
    const double area = 0.5 * d01 * d02 * s;  // ½·|p0p1|·|p0p2|·sin(∠0)
    const double height = (d12 > 1.0e-9) ? (2.0 * area / d12) : 0.0;
    t.valid = height >= min_height_m;
    return t;
}

bool fit_rigid_triangle(const RigidTemplate& templ,
                        const std::array<cv::Vec3d, 3>& measured,
                        const std::array<double, 3>& weights,
                        std::array<cv::Vec3d, 3>& out) {
    if (!templ.valid) return false;
    const double W = weights[0] + weights[1] + weights[2];
    if (W < 1.0e-9) return false;
    for (double w : weights)
        if (w < 0.0) return false;

    // Weighted centroids.
    cv::Vec3d cs{0, 0, 0}, cm{0, 0, 0};
    for (int i = 0; i < 3; ++i) {
        cs += weights[i] * templ.pts[i];
        cm += weights[i] * measured[i];
    }
    cs *= (1.0 / W);
    cm *= (1.0 / W);

    // Cross-covariance H = Σ wᵢ (templ_i − cs)(measured_i − cm)ᵀ.
    cv::Matx33d H = cv::Matx33d::zeros();
    for (int i = 0; i < 3; ++i) {
        const cv::Vec3d a = templ.pts[i] - cs;   // source
        const cv::Vec3d b = measured[i] - cm;    // target
        for (int r = 0; r < 3; ++r)
            for (int cc = 0; cc < 3; ++cc)
                H(r, cc) += weights[i] * a[r] * b[cc];
    }

    cv::Matx33d U, Vt;
    cv::Vec3d w;
    cv::SVD::compute(cv::Mat(H), w, U, Vt);
    // A collapsed covariance (all singular values ~0) means no usable structure.
    if (w[0] < 1.0e-12) return false;
    // The measured points must span a plane (rank >= 2). If they are (nearly)
    // collinear the second singular value vanishes, the template's non-collinear
    // rows leave the rotation about the collinear axis unconstrained, and the
    // SVD's arbitrary null-space choice flips the out-of-plane direction — the
    // fitted triangle then jumps instead of denoising. Reject (caller falls
    // back). Relative gate so it is scale-invariant; a healthy pelvis/girdle
    // triangle has w[1]/w[0] ~ 0.1-0.2, far above this.
    if (w[1] < 1.0e-6 * w[0]) return false;

    // R = V · diag(1,1, sign(det(V·Uᵀ))) · Uᵀ, the sign term forbidding a
    // reflection (Kabsch). H = U·Σ·Vᵀ, so V = Vtᵀ, U = U.
    const cv::Matx33d V = Vt.t();
    const cv::Matx33d Ut = U.t();
    const double d = cv::determinant(V * Ut);
    cv::Matx33d D = cv::Matx33d::eye();
    D(2, 2) = (d < 0.0) ? -1.0 : 1.0;
    const cv::Matx33d R = V * D * Ut;
    const cv::Vec3d t = cm - R * cs;

    for (int i = 0; i < 3; ++i) out[i] = R * templ.pts[i] + t;
    return true;
}

bool apply_segment_rigid_fit(infer::Skeleton3D& skel,
                             const TriangulatedSkeleton& tri,
                             const RigidTemplate& templ,
                             const std::array<int, 3>& idx) {
    if (!templ.valid) return false;
    const std::size_t n = skel.joints.size();
    for (int i : idx)
        if (i < 0 || static_cast<std::size_t>(i) >= n || !skel.joints[i].valid) return false;
    std::array<cv::Vec3d, 3> measured, out;
    std::array<double, 3> w;
    for (int k = 0; k < 3; ++k) {
        const auto& j = skel.joints[idx[k]];
        measured[k] = {j.x, j.y, j.z};
        const double score = std::max(1.0e-3, static_cast<double>(j.score));
        const int vc = std::max(1, tri.view_count[idx[k]]);
        w[k] = score * static_cast<double>(vc);
    }
    if (!fit_rigid_triangle(templ, measured, w, out)) return false;
    for (int k = 0; k < 3; ++k) {
        auto& j = skel.joints[idx[k]];
        j.x = static_cast<float>(out[k][0]);
        j.y = static_cast<float>(out[k][1]);
        j.z = static_cast<float>(out[k][2]);
        j.valid = true;
    }
    return true;
}

void apply_spine_coupling(infer::Skeleton3D& skel, int hip_idx, int neck_idx,
                          const std::array<int, 3>& girdle_idx,
                          double spine_len, double tol) {
    if (spine_len <= 1.0e-6) return;
    const int n = static_cast<int>(skel.joints.size());
    if (hip_idx < 0 || hip_idx >= n || neck_idx < 0 || neck_idx >= n) return;
    const auto& hip = skel.joints[hip_idx];
    const auto& neck = skel.joints[neck_idx];
    if (!hip.valid || !neck.valid) return;
    const cv::Vec3d h{hip.x, hip.y, hip.z};
    const cv::Vec3d nk{neck.x, neck.y, neck.z};
    const cv::Vec3d axis = nk - h;
    const double d = cv::norm(axis);
    if (d < 1.0e-9) return;
    const double lo = spine_len * (1.0 - tol);
    const double hi = spine_len * (1.0 + tol);
    const double d_clamped = std::clamp(d, lo, hi);
    if (std::abs(d_clamped - d) < 1.0e-9) return;  // inside the band: neck free
    const cv::Vec3d n_new = h + axis * (d_clamped / d);
    const cv::Vec3d shift = n_new - nk;
    for (int i : girdle_idx) {
        if (i < 0 || i >= n) continue;
        auto& j = skel.joints[i];
        if (!j.valid) continue;
        j.x += static_cast<float>(shift[0]);
        j.y += static_cast<float>(shift[1]);
        j.z += static_cast<float>(shift[2]);
    }
}

}  // namespace fitra::lift
