#include "lift/rigid_fit.hpp"

#include <algorithm>
#include <cmath>

namespace fitra::lift {

namespace {

double det3(const cv::Matx33d& m) {
    return m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)) -
           m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
           m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
}

}  // namespace

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

    // R = V · diag(1,1, sign(det(V·Uᵀ))) · Uᵀ, the sign term forbidding a
    // reflection (Kabsch). H = U·Σ·Vᵀ, so V = Vtᵀ, U = U.
    const cv::Matx33d V = Vt.t();
    const cv::Matx33d Ut = U.t();
    const double d = det3(V * Ut);
    cv::Matx33d D = cv::Matx33d::eye();
    D(2, 2) = (d < 0.0) ? -1.0 : 1.0;
    const cv::Matx33d R = V * D * Ut;
    const cv::Vec3d t = cm - R * cs;

    for (int i = 0; i < 3; ++i) out[i] = R * templ.pts[i] + t;
    return true;
}

}  // namespace fitra::lift
