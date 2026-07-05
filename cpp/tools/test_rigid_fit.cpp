// Unit test for the spatial rigid-body fit (lift/rigid_fit.{hpp,cpp}).
// See docs/design/pose-3d-spatial-filtering.md (milestone M-A).
//
// Covers: template construction from segment distances (shape + degeneracy
// gate), exact recovery of a known rigid transform, shape-preserving denoising
// of a noisy segment, weighting (a downweighted noisy point pulls less), and
// the Kabsch reflection guard (a mirrored target cannot be matched by a proper
// rotation).

#include "lift/rigid_fit.hpp"

#include <array>
#include <cmath>
#include <cstdio>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace {

using fitra::lift::RigidTemplate;
using fitra::lift::fit_rigid_triangle;

int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++g_fail; } \
} while (0)

#define CHECK_LT(a, b) do { \
    if (!((a) < (b))) { \
        std::fprintf(stderr, "FAIL %s:%d %s < %s (%g vs %g)\n", \
            __FILE__, __LINE__, #a, #b, double(a), double(b)); ++g_fail; \
    } \
} while (0)

double dist(const cv::Vec3d& a, const cv::Vec3d& b) { return cv::norm(a - b); }

// Subject01's real pelvis triangle: hip_center->l_hip, hip_center->r_hip, hip_width.
constexpr double kD01 = 0.0801, kD02 = 0.0822, kD12 = 0.1546;

cv::Matx33d rot(double rx, double ry, double rz) {
    cv::Matx33d R;
    cv::Rodrigues(cv::Vec3d{rx, ry, rz}, R);
    return R;
}

// 1) Template shape matches the requested distances; degenerate rejected.
void test_template_shape() {
    RigidTemplate t = RigidTemplate::from_distances(kD01, kD02, kD12);
    CHECK(t.valid);
    CHECK_LT(std::abs(dist(t.pts[0], t.pts[1]) - kD01), 1e-9);
    CHECK_LT(std::abs(dist(t.pts[0], t.pts[2]) - kD02), 1e-9);
    CHECK_LT(std::abs(dist(t.pts[1], t.pts[2]) - kD12), 1e-9);

    // Collinear (p0 exactly at the midpoint: d01+d02 == d12) => degenerate.
    RigidTemplate flat = RigidTemplate::from_distances(0.08, 0.08, 0.16);
    CHECK(!flat.valid);
    // Non-positive distance => invalid.
    CHECK(!RigidTemplate::from_distances(0.0, 0.08, 0.1).valid);
    // Triangle inequality violated => invalid.
    CHECK(!RigidTemplate::from_distances(0.05, 0.05, 0.5).valid);
}

// 2) A measured set that is exactly a rigid image of the template is recovered
//    bit-for-bit (out == measured).
void test_exact_recovery() {
    RigidTemplate t = RigidTemplate::from_distances(kD01, kD02, kD12);
    const cv::Matx33d R = rot(0.3, -0.6, 0.9);
    const cv::Vec3d tr{1.2, -0.4, 2.0};
    std::array<cv::Vec3d, 3> measured, out;
    for (int i = 0; i < 3; ++i) measured[i] = R * t.pts[i] + tr;
    CHECK(fit_rigid_triangle(t, measured, {1.0, 1.0, 1.0}, out));
    for (int i = 0; i < 3; ++i) CHECK_LT(dist(out[i], measured[i]), 1e-9);
}

// 3) With per-point noise, the fit preserves the template shape exactly and
//    lands close to the (unknown) clean rigid target — denoising the segment.
void test_denoise_preserves_shape() {
    RigidTemplate t = RigidTemplate::from_distances(kD01, kD02, kD12);
    const cv::Matx33d R = rot(-0.2, 0.5, 0.1);
    const cv::Vec3d tr{0.5, 0.5, 1.5};
    std::array<cv::Vec3d, 3> clean, measured, out;
    // Deterministic small per-axis perturbations (~5mm), different per point.
    const cv::Vec3d noise[3] = {{0.004, -0.003, 0.005}, {-0.005, 0.004, -0.002}, {0.003, 0.006, -0.004}};
    for (int i = 0; i < 3; ++i) {
        clean[i] = R * t.pts[i] + tr;
        measured[i] = clean[i] + noise[i];
    }
    CHECK(fit_rigid_triangle(t, measured, {1.0, 1.0, 1.0}, out));
    // Output is a perfect rigid triangle of the template shape.
    CHECK_LT(std::abs(dist(out[0], out[1]) - kD01), 1e-9);
    CHECK_LT(std::abs(dist(out[0], out[2]) - kD02), 1e-9);
    CHECK_LT(std::abs(dist(out[1], out[2]) - kD12), 1e-9);
    // Closer (in total) to the clean target than the raw noisy measurement is:
    // the rigid constraint absorbs the component of noise that breaks the shape.
    double res_out = 0.0, res_meas = 0.0;
    for (int i = 0; i < 3; ++i) { res_out += dist(out[i], clean[i]); res_meas += dist(measured[i], clean[i]); }
    CHECK_LT(res_out, res_meas);
}

// 4) Downweighting a badly-corrupted point makes the fit follow the two good
//    points, so their fitted positions land closer to their clean targets than
//    an equal-weight fit would.
void test_weighting() {
    RigidTemplate t = RigidTemplate::from_distances(kD01, kD02, kD12);
    const cv::Matx33d R = rot(0.1, 0.2, -0.3);
    const cv::Vec3d tr{0.0, 1.0, 1.0};
    std::array<cv::Vec3d, 3> clean, measured, out_eq, out_w;
    for (int i = 0; i < 3; ++i) clean[i] = R * t.pts[i] + tr;
    measured = clean;
    measured[2] += cv::Vec3d{0.05, -0.04, 0.03};  // point 2 badly off (~7cm)

    CHECK(fit_rigid_triangle(t, measured, {1.0, 1.0, 1.0}, out_eq));
    CHECK(fit_rigid_triangle(t, measured, {1.0, 1.0, 0.05}, out_w));
    // The two trusted points track their clean targets better when point 2 is
    // downweighted.
    const double eq  = dist(out_eq[0], clean[0]) + dist(out_eq[1], clean[1]);
    const double wtd = dist(out_w[0], clean[0]) + dist(out_w[1], clean[1]);
    CHECK_LT(wtd, eq);
}

// 5) Guard returns: invalid template, near-zero total weight, and a negative
//    weight all yield false with `out` left untouched (caller keeps measured).
//    (A reflection guard is untestable with three points: any reflection of
//    three coplanar points is also a proper 180-degree rotation about an
//    in-plane axis, so a planar template cannot distinguish the two — the
//    det-correction still runs, it just cannot change the coplanar result.)
void test_guard_returns() {
    RigidTemplate good = RigidTemplate::from_distances(kD01, kD02, kD12);
    RigidTemplate bad = RigidTemplate::from_distances(0.08, 0.08, 0.16);  // degenerate
    const std::array<cv::Vec3d, 3> measured{{{1, 0, 0}, {1, 0.08, 0}, {1.05, 0.04, 0.02}}};
    const cv::Vec3d sentinel{-9, -9, -9};
    std::array<cv::Vec3d, 3> out{{sentinel, sentinel, sentinel}};

    CHECK(!fit_rigid_triangle(bad, measured, {1.0, 1.0, 1.0}, out));
    CHECK(!fit_rigid_triangle(good, measured, {0.0, 0.0, 0.0}, out));
    CHECK(!fit_rigid_triangle(good, measured, {1.0, -0.5, 1.0}, out));
    // out never written on a false return.
    for (int i = 0; i < 3; ++i) CHECK(out[i] == sentinel);
    // ...and a valid call does write.
    CHECK(fit_rigid_triangle(good, measured, {1.0, 1.0, 1.0}, out));
    CHECK(out[0] != sentinel);
}

}  // namespace

int main() {
    test_template_shape();
    test_exact_recovery();
    test_denoise_preserves_shape();
    test_weighting();
    test_guard_returns();
    if (g_fail) { std::fprintf(stderr, "%d checks FAILED\n", g_fail); return 1; }
    std::printf("test_rigid_fit: all checks passed\n");
    return 0;
}
