// Unit test for the typed coordinate-frame layer (geom/frames.hpp,
// geom/world_convention.hpp). See docs/design/pose-3d-typed-coordinate-frames.md.
//
// Covers: Transform composition associativity, inverse round-trip, Point3
// transport, and the single fitra<->VMT basis change matching both the legacy
// kVmtWorldToFitra constant and vmt::world_pos_to_vmt's rotation (the regression
// nets that keep the three historical hand-written conversions in agreement).

#include "geom/frames.hpp"
#include "geom/world_convention.hpp"

#include <cmath>
#include <cstdio>

namespace {

using namespace fitra::geom;

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

double mat_diff(const cv::Matx44d& a, const cv::Matx44d& b) {
    double s = 0.0;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) s = std::max(s, std::abs(a(r, c) - b(r, c)));
    return s;
}

// Tag set used purely to exercise the algebra (distinct from the production
// frame tags so this test owns its own little chain A<-B<-C<-D).
struct A {}; struct B {}; struct C {}; struct D {};

Transform<A, B> rand_AB() {
    return Transform<A, B>::from_raw(
        pose_from_pos_quat(0.3, -0.7, 1.1, 0.1, 0.2, 0.3, 0.9));
}
Transform<B, C> rand_BC() {
    return Transform<B, C>::from_raw(
        pose_from_pos_quat(-0.4, 0.2, 0.5, -0.2, 0.4, -0.1, 0.8));
}
Transform<C, D> rand_CD() {
    return Transform<C, D>::from_raw(
        pose_from_pos_quat(0.9, 0.1, -0.3, 0.05, -0.3, 0.2, 0.95));
}

// 1) Composition is associative and the type threads through correctly.
void test_compose_assoc() {
    Transform<A, B> ab = rand_AB();
    Transform<B, C> bc = rand_BC();
    Transform<C, D> cd = rand_CD();
    Transform<A, D> left  = (ab * bc) * cd;   // (A<-B * B<-C) * C<-D
    Transform<A, D> right = ab * (bc * cd);   // A<-B * (B<-C * C<-D)
    CHECK_LT(mat_diff(left.raw(), right.raw()), 1e-12);
}

// 2) inverse() flips the frames and round-trips to identity both ways.
void test_inverse_roundtrip() {
    Transform<A, B> ab = rand_AB();
    Transform<B, A> ba = ab.inverse();
    Transform<A, A> id1 = ab * ba;
    Transform<B, B> id2 = ba * ab;
    CHECK_LT(mat_diff(id1.raw(), cv::Matx44d::eye()), 1e-12);
    CHECK_LT(mat_diff(id2.raw(), cv::Matx44d::eye()), 1e-12);
}

// 3) Point transport: (T_ab * T_bc) * p_c == T_ab * (T_bc * p_c).
void test_point_transport() {
    Transform<A, B> ab = rand_AB();
    Transform<B, C> bc = rand_BC();
    Point3<C> pc(1.0, -2.0, 3.0);
    Point3<A> via_compose = (ab * bc) * pc;
    Point3<A> via_steps   = ab * (bc * pc);
    CHECK_LT(cv::norm(via_compose.v - via_steps.v), 1e-12);
}

// 4) The single basis change matches the legacy kVmtWorldToFitra constant and
//    vmt::world_pos_to_vmt's rotation; the two directions are mutual inverses.
void test_basis_change() {
    // Legacy constant that lived in extrinsic_calib_session.cpp. It was
    // right-multiplied onto a T_cam_vmtworld; its comment notes it maps fitra
    // coords to VMT coords ((x,y,z)->(x,z,-y)) — i.e. it equals fitra_to_vmt.
    const cv::Matx44d kVmtWorldToFitra_legacy{
        1,  0, 0, 0,
        0,  0, 1, 0,
        0, -1, 0, 0,
        0,  0, 0, 1};
    CHECK_LT(mat_diff(fitra_to_vmt_basis().raw(), kVmtWorldToFitra_legacy), 1e-15);

    // fitra_to_vmt applied to a point must equal vmt::world_pos_to_vmt's
    // documented mapping (x, y, z) -> (x, z, -y). The byte-for-byte cross-check
    // against the actual wire helper lives in test_vmt_protocol (M4) to keep
    // this test fitra_lift-only.
    Point3<frame::FitraWorld> pf(1.3, -2.1, 0.7);
    Point3<frame::VmtWorld> pv = fitra_to_vmt_basis() * pf;
    CHECK_LT(std::abs(pv.v[0] - pf.v[0]), 1e-12);   // x
    CHECK_LT(std::abs(pv.v[1] - pf.v[2]), 1e-12);   // z
    CHECK_LT(std::abs(pv.v[2] - (-pf.v[1])), 1e-12);  // -y

    // The two directions are mutual inverses.
    Transform<frame::FitraWorld, frame::FitraWorld> id =
        vmt_to_fitra_basis() * fitra_to_vmt_basis();
    CHECK_LT(mat_diff(id.raw(), cv::Matx44d::eye()), 1e-15);
}

// 5) Re-expressing a Camera<-VmtWorld extrinsic into Camera<-FitraWorld must
//    equal the legacy right-multiply by kVmtWorldToFitra.
void test_extrinsic_reexpress() {
    cv::Matx44d T_cam_vmt_raw = pose_from_pos_quat(0.5, 0.1, 2.0, 0.1, -0.2, 0.3, 0.9);
    Transform<frame::Camera, frame::VmtWorld> T_cam_vmt =
        Transform<frame::Camera, frame::VmtWorld>::from_raw(T_cam_vmt_raw);
    Transform<frame::Camera, frame::FitraWorld> T_cam_fitra =
        T_cam_vmt * fitra_to_vmt_basis();

    const cv::Matx44d kVmtWorldToFitra_legacy{
        1,  0, 0, 0,
        0,  0, 1, 0,
        0, -1, 0, 0,
        0,  0, 0, 1};
    cv::Matx44d legacy = T_cam_vmt_raw * kVmtWorldToFitra_legacy;
    CHECK_LT(mat_diff(T_cam_fitra.raw(), legacy), 1e-15);
}

}  // namespace

int main() {
    test_compose_assoc();
    test_inverse_roundtrip();
    test_point_transport();
    test_basis_change();
    test_extrinsic_reexpress();
    if (g_fail) {
        std::fprintf(stderr, "test_geom_frames: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_geom_frames: OK\n");
    return 0;
}
