// test_vmt_protocol — verify the world → VMT Driver coordinate transform and
// the TrackerRole → VMT index mapping. The goldens are derived from the
// SteamVR "x-right, y-up, z-back, RH" Driver frame.

#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

#include "vmt/vmt_protocol.hpp"
#include "slimevr/tracker_extract.hpp"
#include "geom/world_convention.hpp"

namespace {

void expect_near(float got, float want, float tol, const std::string& label) {
    if (std::fabs(got - want) > tol) {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "%s: got=%.6f want=%.6f tol=%g",
                      label.c_str(), got, want, tol);
        throw std::runtime_error(buf);
    }
}

void test_pos_cardinals() {
    using fitra::vmt::world_pos_to_vmt;
    // world +X (right)   → vmt +X (right)
    auto x = world_pos_to_vmt(1.0f, 0.0f, 0.0f);
    expect_near(x.x,  1.0f, 1e-6f, "pos +X.x");
    expect_near(x.y,  0.0f, 1e-6f, "pos +X.y");
    expect_near(x.z,  0.0f, 1e-6f, "pos +X.z");
    // world +Y (forward) → vmt -Z (forward in SteamVR Driver is -Z)
    auto y = world_pos_to_vmt(0.0f, 1.0f, 0.0f);
    expect_near(y.x,  0.0f, 1e-6f, "pos +Y.x");
    expect_near(y.y,  0.0f, 1e-6f, "pos +Y.y");
    expect_near(y.z, -1.0f, 1e-6f, "pos +Y.z");
    // world +Z (up)      → vmt +Y (up)
    auto z = world_pos_to_vmt(0.0f, 0.0f, 1.0f);
    expect_near(z.x,  0.0f, 1e-6f, "pos +Z.x");
    expect_near(z.y,  1.0f, 1e-6f, "pos +Z.y");
    expect_near(z.z,  0.0f, 1e-6f, "pos +Z.z");
}

// Regression net: the wire position helper must agree byte-for-byte with the
// single geom::fitra_to_vmt_basis() (the one source of truth). If someone
// edits one without the other, this fails. Checks arbitrary (non-cardinal)
// coordinates so it pins the full basis, not just the axes.
void test_pos_matches_geom_basis() {
    using fitra::vmt::world_pos_to_vmt;
    const double pts[][3] = {
        {1.3, -2.1, 0.7}, {-0.4, 0.9, -1.6}, {2.5, 2.5, 2.5}, {0.0, -3.3, 1.1}};
    for (const auto& p : pts) {
        auto wire = world_pos_to_vmt(static_cast<float>(p[0]),
                                     static_cast<float>(p[1]),
                                     static_cast<float>(p[2]));
        fitra::geom::Point3<fitra::geom::frame::FitraWorld> pf(p[0], p[1], p[2]);
        auto pv = fitra::geom::fitra_to_vmt_basis() * pf;
        expect_near(wire.x, static_cast<float>(pv.v[0]), 1e-5f, "geom basis pos.x");
        expect_near(wire.y, static_cast<float>(pv.v[1]), 1e-5f, "geom basis pos.y");
        expect_near(wire.z, static_cast<float>(pv.v[2]), 1e-5f, "geom basis pos.z");
    }
}

void test_quat_cardinals() {
    using fitra::vmt::world_quat_to_vmt;
    // identity → identity
    {
        auto q = world_quat_to_vmt(1.0f, 0.0f, 0.0f, 0.0f);
        expect_near(q.x, 0.0f, 1e-6f, "quat identity.x");
        expect_near(q.y, 0.0f, 1e-6f, "quat identity.y");
        expect_near(q.z, 0.0f, 1e-6f, "quat identity.z");
        expect_near(q.w, 1.0f, 1e-6f, "quat identity.w");
    }
    // world +X 90° (qwxyz = (k, k, 0, 0), k = sqrt(2)/2)
    //   → vmt wire xyzw = (qx, qz, -qy, qw) = (k, 0, 0, k)
    {
        const float k = std::sqrt(2.0f) / 2.0f;
        auto q = world_quat_to_vmt(k, k, 0.0f, 0.0f);
        expect_near(q.x,  k,    1e-6f, "quat +X90.x");
        expect_near(q.y,  0.0f, 1e-6f, "quat +X90.y");
        expect_near(q.z,  0.0f, 1e-6f, "quat +X90.z");
        expect_near(q.w,  k,    1e-6f, "quat +X90.w");
    }
    // world +Y 90° (qwxyz = (k, 0, k, 0))
    //   → vmt wire xyzw = (0, 0, -k, k)
    {
        const float k = std::sqrt(2.0f) / 2.0f;
        auto q = world_quat_to_vmt(k, 0.0f, k, 0.0f);
        expect_near(q.x,  0.0f, 1e-6f, "quat +Y90.x");
        expect_near(q.y,  0.0f, 1e-6f, "quat +Y90.y");
        expect_near(q.z, -k,    1e-6f, "quat +Y90.z");
        expect_near(q.w,  k,    1e-6f, "quat +Y90.w");
    }
    // world +Z 90° (qwxyz = (k, 0, 0, k))
    //   → vmt wire xyzw = (0, k, 0, k)
    {
        const float k = std::sqrt(2.0f) / 2.0f;
        auto q = world_quat_to_vmt(k, 0.0f, 0.0f, k);
        expect_near(q.x,  0.0f, 1e-6f, "quat +Z90.x");
        expect_near(q.y,  k,    1e-6f, "quat +Z90.y");
        expect_near(q.z,  0.0f, 1e-6f, "quat +Z90.z");
        expect_near(q.w,  k,    1e-6f, "quat +Z90.w");
    }
}

void test_index_mapping() {
    using fitra::vmt::vmt_index_for;
    using fitra::slimevr::TrackerRole;
    if (vmt_index_for(TrackerRole::LeftUpperArm)  != 0) throw std::runtime_error("LeftUpperArm != 0");
    if (vmt_index_for(TrackerRole::RightUpperArm) != 1) throw std::runtime_error("RightUpperArm != 1");
    if (vmt_index_for(TrackerRole::Chest)         != 2) throw std::runtime_error("Chest != 2");
    if (vmt_index_for(TrackerRole::Waist)         != 3) throw std::runtime_error("Waist != 3");
    if (vmt_index_for(TrackerRole::LeftUpperLeg)  != 4) throw std::runtime_error("LeftUpperLeg != 4");
    if (vmt_index_for(TrackerRole::RightUpperLeg) != 5) throw std::runtime_error("RightUpperLeg != 5");
    if (vmt_index_for(TrackerRole::LeftLowerLeg)  != 6) throw std::runtime_error("LeftLowerLeg != 6");
    if (vmt_index_for(TrackerRole::RightLowerLeg) != 7) throw std::runtime_error("RightLowerLeg != 7");
    if (vmt_index_for(TrackerRole::LeftFoot)      != 8) throw std::runtime_error("LeftFoot != 8");
    if (vmt_index_for(TrackerRole::RightFoot)     != 9) throw std::runtime_error("RightFoot != 9");
    if (fitra::slimevr::kTrackerCount             != 10) throw std::runtime_error("kTrackerCount != 10");
}

void test_index_base_mapping() {
    using fitra::vmt::vmt_index_for;
    using fitra::slimevr::TrackerRole;
    if (vmt_index_for(TrackerRole::LeftUpperArm, 10)  != 10) throw std::runtime_error("LeftUpperArm base 10 != 10");
    if (vmt_index_for(TrackerRole::RightUpperArm, 10) != 11) throw std::runtime_error("RightUpperArm base 10 != 11");
    if (vmt_index_for(TrackerRole::Chest, 10)         != 12) throw std::runtime_error("Chest base 10 != 12");
    if (vmt_index_for(TrackerRole::RightFoot, 10)     != 19) throw std::runtime_error("RightFoot base 10 != 19");
}

void test_alignment_identity() {
    fitra::vmt::VmtPos pos{1.0f, 2.0f, 3.0f};
    fitra::vmt::VmtQuat quat{0.0f, 0.0f, 0.0f, 1.0f};
    fitra::vmt::apply_vmt_alignment(pos, quat, fitra::vmt::VmtAlignment{});
    expect_near(pos.x, 1.0f, 1e-6f, "align identity.pos.x");
    expect_near(pos.y, 2.0f, 1e-6f, "align identity.pos.y");
    expect_near(pos.z, 3.0f, 1e-6f, "align identity.pos.z");
    expect_near(quat.x, 0.0f, 1e-6f, "align identity.quat.x");
    expect_near(quat.y, 0.0f, 1e-6f, "align identity.quat.y");
    expect_near(quat.z, 0.0f, 1e-6f, "align identity.quat.z");
    expect_near(quat.w, 1.0f, 1e-6f, "align identity.quat.w");
}

void test_alignment_translation() {
    fitra::vmt::VmtPos pos{1.0f, 2.0f, 3.0f};
    fitra::vmt::VmtQuat quat{0.0f, 0.0f, 0.0f, 1.0f};
    fitra::vmt::VmtAlignment a;
    a.x = 0.5f;
    a.y = -1.0f;
    a.z = 2.0f;
    fitra::vmt::apply_vmt_alignment(pos, quat, a);
    expect_near(pos.x, 1.5f, 1e-6f, "align translation.pos.x");
    expect_near(pos.y, 1.0f, 1e-6f, "align translation.pos.y");
    expect_near(pos.z, 5.0f, 1e-6f, "align translation.pos.z");
}

void test_alignment_yaw_position() {
    fitra::vmt::VmtPos pos{1.0f, 0.0f, 0.0f};
    fitra::vmt::VmtQuat quat{0.0f, 0.0f, 0.0f, 1.0f};
    fitra::vmt::VmtAlignment a;
    a.yaw_deg = 90.0f;
    fitra::vmt::apply_vmt_alignment(pos, quat, a);
    const float k = std::sqrt(2.0f) / 2.0f;
    expect_near(pos.x,  0.0f, 1e-5f, "align yaw.pos.x");
    expect_near(pos.y,  0.0f, 1e-5f, "align yaw.pos.y");
    expect_near(pos.z, -1.0f, 1e-5f, "align yaw.pos.z");
    expect_near(quat.x, 0.0f, 1e-5f, "align yaw.quat.x");
    expect_near(quat.y, k,    1e-5f, "align yaw.quat.y");
    expect_near(quat.z, 0.0f, 1e-5f, "align yaw.quat.z");
    expect_near(quat.w, k,    1e-5f, "align yaw.quat.w");
}

void test_alignment_yaw_quat_left_multiply() {
    const float k = std::sqrt(2.0f) / 2.0f;
    fitra::vmt::VmtPos pos{0.0f, 0.0f, 0.0f};
    // Input is +X 90 degrees in VMT xyzw.
    fitra::vmt::VmtQuat quat{k, 0.0f, 0.0f, k};
    fitra::vmt::VmtAlignment a;
    a.yaw_deg = 90.0f;
    fitra::vmt::apply_vmt_alignment(pos, quat, a);
    expect_near(quat.x,  0.5f, 1e-5f, "align yaw left-mul.quat.x");
    expect_near(quat.y,  0.5f, 1e-5f, "align yaw left-mul.quat.y");
    expect_near(quat.z, -0.5f, 1e-5f, "align yaw left-mul.quat.z");
    expect_near(quat.w,  0.5f, 1e-5f, "align yaw left-mul.quat.w");
}

}  // namespace

int main() {
    try {
        test_pos_cardinals();
        test_pos_matches_geom_basis();
        test_quat_cardinals();
        test_index_mapping();
        test_index_base_mapping();
        test_alignment_identity();
        test_alignment_translation();
        test_alignment_yaw_position();
        test_alignment_yaw_quat_left_multiply();
        std::puts("test_vmt_protocol ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_vmt_protocol failed: %s\n", e.what());
        return 1;
    }
}
