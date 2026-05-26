// Unit tests for solve_tpose and solve_motion.
//
// All inputs/outputs are in VMT Driver frame (Y-up RH). 2D xz plane Procrustes.

#include "vmt/auto_alignment.hpp"

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

namespace {

using fitra::vmt::AutoAlignmentResult;
using fitra::vmt::AutoAlignmentStatus;
using fitra::vmt::HmdPose;
using fitra::vmt::MotionSample;
using fitra::vmt::solve_motion;
using fitra::vmt::solve_tpose;
using fitra::vmt::VmtPos;
using fitra::vmt::VmtQuat;

constexpr float kPi = 3.14159265358979323846f;

int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++g_fail; } \
} while (0)

#define CHECK_NEAR(a, b, eps) do { \
    if (std::fabs(double((a)) - double((b))) > (eps)) { \
        std::fprintf(stderr, "FAIL %s:%d %s != %s (%g vs %g)\n", \
            __FILE__, __LINE__, #a, #b, double(a), double(b)); \
        ++g_fail; \
    } \
} while (0)

VmtQuat yaw_quat(float yaw_rad) {
    // Y-axis rotation quaternion (xyzw).
    const float h = yaw_rad * 0.5f;
    return {0.0f, std::sin(h), 0.0f, std::cos(h)};
}

HmdPose make_hmd(float x, float z, float yaw_rad, bool valid = true) {
    HmdPose h;
    h.valid = valid;
    h.x = x; h.y = 1.7f; h.z = z;
    const VmtQuat q = yaw_quat(yaw_rad);
    h.qx = q.x; h.qy = q.y; h.qz = q.z; h.qw = q.w;
    return h;
}

// Apply a forward (R_y · p + t) to a chest point, mirroring apply_vmt_alignment.
void apply(float yaw_rad, float tx, float tz, float cx, float cz,
           float& out_x, float& out_z) {
    const float c = std::cos(yaw_rad);
    const float s = std::sin(yaw_rad);
    out_x =  c * cx + s * cz + tx;
    out_z = -s * cx + c * cz + tz;
}

// 1) Identity: chest at the HMD with same yaw → alignment ≈ 0.
void test_tpose_identity() {
    HmdPose hmd = make_hmd(0.5f, -0.3f, 0.0f);
    VmtPos chest{0.5f, 0.2f, -0.3f};
    VmtQuat q = yaw_quat(0.0f);
    auto r = solve_tpose(hmd, chest, q);
    CHECK(r.status == AutoAlignmentStatus::Ok);
    CHECK_NEAR(r.alignment.x, 0.0, 1e-5);
    CHECK_NEAR(r.alignment.z, 0.0, 1e-5);
    CHECK_NEAR(r.alignment.y, 0.0, 1e-9);
    CHECK_NEAR(r.alignment.yaw_deg, 0.0, 1e-4);
}

// 2) Pure xz translation, same yaw → alignment captures the offset, yaw=0.
void test_tpose_pure_translation() {
    HmdPose hmd = make_hmd(1.0f, 2.0f, 0.0f);
    VmtPos chest{0.5f, 0.2f, 0.5f};
    auto r = solve_tpose(hmd, chest, yaw_quat(0.0f));
    CHECK(r.status == AutoAlignmentStatus::Ok);
    CHECK_NEAR(r.alignment.x, 0.5, 1e-5);
    CHECK_NEAR(r.alignment.z, 1.5, 1e-5);
    CHECK_NEAR(r.alignment.yaw_deg, 0.0, 1e-4);
}

// 3) Pure yaw, chest at origin → alignment yaw_deg = expected; xz ≈ 0.
void test_tpose_pure_yaw() {
    float yaw_rad = 30.0f * kPi / 180.0f;
    HmdPose hmd = make_hmd(0.0f, 0.0f, yaw_rad);
    VmtPos chest{0.0f, 0.2f, 0.0f};
    auto r = solve_tpose(hmd, chest, yaw_quat(0.0f));
    CHECK(r.status == AutoAlignmentStatus::Ok);
    CHECK_NEAR(r.alignment.x, 0.0, 1e-5);
    CHECK_NEAR(r.alignment.z, 0.0, 1e-5);
    CHECK_NEAR(r.alignment.yaw_deg, 30.0, 1e-3);
}

// 4) HMD invalid → NoHmd status, no crash.
void test_tpose_no_hmd() {
    HmdPose hmd = make_hmd(0.0f, 0.0f, 0.0f, /*valid*/ false);
    VmtPos chest{0.5f, 0.2f, 0.5f};
    auto r = solve_tpose(hmd, chest, yaw_quat(0.0f));
    CHECK(r.status == AutoAlignmentStatus::NoHmd);
}

// 5) Motion identity (chest == HMD): yaw=0, t=0. Sample points are spread
//    across the xz plane so the SVD is non-degenerate.
void test_motion_identity() {
    std::vector<MotionSample> s;
    for (int i = 0; i < 20; ++i) {
        const float angle = i * 0.31f;
        const float r = 0.4f + 0.1f * (i % 3);
        const float cx = r * std::cos(angle);
        const float cz = r * std::sin(angle);
        s.push_back({cx, cz, cx, cz});
    }
    auto r = solve_motion(s);
    CHECK(r.status == AutoAlignmentStatus::Ok);
    CHECK_NEAR(r.alignment.yaw_deg, 0.0, 1e-3);
    CHECK_NEAR(r.alignment.x, 0.0, 1e-4);
    CHECK_NEAR(r.alignment.z, 0.0, 1e-4);
    CHECK(r.residual_m < 1e-4f);
}

// 6) Motion with yaw + translation. We synthesise HMD from chest by applying
//    a known transform, then check the solver recovers it.
void test_motion_yaw_and_translation() {
    const float yaw_rad = 25.0f * kPi / 180.0f;
    const float tx = 0.7f;
    const float tz = -0.4f;
    std::vector<MotionSample> s;
    // Spread chest points around to avoid collinearity.
    for (int i = 0; i < 30; ++i) {
        float angle = i * 0.2f;
        float r = 0.5f + 0.1f * (i % 5);
        float cx = r * std::cos(angle);
        float cz = r * std::sin(angle);
        float hx, hz;
        apply(yaw_rad, tx, tz, cx, cz, hx, hz);
        s.push_back({hx, hz, cx, cz});
    }
    auto r = solve_motion(s);
    CHECK(r.status == AutoAlignmentStatus::Ok);
    CHECK_NEAR(r.alignment.yaw_deg, 25.0, 1e-3);
    CHECK_NEAR(r.alignment.x, tx, 1e-4);
    CHECK_NEAR(r.alignment.z, tz, 1e-4);
    CHECK(r.residual_m < 1e-4f);
}

// 7) Motion with Gaussian noise (σ=0.01m) should still solve, residual small.
void test_motion_noisy() {
    const float yaw_rad = -15.0f * kPi / 180.0f;
    const float tx = 0.3f;
    const float tz = 1.2f;
    std::mt19937 rng(42);
    std::normal_distribution<float> noise(0.0f, 0.01f);
    std::vector<MotionSample> s;
    for (int i = 0; i < 100; ++i) {
        float angle = i * 0.1f;
        float r = 0.4f + 0.2f * std::sin(0.3f * i);
        float cx = r * std::cos(angle);
        float cz = r * std::sin(angle);
        float hx, hz;
        apply(yaw_rad, tx, tz, cx, cz, hx, hz);
        hx += noise(rng); hz += noise(rng);
        s.push_back({hx, hz, cx, cz});
    }
    auto r = solve_motion(s);
    CHECK(r.status == AutoAlignmentStatus::Ok);
    CHECK_NEAR(r.alignment.yaw_deg, -15.0, 0.5);
    CHECK_NEAR(r.alignment.x, tx, 0.02);
    CHECK_NEAR(r.alignment.z, tz, 0.02);
    CHECK(r.residual_m < 0.02f);
}

// 8) Too few samples → NotEnoughSamples. Collinear samples → Degenerate.
void test_motion_edge_cases() {
    std::vector<MotionSample> too_few = {
        {0, 0, 0, 0}, {1, 1, 1, 1},
    };
    auto r1 = solve_motion(too_few);
    CHECK(r1.status == AutoAlignmentStatus::NotEnoughSamples);
    CHECK(r1.n_samples == 2);

    // All HMD and chest points sit on the line z=0 → SVD second singular
    // value is 0 → Degenerate.
    std::vector<MotionSample> collinear;
    for (int i = 0; i < 20; ++i) {
        float t = static_cast<float>(i) * 0.1f;
        collinear.push_back({t, 0.0f, t, 0.0f});
    }
    auto r2 = solve_motion(collinear);
    CHECK(r2.status == AutoAlignmentStatus::Degenerate);
}

}  // namespace

int main() {
    test_tpose_identity();
    test_tpose_pure_translation();
    test_tpose_pure_yaw();
    test_tpose_no_hmd();
    test_motion_identity();
    test_motion_yaw_and_translation();
    test_motion_noisy();
    test_motion_edge_cases();
    if (g_fail) {
        std::fprintf(stderr, "test_auto_alignment: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_auto_alignment: OK\n");
    return 0;
}
