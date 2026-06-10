// Synthetic-data unit test for the controller-marker extrinsic hand-eye solver.
//
// We pick ground-truth per-camera extrinsics (T_cam←world) and per-face mount
// offsets (T_controller←face), generate a sweep of controller poses with real
// rotation diversity, synthesize the marker-as-seen-by-camera observations via
// the forward chain, then check the solver recovers the extrinsics. With no
// noise the recovery must be near-exact; with mm/sub-degree noise the residual
// and recovery error must stay small.

#include "lift/extrinsic_solver.hpp"

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

namespace {

using fitra::lift::CameraExtrinsic;
using fitra::lift::ExtrinsicSample;
using fitra::lift::ExtrinsicSolverOptions;
using fitra::lift::pose_from_pos_quat;
using fitra::lift::rotation_angle_deg;
using fitra::lift::solve_extrinsics;

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

cv::Matx44d rigid(double rx, double ry, double rz, double tx, double ty, double tz) {
    // Build a rotation from an axis-angle-ish Euler (rx,ry,rz radians) via
    // pose_from_pos_quat using a normalized quaternion from small Euler.
    double cx = std::cos(rx / 2), sx = std::sin(rx / 2);
    double cy = std::cos(ry / 2), sy = std::sin(ry / 2);
    double cz = std::cos(rz / 2), sz = std::sin(rz / 2);
    // ZYX intrinsic -> quaternion (w,x,y,z)
    double w = cx * cy * cz + sx * sy * sz;
    double qx = sx * cy * cz - cx * sy * sz;
    double qy = cx * sy * cz + sx * cy * sz;
    double qz = cx * cy * sz - sx * sy * cz;
    return pose_from_pos_quat(tx, ty, tz, qx, qy, qz, w);
}

cv::Matx44d invert_rigid(const cv::Matx44d& T) {
    cv::Matx33d R(T(0,0),T(0,1),T(0,2), T(1,0),T(1,1),T(1,2), T(2,0),T(2,1),T(2,2));
    cv::Vec3d t(T(0,3), T(1,3), T(2,3));
    cv::Matx33d Rt = R.t();
    cv::Vec3d ti = -(Rt * t);
    cv::Matx44d out = cv::Matx44d::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) out(r, c) = Rt(r, c);
        out(r, 3) = ti(r);
    }
    return out;
}

double trans_dist(const cv::Matx44d& a, const cv::Matx44d& b) {
    return std::sqrt((a(0,3)-b(0,3))*(a(0,3)-b(0,3)) +
                     (a(1,3)-b(1,3))*(a(1,3)-b(1,3)) +
                     (a(2,3)-b(2,3))*(a(2,3)-b(2,3)));
}

struct Scene {
    std::vector<cv::Matx44d> T_cam_world_gt;        // per camera
    std::vector<cv::Matx44d> T_controller_face_gt;  // per face
    std::vector<ExtrinsicSample> samples;
};

// Build a scene with `n_cams` cameras, `n_faces` faces, `n_poses` controller
// poses. `pos_noise_m` / `rot_noise_deg` perturb the synthesized PnP A_i.
Scene make_scene(int n_cams, int n_faces, int n_poses,
                 double pos_noise_m, double rot_noise_deg, unsigned seed) {
    Scene sc;
    // Ground-truth camera extrinsics (T_cam←world): spread around a room.
    for (int c = 0; c < n_cams; ++c) {
        double ang = 1.2 * c;
        sc.T_cam_world_gt.push_back(rigid(0.1 * c, ang, -0.2 * c,
                                          0.5 * c, 0.1, 2.0 + 0.3 * c));
    }
    // Ground-truth face mount offsets (T_controller←face): distinct per face.
    for (int f = 0; f < n_faces; ++f) {
        sc.T_controller_face_gt.push_back(
            rigid(0.4 * (f + 1), -0.3 * (f + 1), 0.9 * f,
                  0.03 * (f + 1), -0.02 * f, 0.05));
    }

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> ang(-1.5, 1.5);
    std::uniform_real_distribution<double> pos(-0.4, 0.4);
    std::normal_distribution<double> tn(0.0, pos_noise_m);
    std::normal_distribution<double> rn(0.0, rot_noise_deg * M_PI / 180.0);

    for (int i = 0; i < n_poses; ++i) {
        // Controller pose in world (B_i) with full rotation diversity.
        cv::Matx44d B = rigid(ang(rng), ang(rng), ang(rng),
                              pos(rng), 1.0 + pos(rng), 1.5 + pos(rng));
        for (int c = 0; c < n_cams; ++c) {
            for (int f = 0; f < n_faces; ++f) {
                // Forward chain: A = T_cam←world · T_world←controller · T_controller←face
                cv::Matx44d A = sc.T_cam_world_gt[c] * B * sc.T_controller_face_gt[f];
                // Perturb A (the PnP measurement) if noise requested.
                if (pos_noise_m > 0 || rot_noise_deg > 0) {
                    cv::Matx44d noise = rigid(rn(rng), rn(rng), rn(rng),
                                              tn(rng), tn(rng), tn(rng));
                    A = A * noise;
                }
                ExtrinsicSample s;
                s.cam_index = c;
                s.face_id   = f;
                s.T_cam_marker = fitra::geom::T_cam_marker::from_raw(A);
                s.T_world_controller = fitra::geom::T_world_controller::from_raw(B);
                sc.samples.push_back(s);
            }
        }
    }
    return sc;
}

// 1) Noise-free: recovery must be near-exact and residuals ~0.
void test_exact() {
    Scene sc = make_scene(/*cams*/ 3, /*faces*/ 2, /*poses*/ 16, 0.0, 0.0, 1u);
    auto sol = solve_extrinsics(sc.samples);
    CHECK(sol.ok);
    CHECK(static_cast<int>(sol.cameras.size()) == 3);
    CHECK(static_cast<int>(sol.faces.size()) == 6);  // 3 cams × 2 faces

    for (const auto& fs : sol.faces) {
        CHECK(fs.solved);
        CHECK_LT(fs.residual_trans_rms_m, 1e-9);
        CHECK_LT(fs.residual_rot_rms_deg, 1e-6);
    }
    for (const auto& ce : sol.cameras) {
        const cv::Matx44d& gt = sc.T_cam_world_gt[ce.cam_index];
        CHECK_LT(trans_dist(ce.T_cam_world.raw(), gt), 1e-9);
        CHECK_LT(rotation_angle_deg(ce.T_cam_world.raw(), gt), 1e-6);
        CHECK_LT(ce.face_spread_trans_m, 1e-9);
        CHECK_LT(ce.face_spread_rot_deg, 1e-6);
    }
}

// 2) Relative extrinsic extrinsic(A,B) = T_camA←world · (T_camB←world)⁻¹
//    must match ground truth in the noise-free case.
void test_relative() {
    Scene sc = make_scene(2, 1, 20, 0.0, 0.0, 7u);
    auto sol = solve_extrinsics(sc.samples);
    CHECK(sol.ok);
    CHECK(sol.cameras.size() == 2);
    cv::Matx44d rel_est = sol.cameras[0].T_cam_world.raw() * invert_rigid(sol.cameras[1].T_cam_world.raw());
    cv::Matx44d rel_gt  = sc.T_cam_world_gt[0] * invert_rigid(sc.T_cam_world_gt[1]);
    CHECK_LT(trans_dist(rel_est, rel_gt), 1e-9);
    CHECK_LT(rotation_angle_deg(rel_est, rel_gt), 1e-6);
}

// 3) Noisy: recovery stays close, residual reflects the injected noise.
void test_noisy() {
    Scene sc = make_scene(3, 2, 40, /*pos*/ 0.002, /*rot*/ 0.2, 99u);
    auto sol = solve_extrinsics(sc.samples);
    CHECK(sol.ok);
    for (const auto& ce : sol.cameras) {
        const cv::Matx44d& gt = sc.T_cam_world_gt[ce.cam_index];
        CHECK_LT(trans_dist(ce.T_cam_world.raw(), gt), 0.01);   // < 1 cm
        CHECK_LT(rotation_angle_deg(ce.T_cam_world.raw(), gt), 1.0);  // < 1 deg
    }
    for (const auto& fs : sol.faces) {
        CHECK(fs.solved);
        CHECK_LT(fs.residual_trans_rms_m, 0.02);
        CHECK_LT(fs.residual_rot_rms_deg, 2.0);
    }
}

// 4) Under-sampled groups are reported but skipped (no crash, ok=false).
void test_undersampled() {
    Scene sc = make_scene(1, 1, 2, 0.0, 0.0, 3u);  // only 2 poses < min
    ExtrinsicSolverOptions opts;
    opts.min_samples_per_group = 6;
    auto sol = solve_extrinsics(sc.samples, opts);
    CHECK(!sol.ok);
    CHECK(sol.faces.size() == 1);
    CHECK(!sol.faces[0].solved);
    CHECK(sol.cameras.empty());
}

}  // namespace

int main() {
    test_exact();
    test_relative();
    test_noisy();
    test_undersampled();
    if (g_fail) {
        std::fprintf(stderr, "test_extrinsic_solver: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_extrinsic_solver: OK\n");
    return 0;
}
