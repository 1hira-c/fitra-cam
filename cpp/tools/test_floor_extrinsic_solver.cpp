// Synthetic-data unit test for the floor-AprilTag extrinsic solver.
//
// We pick a ground-truth T_cam←world, a known FloorTagMap, project every tag's
// world corners through that pose with a known camera, feed the pixels back, and
// check the solver recovers the pose. Covers: noise-free round-trip, planar
// degeneracy detection (floor-only vs floor+stand), pixel-noise robustness, and
// graceful failure on too few correspondences.

#include "lift/floor_extrinsic_solver.hpp"
#include "lift/floor_tag_map.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

namespace {

using fitra::lift::FloorCameraInput;
using fitra::lift::FloorTag;
using fitra::lift::FloorTagMap;
using fitra::lift::FloorTagObservation;
using fitra::lift::solve_floor_extrinsics;

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

cv::Mat make_K(double f, double cx, double cy) {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = f;
    K.at<double>(1, 1) = f;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    return K;
}

double rot_angle_deg(const cv::Matx44d& a, const cv::Matx44d& b) {
    cv::Matx33d Ra(a(0,0),a(0,1),a(0,2), a(1,0),a(1,1),a(1,2), a(2,0),a(2,1),a(2,2));
    cv::Matx33d Rb(b(0,0),b(0,1),b(0,2), b(1,0),b(1,1),b(1,2), b(2,0),b(2,1),b(2,2));
    cv::Matx33d Rrel = Ra.t() * Rb;
    double tr = Rrel(0,0) + Rrel(1,1) + Rrel(2,2);
    double c = std::max(-1.0, std::min(1.0, (tr - 1.0) * 0.5));
    return std::acos(c) * 57.2957795;
}

FloorTag make_tag(int id, double size, const cv::Matx33d& R, const cv::Vec3d& t) {
    FloorTag tag;
    tag.id = id;
    tag.size_m = size;
    cv::Matx44d m = cv::Matx44d::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) m(r, c) = R(r, c);
        m(r, 3) = t[r];
    }
    tag.T_world_tag = fitra::geom::T_world_marker::from_raw(m);
    return tag;
}

// Project a map's tags into one camera at the given GT world->camera pose.
FloorCameraInput synth_camera(const FloorTagMap& map, const cv::Matx44d& T_cw,
                              const cv::Mat& K, double noise_px,
                              std::mt19937& rng) {
    cv::Mat R = (cv::Mat_<double>(3, 3) <<
        T_cw(0,0),T_cw(0,1),T_cw(0,2), T_cw(1,0),T_cw(1,1),T_cw(1,2),
        T_cw(2,0),T_cw(2,1),T_cw(2,2));
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << T_cw(0,3), T_cw(1,3), T_cw(2,3));
    cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);
    std::normal_distribution<double> gauss(0.0, noise_px);

    FloorCameraInput cam;
    cam.cam_index = 0;
    cam.K = K;
    cam.dist = dist;
    for (const auto& tag : map.tags) {
        auto wc = map.world_corners(tag);
        std::vector<cv::Point3f> objp;
        for (int i = 0; i < 4; ++i)
            objp.emplace_back((float)wc[i].v[0], (float)wc[i].v[1], (float)wc[i].v[2]);
        std::vector<cv::Point2f> proj;
        cv::projectPoints(objp, rvec, tvec, K, dist, proj);
        FloorTagObservation ob;
        ob.id = tag.id;
        for (int i = 0; i < 4; ++i) {
            ob.corners[i] = proj[i];
            if (noise_px > 0.0) {
                ob.corners[i].x += (float)gauss(rng);
                ob.corners[i].y += (float)gauss(rng);
            }
        }
        cam.obs.push_back(ob);
    }
    return cam;
}

// world->camera GT: camera ~2.5 m above, looking down at the floor, slight tilt.
cv::Matx44d gt_pose() {
    // Camera at world (0.3, -1.0, 2.5), pitched down ~50°.
    double a = -50.0 * 3.14159265 / 180.0;  // about world X (look down)
    cv::Matx33d Rwc(1, 0, 0, 0, std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a));
    // camera->world rotation Rwc; world->camera is its transpose.
    cv::Matx33d Rcw = Rwc.t();
    cv::Vec3d cam_center(0.3, -1.0, 2.5);
    cv::Vec3d t = -(Rcw * cam_center);
    cv::Matx44d T = cv::Matx44d::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) T(r, c) = Rcw(r, c);
        T(r, 3) = t[r];
    }
    return T;
}

// A layout with off-plane tags (a few floor tags + two on a vertical stand).
FloorTagMap layout_with_stand() {
    FloorTagMap map;
    map.tags.push_back(make_tag(0, 0.168, cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)));
    map.tags.push_back(make_tag(1, 0.168, cv::Matx33d::eye(), cv::Vec3d(0.6, 0, 0)));
    map.tags.push_back(make_tag(2, 0.168, cv::Matx33d::eye(), cv::Vec3d(0, 0.6, 0)));
    map.tags.push_back(make_tag(3, 0.168, cv::Matx33d::eye(), cv::Vec3d(0.6, 0.6, 0)));
    // Stand tags: rotated 90° about X (facing -Y), lifted off the floor.
    cv::Matx33d Rx(1, 0, 0, 0, 0, -1, 0, 1, 0);
    map.tags.push_back(make_tag(10, 0.110, Rx, cv::Vec3d(0.3, 0.8, 0.4)));
    map.tags.push_back(make_tag(11, 0.110, Rx, cv::Vec3d(0.3, 0.8, 0.7)));
    return map;
}

FloorTagMap layout_floor_only() {
    FloorTagMap map;
    map.tags.push_back(make_tag(0, 0.168, cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)));
    map.tags.push_back(make_tag(1, 0.168, cv::Matx33d::eye(), cv::Vec3d(0.6, 0, 0)));
    map.tags.push_back(make_tag(2, 0.168, cv::Matx33d::eye(), cv::Vec3d(0, 0.6, 0)));
    map.tags.push_back(make_tag(3, 0.168, cv::Matx33d::eye(), cv::Vec3d(0.6, 0.6, 0)));
    return map;
}

void test_roundtrip_noisefree() {
    cv::Mat K = make_K(900.0, 640.0, 360.0);
    std::mt19937 rng(1);
    FloorTagMap map = layout_with_stand();
    cv::Matx44d gt = gt_pose();
    auto cam = synth_camera(map, gt, K, 0.0, rng);

    auto sol = solve_floor_extrinsics({cam}, map);
    CHECK(sol.ok);
    CHECK(sol.cameras.size() == 1);
    if (!sol.cameras.empty()) {
        const auto& c = sol.cameras[0];
        CHECK(c.solved);
        CHECK(c.n_tags == 6);
        // Object corners are single-precision (tag_object_corners → float), so
        // the noise-free recovery is float-limited, not double-exact.
        CHECK_LT(c.reproj_rms_px, 1e-2);
        const cv::Matx44d& T = c.T_cam_world.raw();
        CHECK_LT(std::abs(T(0,3) - gt(0,3)), 1e-5);
        CHECK_LT(std::abs(T(1,3) - gt(1,3)), 1e-5);
        CHECK_LT(std::abs(T(2,3) - gt(2,3)), 1e-5);
        CHECK_LT(rot_angle_deg(T, gt), 1e-3);
        CHECK(!c.planar_degenerate);  // stand tags break the plane
    }
}

void test_planar_degenerate() {
    cv::Mat K = make_K(900.0, 640.0, 360.0);
    std::mt19937 rng(2);
    FloorTagMap flat = layout_floor_only();
    auto cam = synth_camera(flat, gt_pose(), K, 0.0, rng);
    auto sol = solve_floor_extrinsics({cam}, flat);
    CHECK(sol.cameras.size() == 1);
    if (!sol.cameras.empty()) {
        // A pose still solves (homography), but the cloud is flagged coplanar.
        CHECK(sol.cameras[0].planar_degenerate);
        CHECK_LT(sol.cameras[0].plane_thickness_m, 1e-6);
    }

    FloorTagMap stand = layout_with_stand();
    auto cam2 = synth_camera(stand, gt_pose(), K, 0.0, rng);
    auto sol2 = solve_floor_extrinsics({cam2}, stand);
    CHECK(!sol2.cameras.empty() && !sol2.cameras[0].planar_degenerate);
}

void test_noise() {
    cv::Mat K = make_K(900.0, 640.0, 360.0);
    std::mt19937 rng(3);
    FloorTagMap map = layout_with_stand();
    cv::Matx44d gt = gt_pose();
    auto cam = synth_camera(map, gt, K, 0.5, rng);  // 0.5 px noise
    auto sol = solve_floor_extrinsics({cam}, map);
    CHECK(sol.ok);
    if (!sol.cameras.empty()) {
        const auto& c = sol.cameras[0];
        CHECK(c.solved);
        CHECK_LT(c.reproj_rms_px, 1.0);
        CHECK_LT(rot_angle_deg(c.T_cam_world.raw(), gt), 1.0);
    }
}

void test_too_few() {
    cv::Mat K = make_K(900.0, 640.0, 360.0);
    FloorTagMap map = layout_with_stand();
    FloorCameraInput cam;
    cam.cam_index = 0;
    cam.K = K;
    cam.dist = cv::Mat::zeros(1, 5, CV_64F);
    // Only one tag (4 points) but require min_points=8 default.
    FloorTagObservation ob;
    ob.id = 0;
    for (int i = 0; i < 4; ++i) ob.corners[i] = cv::Point2f(100.f + i, 100.f);
    cam.obs.push_back(ob);
    auto sol = solve_floor_extrinsics({cam}, map);
    CHECK(!sol.ok);
    CHECK(sol.cameras.size() == 1 && !sol.cameras[0].solved);
}

}  // namespace

int main() {
    test_roundtrip_noisefree();
    test_planar_degenerate();
    test_noise();
    test_too_few();
    if (g_fail) {
        std::fprintf(stderr, "test_floor_extrinsic_solver: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_floor_extrinsic_solver: OK\n");
    return 0;
}
