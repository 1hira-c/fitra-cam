// Tests for the floor-AprilTag calibration session.
//
// Drives ingest() with synthetic projected corners (no image / detector),
// solves, writes the YAML, reads it back via load_calibration, and checks the
// persisted T_cw matches ground truth in the fitra Z-up world (no basis change).

#include "pipeline/floor_calib_session.hpp"

#include "lift/calib_io.hpp"
#include "lift/floor_tag_map.hpp"

#include <opencv2/calib3d.hpp>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

using fitra::lift::CalibrationSet;
using fitra::lift::CameraCalibration;
using fitra::lift::FloorTag;
using fitra::lift::FloorTagMap;
using fitra::lift::load_calibration;
using fitra::pipeline::FloorCalibConfig;
using fitra::pipeline::FloorCalibSession;
using fitra::pipeline::FloorCalibState;

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

std::string tmp_path(const char* name) {
    const char* dir = std::getenv("TMPDIR");
    std::string base = dir ? dir : "/tmp";
    return base + "/" + name;
}

cv::Mat make_K(double f, double cx, double cy) {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = f; K.at<double>(1, 1) = f;
    K.at<double>(0, 2) = cx; K.at<double>(1, 2) = cy;
    return K;
}

CameraCalibration make_cam(const std::string& id, const cv::Mat& K) {
    CameraCalibration c;
    c.id = id;
    c.intrinsics.width = 1280;
    c.intrinsics.height = 720;
    c.intrinsics.K = K;
    c.intrinsics.dist = cv::Mat::zeros(1, 5, CV_64F);
    return c;
}

FloorTag make_tag(int id, double size, const cv::Matx33d& R, const cv::Vec3d& t) {
    FloorTag tag; tag.id = id; tag.size_m = size;
    cv::Matx44d m = cv::Matx44d::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) m(r, c) = R(r, c);
        m(r, 3) = t[r];
    }
    tag.T_world_tag = fitra::geom::T_world_marker::from_raw(m);
    return tag;
}

double rot_angle_deg(const cv::Matx44d& a, const cv::Matx44d& b) {
    cv::Matx33d Ra(a(0,0),a(0,1),a(0,2), a(1,0),a(1,1),a(1,2), a(2,0),a(2,1),a(2,2));
    cv::Matx33d Rb(b(0,0),b(0,1),b(0,2), b(1,0),b(1,1),b(1,2), b(2,0),b(2,1),b(2,2));
    cv::Matx33d Rrel = Ra.t() * Rb;
    double tr = Rrel(0,0) + Rrel(1,1) + Rrel(2,2);
    double c = std::max(-1.0, std::min(1.0, (tr - 1.0) * 0.5));
    return std::acos(c) * 57.2957795;
}

FloorTagMap layout() {
    FloorTagMap map;
    map.tags.push_back(make_tag(0, 0.168, cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)));
    map.tags.push_back(make_tag(1, 0.168, cv::Matx33d::eye(), cv::Vec3d(0.6, 0, 0)));
    map.tags.push_back(make_tag(2, 0.168, cv::Matx33d::eye(), cv::Vec3d(0, 0.6, 0)));
    map.tags.push_back(make_tag(3, 0.168, cv::Matx33d::eye(), cv::Vec3d(0.6, 0.6, 0)));
    cv::Matx33d Rx(1, 0, 0, 0, 0, -1, 0, 1, 0);
    map.tags.push_back(make_tag(10, 0.110, Rx, cv::Vec3d(0.3, 0.8, 0.4)));
    map.tags.push_back(make_tag(11, 0.110, Rx, cv::Vec3d(0.3, 0.8, 0.7)));
    return map;
}

// world->camera GT poses for two cameras.
cv::Matx44d gt_pose(const cv::Vec3d& center, double pitch_deg) {
    double a = pitch_deg * 3.14159265 / 180.0;
    cv::Matx33d Rwc(1, 0, 0, 0, std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a));
    cv::Matx33d Rcw = Rwc.t();
    cv::Vec3d t = -(Rcw * center);
    cv::Matx44d T = cv::Matx44d::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) T(r, c) = Rcw(r, c);
        T(r, 3) = t[r];
    }
    return T;
}

void feed_camera(FloorCalibSession& s, std::size_t cam, const FloorTagMap& map,
                 const cv::Matx44d& T_cw, const cv::Mat& K, int frames) {
    cv::Mat R = (cv::Mat_<double>(3, 3) <<
        T_cw(0,0),T_cw(0,1),T_cw(0,2), T_cw(1,0),T_cw(1,1),T_cw(1,2),
        T_cw(2,0),T_cw(2,1),T_cw(2,2));
    cv::Mat rvec; cv::Rodrigues(R, rvec);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << T_cw(0,3), T_cw(1,3), T_cw(2,3));
    cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);
    for (const auto& tag : map.tags) {
        auto wc = map.world_corners(tag);
        std::vector<cv::Point3f> objp;
        for (int i = 0; i < 4; ++i)
            objp.emplace_back((float)wc[i].v[0], (float)wc[i].v[1], (float)wc[i].v[2]);
        std::vector<cv::Point2f> proj;
        cv::projectPoints(objp, rvec, tvec, K, dist, proj);
        std::array<cv::Point2f, 4> corners{proj[0], proj[1], proj[2], proj[3]};
        for (int f = 0; f < frames; ++f) s.ingest(cam, tag.id, corners);
    }
}

void test_solve_write_roundtrip() {
    cv::Mat K = make_K(900.0, 640.0, 360.0);
    CalibrationSet intr;
    intr.schema = "fitra_calibration_v1";
    intr.cameras.push_back(make_cam("cam0", K));
    intr.cameras.push_back(make_cam("cam1", K));

    FloorTagMap map = layout();
    cv::Matx44d gt0 = gt_pose(cv::Vec3d(0.3, -1.0, 2.5), -50.0);
    cv::Matx44d gt1 = gt_pose(cv::Vec3d(0.3,  1.6, 2.5),  50.0);

    FloorCalibConfig cfg;
    cfg.intrinsics = intr;
    cfg.map = map;
    cfg.burst_min = 5;
    cfg.out_path = tmp_path("fitra_floor_extrinsics_test.yaml");

    FloorCalibSession s(cfg);
    // ingest before start() should be refused.
    std::array<cv::Point2f, 4> dummy{};
    CHECK(!s.ingest(0, 0, dummy));

    s.start();
    feed_camera(s, 0, map, gt0, K, 8);
    feed_camera(s, 1, map, gt1, K, 8);
    CHECK(s.ready_group_count() == 12);  // 6 tags × 2 cams

    std::string err;
    bool ok = s.solve_and_write(err);
    CHECK(ok);
    CHECK(s.state() == FloorCalibState::kSolved);

    CalibrationSet got = load_calibration(cfg.out_path);
    CHECK(got.cameras.size() == 2);
    bool found0 = false, found1 = false;
    for (const auto& cam : got.cameras) {
        CHECK(cam.has_extrinsics);
        CHECK(cam.extrinsics.method == "floor_apriltag_pnp");
        const cv::Matx44d gt = (cam.id == "cam0") ? gt0 : gt1;
        cv::Matx44d T = cam.extrinsics.pose().raw();  // typed view of T_cw
        CHECK_LT(std::abs(T(0,3) - gt(0,3)), 1e-4);
        CHECK_LT(std::abs(T(1,3) - gt(1,3)), 1e-4);
        CHECK_LT(std::abs(T(2,3) - gt(2,3)), 1e-4);
        CHECK_LT(rot_angle_deg(T, gt), 1e-2);
        if (cam.id == "cam0") found0 = true;
        if (cam.id == "cam1") found1 = true;
    }
    CHECK(found0 && found1);
    // coordinate_system marks fitra Z-up.
    CHECK(got.coordinate_system.find("z up") != std::string::npos);
    std::remove(cfg.out_path.c_str());
}

}  // namespace

int main() {
    test_solve_write_roundtrip();
    if (g_fail) {
        std::fprintf(stderr, "test_floor_calib_session: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_floor_calib_session: OK\n");
    return 0;
}
