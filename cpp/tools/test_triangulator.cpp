#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>

#include "lift/keypoint_format.hpp"
#include "lift/head_direction.hpp"
#include "lift/triangulator.hpp"

namespace {

void check(bool cond, const std::string& msg) {
    if (!cond) throw std::runtime_error(msg);
}

fitra::lift::CalibrationSet make_calibration(const std::vector<std::string>& ids) {
    fitra::lift::CalibrationSet calib;
    calib.schema = "test";
    calib.unit = "m";
    calib.coordinate_system = "opencv";

    for (std::size_t i = 0; i < ids.size(); ++i) {
        fitra::lift::CameraCalibration cam;
        cam.id = ids[i];
        cam.intrinsics.width = 640;
        cam.intrinsics.height = 480;
        cam.intrinsics.K = (cv::Mat_<double>(3, 3) << 600.0, 0.0, 320.0,
                                                       0.0, 600.0, 240.0,
                                                       0.0, 0.0, 1.0);
        cam.intrinsics.dist = cv::Mat::zeros(1, 5, CV_64F);
        cam.has_extrinsics = true;
        cam.extrinsics.T_cw = cv::Mat::eye(4, 4, CV_64F);
        cam.extrinsics.T_cw.at<double>(0, 3) = -0.45 * static_cast<double>(i);
        calib.cameras.push_back(std::move(cam));
    }
    return calib;
}

std::vector<cv::Point3d> make_points() {
    std::vector<cv::Point3d> points;
    const std::size_t kp_count = fitra::lift::active_kp_count();
    points.reserve(kp_count);
    for (std::size_t k = 0; k < kp_count; ++k) {
        const double x = (static_cast<int>(k % 5) - 2) * 0.08;
        const double y = (static_cast<int>(k / 5) - 1) * 0.07;
        const double z = 3.0 + static_cast<double>(k % 3) * 0.03;
        points.emplace_back(x, y, z);
    }
    return points;
}

fitra::infer::Person project_person(const fitra::lift::CameraCalibration& cam,
                                    const std::vector<cv::Point3d>& points) {
    cv::Mat R = cam.extrinsics.T_cw(cv::Rect(0, 0, 3, 3));
    cv::Mat t = cam.extrinsics.T_cw(cv::Rect(3, 0, 1, 3));
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);

    std::vector<cv::Point2d> image;
    cv::projectPoints(points, rvec, t, cam.intrinsics.K, cam.intrinsics.dist, image);

    fitra::infer::Person person;
    person.bbox = {0.0f, 0.0f, 640.0f, 480.0f, 1.0f};
    person.kp_count = static_cast<std::uint8_t>(points.size());
    for (std::size_t k = 0; k < points.size(); ++k) {
        person.kpts[k].x = static_cast<float>(image[k].x);
        person.kpts[k].y = static_cast<float>(image[k].y);
        person.kpts[k].score = 0.9f;
    }
    return person;
}

void test_round_trip() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Coco17);
    auto calib = make_calibration({"cam0", "cam1"});
    fitra::lift::Triangulator triangulator{calib};
    triangulator.require_camera_ids({"cam0", "cam1"});

    const auto points = make_points();
    auto p0 = project_person(calib.cameras[0], points);
    auto p1 = project_person(calib.cameras[1], points);
    std::vector<fitra::lift::PerCameraObservation> observations{
        {0, &p0},
        {1, &p1},
    };

    auto tri = triangulator.triangulate(observations);
    check(tri.valid_joints == static_cast<int>(fitra::lift::active_kp_count()),
          "expected all joints to triangulate");
    for (std::size_t k = 0; k < points.size(); ++k) {
        const auto& got = tri.skeleton.joints[k];
        check(got.valid, "triangulated joint is invalid");
        const double dx = static_cast<double>(got.x) - points[k].x;
        const double dy = static_cast<double>(got.y) - points[k].y;
        const double dz = static_cast<double>(got.z) - points[k].z;
        const double err_m = std::sqrt(dx * dx + dy * dy + dz * dz);
        check(err_m < 1.0e-3, "triangulated joint drift exceeds 1mm");
        check(tri.max_ray_angle_deg[k] > 5.0f &&
                  tri.max_ray_angle_deg[k] < 12.0f,
              "two-camera inlier ray angle is outside expected geometry");
    }
}

void test_ray_angle_uses_final_inliers_only() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Coco17);
    auto calib = make_calibration({"cam0", "cam1", "cam2"});
    fitra::lift::Triangulator::Options options;
    options.kp_conf_thresh = 0.3f;
    options.max_reproj_px = 6.0f;
    fitra::lift::Triangulator triangulator{calib, options};

    const auto points = make_points();
    auto p0 = project_person(calib.cameras[0], points);
    auto p1 = project_person(calib.cameras[1], points);
    auto p2_clean = project_person(calib.cameras[2], points);
    auto p2_outlier = p2_clean;
    for (std::size_t k = 0; k < points.size(); ++k) {
        // Keep the third view just above the confidence gate but make its
        // reprojection incompatible with the two high-confidence inliers.
        // If max_ray_angle_deg accidentally uses all candidate views, cam2's
        // wider baseline would inflate the reported quality evidence.
        p2_outlier.kpts[k].x += 20.0f;
        p2_outlier.kpts[k].y -= 16.0f;
        p2_outlier.kpts[k].score = 0.31f;
    }

    const auto inlier_pair = triangulator.triangulate({
        {0, &p0}, {1, &p1},
    });
    const auto all_clean = triangulator.triangulate({
        {0, &p0}, {1, &p1}, {2, &p2_clean},
    });
    const auto with_outlier = triangulator.triangulate({
        {0, &p0}, {1, &p1}, {2, &p2_outlier},
    });

    for (std::size_t k = 0; k < points.size(); ++k) {
        check(with_outlier.skeleton.joints[k].valid,
              "two inlier views must still triangulate with one outlier");
        check(with_outlier.view_count[k] == 2,
              "reprojection outlier must be removed from final views");
        check(std::abs(with_outlier.max_ray_angle_deg[k] -
                       inlier_pair.max_ray_angle_deg[k]) < 0.1f,
              "max ray angle included a rejected reprojection outlier");
        check(all_clean.max_ray_angle_deg[k] >
                  with_outlier.max_ray_angle_deg[k] + 4.0f,
              "test geometry does not distinguish candidate and final rays");
    }
}

void test_halpe_face_joints_are_excluded_from_3d() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    auto calib = make_calibration({"cam0", "cam1"});
    fitra::lift::Triangulator triangulator{calib};

    const auto points = make_points();
    auto p0 = project_person(calib.cameras[0], points);
    auto p1 = project_person(calib.cameras[1], points);

    // Deliberately make the face disagree across cameras. These unreliable
    // landmarks must not enter the 3D reprojection metric or body skeleton.
    for (std::size_t k = 0; k <= 4; ++k) {
        p1.kpts[k].x += 120.0f;
        p1.kpts[k].y -= 80.0f;
    }
    std::vector<fitra::lift::PerCameraObservation> observations{
        {0, &p0},
        {1, &p1},
    };

    auto tri = triangulator.triangulate(observations);
    check(tri.valid_joints == 21,
          "Halpe26 3D lift should contain 21 non-facial joints");
    check(tri.median_reproj_px < 1.0e-3,
          "excluded face disagreement must not bias reprojection median");
    const auto& nose = tri.skeleton.joints[0];
    const auto& head_top = tri.skeleton.joints[17];
    const auto& neck = tri.skeleton.joints[18];
    check(nose.valid,
          "nose direction source must produce a synthetic endpoint");
    const double rx = static_cast<double>(nose.x) - head_top.x;
    const double ry = static_cast<double>(nose.y) - head_top.y;
    const double rz = static_cast<double>(nose.z) - head_top.z;
    const double ray_len = std::sqrt(rx * rx + ry * ry + rz * rz);
    check(std::abs(ray_len - fitra::lift::kHeadDirectionLengthM) < 1.0e-4,
          "synthetic head direction must have fixed length");
    const double ax = static_cast<double>(head_top.x) - neck.x;
    const double ay = static_cast<double>(head_top.y) - neck.y;
    const double az = static_cast<double>(head_top.z) - neck.z;
    check(std::abs(rx * ax + ry * ay + rz * az) < 1.0e-5,
          "synthetic head direction must be perpendicular to head axis");
    check(tri.view_count[0] == 0 && tri.reproj_error_px[0] == 0.0f,
          "synthetic nose must not publish reprojection diagnostics");
    for (std::size_t k = 1; k <= 4; ++k) {
        check(!tri.skeleton.joints[k].valid,
              "Halpe26 eye/ear joint must remain invalid in 3D skeleton");
        check(tri.view_count[k] == 0,
              "Halpe26 eye/ear joint must not consume triangulation views");
    }
    for (std::size_t k = 5; k < points.size(); ++k) {
        check(tri.skeleton.joints[k].valid,
              "non-facial Halpe26 joint must still triangulate");
    }
    check(head_top.valid,
          "head_top must stay available for HMD alignment");
    check(neck.valid,
          "neck must stay available for torso tracking");
}

void test_camera_id_validation() {
    auto calib = make_calibration({"left", "right"});
    fitra::lift::Triangulator triangulator{calib};
    bool threw = false;
    try {
        triangulator.require_camera_ids({"cam0", "cam1"});
    } catch (const std::runtime_error&) {
        threw = true;
    }
    check(threw, "expected non-camN calibration ids to be rejected");
}

}  // namespace

int main() {
    try {
        test_round_trip();
        test_ray_angle_uses_final_inliers_only();
        test_halpe_face_joints_are_excluded_from_3d();
        test_camera_id_validation();
        std::puts("test_triangulator ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_triangulator failed: %s\n", e.what());
        return 1;
    }
}
