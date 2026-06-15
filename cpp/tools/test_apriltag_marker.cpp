// Tests for the AprilTag marker module.
//
//  1) solve_tag_pose: pure PnP. Project a known tag pose's object corners with
//     a known camera matrix, feed the pixels back, and check the recovered
//     T_cam←face matches ground truth and reprojects cleanly.
//  2) AprilTagDetector round-trip: render a 36h11 marker, embed it with a quiet
//     zone, detect it, and confirm the configured face is decoded with a valid
//     pose. Exercises the real cv::aruco objdetect path without hardware.

#include "lift/apriltag_marker.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {

using fitra::lift::AprilTagDetector;
using fitra::lift::MarkerBoardConfig;
using fitra::lift::MarkerFace;
using fitra::lift::solve_tag_pose;
using fitra::lift::tag_object_corners;
using fitra::lift::TagDetection;

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

// 1) Pure PnP recovery against ground truth.
void test_solve_tag_pose() {
    const double tag = 0.10;  // 10 cm
    cv::Mat K = make_K(900.0, 640.0, 360.0);
    cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);

    // Ground-truth pose: tag 1.2 m ahead, tilted ~20° about Y and ~10° about X.
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0.17, 0.35, -0.05);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.06, -0.04, 1.2);
    cv::Mat Rgt;
    cv::Rodrigues(rvec, Rgt);
    cv::Matx44d gt = cv::Matx44d::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) gt(r, c) = Rgt.at<double>(r, c);
        gt(r, 3) = tvec.at<double>(r);
    }

    auto objc = tag_object_corners(tag);
    std::vector<cv::Point3f> objv(objc.begin(), objc.end());
    std::vector<cv::Point2f> proj;
    cv::projectPoints(objv, rvec, tvec, K, dist, proj);
    std::array<cv::Point2f, 4> corners{proj[0], proj[1], proj[2], proj[3]};

    fitra::geom::T_cam_marker Tw;
    double rms = 0.0;
    bool ok = solve_tag_pose(corners, tag, K, dist, Tw, rms);
    const cv::Matx44d& T = Tw.raw();
    CHECK(ok);
    CHECK_LT(rms, 1e-3);
    // Translation recovery within a few microns.
    CHECK_LT(std::abs(T(0, 3) - gt(0, 3)), 1e-4);
    CHECK_LT(std::abs(T(1, 3) - gt(1, 3)), 1e-4);
    CHECK_LT(std::abs(T(2, 3) - gt(2, 3)), 1e-4);
    CHECK_LT(rot_angle_deg(T, gt), 0.05);
}

// 2) Render a real 36h11 marker, detect it, recover a pose.
void test_detect_roundtrip() {
    const int   face_id = 7;
    const int   side_px = 240;
    const int   quiet   = 80;   // white border (quiet zone) around the tag
    const double tag_m  = 0.10;

    cv::aruco::Dictionary dict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Mat marker;
    dict.generateImageMarker(face_id, side_px, marker, 1);

    cv::Mat canvas(side_px + 2 * quiet, side_px + 2 * quiet, CV_8UC1,
                   cv::Scalar(255));
    marker.copyTo(canvas(cv::Rect(quiet, quiet, side_px, side_px)));

    int W = canvas.cols, H = canvas.rows;
    cv::Mat K = make_K(800.0, W * 0.5, H * 0.5);
    cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);

    MarkerBoardConfig cfg;
    cfg.faces.push_back(MarkerFace{face_id, tag_m});
    cfg.faces.push_back(MarkerFace{99, tag_m});  // a face that won't appear
    AprilTagDetector detector(cfg);

    auto dets = detector.detect(canvas, K, dist);
    CHECK(dets.size() == 1);
    if (!dets.empty()) {
        CHECK(dets[0].face_id == face_id);
        CHECK(dets[0].pose_ok);
        CHECK_LT(dets[0].reproj_rms_px, 1.0);
        CHECK(dets[0].T_cam_face.raw()(2, 3) > 0.0);  // tag in front of camera
    }
}

// 3) CLAHE front-end: a low-contrast (soft, mid-gray) rendering decodes when
//    use_clahe is on. Mirrors the floor-AprilTag feasibility finding that
//    contrast — not distortion or JPEG — gates detection on soft-focus lenses.
void test_clahe_recovers_low_contrast() {
    const int   face_id = 12;
    const int   side_px = 240;
    const int   quiet   = 80;
    const double tag_m   = 0.10;

    cv::aruco::Dictionary dict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Mat marker;
    dict.generateImageMarker(face_id, side_px, marker, 1);

    cv::Mat canvas(side_px + 2 * quiet, side_px + 2 * quiet, CV_8UC1,
                   cv::Scalar(255));
    marker.copyTo(canvas(cv::Rect(quiet, quiet, side_px, side_px)));

    // Compress the dynamic range toward a narrow band centred on mid-gray:
    // out = 116 + in * (24/255), so black→116, white→140 (≈24 levels span).
    cv::Mat low;
    canvas.convertTo(low, CV_8UC1, 24.0 / 255.0, 116.0);

    int W = low.cols, H = low.rows;
    cv::Mat K = make_K(800.0, W * 0.5, H * 0.5);
    cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);

    MarkerBoardConfig cfg;
    cfg.faces.push_back(MarkerFace{face_id, tag_m});
    cfg.use_clahe  = true;
    cfg.clahe_clip = 2.0;
    cfg.clahe_grid = 8;
    AprilTagDetector detector(cfg);

    auto dets = detector.detect(low, K, dist);
    CHECK(dets.size() == 1);
    if (!dets.empty()) {
        CHECK(dets[0].face_id == face_id);
        CHECK(dets[0].pose_ok);
        CHECK_LT(dets[0].reproj_rms_px, 1.0);
    }

    // Sanity: full-contrast detection is unaffected by the default (off).
    MarkerBoardConfig cfg_off;
    cfg_off.faces.push_back(MarkerFace{face_id, tag_m});
    AprilTagDetector detector_off(cfg_off);
    auto dets_full = detector_off.detect(canvas, K, dist);
    CHECK(dets_full.size() == 1);
}

}  // namespace

int main() {
    test_solve_tag_pose();
    test_detect_roundtrip();
    test_clahe_recovers_low_contrast();
    if (g_fail) {
        std::fprintf(stderr, "test_apriltag_marker: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_apriltag_marker: OK\n");
    return 0;
}
