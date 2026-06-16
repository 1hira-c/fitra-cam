// Intrinsic calibration session: synthesize ChArUco views by projecting the
// board's known 3D corners through a ground-truth camera at varied poses, feed
// them via ingest(), solve, and check the recovered K + rms. Covers pinhole and
// fisheye, plus the "too few views" failure.

#include "pipeline/intrinsic_calib_session.hpp"

#include "lift/calib_io.hpp"
#include "lift/charuco_board.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

using fitra::lift::CharucoBoardConfig;
using fitra::lift::CharucoBoardDetector;
using fitra::lift::CharucoView;
using fitra::lift::load_calibration;
using fitra::pipeline::IntrinsicCalibConfig;
using fitra::pipeline::IntrinsicCalibSession;
using fitra::pipeline::IntrinsicCalibState;

int g_fail = 0;
#define CHECK(cond) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++g_fail; } \
} while (0)
#define CHECK_LT(a, b) do { \
    if (!((a) < (b))) { std::fprintf(stderr, "FAIL %s:%d %s < %s (%g vs %g)\n", \
        __FILE__, __LINE__, #a, #b, double(a), double(b)); ++g_fail; } \
} while (0)

std::string tmp_path(const char* n) {
    const char* d = std::getenv("TMPDIR");
    return std::string(d ? d : "/tmp") + "/" + n;
}

CharucoBoardConfig board_cfg() {
    CharucoBoardConfig b;
    b.squares_x = 5; b.squares_y = 7;
    b.square_len_m = 0.04; b.marker_len_m = 0.03;
    return b;
}

// Generate diverse board poses for calibration. The diversity gate accepts a
// view only if its corner centroid/scale differs from every prior one, so the
// poses must spread the board ACROSS the image (distinct tx/ty → distinct
// centroids) — not just re-tilt it in place. Each pose also carries a varying
// tilt so cv::calibrateCamera can observe K.
std::vector<cv::Vec6d> gen_poses() {
    std::vector<cv::Vec6d> poses;  // rx,ry,rz, tx,ty,tz
    // Wide tx/ty spacing so centroids stay distinct even at the fisheye test's
    // shorter focal length; two tz layers add scale diversity (area differs) so
    // the gate accepts every pose. Per-pose tilt gives K observability.
    const double txs[] = {-0.15, 0.0, 0.15};
    const double tys[] = {-0.10, 0.10};
    const double tzs[] = {0.50, 0.80};
    int i = 0;
    for (double tz : tzs) {
        for (double tx : txs) {
            for (double ty : tys) {
                const double rx = 0.30 * std::sin(0.7 * i);
                const double ry = 0.30 * std::cos(0.5 * i);
                poses.push_back({rx, ry, 0.05, tx, ty, tz});
                ++i;
            }
        }
    }
    return poses;  // 12 poses, distinct centroids + 2 scale layers
}

template <class ProjFn>
int feed_views(IntrinsicCalibSession& s, std::size_t cam,
               const std::vector<cv::Point3f>& board3d, int w, int h,
               ProjFn project) {
    int accepted = 0;
    for (const auto& p : gen_poses()) {
        cv::Mat rvec = (cv::Mat_<double>(3, 1) << p[0], p[1], p[2]);
        cv::Mat tvec = (cv::Mat_<double>(3, 1) << p[3], p[4], p[5]);
        std::vector<cv::Point2f> img = project(board3d, rvec, tvec);
        CharucoView v;
        v.corners = img;
        for (int i = 0; i < static_cast<int>(board3d.size()); ++i) v.ids.push_back(i);
        if (s.ingest(cam, v, w, h)) ++accepted;
    }
    return accepted;
}

void test_pinhole_recovery() {
    const int W = 640, H = 480;
    cv::Mat Kgt = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
    cv::Mat Dgt = cv::Mat::zeros(1, 5, CV_64F);

    CharucoBoardDetector det(board_cfg());
    // board 3D corners (board frame); reuse via a temporary session's detector
    // is awkward, so rebuild the board here.
    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::CharucoBoard board(cv::Size(5, 7), 0.04f, 0.03f, dict);
    std::vector<cv::Point3f> board3d = board.getChessboardCorners();

    IntrinsicCalibConfig cfg;
    cfg.board = board_cfg();
    cfg.distortion_model = "pinhole";
    cfg.num_cams = 1;
    cfg.min_views = 10;
    cfg.out_path = tmp_path("fitra_intr_pinhole.yaml");

    IntrinsicCalibSession s(cfg);
    s.start();
    int acc = feed_views(s, 0, board3d, W, H,
        [&](const std::vector<cv::Point3f>& o, const cv::Mat& rv, const cv::Mat& tv) {
            std::vector<cv::Point2f> img;
            cv::projectPoints(o, rv, tv, Kgt, Dgt, img);
            return img;
        });
    CHECK(acc >= 10);

    std::string err;
    bool ok = s.solve_and_write(err);
    if (!ok) std::fprintf(stderr, "pinhole solve err: %s\n", err.c_str());
    CHECK(ok);
    CHECK(s.state() == IntrinsicCalibState::kSolved);

    auto got = load_calibration(cfg.out_path);
    CHECK(got.cameras.size() == 1);
    if (!got.cameras.empty()) {
        const cv::Mat& K = got.cameras[0].intrinsics.K;
        CHECK(got.cameras[0].intrinsics.distortion_model == "pinhole");
        CHECK_LT(std::abs(K.at<double>(0, 0) - 600.0), 6.0);  // fx within 1%
        CHECK_LT(std::abs(K.at<double>(1, 1) - 600.0), 6.0);
        CHECK_LT(got.cameras[0].intrinsics.rms_px, 0.5);
    }
    std::remove(cfg.out_path.c_str());
}

void test_fisheye_recovery() {
    const int W = 640, H = 480;
    cv::Mat Kgt = (cv::Mat_<double>(3, 3) << 300, 0, 320, 0, 300, 240, 0, 0, 1);
    cv::Mat Dgt = (cv::Mat_<double>(1, 4) << 0.02, -0.005, 0.001, -0.0002);

    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::CharucoBoard board(cv::Size(5, 7), 0.04f, 0.03f, dict);
    std::vector<cv::Point3f> board3d = board.getChessboardCorners();

    IntrinsicCalibConfig cfg;
    cfg.board = board_cfg();
    cfg.distortion_model = "fisheye";
    cfg.num_cams = 1;
    cfg.min_views = 10;
    cfg.out_path = tmp_path("fitra_intr_fisheye.yaml");

    IntrinsicCalibSession s(cfg);
    s.start();
    int acc = feed_views(s, 0, board3d, W, H,
        [&](const std::vector<cv::Point3f>& o, const cv::Mat& rv, const cv::Mat& tv) {
            std::vector<cv::Point3d> od(o.begin(), o.end());
            std::vector<cv::Point2d> img;
            cv::fisheye::projectPoints(od, img, rv, tv, Kgt, Dgt);
            std::vector<cv::Point2f> imgf;
            for (auto& p : img) imgf.push_back({(float)p.x, (float)p.y});
            return imgf;
        });
    CHECK(acc >= 10);

    std::string err;
    bool ok = s.solve_and_write(err);
    if (!ok) std::fprintf(stderr, "fisheye solve err: %s\n", err.c_str());
    CHECK(ok);
    auto got = load_calibration(cfg.out_path);
    if (!got.cameras.empty()) {
        const cv::Mat& K = got.cameras[0].intrinsics.K;
        CHECK(got.cameras[0].intrinsics.distortion_model == "fisheye");
        CHECK(got.cameras[0].intrinsics.dist.cols == 4);
        CHECK_LT(std::abs(K.at<double>(0, 0) - 300.0), 9.0);  // fx within ~3%
        CHECK_LT(got.cameras[0].intrinsics.rms_px, 1.0);
    }
    std::remove(cfg.out_path.c_str());
}

void test_too_few_views() {
    IntrinsicCalibConfig cfg;
    cfg.board = board_cfg();
    cfg.num_cams = 1;
    cfg.min_views = 12;
    cfg.out_path = tmp_path("fitra_intr_few.yaml");
    IntrinsicCalibSession s(cfg);
    s.start();
    CharucoView v;
    for (int i = 0; i < 24; ++i) { v.ids.push_back(i); v.corners.emplace_back(10.f * i, 100.f); }
    s.ingest(0, v, 640, 480);  // one view only
    std::string err;
    CHECK(!s.solve_and_write(err));
    CHECK(s.state() == IntrinsicCalibState::kFailed);
}

}  // namespace

int main() {
    test_pinhole_recovery();
    test_fisheye_recovery();
    test_too_few_views();
    if (g_fail) {
        std::fprintf(stderr, "test_intrinsic_calib_session: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_intrinsic_calib_session: OK\n");
    return 0;
}
