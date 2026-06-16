// ChArUco board detection: render the board, detect it, and check the matched
// object/image points are sane (planar, count matches, ids in range).

#include "lift/charuco_board.hpp"

#include <opencv2/core.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

using fitra::lift::CharucoBoardConfig;
using fitra::lift::CharucoBoardDetector;
using fitra::lift::CharucoView;

int g_fail = 0;
#define CHECK(cond) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++g_fail; } \
} while (0)

void test_detect_render() {
    CharucoBoardConfig cfg;
    cfg.squares_x = 5;
    cfg.squares_y = 7;
    cfg.square_len_m = 0.04;
    cfg.marker_len_m = 0.03;

    CharucoBoardDetector det(cfg);
    CHECK(det.total_corners() == (5 - 1) * (7 - 1));  // 24 inner corners

    // Render a clean board image (re-build a matching board for generateImage).
    cv::aruco::Dictionary dict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::CharucoBoard board(cv::Size(5, 7), 0.04f, 0.03f, dict);
    cv::Mat img;
    board.generateImage(cv::Size(500, 700), img, 20, 1);

    CharucoView v = det.detect(img);
    // A clean full-board render should recover (nearly) all inner corners.
    CHECK(v.count() >= det.total_corners() - 2);
    CHECK(v.corners.size() == v.ids.size());

    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> imgp;
    det.match_points(v, obj, imgp);
    CHECK(obj.size() == static_cast<std::size_t>(v.count()));
    CHECK(imgp.size() == obj.size());
    // Object points are planar (Z=0) and within the board extent.
    for (const auto& p : obj) {
        CHECK(std::abs(p.z) < 1e-6);
        CHECK(p.x >= -1e-6 && p.x <= 5 * 0.04 + 1e-6);
        CHECK(p.y >= -1e-6 && p.y <= 7 * 0.04 + 1e-6);
    }
}

void test_blank_image() {
    CharucoBoardConfig cfg;
    CharucoBoardDetector det(cfg);
    cv::Mat blank(480, 640, CV_8UC1, cv::Scalar(255));
    CharucoView v = det.detect(blank);
    CHECK(v.count() == 0);
    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> imgp;
    det.match_points(v, obj, imgp);  // must not crash on empty
    CHECK(obj.empty() && imgp.empty());
}

}  // namespace

int main() {
    test_detect_render();
    test_blank_image();
    if (g_fail) {
        std::fprintf(stderr, "test_charuco_board: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_charuco_board: OK\n");
    return 0;
}
