#pragma once
//
// ChArUco board detection for intrinsic calibration (C++ side).
//
// See docs/design/pose-3d-intrinsic-calibration.md. Mirrors the board used by
// python/scripts/calibrate_intrinsics_charuco.py (ArUco dictionary + a
// squaresX×squaresY ChArUco board). Each frame yields the detected inner
// chessboard corners + their ids; match_points() pairs them with the board's
// known 3D corner positions (Z=0 plane) to feed cv::calibrateCamera /
// cv::fisheye::calibrate.

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>

namespace fitra::lift {

struct CharucoBoardConfig {
    int    squares_x    = 5;       // board columns (squares)
    int    squares_y    = 7;       // board rows (squares)
    double square_len_m = 0.04;    // chessboard square side, metres
    double marker_len_m = 0.03;    // embedded ArUco marker side, metres
    int    dictionary   = -1;      // -1 → DICT_4X4_50 (python default)
};

// One frame's detected charuco corners (image px) + their board corner ids.
struct CharucoView {
    std::vector<cv::Point2f> corners;
    std::vector<int>         ids;
    int count() const { return static_cast<int>(ids.size()); }
};

class CharucoBoardDetector {
public:
    explicit CharucoBoardDetector(CharucoBoardConfig cfg);

    CharucoBoardDetector(const CharucoBoardDetector&) = delete;
    CharucoBoardDetector& operator=(const CharucoBoardDetector&) = delete;

    // Detect the board in `image` (BGR or grayscale). Returns empty corners/ids
    // when nothing is found.
    //
    // NOT thread-safe: the wrapped cv::aruco::CharucoDetector carries state and
    // is mutated here (the `const` is a convenience for the detection-only call
    // sites, via const_cast — it does NOT imply concurrency safety). Call from a
    // single thread only; the intrinsic calibration session's frame tap and the
    // offline tools hand frames in serially, which is fine.
    CharucoView detect(const cv::Mat& image) const;

    // Pair a detected view's corners with the board's known 3D corner positions
    // (board frame, Z=0). obj[i] is the 3D position of charuco corner ids[i];
    // img[i] is its detected pixel. Sizes equal view.count().
    void match_points(const CharucoView& view,
                      std::vector<cv::Point3f>& obj,
                      std::vector<cv::Point2f>& img) const;

    // Number of inner chessboard corners = (squares_x-1) * (squares_y-1).
    int total_corners() const;

    const CharucoBoardConfig& config() const { return cfg_; }

private:
    CharucoBoardConfig         cfg_;
    cv::aruco::CharucoBoard    board_;
    cv::aruco::CharucoDetector detector_;
};

}  // namespace fitra::lift
