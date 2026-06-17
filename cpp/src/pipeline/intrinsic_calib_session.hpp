#pragma once
//
// Intrinsic (per-camera K + distortion) calibration collection loop.
//
// See docs/design/pose-3d-intrinsic-calibration.md. Each camera independently
// collects diverse ChArUco views; on solve, pinhole uses cv::calibrateCamera and
// fisheye uses cv::fisheye::calibrate, and the result is written as an intrinsics
// CalibrationSet (with distortion_model). VR-free, static-board capture.
//
// Threading: on_frame() runs on the capture thread; read methods + solve may run
// on the Crow / main thread. Shared state is mutex-guarded.

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "lift/calib_io.hpp"
#include "lift/charuco_board.hpp"

namespace fitra::pipeline {

enum class IntrinsicCalibState {
    kIdle,
    kCollecting,
    kSolving,
    kSolved,
    kFailed,
};

const char* intrinsic_calib_state_name(IntrinsicCalibState s);

struct IntrinsicCalibConfig {
    lift::CharucoBoardConfig board;
    // "pinhole" (cv::calibrateCamera, 5 coeffs) | "fisheye" (cv::fisheye::calibrate, 4).
    std::string distortion_model = "pinhole";
    std::size_t num_cams = 1;
    std::vector<std::string> cam_ids;  // optional; default "cam0","cam1",...

    int    min_corners = 8;       // per accepted view
    int    min_views   = 12;      // per camera to solve
    int    max_views   = 40;      // cap per camera
    // Acceptance gate: a wrong board spec (squares_x/y transposed, wrong
    // square/marker length, wrong dictionary) still "solves" but lands a huge
    // reprojection RMS and/or an anisotropic K. Reject above these so a
    // degenerate calibration never gets written and silently poison the
    // downstream extrinsic / triangulation. 0 disables the RMS gate.
    double max_rms_px        = 1.5;   // reject solve whose rms_px exceeds this
    double max_fxfy_aniso    = 0.25;  // reject |fx-fy|/max(fx,fy) above this
    // View-diversity gate: accept a view only if its corner centroid is far
    // enough from all accepted views (spread across the image) or its scale
    // (bbox area) differs enough (spread across distance).
    double min_center_sep_frac = 0.06;  // of image diagonal
    double min_area_ratio_diff = 0.20;

    std::string out_path = "calibrations/intrinsics.yaml";
};

class IntrinsicCalibSession {
public:
    explicit IntrinsicCalibSession(IntrinsicCalibConfig cfg);

    IntrinsicCalibSession(const IntrinsicCalibSession&) = delete;
    IntrinsicCalibSession& operator=(const IntrinsicCalibSession&) = delete;

    void start();
    void stop_collecting();

    void on_frame(std::size_t cam_idx, const cv::Mat& bgr, double ts_ms = 0.0);

    // Testable core: offer a pre-detected view to a camera. Returns true if it
    // passed the diversity gate and was accepted.
    bool ingest(std::size_t cam_idx, const lift::CharucoView& view,
                int img_w, int img_h);

    bool solve_and_write(std::string& err);

    void set_on_solved(std::function<void()> fn) { on_solved_ = std::move(fn); }

    IntrinsicCalibState state() const;
    std::size_t accepted_views(std::size_t cam_idx) const;
    std::string state_json() const;

private:
    struct CamData {
        int img_w = 0, img_h = 0;
        std::vector<lift::CharucoView> views;
        std::vector<cv::Point2f> centroids;   // per accepted view
        std::vector<double>      areas;        // per accepted view (bbox)
        unsigned coverage_cells = 0;           // 3x3 bitmask of touched cells
        double   rms_px = 0.0;
        bool     solved = false;
    };

    // Diversity gate; call with mu_ held. Updates centroid/area/coverage if kept.
    bool accept_view_(CamData& cam, const lift::CharucoView& v, int w, int h);

    IntrinsicCalibConfig cfg_;
    mutable std::mutex mu_;
    IntrinsicCalibState state_ = IntrinsicCalibState::kIdle;
    std::map<std::size_t, CamData> cams_;
    std::string last_error_;
    std::function<void()> on_solved_;

    std::unique_ptr<lift::CharucoBoardDetector> detector_;
};

}  // namespace fitra::pipeline
