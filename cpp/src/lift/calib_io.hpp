#pragma once
//
// Camera calibration YAML loader shared by future 3D lifting modules.
//
// The YAML is produced by python/scripts/calibrate_intrinsics_charuco.py and
// python/scripts/measure_extrinsics_web.py in OpenCV FileStorage format.

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "geom/frames.hpp"

namespace fitra::lift {

struct Intrinsics {
    int width = 0;
    int height = 0;
    double rms_px = 0.0;
    std::string source;
    // Lens distortion model the K/dist pair was fit under: "pinhole"
    // (cv::calibrateCamera, dist = k1,k2,p1,p2[,k3...]) or "fisheye"
    // (cv::fisheye::calibrate, dist = k1,k2,k3,k4). Consumers branch on this —
    // NOT on the coefficient count — so a fisheye calibration is undistorted
    // with the matching model. Absent in the YAML → "pinhole" (back-compat).
    std::string distortion_model = "pinhole";
    cv::Mat K;     // 3x3 CV_64F
    cv::Mat dist;  // 1xN CV_64F

    bool is_fisheye() const { return distortion_model == "fisheye"; }
};

struct Extrinsics {
    std::string method;
    cv::Mat T_cw;             // 4x4 CV_64F, world -> camera (fitra Z-up world)
    cv::Vec3d camera_center_w{0.0, 0.0, 0.0};

    // Typed view of T_cw for the SE(3)-layer consumers (Triangulator). The
    // frame is fitra Z-up world by the file contract (extrinsic_calib_session
    // re-expresses the solver's VMT Y-up output before writing). Defensive:
    // returns identity for an unset / wrong-shape T_cw (e.g. a default-
    // constructed Extrinsics) rather than dereferencing a null/short buffer, and
    // reads element-wise so a non-contiguous ROI is handled correctly.
    geom::T_cam_world pose() const {
        cv::Matx44d m = cv::Matx44d::eye();
        if (T_cw.rows == 4 && T_cw.cols == 4 && T_cw.type() == CV_64F) {
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c) m(r, c) = T_cw.at<double>(r, c);
        }
        return geom::T_cam_world::from_raw(m);
    }
};

struct CameraCalibration {
    std::string id;
    Intrinsics intrinsics;
    bool has_extrinsics = false;
    Extrinsics extrinsics;
};

struct CalibrationSet {
    std::string schema;
    std::string unit;
    std::string coordinate_system;
    std::vector<CameraCalibration> cameras;
};

CalibrationSet load_calibration(const std::string& path);
void validate_calibration(const CalibrationSet& calib);

// Write `calib` to `path` in the same OpenCV FileStorage layout
// load_calibration reads (schema / unit / coordinate_system + per-camera
// intrinsics map + optional per-camera extrinsics map). Cameras with
// has_extrinsics=false are emitted under intrinsics only. Throws on open
// failure. Used by the extrinsic calibration session to persist results.
void write_calibration(const std::string& path, const CalibrationSet& calib);

}  // namespace fitra::lift
