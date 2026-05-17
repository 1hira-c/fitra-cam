#pragma once
//
// Camera calibration YAML loader shared by future 3D lifting modules.
//
// The YAML is produced by python/scripts/calibrate_intrinsics_charuco.py and
// python/scripts/measure_extrinsics_web.py in OpenCV FileStorage format.

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace fitra::lift {

struct Intrinsics {
    int width = 0;
    int height = 0;
    double rms_px = 0.0;
    std::string source;
    cv::Mat K;     // 3x3 CV_64F
    cv::Mat dist;  // 1xN CV_64F
};

struct Extrinsics {
    std::string method;
    cv::Mat T_cw;             // 4x4, world -> camera
    cv::Vec3d camera_center_w{0.0, 0.0, 0.0};
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

}  // namespace fitra::lift
