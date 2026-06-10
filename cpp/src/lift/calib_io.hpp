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
    cv::Mat K;     // 3x3 CV_64F
    cv::Mat dist;  // 1xN CV_64F
};

struct Extrinsics {
    std::string method;
    cv::Mat T_cw;             // 4x4 CV_64F, world -> camera (fitra Z-up world)
    cv::Vec3d camera_center_w{0.0, 0.0, 0.0};

    // Typed view of T_cw for the SE(3)-layer consumers (Triangulator). The
    // stored T_cw is the validated serialized representation (validate_calibration
    // checks its shape/finiteness); this wraps it once it is known 4x4 CV_64F.
    // The frame is fitra Z-up world by the file contract (extrinsic_calib_session
    // re-expresses the solver's VMT Y-up output before writing).
    geom::T_cam_world pose() const {
        return geom::T_cam_world::from_raw(cv::Matx44d(T_cw.ptr<double>()));
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
