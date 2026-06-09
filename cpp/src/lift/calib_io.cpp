#include "lift/calib_io.hpp"

#include <cmath>
#include <stdexcept>

namespace fitra::lift {

namespace {

std::string node_string(const cv::FileNode& node) {
    return node.empty() ? std::string{} : static_cast<std::string>(node);
}

double node_real(const cv::FileNode& node, double fallback = 0.0) {
    return node.empty() ? fallback : static_cast<double>(node);
}

cv::Mat read_matrix(const cv::FileNode& parent, const char* name) {
    cv::Mat mat;
    parent[name] >> mat;
    if (mat.empty()) {
        throw std::runtime_error(std::string("missing matrix: ") + name);
    }
    if (mat.depth() != CV_64F) {
        mat.convertTo(mat, CV_64F);
    }
    return mat;
}

bool all_finite(const cv::Mat& mat) {
    for (int r = 0; r < mat.rows; ++r) {
        for (int c = 0; c < mat.cols; ++c) {
            if (!std::isfinite(mat.at<double>(r, c))) return false;
        }
    }
    return true;
}

cv::Vec3d camera_center_from_tcw(const cv::Mat& T_cw) {
    cv::Mat R = T_cw(cv::Rect(0, 0, 3, 3));
    cv::Mat t = T_cw(cv::Rect(3, 0, 1, 3));
    cv::Mat center = -R.t() * t;
    return {center.at<double>(0, 0), center.at<double>(1, 0), center.at<double>(2, 0)};
}

}  // namespace

CalibrationSet load_calibration(const std::string& path) {
    cv::FileStorage fs{path, cv::FileStorage::READ};
    if (!fs.isOpened()) {
        throw std::runtime_error("failed to open calibration file: " + path);
    }

    CalibrationSet out;
    out.schema = node_string(fs["schema"]);
    out.unit = node_string(fs["unit"]);
    out.coordinate_system = node_string(fs["coordinate_system"]);

    cv::FileNode intrinsics = fs["intrinsics"];
    if (intrinsics.empty() || !intrinsics.isMap()) {
        throw std::runtime_error("calibration file does not contain intrinsics map");
    }
    cv::FileNode extrinsics = fs["extrinsics"];

    for (auto it = intrinsics.begin(); it != intrinsics.end(); ++it) {
        CameraCalibration cam;
        cam.id = (*it).name();
        if (cam.id.empty()) {
            throw std::runtime_error("intrinsics entry has no camera id");
        }
        const cv::FileNode node = *it;
        cam.intrinsics.width = static_cast<int>(node_real(node["width"]));
        cam.intrinsics.height = static_cast<int>(node_real(node["height"]));
        cam.intrinsics.rms_px = node_real(node["rms_px"]);
        cam.intrinsics.source = node_string(node["source"]);
        cam.intrinsics.K = read_matrix(node, "K");
        cam.intrinsics.dist = read_matrix(node, "dist").reshape(1, 1);

        if (!extrinsics.empty()) {
            cv::FileNode ex = extrinsics[cam.id];
            if (!ex.empty()) {
                cam.has_extrinsics = true;
                cam.extrinsics.method = node_string(ex["method"]);
                cam.extrinsics.T_cw = read_matrix(ex, "T_cw");
                cv::Mat center;
                ex["camera_center_w"] >> center;
                if (!center.empty()) {
                    if (center.depth() != CV_64F) center.convertTo(center, CV_64F);
                    center = center.reshape(1, 1);
                    if (center.cols >= 3) {
                        cam.extrinsics.camera_center_w = {
                            center.at<double>(0, 0),
                            center.at<double>(0, 1),
                            center.at<double>(0, 2),
                        };
                    }
                } else {
                    cam.extrinsics.camera_center_w = camera_center_from_tcw(cam.extrinsics.T_cw);
                }
            }
        }
        out.cameras.push_back(std::move(cam));
    }

    validate_calibration(out);
    return out;
}

void write_calibration(const std::string& path, const CalibrationSet& calib) {
    validate_calibration(calib);
    cv::FileStorage fs{path, cv::FileStorage::WRITE};
    if (!fs.isOpened()) {
        throw std::runtime_error("failed to open calibration file for write: " + path);
    }
    fs << "schema" << (calib.schema.empty() ? std::string("fitra_calibration_v1")
                                            : calib.schema);
    fs << "unit" << (calib.unit.empty() ? std::string("m") : calib.unit);
    fs << "coordinate_system"
       << (calib.coordinate_system.empty() ? std::string("world") : calib.coordinate_system);

    fs << "intrinsics" << "{";
    for (const auto& cam : calib.cameras) {
        fs << cam.id << "{";
        fs << "width" << cam.intrinsics.width;
        fs << "height" << cam.intrinsics.height;
        fs << "rms_px" << cam.intrinsics.rms_px;
        if (!cam.intrinsics.source.empty()) fs << "source" << cam.intrinsics.source;
        fs << "K" << cam.intrinsics.K;
        fs << "dist" << cam.intrinsics.dist;
        fs << "}";
    }
    fs << "}";

    bool any_ext = false;
    for (const auto& cam : calib.cameras) any_ext |= cam.has_extrinsics;
    if (any_ext) {
        fs << "extrinsics" << "{";
        for (const auto& cam : calib.cameras) {
            if (!cam.has_extrinsics) continue;
            fs << cam.id << "{";
            fs << "method" << (cam.extrinsics.method.empty()
                               ? std::string("unknown") : cam.extrinsics.method);
            fs << "T_cw" << cam.extrinsics.T_cw;
            cv::Mat center = (cv::Mat_<double>(1, 3) <<
                cam.extrinsics.camera_center_w[0],
                cam.extrinsics.camera_center_w[1],
                cam.extrinsics.camera_center_w[2]);
            fs << "camera_center_w" << center;
            fs << "}";
        }
        fs << "}";
    }
    fs.release();
}

void validate_calibration(const CalibrationSet& calib) {
    if (calib.cameras.empty()) {
        throw std::runtime_error("calibration has no cameras");
    }
    for (const auto& cam : calib.cameras) {
        if (cam.id.empty()) {
            throw std::runtime_error("camera id is empty");
        }
        if (cam.intrinsics.width <= 0 || cam.intrinsics.height <= 0) {
            throw std::runtime_error("invalid image size for " + cam.id);
        }
        if (cam.intrinsics.K.rows != 3 || cam.intrinsics.K.cols != 3 ||
            cam.intrinsics.K.type() != CV_64F || !all_finite(cam.intrinsics.K)) {
            throw std::runtime_error("invalid K for " + cam.id);
        }
        if (cam.intrinsics.K.at<double>(0, 0) <= 0.0 ||
            cam.intrinsics.K.at<double>(1, 1) <= 0.0) {
            throw std::runtime_error("non-positive focal length for " + cam.id);
        }
        if (cam.intrinsics.dist.empty() || cam.intrinsics.dist.type() != CV_64F ||
            !all_finite(cam.intrinsics.dist)) {
            throw std::runtime_error("invalid dist coefficients for " + cam.id);
        }
        if (cam.has_extrinsics) {
            if (cam.extrinsics.T_cw.rows != 4 || cam.extrinsics.T_cw.cols != 4 ||
                cam.extrinsics.T_cw.type() != CV_64F || !all_finite(cam.extrinsics.T_cw)) {
                throw std::runtime_error("invalid T_cw for " + cam.id);
            }
            const double bottom[] = {
                cam.extrinsics.T_cw.at<double>(3, 0),
                cam.extrinsics.T_cw.at<double>(3, 1),
                cam.extrinsics.T_cw.at<double>(3, 2),
                cam.extrinsics.T_cw.at<double>(3, 3),
            };
            if (std::abs(bottom[0]) > 1e-9 || std::abs(bottom[1]) > 1e-9 ||
                std::abs(bottom[2]) > 1e-9 || std::abs(bottom[3] - 1.0) > 1e-9) {
                throw std::runtime_error("invalid homogeneous row in T_cw for " + cam.id);
            }
        }
    }
}

}  // namespace fitra::lift
