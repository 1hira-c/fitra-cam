#pragma once

// Synthetic image observations for markerless-geometry tests. Ground-truth
// camera poses are used ONLY to project joints and to evaluate recovered
// relative poses; the solver receives fixed intrinsics and image points.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>

#include "lift/markerless_extrinsic.hpp"

namespace markerless_test {

enum class Structure { Articulated, Planar, Collinear };

struct SceneOptions {
    int camera_count = 3;
    int sample_count = 20;
    bool fisheye = false;
    bool mixed_distortion = false;
    bool pure_rotation = false;
    double baseline_scale = 1.0;
    double noise_px = 0.0;
    bool outliers = false;
    bool missing = false;
    Structure structure = Structure::Articulated;
    unsigned seed = 1949;
};

struct Scene {
    fitra::lift::CalibrationSet intrinsics;
    std::vector<fitra::lift::MarkerlessFrameSet> samples;
    std::vector<cv::Matx44d> T_camera_reference_gt;
};

inline cv::Matx33d rotation(const cv::Matx44d& T) {
    return {T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2),
            T(2, 0), T(2, 1), T(2, 2)};
}

inline cv::Vec3d translation(const cv::Matx44d& T) {
    return {T(0, 3), T(1, 3), T(2, 3)};
}

inline double rotation_error_deg(const cv::Matx44d& a, const cv::Matx44d& b) {
    const auto d = rotation(a) * rotation(b).t();
    const double c = std::clamp((d(0, 0) + d(1, 1) + d(2, 2) - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(c) * 180.0 / CV_PI;
}

inline cv::Matx44d pose_from_center(const cv::Vec3d& rod, const cv::Vec3d& center) {
    cv::Matx33d R;
    cv::Rodrigues(rod, R);
    const auto t = -(R * center);
    cv::Matx44d T = cv::Matx44d::eye();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) T(row, col) = R(row, col);
        T(row, 3) = t[row];
    }
    return T;
}

inline std::array<cv::Point3d, 26> articulated_pose(int sample) {
    // OpenCV reference axes: +Y down, +Z forward. These dimensions are not
    // supplied to the solver and carry no metric/ground-plane prior.
    std::array<cv::Point3d, 26> p{{
        {0, -.78, -.04}, {-.04, -.81, -.04}, {.04, -.81, -.04},
        {-.08, -.79, 0}, {.08, -.79, 0},
        {-.23, -.52, 0}, {.23, -.52, 0},
        {-.44, -.18, .10}, {.44, -.18, -.10},
        {-.53, .12, .15}, {.53, .12, -.14},
        {-.16, 0, 0}, {.16, 0, .01},
        {-.17, .38, .08}, {.17, .38, -.07},
        {-.17, .82, 0}, {.17, .82, 0},
        {0, -.92, 0}, {0, -.61, 0}, {0, 0, 0},
        {-.16, .85, -.16}, {.16, .85, -.16},
        {-.23, .85, -.13}, {.23, .85, -.13},
        {-.17, .85, .08}, {.17, .85, .08},
    }};
    const double phase = static_cast<double>(sample);
    for (int k : {7, 9}) {
        p[k].y -= .24 * std::sin(.41 * phase);
        p[k].z += .22 * std::cos(.43 * phase);
    }
    for (int k : {8, 10}) {
        p[k].y -= .18 * std::cos(.37 * phase);
        p[k].z += .20 * std::sin(.47 * phase);
    }
    const double yaw = .46 * std::sin(.39 * phase);
    const cv::Matx33d R(std::cos(yaw), 0, std::sin(yaw), 0, 1, 0,
                        -std::sin(yaw), 0, std::cos(yaw));
    const cv::Vec3d root(.44 * std::sin(.41 * phase), -.08 * std::cos(.61 * phase),
                        3.6 + .65 * std::sin(.31 * phase));
    for (auto& v : p) {
        const auto q = R * cv::Vec3d(v.x, v.y, v.z) + root;
        v = {q[0], q[1], q[2]};
    }
    return p;
}

inline Scene make_scene(const SceneOptions& options = {}) {
    using namespace fitra::lift;
    Scene scene;
    scene.intrinsics.schema = "fitra_calibration_v1";
    scene.intrinsics.unit = "m";
    scene.intrinsics.coordinate_system = "fitra_z_up";
    const std::array<cv::Vec3d, 3> centers{{{0, 0, 0}, {1.0, .04, .12}, {-.9, .12, .25}}};
    const std::array<cv::Vec3d, 3> rotations{{{0, 0, 0}, {.025, .22, -.015}, {.01, -.20, .02}}};
    for (int cam = 0; cam < options.camera_count; ++cam) {
        CameraCalibration camera;
        camera.id = "cam" + std::to_string(cam);
        camera.intrinsics.width = 1280;
        camera.intrinsics.height = 960;
        camera.intrinsics.K = (cv::Mat_<double>(3, 3) <<
            760.0 + 17 * cam, 0, 640.0 + 3 * cam,
            0, 735.0 + 13 * cam, 480.0 - 2 * cam, 0, 0, 1);
        const bool fish = options.fisheye || (options.mixed_distortion && cam == 1);
        camera.intrinsics.distortion_model = fish ? "fisheye" : "pinhole";
        camera.intrinsics.dist = fish
            ? (cv::Mat_<double>(1, 4) << -.035, .003, -.0004, .00002)
            : (cv::Mat_<double>(1, 5) << -.055, .008, .0004, -.0003, -.001);
        scene.intrinsics.cameras.push_back(std::move(camera));
        const auto center = options.pure_rotation ? cv::Vec3d(0, 0, 0)
                                                  : centers[cam] * options.baseline_scale;
        scene.T_camera_reference_gt.push_back(pose_from_center(rotations[cam], center));
    }
    std::mt19937 rng(options.seed);
    std::normal_distribution<double> noise(0.0, options.noise_px);
    for (int sample = 0; sample < options.sample_count; ++sample) {
        auto points = articulated_pose(sample);
        if (options.structure == Structure::Planar) {
            for (auto& p : points) p.z = 3.6;
        } else if (options.structure == Structure::Collinear) {
            for (std::size_t joint = 0; joint < points.size(); ++joint)
                points[joint] = {0.015 * static_cast<double>(joint),
                                 -.8 + .06 * static_cast<double>(joint), 3.6};
        }
        MarkerlessFrameSet set;
        set.sample_id = static_cast<std::uint64_t>(sample + 1);
        set.continuity_id = "synthetic-subject-session";
        for (int cam = 0; cam < options.camera_count; ++cam) {
            const auto& intr = scene.intrinsics.cameras[cam].intrinsics;
            const auto& T = scene.T_camera_reference_gt[cam];
            cv::Vec3d rod;
            cv::Rodrigues(rotation(T), rod);
            std::vector<cv::Point2d> pixels;
            const std::vector<cv::Point3d> objects(points.begin(), points.end());
            if (intr.is_fisheye())
                cv::fisheye::projectPoints(objects, pixels, rod, translation(T), intr.K, intr.dist);
            else
                cv::projectPoints(objects, rod, translation(T), intr.K, intr.dist, pixels);
            MarkerlessCameraFrame frame;
            frame.camera_id = scene.intrinsics.cameras[cam].id;
            frame.source_seq = static_cast<std::uint64_t>(sample + 1);
            frame.stream_epoch = 1;
            frame.capture_mono_ns = 1000000000LL + sample * 300000000LL;
            frame.timestamp_kind = MarkerlessTimestampKind::SyntheticMonotonic;
            frame.width = intr.width;
            frame.height = intr.height;
            for (int joint = 0; joint < 26; ++joint) {
                if (options.missing && joint >= 5 && (sample + 2 * joint + cam) % 17 == 0)
                    continue;
                auto pixel = pixels[joint];
                pixel.x += noise(rng);
                pixel.y += noise(rng);
                if (options.outliers && joint >= 5 && (sample + 7 * joint + 11 * cam) % 41 == 0) {
                    pixel.x += 60.0;
                    pixel.y -= 45.0;
                }
                frame.keypoints.push_back({joint, pixel, .95});
            }
            set.frames.push_back(std::move(frame));
        }
        scene.samples.push_back(std::move(set));
    }
    return scene;
}

}  // namespace markerless_test
