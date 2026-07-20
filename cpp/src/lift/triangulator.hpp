#pragma once

#include <array>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "infer/types.hpp"
#include "lift/calib_io.hpp"

namespace fitra::lift {

struct PerCameraObservation {
    int cam_index = 0;
    const infer::Person* person = nullptr;
};

struct TriangulatedSkeleton {
    infer::Skeleton3D skeleton{};
    // Sized for the largest supported topology; the per-frame triangulate()
    // populates only the leading active-kp-count entries.
    std::array<float, infer::kMaxKeypoints> reproj_error_px{};
    std::array<int, infer::kMaxKeypoints> view_count{};
    double median_reproj_px = 0.0;
    // Number of triangulated body joints contributing to quality metrics.
    // Halpe26's synthetic nose direction endpoint is deliberately not counted.
    int valid_joints = 0;
};

class Triangulator {
public:
    struct Options {
        float kp_conf_thresh = 0.3f;
        float max_reproj_px = 6.0f;
    };

    explicit Triangulator(const CalibrationSet& calib);
    Triangulator(const CalibrationSet& calib, Options opts);

    TriangulatedSkeleton triangulate(const std::vector<PerCameraObservation>& observations) const;
    bool project(int cam_index, const infer::Joint3D& joint, cv::Point2f& out) const;
    std::size_t camera_count() const { return cameras_.size(); }
    const std::string& camera_id(std::size_t i) const { return cameras_[i].id; }
    void require_camera_ids(const std::vector<std::string>& expected_ids) const;

    // Static camera placement in the fitra Z-up world frame, derived from the
    // stored world->camera extrinsics: center_w = -Rᵀ·t, and quat_wxyz is the
    // camera->world rotation (Rᵀ) as a unit quaternion (w,x,y,z). Used by the 3D
    // viewer to draw camera frustums. Precomputed once in the constructor (the
    // placement is static) and returned by const reference, so the per-frame 3D
    // publish path pays no matrix/quaternion work or allocation.
    struct CameraPose {
        std::string id;
        cv::Vec3d   center_w{0.0, 0.0, 0.0};
        cv::Vec4d   quat_wxyz{1.0, 0.0, 0.0, 0.0};
    };
    const std::vector<CameraPose>& camera_poses() const { return camera_poses_; }

private:
    struct CameraModel {
        std::string id;
        cv::Mat K;       // 3x3 CV_64F
        cv::Mat dist;    // 1xN CV_64F
        bool    fisheye = false;  // undistort/project with the fisheye model
        cv::Mat R;       // 3x3 CV_64F world -> camera
        cv::Mat t;       // 3x1 CV_64F world -> camera
        cv::Mat rvec;    // 3x1 CV_64F
        cv::Mat Pn;      // 3x4 CV_64F, normalized projection [R|t]
    };

    struct JointView {
        int cam_index = 0;
        cv::Point2d norm;
        cv::Point2f pixel;
        float score = 0.0f;
    };

    bool triangulate_joint(const std::vector<JointView>& views,
                           infer::Joint3D& joint,
                           float& mean_reproj,
                           int& used_views) const;
    // Position-only DLT for the raw nose direction source. No reprojection
    // calculation/outlier pass: the result is normalized into a fixed-length
    // synthetic endpoint and never enters 3D quality metrics.
    bool triangulate_position_only(const std::vector<JointView>& views,
                                   infer::Joint3D& joint) const;
    bool solve_dlt(const std::vector<JointView>& views,
                   const std::vector<int>& indices,
                   cv::Point3d& out) const;
    float reproj_error_px(const JointView& view, const cv::Point3d& point_w) const;

    std::vector<CameraModel> cameras_;
    std::vector<CameraPose>  camera_poses_;  // precomputed in the constructor
    Options opts_;
};

}  // namespace fitra::lift
