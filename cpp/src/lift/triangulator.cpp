#include "lift/triangulator.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>

#include <opencv2/calib3d.hpp>

#include "geom/frames.hpp"
#include "lift/keypoint_format.hpp"

namespace fitra::lift {

namespace {

double median(std::vector<double> vals) {
    if (vals.empty()) return 0.0;
    auto mid = vals.begin() + static_cast<std::ptrdiff_t>(vals.size() / 2);
    std::nth_element(vals.begin(), mid, vals.end());
    double m = *mid;
    if (vals.size() % 2 == 0) {
        auto mid2 = vals.begin() + static_cast<std::ptrdiff_t>(vals.size() / 2 - 1);
        std::nth_element(vals.begin(), mid2, vals.end());
        m = 0.5 * (m + *mid2);
    }
    return m;
}

}  // namespace

void Triangulator::require_camera_ids(const std::vector<std::string>& expected_ids) const {
    auto describe = [](const auto& ids) {
        std::string out;
        for (std::size_t i = 0; i < ids.size(); ++i) {
            if (i) out += ",";
            out += ids[i];
        }
        return out;
    };
    std::vector<std::string> actual;
    actual.reserve(cameras_.size());
    for (const auto& cam : cameras_) actual.push_back(cam.id);
    if (cameras_.size() != expected_ids.size()) {
        throw std::runtime_error(
            "calibration camera ids must match runtime order exactly: expected [" +
            describe(expected_ids) + "], got [" + describe(actual) + "]");
    }
    for (std::size_t i = 0; i < expected_ids.size(); ++i) {
        if (cameras_[i].id != expected_ids[i]) {
            throw std::runtime_error(
                "calibration camera ids must match runtime order exactly: expected [" +
                describe(expected_ids) + "], got [" + describe(actual) + "]");
        }
    }
}

Triangulator::Triangulator(const CalibrationSet& calib)
    : Triangulator(calib, Options{}) {}

Triangulator::Triangulator(const CalibrationSet& calib, Options opts)
    : opts_{opts} {
    validate_calibration(calib);
    cameras_.reserve(calib.cameras.size());
    for (const auto& cam : calib.cameras) {
        if (!cam.has_extrinsics) {
            throw std::runtime_error("camera " + cam.id + " lacks extrinsics");
        }
        CameraModel m;
        m.id = cam.id;
        m.K = cam.intrinsics.K.clone();
        m.dist = cam.intrinsics.dist.clone();
        m.fisheye = cam.intrinsics.is_fisheye();
        // Typed fitra Z-up world->camera extrinsic; unwrap to cv::Mat R/t for
        // the OpenCV DLT / projectPoints math below.
        const geom::T_cam_world T_cw = cam.extrinsics.pose();
        m.R = cv::Mat(T_cw.rot());
        m.t = cv::Mat(T_cw.trans());
        cv::Rodrigues(m.R, m.rvec);
        m.Pn = cv::Mat::zeros(3, 4, CV_64F);
        m.R.copyTo(m.Pn(cv::Rect(0, 0, 3, 3)));
        m.t.copyTo(m.Pn(cv::Rect(3, 0, 1, 3)));
        cameras_.push_back(std::move(m));
    }
    if (cameras_.size() < 2) {
        throw std::runtime_error("triangulation requires at least 2 calibrated cameras");
    }
}

std::vector<Triangulator::CameraPose> Triangulator::camera_poses() const {
    std::vector<CameraPose> out;
    out.reserve(cameras_.size());
    for (const auto& cam : cameras_) {
        CameraPose p;
        p.id = cam.id;
        // R/t are world->camera. Camera center in world = -Rᵀ·t,
        // camera->world rotation = Rᵀ.
        const cv::Mat Rt = cam.R.t();
        const cv::Mat c = -Rt * cam.t;  // 3x1 CV_64F
        p.center_w = cv::Vec3d(c.at<double>(0), c.at<double>(1), c.at<double>(2));
        cv::Matx33d R_wc;
        for (int r = 0; r < 3; ++r)
            for (int col = 0; col < 3; ++col) R_wc(r, col) = Rt.at<double>(r, col);
        p.quat_wxyz = geom::mat_to_quat(R_wc);
        out.push_back(std::move(p));
    }
    return out;
}

TriangulatedSkeleton Triangulator::triangulate(
    const std::vector<PerCameraObservation>& observations) const {
    TriangulatedSkeleton out;
    std::vector<double> valid_errors;
    const std::size_t kp_count = active_kp_count();
    out.skeleton.kp_count = static_cast<std::uint8_t>(kp_count);

    // Per-call scratch reused across keypoints/views to avoid per-keypoint and
    // per-view heap allocations. thread_local (not mutable members) keeps the
    // const contract intact: triangulate() touches no shared state, so it stays
    // thread-safe even if the pipeline later triangulates frames in parallel,
    // and Triangulator remains cheap to copy/move.
    static thread_local std::vector<JointView>   views;
    static thread_local std::vector<cv::Point2f> undist_src;
    static thread_local std::vector<cv::Point2f> undist_dst;

    for (std::size_t k = 0; k < kp_count; ++k) {
        views.clear();
        for (const auto& obs : observations) {
            if (!obs.person || obs.cam_index < 0 ||
                static_cast<std::size_t>(obs.cam_index) >= cameras_.size()) {
                continue;
            }
            const auto& kp = obs.person->kpts[k];
            if (kp.score < opts_.kp_conf_thresh) continue;

            const auto& cam = cameras_[static_cast<std::size_t>(obs.cam_index)];
            undist_src.assign(1, cv::Point2f(kp.x, kp.y));
            if (cam.fisheye) {
                cv::fisheye::undistortPoints(undist_src, undist_dst, cam.K, cam.dist);
            } else {
                cv::undistortPoints(undist_src, undist_dst, cam.K, cam.dist);
            }
            if (undist_dst.empty()) continue;

            JointView v;
            v.cam_index = obs.cam_index;
            v.norm = cv::Point2d(undist_dst[0].x, undist_dst[0].y);
            v.pixel = cv::Point2f(kp.x, kp.y);
            v.score = kp.score;
            views.push_back(v);
        }

        float err = 0.0f;
        int used = 0;
        if (triangulate_joint(views, out.skeleton.joints[k], err, used)) {
            out.reproj_error_px[k] = err;
            out.view_count[k] = used;
            out.valid_joints += 1;
            valid_errors.push_back(err);
        }
    }

    out.median_reproj_px = median(valid_errors);
    return out;
}

bool Triangulator::triangulate_joint(const std::vector<JointView>& views,
                                     infer::Joint3D& joint,
                                     float& mean_reproj,
                                     int& used_views) const {
    if (views.size() < 2) return false;

    // thread_local scratch (see triangulate()): allocation-free across calls,
    // const-safe, no shared mutable members.
    static thread_local std::vector<int> indices;
    static thread_local std::vector<int> kept;

    indices.resize(views.size());
    std::iota(indices.begin(), indices.end(), 0);

    cv::Point3d point;
    if (!solve_dlt(views, indices, point)) return false;

    if (views.size() > 2) {
        kept.clear();
        kept.reserve(indices.size());
        for (int idx : indices) {
            if (reproj_error_px(views[static_cast<std::size_t>(idx)], point) <= opts_.max_reproj_px) {
                kept.push_back(idx);
            }
        }
        if (kept.size() >= 2 && kept.size() < indices.size()) {
            cv::Point3d refined;
            if (solve_dlt(views, kept, refined)) {
                point = refined;
                indices = kept;
            }
        }
    }

    double err_sum = 0.0;
    double score_sum = 0.0;
    for (int idx : indices) {
        const auto& view = views[static_cast<std::size_t>(idx)];
        err_sum += reproj_error_px(view, point);
        score_sum += view.score;
    }
    used_views = static_cast<int>(indices.size());
    mean_reproj = static_cast<float>(err_sum / std::max(1, used_views));
    joint.x = static_cast<float>(point.x);
    joint.y = static_cast<float>(point.y);
    joint.z = static_cast<float>(point.z);
    joint.score = static_cast<float>(score_sum / std::max(1, used_views));
    joint.valid = true;
    return true;
}

bool Triangulator::solve_dlt(const std::vector<JointView>& views,
                             const std::vector<int>& indices,
                             cv::Point3d& out) const {
    if (indices.size() < 2) return false;
    cv::Mat A(static_cast<int>(indices.size() * 2), 4, CV_64F);
    int row = 0;
    for (int idx : indices) {
        const auto& v = views[static_cast<std::size_t>(idx)];
        const auto& P = cameras_[static_cast<std::size_t>(v.cam_index)].Pn;
        const double w = std::sqrt(std::max(1.0e-6, static_cast<double>(v.score)));
        for (int c = 0; c < 4; ++c) {
            A.at<double>(row, c) =
                w * (v.norm.x * P.at<double>(2, c) - P.at<double>(0, c));
            A.at<double>(row + 1, c) =
                w * (v.norm.y * P.at<double>(2, c) - P.at<double>(1, c));
        }
        row += 2;
    }

    cv::SVD svd(A);
    cv::Mat h = svd.vt.row(3);
    double hw = h.at<double>(0, 3);
    if (std::abs(hw) < 1.0e-12 || !std::isfinite(hw)) return false;
    out.x = h.at<double>(0, 0) / hw;
    out.y = h.at<double>(0, 1) / hw;
    out.z = h.at<double>(0, 2) / hw;
    return std::isfinite(out.x) && std::isfinite(out.y) && std::isfinite(out.z);
}

float Triangulator::reproj_error_px(const JointView& view, const cv::Point3d& point_w) const {
    cv::Point2f proj;
    infer::Joint3D j;
    j.x = static_cast<float>(point_w.x);
    j.y = static_cast<float>(point_w.y);
    j.z = static_cast<float>(point_w.z);
    j.valid = true;
    if (!project(view.cam_index, j, proj)) return 1.0e9f;
    const double dx = static_cast<double>(proj.x) - view.pixel.x;
    const double dy = static_cast<double>(proj.y) - view.pixel.y;
    return static_cast<float>(std::sqrt(dx * dx + dy * dy));
}

bool Triangulator::project(int cam_index, const infer::Joint3D& joint, cv::Point2f& out) const {
    if (!joint.valid || cam_index < 0 || static_cast<std::size_t>(cam_index) >= cameras_.size()) {
        return false;
    }
    const auto& cam = cameras_[static_cast<std::size_t>(cam_index)];
    std::vector<cv::Point3d> obj{{joint.x, joint.y, joint.z}};
    std::vector<cv::Point2d> img;
    if (cam.fisheye) {
        cv::fisheye::projectPoints(obj, img, cam.rvec, cam.t, cam.K, cam.dist);
    } else {
        cv::projectPoints(obj, cam.rvec, cam.t, cam.K, cam.dist, img);
    }
    if (img.empty() || !std::isfinite(img[0].x) || !std::isfinite(img[0].y)) return false;
    out = cv::Point2f(static_cast<float>(img[0].x), static_cast<float>(img[0].y));
    return true;
}

}  // namespace fitra::lift
