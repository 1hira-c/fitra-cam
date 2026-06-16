#include "pipeline/intrinsic_calib_session.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <cstdio>
#include <sstream>

namespace fitra::pipeline {

const char* intrinsic_calib_state_name(IntrinsicCalibState s) {
    switch (s) {
        case IntrinsicCalibState::kIdle:       return "idle";
        case IntrinsicCalibState::kCollecting: return "collecting";
        case IntrinsicCalibState::kSolving:    return "solving";
        case IntrinsicCalibState::kSolved:     return "solved";
        case IntrinsicCalibState::kFailed:     return "failed";
    }
    return "unknown";
}

namespace {

std::string json_escape(const std::string& s) {
    std::string out;
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            default: out += c;
        }
    }
    return out;
}

cv::Point2f centroid(const std::vector<cv::Point2f>& pts) {
    cv::Point2f c(0, 0);
    for (const auto& p : pts) c += p;
    if (!pts.empty()) c *= 1.0f / static_cast<float>(pts.size());
    return c;
}

double bbox_area(const std::vector<cv::Point2f>& pts) {
    if (pts.empty()) return 0.0;
    float xmin = pts[0].x, xmax = pts[0].x, ymin = pts[0].y, ymax = pts[0].y;
    for (const auto& p : pts) {
        xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
    }
    return static_cast<double>(xmax - xmin) * static_cast<double>(ymax - ymin);
}

}  // namespace

IntrinsicCalibSession::IntrinsicCalibSession(IntrinsicCalibConfig cfg)
    : cfg_(std::move(cfg)),
      detector_(std::make_unique<lift::CharucoBoardDetector>(cfg_.board)) {}

void IntrinsicCalibSession::start() {
    std::lock_guard<std::mutex> g(mu_);
    state_ = IntrinsicCalibState::kCollecting;
}

void IntrinsicCalibSession::stop_collecting() {
    std::lock_guard<std::mutex> g(mu_);
    if (state_ == IntrinsicCalibState::kCollecting) state_ = IntrinsicCalibState::kIdle;
}

bool IntrinsicCalibSession::accept_view_(CamData& cam, const lift::CharucoView& v,
                                         int w, int h) {
    if (v.count() < cfg_.min_corners) return false;
    if (static_cast<int>(cam.views.size()) >= cfg_.max_views) return false;
    if (cam.img_w == 0) { cam.img_w = w; cam.img_h = h; }
    else if (cam.img_w != w || cam.img_h != h) return false;  // resolution change

    const cv::Point2f c = centroid(v.corners);
    const double area = bbox_area(v.corners);
    const double diag = std::sqrt(static_cast<double>(w) * w + static_cast<double>(h) * h);
    const double sep_min = cfg_.min_center_sep_frac * diag;

    bool novel = cam.views.empty();
    for (std::size_t i = 0; i < cam.centroids.size() && !novel; ++i) {
        const double d = cv::norm(c - cam.centroids[i]);
        const double a0 = cam.areas[i];
        const double area_diff = a0 > 0 ? std::abs(area - a0) / a0 : 1.0;
        if (d >= sep_min || area_diff >= cfg_.min_area_ratio_diff) novel = true;
    }
    if (!novel) return false;

    cam.views.push_back(v);
    cam.centroids.push_back(c);
    cam.areas.push_back(area);
    // 3x3 coverage cell of the centroid.
    const int cx = std::min(2, std::max(0, static_cast<int>(c.x * 3.0f / w)));
    const int cy = std::min(2, std::max(0, static_cast<int>(c.y * 3.0f / h)));
    cam.coverage_cells |= (1u << (cy * 3 + cx));
    return true;
}

bool IntrinsicCalibSession::ingest(std::size_t cam_idx, const lift::CharucoView& view,
                                   int img_w, int img_h) {
    std::lock_guard<std::mutex> g(mu_);
    if (state_ != IntrinsicCalibState::kCollecting) return false;
    if (cam_idx >= cfg_.num_cams) return false;
    return accept_view_(cams_[cam_idx], view, img_w, img_h);
}

void IntrinsicCalibSession::on_frame(std::size_t cam_idx, const cv::Mat& bgr, double) {
    {
        std::lock_guard<std::mutex> g(mu_);
        if (state_ != IntrinsicCalibState::kCollecting) return;
        if (cam_idx >= cfg_.num_cams) return;
    }
    // detector_ is not thread-safe; on_frame runs on the single capture thread.
    lift::CharucoView v = detector_->detect(bgr);
    std::lock_guard<std::mutex> g(mu_);
    if (state_ != IntrinsicCalibState::kCollecting) return;
    accept_view_(cams_[cam_idx], v, bgr.cols, bgr.rows);
}

bool IntrinsicCalibSession::solve_and_write(std::string& err) {
    std::map<std::size_t, CamData> snap;
    {
        std::lock_guard<std::mutex> g(mu_);
        state_ = IntrinsicCalibState::kSolving;
        snap = cams_;
    }

    lift::CalibrationSet out;
    out.schema = "fitra_calibration_v1";
    out.unit = "m";
    out.coordinate_system = "world: x/y on floor, z up (FitraWorld); extrinsics are T_cw";

    const bool fisheye = (cfg_.distortion_model == "fisheye");
    std::string msg;
    bool all_ok = true;

    for (std::size_t i = 0; i < cfg_.num_cams; ++i) {
        lift::CameraCalibration cam;
        cam.id = (i < cfg_.cam_ids.size() && !cfg_.cam_ids[i].empty())
                     ? cfg_.cam_ids[i] : ("cam" + std::to_string(i));
        cam.intrinsics.distortion_model = cfg_.distortion_model;

        auto it = snap.find(i);
        const int nviews = it == snap.end() ? 0 : static_cast<int>(it->second.views.size());
        if (nviews < cfg_.min_views) {
            all_ok = false;
            msg += cam.id + ": only " + std::to_string(nviews) + "/" +
                   std::to_string(cfg_.min_views) + " views; ";
            continue;
        }
        CamData& cd = it->second;

        std::vector<std::vector<cv::Point3f>> obj_pts;
        std::vector<std::vector<cv::Point2f>> img_pts;
        for (const auto& v : cd.views) {
            std::vector<cv::Point3f> o;
            std::vector<cv::Point2f> ip;
            detector_->match_points(v, o, ip);
            if (static_cast<int>(o.size()) < cfg_.min_corners) continue;
            obj_pts.push_back(std::move(o));
            img_pts.push_back(std::move(ip));
        }
        if (static_cast<int>(obj_pts.size()) < cfg_.min_views) {
            all_ok = false;
            msg += cam.id + ": too few usable views after match; ";
            continue;
        }

        cv::Size img_size(cd.img_w, cd.img_h);
        cv::Mat K, dist;
        double rms = 0.0;
        try {
            if (fisheye) {
                cv::Mat D;
                std::vector<cv::Mat> rvecs, tvecs;
                int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
                            cv::fisheye::CALIB_FIX_SKEW;
                rms = cv::fisheye::calibrate(obj_pts, img_pts, img_size, K, D,
                                             rvecs, tvecs, flags);
                dist = D.reshape(1, 1);  // 1x4
            } else {
                std::vector<cv::Mat> rvecs, tvecs;
                rms = cv::calibrateCamera(obj_pts, img_pts, img_size, K, dist,
                                          rvecs, tvecs);
                dist = dist.reshape(1, 1);
            }
        } catch (const std::exception& e) {
            all_ok = false;
            msg += cam.id + ": calibrate failed (" + e.what() + "); ";
            continue;
        }

        if (K.type() != CV_64F) K.convertTo(K, CV_64F);
        if (dist.type() != CV_64F) dist.convertTo(dist, CV_64F);
        cam.intrinsics.width = cd.img_w;
        cam.intrinsics.height = cd.img_h;
        cam.intrinsics.rms_px = rms;
        cam.intrinsics.source = fisheye ? "charuco_fisheye_cpp" : "charuco_pinhole_cpp";
        cam.intrinsics.K = K;
        cam.intrinsics.dist = dist;
        cd.rms_px = rms;
        cd.solved = true;
        out.cameras.push_back(std::move(cam));
    }

    if (!all_ok || out.cameras.empty()) {
        std::lock_guard<std::mutex> g(mu_);
        cams_ = snap;
        state_ = IntrinsicCalibState::kFailed;
        last_error_ = msg.empty() ? "no cameras solved" : msg;
        err = last_error_;
        return false;
    }

    try {
        lift::write_calibration(cfg_.out_path, out);
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> g(mu_);
        cams_ = snap;
        state_ = IntrinsicCalibState::kFailed;
        last_error_ = std::string("write failed: ") + e.what();
        err = last_error_;
        return false;
    }

    {
        std::lock_guard<std::mutex> g(mu_);
        cams_ = snap;
        state_ = IntrinsicCalibState::kSolved;
    }
    if (on_solved_) on_solved_();
    return true;
}

IntrinsicCalibState IntrinsicCalibSession::state() const {
    std::lock_guard<std::mutex> g(mu_);
    return state_;
}

std::size_t IntrinsicCalibSession::accepted_views(std::size_t cam_idx) const {
    std::lock_guard<std::mutex> g(mu_);
    auto it = cams_.find(cam_idx);
    return it == cams_.end() ? 0 : it->second.views.size();
}

std::string IntrinsicCalibSession::state_json() const {
    std::lock_guard<std::mutex> g(mu_);
    std::ostringstream os;
    os << "{\"method\":\"intrinsic\"";
    os << ",\"state\":\"" << intrinsic_calib_state_name(state_) << "\"";
    os << ",\"model\":\"" << cfg_.distortion_model << "\"";
    os << ",\"num_cams\":" << cfg_.num_cams;
    os << ",\"min_views\":" << cfg_.min_views;
    os << ",\"min_corners\":" << cfg_.min_corners;
    os << ",\"cameras\":[";
    for (std::size_t i = 0; i < cfg_.num_cams; ++i) {
        if (i) os << ",";
        auto it = cams_.find(i);
        int views = it == cams_.end() ? 0 : static_cast<int>(it->second.views.size());
        unsigned cells = it == cams_.end() ? 0u : it->second.coverage_cells;
        int touched = 0;
        for (int b = 0; b < 9; ++b) if (cells & (1u << b)) ++touched;
        double rms = it == cams_.end() ? 0.0 : it->second.rms_px;
        bool solved = it == cams_.end() ? false : it->second.solved;
        os << "{\"cam\":" << i
           << ",\"views\":" << views
           << ",\"coverage\":" << (touched / 9.0)
           << ",\"rms_px\":" << rms
           << ",\"solved\":" << (solved ? "true" : "false") << "}";
    }
    os << "]";
    if (state_ == IntrinsicCalibState::kFailed && !last_error_.empty()) {
        os << ",\"error\":\"" << json_escape(last_error_) << "\"";
    }
    os << "}";
    return os.str();
}

}  // namespace fitra::pipeline
