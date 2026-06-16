#include "pipeline/floor_calib_session.hpp"

#include <algorithm>
#include <cstdio>
#include <sstream>

namespace fitra::pipeline {

const char* floor_calib_state_name(FloorCalibState s) {
    switch (s) {
        case FloorCalibState::kIdle:       return "idle";
        case FloorCalibState::kCollecting: return "collecting";
        case FloorCalibState::kSolving:    return "solving";
        case FloorCalibState::kSolved:     return "solved";
        case FloorCalibState::kFailed:     return "failed";
    }
    return "unknown";
}

namespace {

std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

// Build a detector board from the map's tags (ID + size), CLAHE forced on.
lift::MarkerBoardConfig board_from(const FloorCalibConfig& cfg) {
    lift::MarkerBoardConfig b = cfg.board;
    if (b.faces.empty()) {
        for (const auto& t : cfg.map.tags) {
            b.faces.push_back(lift::MarkerFace{t.id, t.size_m});
        }
    }
    b.use_clahe = true;
    // Per-frame PnP (the reproj quality gate) must match the intrinsics model.
    // Cameras share lenses here; treat fisheye if forced or any camera is fisheye.
    bool any_fisheye = cfg.fisheye;
    for (const auto& cam : cfg.intrinsics.cameras) {
        any_fisheye = any_fisheye || cam.intrinsics.is_fisheye();
    }
    b.fisheye = any_fisheye;
    return b;
}

}  // namespace

FloorCalibSession::FloorCalibSession(FloorCalibConfig cfg)
    : cfg_(std::move(cfg)),
      detector_(std::make_unique<lift::AprilTagDetector>(board_from(cfg_))) {}

void FloorCalibSession::start() {
    std::lock_guard<std::mutex> g(mu_);
    state_ = FloorCalibState::kCollecting;
}

void FloorCalibSession::stop_collecting() {
    std::lock_guard<std::mutex> g(mu_);
    if (state_ == FloorCalibState::kCollecting) state_ = FloorCalibState::kIdle;
}

bool FloorCalibSession::ingest(std::size_t cam_idx, int tag_id,
                               const std::array<cv::Point2f, 4>& corners) {
    std::lock_guard<std::mutex> g(mu_);
    if (state_ != FloorCalibState::kCollecting) return false;
    CornerAccum& a = accum_[GroupKey{cam_idx, tag_id}];
    if (a.count >= cfg_.burst_max) return true;  // saturated; keep state
    for (int i = 0; i < 4; ++i) {
        a.sum[i].x += corners[i].x;
        a.sum[i].y += corners[i].y;
    }
    ++a.count;
    return true;
}

void FloorCalibSession::on_frame(std::size_t cam_idx, const cv::Mat& bgr,
                                 double ts_ms) {
    {
        std::lock_guard<std::mutex> g(mu_);
        if (state_ != FloorCalibState::kCollecting) return;
        if (cam_idx >= cfg_.intrinsics.cameras.size()) return;
    }
    const auto& cam = cfg_.intrinsics.cameras[cam_idx];
    // detector_ is not thread-safe; on_frame runs on the single capture thread.
    auto dets = detector_->detect(bgr, cam.intrinsics.K, cam.intrinsics.dist);

    {
        std::lock_guard<std::mutex> g(mu_);
        CamLive& live = cam_live_[cam_idx];
        live.last_ts_ms = ts_ms;
        live.dets.clear();
        for (const auto& d : dets) {
            live.dets.push_back(LiveDetection{d.face_id, d.reproj_rms_px, d.pose_ok});
        }
        last_frame_ts_ms_ = std::max(last_frame_ts_ms_, ts_ms);
    }

    for (const auto& d : dets) {
        if (!d.pose_ok) continue;
        if (d.reproj_rms_px > cfg_.max_pnp_reproj_px) continue;
        ingest(cam_idx, d.face_id, d.corners);
    }
}

bool FloorCalibSession::solve_and_write(std::string& err) {
    std::map<GroupKey, CornerAccum> accum_copy;
    {
        std::lock_guard<std::mutex> g(mu_);
        state_ = FloorCalibState::kSolving;
        accum_copy = accum_;
    }

    // Only solve the cameras actually in use this session (num_cams), not every
    // camera the intrinsics file happens to list — an unused camera would have
    // no observations and fail the whole solve.
    std::size_t n_cams = cfg_.num_cams != 0 ? cfg_.num_cams
                                            : cfg_.intrinsics.cameras.size();
    if (n_cams > cfg_.intrinsics.cameras.size()) n_cams = cfg_.intrinsics.cameras.size();
    std::vector<lift::FloorCameraInput> inputs(n_cams);
    for (std::size_t i = 0; i < n_cams; ++i) {
        const auto& intr = cfg_.intrinsics.cameras[i].intrinsics;
        inputs[i].cam_index = static_cast<int>(i);
        inputs[i].K       = intr.K;
        inputs[i].dist    = intr.dist;
        // Data-driven from the calibration's distortion model; cfg_.fisheye is a
        // legacy force-all override.
        inputs[i].fisheye = cfg_.fisheye || intr.is_fisheye();
    }
    for (const auto& [key, acc] : accum_copy) {
        if (acc.count < cfg_.burst_min) continue;
        if (key.cam >= n_cams) continue;
        lift::FloorTagObservation ob;
        ob.id = key.tag;
        const double inv = 1.0 / static_cast<double>(acc.count);
        for (int i = 0; i < 4; ++i) {
            ob.corners[i] = cv::Point2f(static_cast<float>(acc.sum[i].x * inv),
                                        static_cast<float>(acc.sum[i].y * inv));
        }
        inputs[key.cam].obs.push_back(ob);
    }

    lift::FloorExtrinsicSolution sol =
        lift::solve_floor_extrinsics(inputs, cfg_.map, cfg_.solver);

    if (!sol.ok) {
        std::lock_guard<std::mutex> g(mu_);
        solution_ = sol;
        state_ = FloorCalibState::kFailed;
        last_error_ = sol.message.empty() ? "floor extrinsic solve failed" : sol.message;
        err = last_error_;
        return false;
    }

    // The floor map IS the fitra Z-up world, so the solved T_cam←world is
    // persisted with NO basis change (unlike the controller-marker path).
    lift::CalibrationSet result =
        cfg_.out_intrinsics.cameras.empty() ? cfg_.intrinsics : cfg_.out_intrinsics;
    result.coordinate_system =
        "world: x/y on floor, z up (FitraWorld); extrinsics are T_cw";
    for (auto& cam : result.cameras) cam.has_extrinsics = false;

    for (const auto& ce : sol.cameras) {
        if (!ce.solved) continue;
        if (ce.cam_index < 0 ||
            ce.cam_index >= static_cast<int>(cfg_.intrinsics.cameras.size())) {
            continue;
        }
        // Match by camera id, not index: `result` may be a SEPARATE
        // out_intrinsics file whose camera order/count need not match the PnP
        // intrinsics the solver indexed. Index-based assignment would attach a
        // pose to the wrong camera (or out of range) silently.
        const std::string& id = cfg_.intrinsics.cameras[ce.cam_index].id;
        auto it = std::find_if(result.cameras.begin(), result.cameras.end(),
                               [&](const lift::CameraCalibration& c) { return c.id == id; });
        if (it == result.cameras.end()) {
            err += "output intrinsics has no camera '" + id + "'; ";
            continue;
        }
        auto& cam = *it;
        cam.has_extrinsics = true;
        cam.extrinsics.method = "floor_apriltag_pnp";
        cam.extrinsics.T_cw = cv::Mat(ce.T_cam_world.raw()).clone();  // 4x4 CV_64F
        cv::Mat R = cam.extrinsics.T_cw(cv::Rect(0, 0, 3, 3));
        cv::Mat t = cam.extrinsics.T_cw(cv::Rect(3, 0, 1, 3));
        cv::Mat c = -R.t() * t;
        cam.extrinsics.camera_center_w = {c.at<double>(0), c.at<double>(1), c.at<double>(2)};
    }
    // If a separate out_intrinsics dropped any solved camera, fail loudly rather
    // than writing a calibration that silently lacks an extrinsic.
    if (!err.empty()) {
        std::lock_guard<std::mutex> g(mu_);
        solution_ = sol;
        state_ = FloorCalibState::kFailed;
        last_error_ = err;
        return false;
    }

    try {
        lift::write_calibration(cfg_.out_path, result);
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> g(mu_);
        solution_ = sol;
        state_ = FloorCalibState::kFailed;
        last_error_ = std::string("write failed: ") + e.what();
        err = last_error_;
        return false;
    }

    {
        std::lock_guard<std::mutex> g(mu_);
        solution_ = sol;
        state_ = FloorCalibState::kSolved;
    }
    if (on_solved_) on_solved_();
    return true;
}

FloorCalibState FloorCalibSession::state() const {
    std::lock_guard<std::mutex> g(mu_);
    return state_;
}

std::size_t FloorCalibSession::ready_group_count() const {
    std::lock_guard<std::mutex> g(mu_);
    std::size_t n = 0;
    for (const auto& [k, a] : accum_) {
        (void)k;
        if (a.count >= cfg_.burst_min) ++n;
    }
    return n;
}

std::string FloorCalibSession::state_json() const {
    std::lock_guard<std::mutex> g(mu_);
    std::ostringstream os;
    std::size_t ready = 0;
    for (const auto& [k, a] : accum_) {
        (void)k;
        if (a.count >= cfg_.burst_min) ++ready;
    }

    os << "{\"method\":\"floor\"";
    os << ",\"state\":\"" << floor_calib_state_name(state_) << "\"";
    os << ",\"samples\":" << ready;
    os << ",\"burst_min\":" << cfg_.burst_min;
    os << ",\"num_cams\":" << cfg_.intrinsics.cameras.size();

    os << ",\"tags\":[";
    for (std::size_t i = 0; i < cfg_.map.tags.size(); ++i) {
        if (i) os << ",";
        os << cfg_.map.tags[i].id;
    }
    os << "]";

    os << ",\"detections\":[";
    bool dfirst = true;
    for (const auto& [cam, live] : cam_live_) {
        if (!dfirst) os << ",";
        dfirst = false;
        os << "{\"cam\":" << cam
           << ",\"age_ms\":" << (last_frame_ts_ms_ - live.last_ts_ms)
           << ",\"tags\":[";
        for (std::size_t i = 0; i < live.dets.size(); ++i) {
            if (i) os << ",";
            os << "{\"id\":" << live.dets[i].tag_id
               << ",\"reproj\":" << live.dets[i].reproj_rms_px
               << ",\"ok\":" << (live.dets[i].pose_ok ? "true" : "false") << "}";
        }
        os << "]}";
    }
    os << "]";

    os << ",\"coverage\":[";
    bool cfirst = true;
    for (const auto& [k, a] : accum_) {
        if (!cfirst) os << ",";
        cfirst = false;
        os << "{\"cam\":" << k.cam << ",\"tag\":" << k.tag << ",\"count\":" << a.count << "}";
    }
    os << "]";

    os << ",\"cameras\":[";
    bool first = true;
    for (const auto& ce : solution_.cameras) {
        if (!first) os << ",";
        first = false;
        os << "{\"cam\":" << ce.cam_index
           << ",\"n_tags\":" << ce.n_tags
           << ",\"reproj_rms_px\":" << ce.reproj_rms_px
           << ",\"planar_degenerate\":" << (ce.planar_degenerate ? "true" : "false")
           << ",\"plane_thickness_m\":" << ce.plane_thickness_m
           << ",\"solved\":" << (ce.solved ? "true" : "false") << "}";
    }
    os << "]";

    if (state_ == FloorCalibState::kFailed && !last_error_.empty()) {
        os << ",\"error\":\"" << json_escape(last_error_) << "\"";
    }
    os << "}";
    return os.str();
}

}  // namespace fitra::pipeline
