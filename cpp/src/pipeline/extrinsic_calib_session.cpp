#include "pipeline/extrinsic_calib_session.hpp"

#include "geom/world_convention.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <vector>

namespace fitra::pipeline {

const char* extrinsic_calib_state_name(ExtrinsicCalibState s) {
    switch (s) {
        case ExtrinsicCalibState::kIdle:       return "idle";
        case ExtrinsicCalibState::kCollecting: return "collecting";
        case ExtrinsicCalibState::kSolving:    return "solving";
        case ExtrinsicCalibState::kSolved:     return "solved";
        case ExtrinsicCalibState::kFailed:     return "failed";
    }
    return "unknown";
}

namespace {

cv::Matx44d controller_pose(const ControllerObservation& c) {
    return lift::pose_from_pos_quat(c.x, c.y, c.z, c.qx, c.qy, c.qz, c.qw);
}

// Minimal JSON string escaper. camera ids come from the calibration file and
// validate_calibration only guarantees non-empty, so an id containing '"' or
// '\' (or a control char) would otherwise break the response JSON.
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

// The controller poses fed to the hand-eye solver come from the VR runtime in
// the VMT/SteamVR world frame (Y-up RH, X-right, Z-back; see
// controller_pose_receiver.hpp), so the solved T_cam<-world is in those axes
// (typed geom::T_cam_vmtworld). Everything downstream -- floor calibration,
// triangulation, IK, the WebUI 3D viewer, SlimeVR output -- assumes the fitra
// world frame (Z-up RH, X-right, Y-forward). Re-express the *world* axes without
// moving the cameras by right-composing the single basis change
// geom::fitra_to_vmt_basis() (Vmt<-Fitra); the type system then enforces
// Cam<-Vmt * Vmt<-Fitra = Cam<-Fitra (a direction mistake won't compile).
geom::T_cam_world to_fitra_world(const geom::T_cam_vmtworld& T_cam_vmtworld) {
    return T_cam_vmtworld * geom::fitra_to_vmt_basis();
}

}  // namespace

ExtrinsicCalibSession::ExtrinsicCalibSession(ExtrinsicCalibConfig cfg)
    : cfg_(std::move(cfg)),
      detector_(std::make_unique<lift::AprilTagDetector>(cfg_.board)) {}

void ExtrinsicCalibSession::start() {
    std::lock_guard<std::mutex> g(mu_);
    state_ = ExtrinsicCalibState::kCollecting;
}

void ExtrinsicCalibSession::stop_collecting() {
    std::lock_guard<std::mutex> g(mu_);
    flush_all_bursts_();
    if (state_ == ExtrinsicCalibState::kCollecting) {
        state_ = ExtrinsicCalibState::kIdle;
    }
}

void ExtrinsicCalibSession::update_velocity_(const ControllerObservation& ctrl) {
    if (!have_prev_ctrl_) {
        prev_ctrl_      = ctrl;
        have_prev_ctrl_ = true;
        last_lin_vel_mps_ = last_ang_vel_dps_ = 0.0;
        return;
    }
    double dt = (ctrl.ts_ms - prev_ctrl_.ts_ms) / 1000.0;
    if (dt <= 1e-6) {
        // Same controller sample (e.g. a second face in the same frame) or out
        // of order — keep the previous velocity estimate and prev_ctrl_.
        return;
    }
    double dx = ctrl.x - prev_ctrl_.x;
    double dy = ctrl.y - prev_ctrl_.y;
    double dz = ctrl.z - prev_ctrl_.z;
    last_lin_vel_mps_ = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
    cv::Matx44d ra = lift::pose_from_pos_quat(0, 0, 0, prev_ctrl_.qx, prev_ctrl_.qy,
                                              prev_ctrl_.qz, prev_ctrl_.qw);
    cv::Matx44d rb = lift::pose_from_pos_quat(0, 0, 0, ctrl.qx, ctrl.qy,
                                              ctrl.qz, ctrl.qw);
    last_ang_vel_dps_ = lift::rotation_angle_deg(ra, rb) / dt;
    prev_ctrl_ = ctrl;
}

bool ExtrinsicCalibSession::ingest(std::size_t cam_idx, int face_id,
                                   const geom::T_cam_marker& T_cam_face,
                                   const ControllerObservation& ctrl) {
    std::lock_guard<std::mutex> g(mu_);
    if (state_ != ExtrinsicCalibState::kCollecting) return false;

    const bool had_velocity = have_prev_ctrl_;
    update_velocity_(ctrl);

    const bool gate = ctrl.running_ok && had_velocity &&
                      last_lin_vel_mps_ <= cfg_.lin_vel_max_mps &&
                      last_ang_vel_dps_ <= cfg_.ang_vel_max_dps;
    if (!gate) {
        // Motion (or lost tracking): the held pose ended — flush everything.
        flush_all_bursts_();
        return false;
    }

    GroupKey key{cam_idx, face_id};
    // Flush a stale prior burst first if the gap exceeds burst_gap_ms. We must
    // not hold a reference across the erase — std::map::erase invalidates it,
    // and writing back through that reference is UB. Use find() inside its own
    // scope, then re-bind a fresh reference for the append.
    {
        auto it = bursts_.find(key);
        if (it != bursts_.end() && !it->second.T_cam_face.empty() &&
            (ctrl.ts_ms - it->second.last_ts_ms) > cfg_.burst_gap_ms) {
            flush_burst_(key);
        }
    }
    Burst& b = bursts_[key];
    b.T_cam_face.push_back(T_cam_face.raw());
    b.T_world_controller.push_back(controller_pose(ctrl));
    b.last_ts_ms = ctrl.ts_ms;
    if (static_cast<int>(b.T_cam_face.size()) >= cfg_.burst_max) {
        flush_burst_(key);   // erases the entry; do not touch `b` afterwards
    }
    return true;
}

void ExtrinsicCalibSession::flush_burst_(const GroupKey& k) {
    auto it = bursts_.find(k);
    if (it == bursts_.end()) return;
    Burst& b = it->second;
    if (static_cast<int>(b.T_cam_face.size()) >= cfg_.burst_min) {
        lift::ExtrinsicSample s;
        s.cam_index = static_cast<int>(k.cam);
        s.face_id   = k.face;
        // Burst buffers stay raw (internal accumulation); wrap into the typed
        // sample at this single boundary.
        s.T_cam_marker = geom::T_cam_marker::from_raw(lift::average_poses(b.T_cam_face));
        s.T_world_controller =
            geom::T_world_controller::from_raw(lift::average_poses(b.T_world_controller));
        samples_.push_back(s);
        ++coverage_[k];
    }
    bursts_.erase(it);
}

void ExtrinsicCalibSession::flush_all_bursts_() {
    std::vector<GroupKey> keys;
    keys.reserve(bursts_.size());
    for (const auto& [k, _] : bursts_) keys.push_back(k);
    for (const auto& k : keys) flush_burst_(k);
}

void ExtrinsicCalibSession::on_frame(std::size_t cam_idx, const cv::Mat& bgr,
                                     const ControllerObservation& ctrl) {
    {
        std::lock_guard<std::mutex> g(mu_);
        if (state_ != ExtrinsicCalibState::kCollecting) return;
        if (cam_idx >= cfg_.intrinsics.cameras.size()) return;
    }
    const auto& cam = cfg_.intrinsics.cameras[cam_idx];
    // detector_ is non-thread-safe (cv::aruco::ArucoDetector carries state);
    // on_frame is called from the single driver frame-worker thread.
    auto dets = detector_->detect(bgr, cam.intrinsics.K, cam.intrinsics.dist);

    // Record the per-camera live summary for the UI (under the lock).
    {
        std::lock_guard<std::mutex> g(mu_);
        CamLive& live = cam_live_[cam_idx];
        live.last_ts_ms = ctrl.ts_ms;
        live.ctrl_running_ok = ctrl.running_ok;
        live.dets.clear();
        for (const auto& d : dets) {
            live.dets.push_back(LiveDetection{d.face_id, d.reproj_rms_px, d.pose_ok});
        }
        last_frame_ts_ms_ = std::max(last_frame_ts_ms_, ctrl.ts_ms);
    }

    for (const auto& d : dets) {
        if (!d.pose_ok) continue;
        if (d.reproj_rms_px > cfg_.max_pnp_reproj_px) continue;
        ingest(cam_idx, d.face_id, d.T_cam_face, ctrl);
    }
}

bool ExtrinsicCalibSession::solve_and_write(std::string& err) {
    std::vector<lift::ExtrinsicSample> samples_copy;
    {
        std::lock_guard<std::mutex> g(mu_);
        flush_all_bursts_();
        state_ = ExtrinsicCalibState::kSolving;
        samples_copy = samples_;
    }

    lift::ExtrinsicSolverOptions o;
    o.min_samples_per_group = cfg_.min_samples_per_group;
    lift::ExtrinsicSolution sol = lift::solve_extrinsics(samples_copy, o);

    if (!sol.ok) {
        std::lock_guard<std::mutex> g(mu_);
        solution_ = sol;
        state_ = ExtrinsicCalibState::kFailed;
        last_error_ = sol.message.empty() ? "extrinsic solve failed" : sol.message;
        err = last_error_;
        return false;
    }

    // The solver works in the VMT (Y-up) world frame the controller poses live
    // in. Keep `solution_` (→ extrinsics_json → the /extrinsic-calib scene) in
    // that frame: that verification scene overlays the solved cameras with the
    // live HMD/controller poses (also VMT Y-up) and a floor at Y=0, so the
    // cameras must stay in the same frame as the poses. Only the *persisted*
    // YAML is re-expressed into the fitra (Z-up) world frame below, since the
    // downstream pipeline (triangulation, IK, main ws3d viewer via the
    // hot-swap that reloads the written file, SlimeVR output) assumes Z-up.

    // Build the output CalibrationSet: copy intrinsics, fill in extrinsics.
    lift::CalibrationSet result = cfg_.intrinsics;
    // Persisted extrinsics are converted to the fitra Z-up world frame below,
    // so label the file accordingly regardless of what the intrinsics source
    // claimed.
    result.coordinate_system = "world: x/y measured on floor, z up; extrinsics are T_cw";
    for (auto& cam : result.cameras) cam.has_extrinsics = false;

    for (const auto& ce : sol.cameras) {
        if (ce.cam_index < 0 ||
            ce.cam_index >= static_cast<int>(result.cameras.size())) {
            continue;
        }
        auto& cam = result.cameras[ce.cam_index];
        cam.has_extrinsics = true;
        cam.extrinsics.method = "controller_marker_handeye";
        // VMT (Y-up) world → fitra (Z-up) world, persisted-file only.
        cam.extrinsics.T_cw =
            cv::Mat(to_fitra_world(ce.T_cam_world).raw()).clone();  // 4x4 CV_64F
        cv::Mat R = cam.extrinsics.T_cw(cv::Rect(0, 0, 3, 3));
        cv::Mat t = cam.extrinsics.T_cw(cv::Rect(3, 0, 1, 3));
        cv::Mat c = -R.t() * t;
        cam.extrinsics.camera_center_w = {c.at<double>(0), c.at<double>(1), c.at<double>(2)};
    }

    try {
        lift::write_calibration(cfg_.out_path, result);
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> g(mu_);
        solution_ = sol;
        state_ = ExtrinsicCalibState::kFailed;
        last_error_ = std::string("write failed: ") + e.what();
        err = last_error_;
        return false;
    }

    {
        std::lock_guard<std::mutex> g(mu_);
        solution_ = sol;
        state_ = ExtrinsicCalibState::kSolved;
    }
    if (on_solved_) on_solved_();
    return true;
}

std::string ExtrinsicCalibSession::extrinsics_json() const {
    std::lock_guard<std::mutex> g(mu_);
    std::ostringstream os;
    os << "{\"solved\":" << (state_ == ExtrinsicCalibState::kSolved ? "true" : "false");
    os << ",\"unit\":\"m\",\"cameras\":[";
    bool first = true;
    for (const auto& ce : solution_.cameras) {
        if (ce.cam_index < 0 ||
            ce.cam_index >= static_cast<int>(cfg_.intrinsics.cameras.size())) {
            continue;
        }
        const auto& cam = cfg_.intrinsics.cameras[ce.cam_index];
        const cv::Mat& K = cam.intrinsics.K;
        // extrinsics_json stays in the solver's VMT Y-up frame (the verification
        // scene overlays live VMT poses) — raw out, no basis change here.
        const cv::Matx44d& T = ce.T_cam_world.raw();
        // camera centre in world = -R^T t.
        cv::Matx33d R(T(0,0),T(0,1),T(0,2), T(1,0),T(1,1),T(1,2), T(2,0),T(2,1),T(2,2));
        cv::Vec3d t(T(0,3), T(1,3), T(2,3));
        cv::Vec3d c = -(R.t() * t);

        if (!first) os << ",";
        first = false;
        os << "{\"cam\":" << ce.cam_index
           << ",\"id\":\"" << json_escape(cam.id) << "\""
           << ",\"width\":" << cam.intrinsics.width
           << ",\"height\":" << cam.intrinsics.height
           << ",\"fx\":" << K.at<double>(0, 0)
           << ",\"fy\":" << K.at<double>(1, 1)
           << ",\"cx\":" << K.at<double>(0, 2)
           << ",\"cy\":" << K.at<double>(1, 2)
           << ",\"T_cam_world\":[";
        for (int r = 0; r < 4; ++r) {
            for (int cc = 0; cc < 4; ++cc) {
                if (r || cc) os << ",";
                os << T(r, cc);
            }
        }
        os << "],\"center\":[" << c[0] << "," << c[1] << "," << c[2] << "]}";
    }
    os << "]}";
    return os.str();
}

ExtrinsicCalibState ExtrinsicCalibSession::state() const {
    std::lock_guard<std::mutex> g(mu_);
    return state_;
}

std::size_t ExtrinsicCalibSession::sample_count() const {
    std::lock_guard<std::mutex> g(mu_);
    return samples_.size();
}

namespace {
constexpr double kTagRecencyMs = 750.0;  // a tag counts as "live" within this
}

const char* ExtrinsicCalibSession::gate_reason_() const {
    if (state_ != ExtrinsicCalibState::kCollecting) return "IDLE";
    bool any_tag = false;
    for (const auto& [cam, live] : cam_live_) {
        (void)cam;
        if (last_frame_ts_ms_ - live.last_ts_ms > kTagRecencyMs) continue;
        for (const auto& d : live.dets) {
            if (d.pose_ok) { any_tag = true; break; }
        }
        if (any_tag) break;
    }
    if (!any_tag) return "NO_TAG";
    if (!prev_ctrl_.running_ok) return "NO_POSE";
    if (!have_prev_ctrl_ ||
        last_lin_vel_mps_ > cfg_.lin_vel_max_mps ||
        last_ang_vel_dps_ > cfg_.ang_vel_max_dps) {
        return "MOVING";
    }
    return "GOOD";
}

std::string ExtrinsicCalibSession::state_json() const {
    std::lock_guard<std::mutex> g(mu_);
    std::ostringstream os;
    os << "{";
    os << "\"state\":\"" << extrinsic_calib_state_name(state_) << "\"";
    os << ",\"samples\":" << samples_.size();
    os << ",\"min_samples\":" << cfg_.min_samples_per_group;
    os << ",\"num_cams\":" << cfg_.intrinsics.cameras.size();
    os << ",\"lin_vel_mps\":" << last_lin_vel_mps_;
    os << ",\"ang_vel_dps\":" << last_ang_vel_dps_;
    os << ",\"gate\":{\"lin_max\":" << cfg_.lin_vel_max_mps
       << ",\"ang_max\":" << cfg_.ang_vel_max_dps << "}";
    os << ",\"gate_reason\":\"" << gate_reason_() << "\"";

    // Per-camera live detection summary (for the glanceable UI).
    os << ",\"detections\":[";
    bool dfirst = true;
    for (const auto& [cam, live] : cam_live_) {
        if (!dfirst) os << ",";
        dfirst = false;
        os << "{\"cam\":" << cam
           << ",\"age_ms\":" << (last_frame_ts_ms_ - live.last_ts_ms)
           << ",\"ctrl_ok\":" << (live.ctrl_running_ok ? "true" : "false")
           << ",\"faces\":[";
        for (std::size_t i = 0; i < live.dets.size(); ++i) {
            if (i) os << ",";
            os << "{\"id\":" << live.dets[i].face_id
               << ",\"reproj\":" << live.dets[i].reproj_rms_px
               << ",\"ok\":" << (live.dets[i].pose_ok ? "true" : "false") << "}";
        }
        os << "]}";
    }
    os << "]";

    // Configured marker faces (so the UI can render a full coverage matrix
    // including not-yet-seen (cam,face) cells).
    os << ",\"faces\":[";
    for (std::size_t i = 0; i < cfg_.board.faces.size(); ++i) {
        if (i) os << ",";
        os << cfg_.board.faces[i].face_id;
    }
    os << "]";

    os << ",\"coverage\":[";
    bool first = true;
    for (const auto& [k, n] : coverage_) {
        if (!first) os << ",";
        first = false;
        os << "{\"cam\":" << k.cam << ",\"face\":" << k.face << ",\"count\":" << n << "}";
    }
    os << "]";

    os << ",\"cameras\":[";
    first = true;
    for (const auto& ce : solution_.cameras) {
        if (!first) os << ",";
        first = false;
        os << "{\"cam\":" << ce.cam_index
           << ",\"n_faces\":" << ce.n_faces
           << ",\"n_samples\":" << ce.n_samples
           << ",\"face_spread_trans_m\":" << ce.face_spread_trans_m
           << ",\"face_spread_rot_deg\":" << ce.face_spread_rot_deg << "}";
    }
    os << "]";
    os << "}";
    return os.str();
}

}  // namespace fitra::pipeline
