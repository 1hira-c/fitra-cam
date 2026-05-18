#include "pipeline/calibration_session.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <sys/wait.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include "util/logging.hpp"

namespace fitra::pipeline {

namespace fs = std::filesystem;

const std::array<lift::TargetPose, CalibrationSession::kPoseCount>
CalibrationSession::kSequence{{
    lift::TargetPose::kStanding,
    lift::TargetPose::kTPose,
    lift::TargetPose::kElbowFlex,
    lift::TargetPose::kKneeFlex,
}};

const char* calib_state_name(CalibState s) {
    switch (s) {
        case CalibState::kIdle:       return "idle";
        case CalibState::kReady:      return "ready";
        case CalibState::kAwaitHold:  return "await_hold";
        case CalibState::kRecording:  return "recording";
        case CalibState::kFinalizing: return "finalizing";
        case CalibState::kAnalyzing:  return "analyzing";
        case CalibState::kReview:     return "review";
        case CalibState::kApproving:  return "approving";
        case CalibState::kApproved:   return "approved";
        case CalibState::kCanceled:   return "canceled";
        case CalibState::kFailed:     return "failed";
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
                    std::snprintf(buf, sizeof(buf), "\\u%04x",
                                  static_cast<unsigned>(c) & 0xff);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

std::string shell_quote(const std::string& s) {
    // Wrap in single quotes; escape embedded single quotes by closing/escaping.
    std::string out;
    out.reserve(s.size() + 2);
    out += '\'';
    for (char c : s) {
        if (c == '\'') {
            out += "'\\''";
        } else {
            out += c;
        }
    }
    out += '\'';
    return out;
}

}  // namespace

CalibrationSession::CalibrationSession() : recognizer_{15.0} {}

CalibrationSession::~CalibrationSession() {
    if (finalize_thread_.joinable()) finalize_thread_.join();
    if (analyzer_thread_.joinable())  analyzer_thread_.join();
}

void CalibrationSession::set_fps_hint(double fps) {
    std::lock_guard<std::mutex> g(mu_);
    recognizer_.set_fps_hint(fps);
}

std::string CalibrationSession::sanitize_id(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (char c : in) {
        if ((c >= 'a' && c <= 'z') ||
            (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') ||
            c == '_' || c == '-') {
            out += c;
        }
    }
    if (out.empty()) out = "subject";
    return out;
}

std::string CalibrationSession::iso_timestamp_now() {
    std::time_t t = std::time(nullptr);
    std::tm lt{};
    localtime_r(&t, &lt);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &lt);
    return std::string{buf};
}

void CalibrationSession::log_line(const std::string& s) const {
    if (log_fn_) log_fn_(s);
    FITRA_LOG_INFO("[calib] {}", s);
}

void CalibrationSession::set_state_(CalibState s) {
    state_.store(s);
    log_line(std::string{"state="} + calib_state_name(s));
}

fs::path CalibrationSession::session_dir() const {
    std::lock_guard<std::mutex> g(mu_);
    return session_dir_;
}

bool CalibrationSession::preflight(const CalibPreflight& in, std::string& err) {
    err.clear();
    CalibState s = state_.load();
    if (s != CalibState::kIdle && s != CalibState::kCanceled && s != CalibState::kApproved
        && s != CalibState::kFailed) {
        err = "session is busy; cancel first";
        return false;
    }
    if (in.subject_id.empty()) { err = "subject_id required"; return false; }
    if (in.subject_height_m < 1.0 || in.subject_height_m > 2.3) {
        err = "subject_height_m must be in [1.0, 2.3] m";
        return false;
    }
    if (in.calib_yaml.empty() || !fs::exists(in.calib_yaml)) {
        err = "calibration YAML not found: " + in.calib_yaml;
        return false;
    }
    if (in.det_engine.empty() || !fs::exists(in.det_engine)) {
        err = "det engine not found: " + in.det_engine;
        return false;
    }
    if (in.pose_engine.empty() || !fs::exists(in.pose_engine)) {
        err = "pose engine not found: " + in.pose_engine;
        return false;
    }
    if (in.dump_tool_path.empty() || !fs::exists(in.dump_tool_path)) {
        err = "dump_keypoints_3d not found: " + in.dump_tool_path;
        return false;
    }

    CalibPreflight applied;  // copy of pre_ after sanitize, for the callback
    {
        std::lock_guard<std::mutex> g(mu_);
        pre_ = in;
        pre_.subject_id = sanitize_id(pre_.subject_id);
        created_at_ = iso_timestamp_now();

        subject_dir_ = fs::path(pre_.subjects_dir) / pre_.subject_id;
        std::time_t t = std::time(nullptr);
        std::tm lt{};
        localtime_r(&t, &lt);
        char tbuf[32];
        std::strftime(tbuf, sizeof(tbuf), "%Y%m%d_%H%M%S", &lt);
        session_dir_ = subject_dir_ / "sessions" / tbuf;

        std::error_code ec;
        fs::create_directories(session_dir_ / "raw", ec);
        if (ec) {
            err = "cannot create session dir: " + ec.message();
            return false;
        }
        fs::create_directories(session_dir_ / "overlays", ec);

        subject_profile_yaml_ = session_dir_ / "subject_profile.yaml";
        quality_json_path_    = session_dir_ / "quality.json";
        pose_session_path_    = session_dir_ / "pose_session.json";
        joints3d_path_        = session_dir_ / "joints3d.jsonl";
        summary_path_         = session_dir_ / "summary.json";
        overlay_dir_          = session_dir_ / "overlays";
        latest_profile_path_  = subject_dir_ / "latest_profile.yaml";

        for (std::size_t i = 0; i < kPoseCount; ++i) {
            records_[i] = CalibPoseRecord{};
            records_[i].name = lift::target_pose_name(kSequence[i]);
            for (auto& b : buffers_[i]) b.clear();
        }
        recognizer_.set_required_hold_sec(pre_.required_hold_sec);
        target_pose_idx_ = 0;
        analyze_log_tail_.clear();
        analyze_exit_code_ = -1;
        quality_status_.clear();
        quality_ok_ = false;
        quality_summary_.clear();
        approval_done_.store(false);
        last_error_.clear();
        applied = pre_;
    }
    set_state_(CalibState::kReady);
    // Callback dispatched without mu_ so it can safely re-enter the session
    // (e.g., to read state_json). Used by main to prime IkSolver with the
    // subject height before recording starts.
    if (on_preflight_) on_preflight_(applied);
    return true;
}

bool CalibrationSession::start(std::string& err) {
    err.clear();
    if (state_.load() != CalibState::kReady) {
        err = "preflight required";
        return false;
    }
    started_once_.store(true);
    advance_to_pose_(0);
    return true;
}

void CalibrationSession::advance_to_pose_(std::size_t idx) {
    std::lock_guard<std::mutex> g(mu_);
    target_pose_idx_ = idx;
    recognizer_.set_target(kSequence[idx]);
    recognizer_.reset();
    set_state_(CalibState::kAwaitHold);
    log_line(std::string{"AWAIT "} + records_[idx].name);
}

bool CalibrationSession::retake(const std::string& pose_name, std::string& err) {
    err.clear();
    std::size_t found = kPoseCount;
    for (std::size_t i = 0; i < kPoseCount; ++i) {
        if (records_[i].name == pose_name) { found = i; break; }
    }
    if (found == kPoseCount) { err = "unknown pose: " + pose_name; return false; }
    CalibState s = state_.load();
    if (s == CalibState::kAnalyzing || s == CalibState::kApproving
        || s == CalibState::kFinalizing) {
        err = "session busy";
        return false;
    }
    {
        std::lock_guard<std::mutex> g(mu_);
        for (auto& b : buffers_[found]) b.clear();
        records_[found] = CalibPoseRecord{};
        records_[found].name = lift::target_pose_name(kSequence[found]);
    }
    advance_to_pose_(found);
    return true;
}

bool CalibrationSession::cancel(std::string& err) {
    err.clear();
    set_state_(CalibState::kCanceled);
    std::lock_guard<std::mutex> g(mu_);
    for (auto& p : buffers_) for (auto& b : p) b.clear();
    return true;
}

bool CalibrationSession::approve(bool force, std::string& err) {
    err.clear();
    if (state_.load() != CalibState::kReview) {
        err = "session not in review state";
        return false;
    }
    std::string qstatus;
    fs::path subject_profile_yaml;
    fs::path subject_dir;
    fs::path latest_profile_path;
    {
        std::lock_guard<std::mutex> g(mu_);
        qstatus = quality_status_;
        subject_profile_yaml = subject_profile_yaml_;
        subject_dir = subject_dir_;
        latest_profile_path = latest_profile_path_;
    }
    if (qstatus == "fail") {
        err = "quality=fail cannot be approved";
        return false;
    }
    if (qstatus == "warn" && !force) {
        err = "quality=warn requires force=true";
        return false;
    }

    set_state_(CalibState::kApproving);
    try {
        lift::SubjectProfile profile =
            lift::load_subject_profile(subject_profile_yaml.string());
        std::error_code ec;
        fs::create_directories(subject_dir, ec);
        fs::path tmp = latest_profile_path;
        tmp += ".tmp";
        lift::write_subject_profile(tmp.string(), profile);
        fs::rename(tmp, latest_profile_path, ec);
        if (ec) {
            err = "rename failed: " + ec.message();
            set_state_(CalibState::kFailed);
            return false;
        }
        log_line("approved profile -> " + latest_profile_path.string());
        approval_done_.store(true);
        set_state_(CalibState::kApproved);
        if (on_approved_) on_approved_(profile);
        if (auto_exit_.load() && on_exit_requested_) on_exit_requested_();
        return true;
    } catch (const std::exception& e) {
        err = e.what();
        {
            std::lock_guard<std::mutex> g(mu_);
            last_error_ = err;
        }
        set_state_(CalibState::kFailed);
        return false;
    }
}

void CalibrationSession::on_frame(std::size_t cam_idx, const cv::Mat& bgr, double ts_ms) {
    if (state_.load() != CalibState::kRecording) return;
    if (cam_idx >= 2) return;
    if (bgr.empty()) return;

    std::lock_guard<std::mutex> g(mu_);
    auto& buf = buffers_[target_pose_idx_][cam_idx];
    if (static_cast<int>(buf.size()) >= pre_.recording_frames_per_cam) return;
    FrameItem item;
    item.bgr = bgr.clone();
    item.ts_ms = ts_ms;
    buf.push_back(std::move(item));

    bool full0 = static_cast<int>(buffers_[target_pose_idx_][0].size())
                  >= pre_.recording_frames_per_cam;
    bool full1 = static_cast<int>(buffers_[target_pose_idx_][1].size())
                  >= pre_.recording_frames_per_cam;
    if (full0 && full1) {
        // Defer pose-finalize work to a background thread to avoid blocking
        // the pipeline tap.
        std::size_t idx = target_pose_idx_;
        log_line("buffer full for " + records_[idx].name);
        if (idx + 1 < kPoseCount) {
            // Move to next pose AwaitHold immediately; finalize MP4 lazily
            // when all poses are buffered.
            target_pose_idx_ = idx + 1;
            recognizer_.set_target(kSequence[target_pose_idx_]);
            recognizer_.reset();
            set_state_(CalibState::kAwaitHold);
            log_line(std::string{"AWAIT "} + records_[target_pose_idx_].name);
        } else {
            set_state_(CalibState::kFinalizing);
            launch_finalize_();
        }
    }
}

void CalibrationSession::on_skeleton3d(const infer::Skeleton3D& skel,
                                        double bone_drift_pct) {
    last_bone_drift_pct_ = bone_drift_pct;
    auto s = state_.load();
    if (s != CalibState::kAwaitHold && s != CalibState::kRecording) return;

    lift::PoseDetectionState det;
    {
        std::lock_guard<std::mutex> g(mu_);
        det = recognizer_.update(skel, bone_drift_pct);
        last_detection_ = det;
    }
    if (s == CalibState::kAwaitHold && det.in_band
        && det.consecutive_ok >= det.consecutive_required) {
        begin_recording_();
    }
}

void CalibrationSession::begin_recording_() {
    log_line(std::string{"RECORDING "} + records_[target_pose_idx_].name);
    set_state_(CalibState::kRecording);
}

void CalibrationSession::launch_finalize_() {
    if (finalize_thread_.joinable()) finalize_thread_.join();
    finalize_thread_ = std::thread{&CalibrationSession::finalize_thread_main_, this};
}

void CalibrationSession::finalize_thread_main_() {
    try {
        // Snapshot pose names + take ownership of the per-pose frame buffers
        // under the lock so /api/calib/state polling and the (rare) cancel()
        // see a consistent view. After this block, buffers_ is empty and the
        // local copy is owned exclusively by this thread.
        std::array<std::array<std::vector<FrameItem>, 2>, kPoseCount> local_buffers;
        std::array<std::string, kPoseCount> pose_names;
        {
            std::lock_guard<std::mutex> g(mu_);
            for (std::size_t pi = 0; pi < kPoseCount; ++pi) {
                pose_names[pi] = records_[pi].name;
                for (std::size_t ci = 0; ci < 2; ++ci) {
                    local_buffers[pi][ci] = std::move(buffers_[pi][ci]);
                    buffers_[pi][ci].clear();
                }
            }
        }

        // Write MP4s with no lock held -- VideoWriter is slow and we don't
        // want to block the WebUI poll.
        int width = 0, height = 0;
        for (std::size_t pi = 0; pi < kPoseCount; ++pi) {
            for (std::size_t ci = 0; ci < 2; ++ci) {
                auto& buf = local_buffers[pi][ci];
                if (buf.empty()) continue;
                if (width == 0) {
                    width = buf.front().bgr.cols;
                    height = buf.front().bgr.rows;
                }
                double fps = 0.0;
                if (buf.size() >= 2) {
                    double span_ms = buf.back().ts_ms - buf.front().ts_ms;
                    if (span_ms > 0.0) {
                        fps = (static_cast<double>(buf.size()) - 1.0) / (span_ms / 1000.0);
                    }
                }
                if (fps < 1.0) fps = 15.0;
                std::string rel = std::string{"raw/"} + pose_names[pi] + "_cam"
                                + std::to_string(ci) + ".mp4";
                fs::path out_path = session_dir_ / rel;
                int fourcc = cv::VideoWriter::fourcc('m','p','4','v');
                cv::VideoWriter writer{out_path.string(), fourcc, fps,
                                       {buf.front().bgr.cols, buf.front().bgr.rows}};
                if (!writer.isOpened()) {
                    throw std::runtime_error("VideoWriter open failed: " + out_path.string());
                }
                for (auto& it : buf) writer.write(it.bgr);
                writer.release();

                // Commit metadata under the lock so concurrent state_json()
                // calls see a coherent record.
                {
                    std::lock_guard<std::mutex> g(mu_);
                    records_[pi].clips.clear();
                    records_[pi].clips.push_back(rel);
                    records_[pi].frames[ci] = static_cast<int>(buf.size());
                    records_[pi].fps[ci]    = fps;
                    records_[pi].recorded   = true;
                }
                buf.clear();  // free local memory immediately
            }

            // Normalize clip ordering (cam0 then cam1) once both have written.
            std::vector<std::string> clips;
            for (std::size_t ci = 0; ci < 2; ++ci) {
                std::string rel = std::string{"raw/"} + pose_names[pi] + "_cam"
                                  + std::to_string(ci) + ".mp4";
                if (fs::exists(session_dir_ / rel)) clips.push_back(rel);
            }
            {
                std::lock_guard<std::mutex> g(mu_);
                records_[pi].clips = std::move(clips);
            }
        }
        if (width == 0) width = 640;
        if (height == 0) height = 480;

        // Build pose_session.json under the lock so the records_ snapshot is
        // consistent with what we just wrote (and with what state_json sees).
        std::string ps_json;
        {
            std::lock_guard<std::mutex> g(mu_);
            std::ostringstream ps;
            ps << "{\n";
            ps << "  \"schema\": \"fitra_pose_session_v1\",\n";
            ps << "  \"subject_id\": \"" << json_escape(pre_.subject_id) << "\",\n";
            ps << "  \"created_at\": \"" << json_escape(created_at_) << "\",\n";
            ps << "  \"subject_height_m\": " << pre_.subject_height_m << ",\n";
            ps << "  \"calib\": \"" << json_escape(pre_.calib_yaml) << "\",\n";
            ps << "  \"camera_count\": 2,\n";
            ps << "  \"cameras\": [\n";
            ps << "    {\"id\": \"cam0\"},\n";
            ps << "    {\"id\": \"cam1\"}\n";
            ps << "  ],\n";
            ps << "  \"poses\": [\n";
            bool first = true;
            for (std::size_t pi = 0; pi < kPoseCount; ++pi) {
                const auto& rec = records_[pi];
                if (!rec.recorded) continue;
                if (!first) ps << ",\n";
                first = false;
                ps << "    {\n";
                ps << "      \"name\": \"" << json_escape(rec.name) << "\",\n";
                ps << "      \"clips\": [";
                for (std::size_t ci = 0; ci < rec.clips.size(); ++ci) {
                    if (ci > 0) ps << ", ";
                    ps << "\"" << json_escape(rec.clips[ci]) << "\"";
                }
                ps << "],\n";
                ps << "      \"frames\": [" << rec.frames[0] << ", " << rec.frames[1] << "],\n";
                ps << "      \"fps\": [" << rec.fps[0] << ", " << rec.fps[1] << "],\n";
                ps << "      \"status\": \"recorded\"\n";
                ps << "    }";
            }
            ps << "\n  ]\n";
            ps << "}\n";
            ps_json = ps.str();
        }
        {
            std::ofstream out{pose_session_path_};
            out << ps_json;
        }
        log_line("pose_session.json written");

        set_state_(CalibState::kAnalyzing);
        launch_analyzer_();
    } catch (const std::exception& e) {
        {
            std::lock_guard<std::mutex> g(mu_);
            last_error_ = e.what();
        }
        log_line(std::string{"finalize failed: "} + e.what());
        set_state_(CalibState::kFailed);
    }
}

void CalibrationSession::launch_analyzer_() {
    if (analyzer_thread_.joinable()) analyzer_thread_.join();
    analyzer_thread_ = std::thread{&CalibrationSession::analyzer_thread_main_, this};
}

void CalibrationSession::analyzer_thread_main_() {
    std::ostringstream cmd;
    cmd << shell_quote(pre_.dump_tool_path)
        << " --pose-session " << shell_quote(pose_session_path_.string())
        << " --calib "        << shell_quote(pre_.calib_yaml)
        << " --det-engine "   << shell_quote(pre_.det_engine)
        << " --pose-engine "  << shell_quote(pre_.pose_engine)
        << " --out "          << shell_quote(joints3d_path_.string())
        << " --summary "      << shell_quote(summary_path_.string())
        << " --overlay-dir "  << shell_quote(overlay_dir_.string())
        << " --subject-profile-out " << shell_quote(subject_profile_yaml_.string())
        << " --quality-out "  << shell_quote(quality_json_path_.string());
    if (pre_.subject_height_m > 0.0) {
        cmd << " --subject-height-m " << pre_.subject_height_m;
    }
    cmd << " 2>&1";
    log_line("analyze cmd: " + cmd.str());

    FILE* pipe = popen(cmd.str().c_str(), "r");
    if (!pipe) {
        {
            std::lock_guard<std::mutex> g(mu_);
            last_error_ = "popen failed";
        }
        set_state_(CalibState::kFailed);
        return;
    }
    char buf[2048];
    while (std::fgets(buf, sizeof(buf), pipe)) {
        std::string line{buf};
        if (!line.empty() && line.back() == '\n') line.pop_back();
        log_line(std::string{"[analyze] "} + line);
        std::lock_guard<std::mutex> g(mu_);
        analyze_log_tail_ += line;
        analyze_log_tail_ += '\n';
        if (analyze_log_tail_.size() > 8192) {
            analyze_log_tail_.erase(0, analyze_log_tail_.size() - 8192);
        }
    }
    int rc = pclose(pipe);
    int exit_code = (WIFEXITED(rc) ? WEXITSTATUS(rc) : -1);
    {
        std::lock_guard<std::mutex> g(mu_);
        analyze_exit_code_ = exit_code;
    }
    log_line("analyze exit=" + std::to_string(exit_code));

    if (exit_code != 0) {
        {
            std::lock_guard<std::mutex> g(mu_);
            last_error_ = "dump_keypoints_3d exited " + std::to_string(exit_code);
        }
        set_state_(CalibState::kFailed);
        return;
    }

    // Read quality.json to extract status. Best-effort string scan to avoid a
    // JSON dependency.
    std::string qstatus = "unknown";
    std::string qsummary;
    {
        std::ifstream f{quality_json_path_};
        if (f) {
            std::stringstream ss;
            ss << f.rdbuf();
            qsummary = ss.str();
            auto pos = qsummary.find("\"status\"");
            if (pos != std::string::npos) {
                auto colon = qsummary.find(':', pos);
                if (colon != std::string::npos) {
                    auto q1 = qsummary.find('"', colon);
                    auto q2 = (q1 != std::string::npos)
                                ? qsummary.find('"', q1 + 1) : std::string::npos;
                    if (q1 != std::string::npos && q2 != std::string::npos) {
                        qstatus = qsummary.substr(q1 + 1, q2 - q1 - 1);
                    }
                }
            }
        }
    }
    {
        std::lock_guard<std::mutex> g(mu_);
        quality_status_ = qstatus;
        quality_summary_ = qsummary;
        quality_ok_ = (qstatus == "pass" || qstatus == "warn");
    }
    log_line("quality status=" + qstatus);
    set_state_(CalibState::kReview);
    try_auto_approve_();
}

void CalibrationSession::try_auto_approve_() {
    if (!auto_approve_.load()) return;
    if (quality_status_ != "pass") {
        // warn / fail are left in kReview for human review (via WebUI
        // force-approve or retake). We deliberately do NOT fire
        // on_exit_requested_ here -- doing so would steal the operator's
        // chance to approve a warn result. With --no-web, the operator
        // sees the [calib] log and Ctrl-Cs the process.
        log_line(std::string{"auto_approve skipped: quality="} + quality_status_
                 + " (manual review required)");
        return;
    }
    std::string err;
    approve(/*force=*/false, err);
    if (!err.empty()) log_line("auto_approve failed: " + err);
}

std::string CalibrationSession::state_json() const {
    std::lock_guard<std::mutex> g(mu_);
    std::ostringstream o;
    o << "{";
    o << "\"state\":\"" << calib_state_name(state_.load()) << "\"";
    o << ",\"started_once\":" << (started_once_.load() ? "true" : "false");
    o << ",\"subject_id\":\"" << json_escape(pre_.subject_id) << "\"";
    o << ",\"subject_height_m\":" << pre_.subject_height_m;
    o << ",\"session_dir\":\"" << json_escape(session_dir_.string()) << "\"";
    o << ",\"target_pose_idx\":" << target_pose_idx_;
    o << ",\"target_pose\":\""
      << (target_pose_idx_ < kPoseCount
            ? json_escape(records_[target_pose_idx_].name) : "")
      << "\"";
    o << ",\"hold_progress\":" << last_detection_.hold_progress;
    o << ",\"in_band\":" << (last_detection_.in_band ? "true" : "false");
    o << ",\"angles_valid\":" << (last_detection_.angles_valid ? "true" : "false");
    o << ",\"failing_axis\":\"" << json_escape(last_detection_.failing_axis) << "\"";
    o << ",\"bone_drift_pct\":" << last_detection_.bone_drift_pct;
    if (last_detection_.angles_valid) {
        o << ",\"angles\":{";
        o << "\"l_elbow\":" << last_detection_.angles.left_elbow_flex
          << ",\"r_elbow\":" << last_detection_.angles.right_elbow_flex
          << ",\"l_sh_abd\":" << last_detection_.angles.left_shoulder_abduction
          << ",\"r_sh_abd\":" << last_detection_.angles.right_shoulder_abduction
          << ",\"l_knee\":" << last_detection_.angles.left_knee_flex
          << ",\"r_knee\":" << last_detection_.angles.right_knee_flex
          << ",\"torso_tilt\":" << last_detection_.angles.torso_tilt;
        o << "}";
    }
    o << ",\"poses\":[";
    for (std::size_t i = 0; i < kPoseCount; ++i) {
        if (i) o << ",";
        const auto& r = records_[i];
        int cam0_buf = static_cast<int>(buffers_[i][0].size());
        int cam1_buf = static_cast<int>(buffers_[i][1].size());
        o << "{\"name\":\"" << json_escape(r.name) << "\""
          << ",\"recorded\":" << (r.recorded ? "true" : "false")
          << ",\"buffered\":[" << cam0_buf << "," << cam1_buf << "]"
          << ",\"frames\":[" << r.frames[0] << "," << r.frames[1] << "]"
          << ",\"fps\":[" << r.fps[0] << "," << r.fps[1] << "]"
          << ",\"clips\":[";
        for (std::size_t ci = 0; ci < r.clips.size(); ++ci) {
            if (ci) o << ",";
            o << "\"" << json_escape(r.clips[ci]) << "\"";
        }
        o << "]}";
    }
    o << "]";
    o << ",\"quality_status\":\"" << json_escape(quality_status_) << "\"";
    o << ",\"quality_ok\":" << (quality_ok_ ? "true" : "false");
    o << ",\"analyze_exit\":" << analyze_exit_code_;
    o << ",\"analyze_log_tail\":\"" << json_escape(analyze_log_tail_) << "\"";
    o << ",\"latest_profile\":\""
      << json_escape(approval_done_.load() ? latest_profile_path_.string() : "")
      << "\"";
    o << ",\"recording_frames_per_cam\":" << pre_.recording_frames_per_cam;
    o << ",\"required_hold_sec\":" << pre_.required_hold_sec;
    o << ",\"last_error\":\"" << json_escape(last_error_) << "\"";
    o << "}";
    return o.str();
}

}  // namespace fitra::pipeline
