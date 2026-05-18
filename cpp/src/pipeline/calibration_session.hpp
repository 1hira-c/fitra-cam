#pragma once
//
// Phase 8 subject-profile calibration orchestrator.
//
// Lives next to MultiCameraDriver and consumes its frame/skeleton taps. The
// state machine cycles through 4 target poses (standing, t_pose, elbow_flex,
// knee_flex), records short clips per camera when the PoseRecognizer reports
// a stable hold, writes a pose_session.json, then runs the existing
// dump_keypoints_3d offline tool as a subprocess to produce
// subject_profile.yaml and quality.json. Approving copies the profile to
// calibrations/subjects/<id>/latest_profile.yaml and invokes the
// approval callback so the live IK can be hot-reloaded.

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "infer/types.hpp"
#include "lift/pose_recognizer.hpp"
#include "lift/subject_profile.hpp"

namespace fitra::pipeline {

enum class CalibState {
    kIdle,
    kReady,
    kAwaitHold,
    kRecording,
    kFinalizing,
    kAnalyzing,
    kReview,
    kApproving,
    kApproved,
    kCanceled,
    kFailed,
};

const char* calib_state_name(CalibState s);

struct CalibPoseRecord {
    std::string name;
    std::vector<std::string> clips;       // paths relative to session_dir
    std::array<int, 2> frames{{0, 0}};
    std::array<double, 2> fps{{0.0, 0.0}};
    bool recorded = false;
};

struct CalibPreflight {
    std::string subject_id;
    double      subject_height_m = 0.0;
    std::string subjects_dir = "calibrations/subjects";
    std::string calib_yaml;
    std::string det_engine;
    std::string pose_engine;
    int         recording_frames_per_cam = 75;   // ~5s at 15 fps
    double      required_hold_sec = 1.5;
    std::string dump_tool_path;                  // ./cpp/build/tools/dump_keypoints_3d
};

class CalibrationSession {
public:
    using ApprovedFn = std::function<void(const lift::SubjectProfile&)>;
    using ExitFn = std::function<void()>;
    using LogFn = std::function<void(const std::string&)>;
    using PreflightFn = std::function<void(const CalibPreflight&)>;

    CalibrationSession();
    ~CalibrationSession();

    CalibrationSession(const CalibrationSession&) = delete;
    CalibrationSession& operator=(const CalibrationSession&) = delete;

    // API surface (web + main).
    bool preflight(const CalibPreflight& in, std::string& err);
    bool start(std::string& err);
    bool retake(const std::string& pose_name, std::string& err);
    bool cancel(std::string& err);
    bool approve(bool force, std::string& err);

    // Hot reload hook -- main registers this so approve() flips the live IK.
    void set_on_approved(ApprovedFn fn) { on_approved_ = std::move(fn); }
    // Called from preflight() on success with the validated CalibPreflight.
    // main wires this up to IkSolver::apply_subject_height() so the IK is
    // primed for 3D angle recognition before recording begins.
    void set_on_preflight(PreflightFn fn) { on_preflight_ = std::move(fn); }
    void set_on_exit_requested(ExitFn fn) { on_exit_requested_ = std::move(fn); }
    void set_log(LogFn fn) { log_fn_ = std::move(fn); }
    void set_auto_approve(bool yes) { auto_approve_.store(yes); }
    void set_auto_exit(bool yes) { auto_exit_.store(yes); }
    void set_fps_hint(double fps);

    // Taps (pipeline thread).
    void on_frame(std::size_t cam_idx, const cv::Mat& bgr, double ts_ms);
    void on_skeleton3d(const infer::Skeleton3D& skel, double bone_drift_pct);

    // Read.
    CalibState state() const { return state_.load(); }
    std::string state_json() const;            // self-contained JSON
    std::filesystem::path session_dir() const;
    bool has_been_started() const { return started_once_.load(); }

private:
    void log_line(const std::string& s) const;
    void set_state_(CalibState s);
    void advance_to_pose_(std::size_t idx);
    void begin_recording_();
    void on_pose_buffer_full_();
    void launch_finalize_();
    void finalize_thread_main_();
    void launch_analyzer_();
    void analyzer_thread_main_();
    void try_auto_approve_();
    static std::string sanitize_id(const std::string& in);
    static std::string iso_timestamp_now();

    static constexpr std::size_t kPoseCount = 4;
    static const std::array<lift::TargetPose, kPoseCount> kSequence;

    mutable std::mutex mu_;
    std::atomic<CalibState> state_{CalibState::kIdle};
    std::atomic<bool>       started_once_{false};

    CalibPreflight pre_;
    std::filesystem::path session_dir_;
    std::filesystem::path subject_dir_;
    std::filesystem::path subject_profile_yaml_;
    std::filesystem::path quality_json_path_;
    std::filesystem::path pose_session_path_;
    std::filesystem::path joints3d_path_;
    std::filesystem::path summary_path_;
    std::filesystem::path overlay_dir_;
    std::filesystem::path latest_profile_path_;
    std::string           created_at_;

    struct FrameItem {
        cv::Mat bgr;
        double  ts_ms = 0.0;
    };
    std::array<std::array<std::vector<FrameItem>, 2>, kPoseCount> buffers_;
    std::array<CalibPoseRecord, kPoseCount> records_;

    std::size_t target_pose_idx_ = 0;

    lift::PoseRecognizer        recognizer_;
    lift::PoseDetectionState    last_detection_{};
    double                      last_bone_drift_pct_ = 0.0;

    std::thread finalize_thread_;
    std::thread analyzer_thread_;
    std::string analyze_log_tail_;
    int analyze_exit_code_ = -1;
    std::string quality_status_;
    bool quality_ok_ = false;
    std::string quality_summary_;

    ApprovedFn  on_approved_;
    PreflightFn on_preflight_;
    ExitFn      on_exit_requested_;
    LogFn       log_fn_;
    std::atomic<bool> auto_approve_{false};
    std::atomic<bool> auto_exit_{false};
    std::atomic<bool> approval_done_{false};

    std::string last_error_;
};

}  // namespace fitra::pipeline
