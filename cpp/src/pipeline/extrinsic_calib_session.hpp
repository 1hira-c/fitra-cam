#pragma once
//
// Controller-marker multi-camera extrinsic calibration collection loop.
//
// See docs/design/pose-3d-controller-marker-extrinsic.md (M2). Skeleton: it
// taps MultiCameraDriver's per-camera frames, detects the multi-face AprilTag
// marker, pairs each face detection with the latest VR controller pose, applies
// a motion gate + burst average, and accumulates ExtrinsicSamples. On finalize
// it runs the hand-eye solver and writes the extrinsics back into a
// CalibrationSet YAML.
//
// Decoupled from fitra_vmt: the controller pose is handed in as a plain
// ControllerObservation (built by main from the ControllerPoseBus snapshot) so
// fitra_pipeline does not need to depend on fitra_vmt (which would be circular —
// fitra_vmt → fitra_slimevr → fitra_pipeline).
//
// Threading: on_frame() is called from the pipeline thread; the public read
// methods (state/state_json/sample_count) and solve_and_write() may be called
// from the Crow / main thread. All shared state is mutex-guarded.

#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "lift/apriltag_marker.hpp"
#include "lift/calib_io.hpp"
#include "lift/extrinsic_solver.hpp"

namespace fitra::pipeline {

// Controller pose at a moment in time, in the same world frame as the cameras
// will be expressed in (SteamVR Standing + VMT). `running_ok` folds OpenVR's
// bPoseIsValid && eTrackingResult == Running_OK.
struct ControllerObservation {
    bool   running_ok = false;
    double x = 0, y = 0, z = 0;
    double qx = 0, qy = 0, qz = 0, qw = 1;
    double ts_ms = 0.0;
};

enum class ExtrinsicCalibState {
    kIdle,        // not collecting
    kCollecting,  // taps feeding the gate/burst accumulator
    kSolving,     // solve in progress
    kSolved,      // solve succeeded, extrinsics written
    kFailed,      // solve failed
};

const char* extrinsic_calib_state_name(ExtrinsicCalibState s);

struct ExtrinsicCalibConfig {
    // Per-camera intrinsics. Camera index in on_frame() maps to
    // intrinsics.cameras[idx]; extrinsics (if present) are ignored — we compute
    // them. Required.
    lift::CalibrationSet    intrinsics;
    // Marker faces (AprilTag IDs + physical sizes). Required.
    lift::MarkerBoardConfig board;

    // Motion gate: a frame is accepted only if the controller is running_ok and
    // both its linear and angular speed (estimated from successive samples) are
    // at or below these thresholds.
    double lin_vel_max_mps = 0.03;   // 3 cm/s
    double ang_vel_max_dps = 8.0;    // 8 deg/s

    // Burst averaging: consecutive accepted observations of the same (cam,face)
    // are averaged into one sample. A burst flushes when motion resumes, the
    // controller-sample gap exceeds burst_gap_ms, or burst_max is reached. Only
    // bursts with at least burst_min frames emit a sample.
    int    burst_min   = 5;
    int    burst_max   = 40;
    double burst_gap_ms = 250.0;

    // Hand-eye solve gate: minimum samples per (cam, face) group.
    int    min_samples_per_group = 8;

    // Where solve_and_write() writes the resulting CalibrationSet.
    std::string out_path = "calibrations/extrinsics.yaml";

    // Reprojection error (px) above which a face PnP detection is rejected.
    double max_pnp_reproj_px = 3.0;
};

class ExtrinsicCalibSession {
public:
    explicit ExtrinsicCalibSession(ExtrinsicCalibConfig cfg);

    ExtrinsicCalibSession(const ExtrinsicCalibSession&) = delete;
    ExtrinsicCalibSession& operator=(const ExtrinsicCalibSession&) = delete;

    void start();             // → kCollecting
    void stop_collecting();   // → kIdle (samples retained)

    // Pipeline frame tap. `bgr` need not be retained past the call.
    void on_frame(std::size_t cam_idx, const cv::Mat& bgr,
                  const ControllerObservation& ctrl);

    // Testable core: feed a pre-computed face detection + controller pose
    // through the motion gate and burst accumulator (no image / PnP). Returns
    // true if the observation passed the motion gate (was buffered).
    bool ingest(std::size_t cam_idx, int face_id,
                const geom::T_cam_marker& T_cam_face,
                const ControllerObservation& ctrl);

    // A single face decoded in one camera's most recent frame (for the live UI).
    struct LiveDetection {
        int    face_id       = 0;
        double reproj_rms_px = 0.0;
        bool   pose_ok       = false;
    };

    // Solve from accumulated samples; on success write the extrinsics YAML.
    bool solve_and_write(std::string& err);

    ExtrinsicCalibState state() const;
    std::size_t         sample_count() const;
    std::string         state_json() const;        // self-contained, for Crow
    // Per-camera intrinsics + solved T_cam_world (world→camera) for the 3D
    // verification scene. Empty `cameras` until a solve succeeds.
    std::string         extrinsics_json() const;

    const lift::ExtrinsicSolution& last_solution() const { return solution_; }

private:
    struct GroupKey {
        std::size_t cam;
        int         face;
        bool operator<(const GroupKey& o) const {
            return cam != o.cam ? cam < o.cam : face < o.face;
        }
    };
    struct Burst {
        std::vector<cv::Matx44d> T_cam_face;
        std::vector<cv::Matx44d> T_world_controller;
        double last_ts_ms = 0.0;
    };

    struct CamLive {
        double last_ts_ms = 0.0;
        bool   ctrl_running_ok = false;
        std::vector<LiveDetection> dets;
    };

    void flush_all_bursts_();              // call with mu_ held
    void flush_burst_(const GroupKey& k);  // call with mu_ held
    void update_velocity_(const ControllerObservation& ctrl);  // mu_ held
    const char* gate_reason_() const;      // call with mu_ held

    ExtrinsicCalibConfig cfg_;

    mutable std::mutex mu_;
    ExtrinsicCalibState state_ = ExtrinsicCalibState::kIdle;

    std::map<GroupKey, Burst>      bursts_;
    std::vector<lift::ExtrinsicSample> samples_;
    std::map<GroupKey, int>        coverage_;   // emitted samples per group

    // Latest per-camera detection summary for the live UI + gate reason.
    std::map<std::size_t, CamLive> cam_live_;
    double last_frame_ts_ms_ = 0.0;   // max ctrl.ts_ms seen across cameras

    // Velocity estimation state.
    bool   have_prev_ctrl_ = false;
    ControllerObservation prev_ctrl_{};
    double last_lin_vel_mps_ = 0.0;
    double last_ang_vel_dps_ = 0.0;

    lift::ExtrinsicSolution solution_;
    std::string             last_error_;

    // Pre-built AprilTag detector. The inner cv::aruco::ArucoDetector is heavy
    // to construct (dictionary lookup + parameter setup); on_frame() is the hot
    // path, so we build it once and reuse. Owned by unique_ptr so AprilTagDetector
    // — which is non-copyable / non-default-constructible — stays out of the
    // member initializer list ordering constraints.
    std::unique_ptr<lift::AprilTagDetector> detector_;
};

}  // namespace fitra::pipeline
