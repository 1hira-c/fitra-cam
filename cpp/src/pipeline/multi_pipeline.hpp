#pragma once
//
// N-camera driver.
//
// Each camera has its own FrameSource (own V4L2 thread + own decode/YOLOX
// thread with its own TRT execution context). This central driver only
// polls the per-camera ready slots, batches the resulting (frame, bbox)
// requests into one RTMPose call, and distributes the persons back to
// per-camera snapshots.
//
// The Yolox / Yolox-engine plumbing now lives in main.cpp (constructs one
// shared ICudaEngine for YOLOX and N per-camera Yolox/IExecutionContexts)
// and inside FrameSource, not here.

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "camera/frame_source.hpp"
#include "infer/rtmpose.hpp"
#include "lift/floor_grounding.hpp"
#include "lift/ik.hpp"
#include "lift/kalman.hpp"
#include "lift/rigid_fit.hpp"
#include "lift/triangulator.hpp"
#include "pipeline/pose_pipeline.hpp"
#include "pipeline/snapshot.hpp"

namespace fitra::pipeline {

class MultiCameraDriver {
public:
    struct ThreeDConfig {
        std::shared_ptr<lift::Triangulator> triangulator;
        Skeleton3DBus* bus = nullptr;
        double sync_window_ms = 15.0;
        bool kalman_enabled = true;
        bool ik_enabled = true;
        // Spatial pelvis rigid fit (spatial-filtering M-A). Only takes effect
        // when a subject profile is present (its distances form the pelvis
        // template) and the topology is Halpe26; otherwise a no-op.
        bool rigid_pelvis = false;
        // Spatiotemporal filter (spatiotemporal-filter M-C3). When true the chain
        // Kalman is weakened toward measurement-tracking (predict/hold only) so
        // it does not compound lag with the tracker-stage regime filter, which
        // now owns the smoothing. No effect on the tracker stage itself (that is
        // gated by TrackerExtractorOptions::st_filter). Default off = byte-identical.
        bool st_filter = false;
        // Floor-contact grounding (spatial-filtering M-D). When true the LAST 3D
        // stage clamps below-floor foot sole points to the floor (Z=0) and snaps
        // near-floor low-speed (stance) points onto it, fixing heel-sink /
        // penetration. Halpe26 only (needs toe/heel). Default off = byte-identical.
        bool floor_grounding = false;
        double floor_z_m = 0.0;
        double floor_stance_vel_mps = 0.15;
        double floor_snap_band_m = 0.03;
        int bone_calib_frames = 150;
        double subject_height_m = 0.0;
        bool has_subject_profile = false;
        lift::SubjectProfile subject_profile;
    };

    MultiCameraDriver(std::vector<std::unique_ptr<camera::FrameSource>> sources,
                      infer::RtmPose& rtmpose,
                      SnapshotBus& bus);
    MultiCameraDriver(std::vector<std::unique_ptr<camera::FrameSource>> sources,
                      infer::RtmPose& rtmpose,
                      SnapshotBus& bus,
                      ThreeDConfig threed);
    ~MultiCameraDriver();

    MultiCameraDriver(const MultiCameraDriver&) = delete;
    MultiCameraDriver& operator=(const MultiCameraDriver&) = delete;

    void start();
    void stop();

    std::size_t camera_count() const { return sources_.size(); }
    const PipelineStats& stats_for(std::size_t i) const { return per_cam_[i].stats; }
    double recv_fps_for(std::size_t i) const { return sources_[i]->recv_fps(); }
    std::uint64_t pending_for(std::size_t i) const {
        auto recv = sources_[i]->total_received();
        auto pr   = per_cam_[i].stats.processed_count;
        return recv > pr ? recv - pr : 0;
    }

    // Taps for the subject calibration wizard. Set/replace before start(),
    // or while running -- writes to the std::function are protected by a
    // tiny mutex on the call side so they're safe to swap from another
    // thread. Both taps default to no-op.
    //
    // The frame tap is called from the pipeline thread immediately after a
    // FrameSource pop, with the decoded BGR Mat and a steady_clock-based
    // timestamp (milliseconds since program start). The callback receives a
    // const reference; the callee MUST clone() if it wants to keep the
    // pixels past the call.
    using FrameTapFn = std::function<void(std::size_t cam_idx,
                                          const cv::Mat& bgr,
                                          double ts_ms)>;
    // Called with the filtered pre-IK 3D skeleton. Subject calibration uses
    // anatomical joint angles from the measured pose; feeding the post-IK
    // skeleton would let hinge/length clamps bias the hold detector.
    using Skeleton3DTapFn = std::function<void(const infer::Skeleton3D& skel,
                                               double bone_drift_pct)>;

    void set_frame_tap(FrameTapFn fn);
    void set_skeleton3d_tap(Skeleton3DTapFn fn);

    // Idle/standby gate (issue #37). `idle_flag` points at the shared idle
    // atomic (owned by the IdleState in mode_run; must outlive the driver);
    // null disables idling (calib modes never set it). While idle the central
    // loop skips the 3D update and throttles to `idle_tick_hz`. Set before
    // start(). A plain atomic pointer (not the app::IdleState type) keeps this
    // layer free of an app/ dependency, mirroring calib_recording_flag.
    void set_idle_gate(const std::atomic<bool>* idle_flag, double idle_tick_hz);

    // For the calib-subject preflight: primes the live IK with the subject
    // height (apply_subject_height) so the 3D angle recognizer has a sensible
    // bone-length lock from frame 1. Profile loading happens at construction
    // via ThreeDConfig — there is no runtime profile reinjection.
    lift::IkSolver& ik() { return ik_; }

private:
    struct CamState {
        PipelineStats        stats;
        std::deque<std::chrono::steady_clock::time_point> recent;
        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    };

    void loop();
    void update_stats(CamState& cs,
                      std::chrono::steady_clock::time_point now,
                      std::chrono::steady_clock::time_point captured_at);
    void maybe_update_3d(std::chrono::steady_clock::time_point now,
                         std::chrono::system_clock::time_point wall_now);
    // Idle/standby edge handler: on entry push one "no fresh data" 3D snapshot
    // so consumers drop the frozen pose; on resume reset the smoothers (M4).
    void handle_idle_transition(bool now_idle);

    std::vector<std::unique_ptr<camera::FrameSource>> sources_;
    infer::RtmPose&      rtmpose_;
    SnapshotBus&         bus_;
    ThreeDConfig         threed_;
    lift::SkeletonKalman kalman_;
    lift::IkSolver       ik_;
    // Spatial pelvis rigid-fit template + gate, built once in the constructor
    // from threed_.subject_profile. rigid_pelvis_active_ is true only when the
    // flag is set, a profile is loaded, and the pelvis template is non-degenerate
    // — otherwise the 3D path is byte-identical to the pre-M-A behavior.
    lift::RigidTemplate  pelvis_template_{};
    bool                 rigid_pelvis_active_ = false;
    // Floor-contact grounding (M-D). opts built once from threed_; state holds
    // the per-foot-point prev anchor for the stance-speed estimate. Only touched
    // when threed_.floor_grounding is set.
    lift::FloorGroundingOptions floor_opts_{};
    lift::FloorGroundingState   floor_state_{};
    std::mutex           threed_mu_;

    // Tap callbacks. Loop reads these via a local snapshot to avoid holding
    // the mutex across the user callback.
    std::mutex           tap_mu_;
    FrameTapFn           frame_tap_;
    Skeleton3DTapFn      skeleton3d_tap_;
    std::chrono::steady_clock::time_point loop_t0_{};
    bool                 loop_t0_set_ = false;

    // Latest decoded frame + bboxes per camera, kept alive across the
    // RTMPose batched call so we can hand cv::Mat pointers into reqs.
    std::vector<camera::DecodedFrame> latest_per_cam_;
    std::vector<CameraSnapshot>        latest_snapshots_;
    std::vector<std::uint64_t>         last_3d_input_seqs_;
    std::vector<CamState>             per_cam_;
    std::deque<std::chrono::steady_clock::time_point> tri_recent_;
    std::chrono::steady_clock::time_point last_3d_update_{};
    std::uint64_t                     tri_processed_ = 0;
    std::uint64_t                     tri_sync_miss_ = 0;
    bool                              has_last_3d_update_ = false;
    // Idle/standby gate state (issue #37).
    const std::atomic<bool>*          idle_flag_   = nullptr;
    double                            idle_tick_hz_ = 2.0;
    bool                              idle_active_ = false;   // edge tracker
    std::thread                       worker_;
    std::atomic<bool>                 stop_{false};
};

}  // namespace fitra::pipeline
