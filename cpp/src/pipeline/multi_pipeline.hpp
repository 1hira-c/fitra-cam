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
#include "lift/ik.hpp"
#include "lift/kalman.hpp"
#include "lift/triangulator.hpp"
#include "pipeline/pose_pipeline.hpp"
#include "pipeline/snapshot.hpp"

namespace fitra::pipeline {

class MultiCameraDriver {
public:
    struct ThreeDConfig {
        lift::Triangulator* triangulator = nullptr;
        Skeleton3DBus* bus = nullptr;
        double sync_window_ms = 15.0;
        bool kalman_enabled = true;
        bool ik_enabled = true;
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

    // Taps for the Phase 8 calibration wizard. Set/replace before start(),
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
    using Skeleton3DTapFn = std::function<void(const infer::Skeleton3D& skel,
                                               double bone_drift_pct)>;

    void set_frame_tap(FrameTapFn fn);
    void set_skeleton3d_tap(Skeleton3DTapFn fn);

    // For the calibration session approval flow: lets the API layer call
    // ik().reload_from_profile() once a new profile is approved.
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

    std::vector<std::unique_ptr<camera::FrameSource>> sources_;
    infer::RtmPose&      rtmpose_;
    SnapshotBus&         bus_;
    ThreeDConfig         threed_;
    lift::SkeletonKalman kalman_;
    lift::IkSolver       ik_;

    // Phase 8 taps. Loop reads these via a local snapshot to avoid holding
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
    std::thread                       worker_;
    std::atomic<bool>                 stop_{false};
};

}  // namespace fitra::pipeline
