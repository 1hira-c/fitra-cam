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
#include <memory>
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
