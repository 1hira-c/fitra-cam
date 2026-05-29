#pragma once
//
// Per-camera worker.
//
// Owns one V4l2Capture (own thread for V4L2 dequeue) and runs a second
// thread that does (decode → optional YOLOX). Publishes the latest
// cached bboxes and optional pre-baked RTMPose inputs so the central
// inference thread only has to batch them into a single RTMPose call.
//
// Why "optional" YOLOX: if the caller passes a Yolox*, the per-cam thread
// runs detection inline. With nullptr, the source is decode-only. Each
// Yolox is built from a TrtEngine that shares ICudaEngine via
// TrtEngine::from_shared — TRT execution contexts themselves are not
// thread-safe, so per-cam Yolox = per-cam context.
//
// State for det-frequency decimation + single-person filter lives here
// rather than in the central pipeline, so each camera tracks its own
// detection schedule independently of any frames the inference thread
// dropped (latest-wins).

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "camera/jpeg_decoder.hpp"
#include "camera/v4l2_capture.hpp"
#include "infer/rtmpose.hpp"
#include "infer/types.hpp"
#include "infer/yolox.hpp"

namespace fitra::camera {

struct DecodedFrame {
    // Populated for decode-only sources, and for prebaked RTMPose sources
    // when Options::retain_bgr is enabled (calibration recording tap).
    cv::Mat                              bgr;
    std::uint64_t                        seq{0};
    std::chrono::steady_clock::time_point captured_at{};  // t_capture (V4L2 DQBUF)
    // Per-stage latency timestamps (steady_clock), set on the per-camera
    // worker as each stage completes. Skipped stages are filled with the
    // previous stage's value so deltas stay 0 (never default/epoch, which
    // would poison the rolling average downstream).
    std::chrono::steady_clock::time_point t_decode{};   // after JPEG/YUYV -> BGR
    std::chrono::steady_clock::time_point t_detect{};   // after YOLOX block
    std::chrono::steady_clock::time_point t_prebake{};  // after RTMPose prebake
    std::vector<infer::Bbox>             bboxes;  // empty if no Yolox or no person
    // Pre-baked RTMPose inputs aligned 1:1 with `bboxes`. The chw buffer is a
    // single contiguous block of bboxes.size() * 3*input_h*input_w floats.
    // Populated when the FrameSource was given a non-null rtmpose_opts.
    std::vector<float>                   chw_concat;
    std::vector<cv::Mat>                 M_invs;

    bool has_prebaked_pose_inputs(std::size_t per_item) const {
        return bboxes.empty()
            || (per_item > 0
                && chw_concat.size() == bboxes.size() * per_item
                && M_invs.size() == bboxes.size());
    }
};

class FrameSource {
public:
    struct Options {
        int  det_frequency = 10;
        bool single_person = true;
        // Keep a BGR copy in DecodedFrame even when RTMPose prebaking is
        // enabled. This costs a full-frame clone per decoded frame, so callers
        // should enable it only when another consumer needs raw pixels.
        bool retain_bgr = false;
        // "Calibration is actively recording" flag. When set, the
        // decode worker (a) retains the BGR copy and (b) skips the RTMPose
        // pre-bake (and YOLOX) so the per-camera worker can keep up with
        // V4L2 dequeue while disk I/O is the bottleneck. The central
        // inference thread naturally sees no bboxes => no pose work either.
        // Stays empty when no shared flag is needed.
        std::shared_ptr<std::atomic<bool>> calib_recording_flag;
        // Debug/bench: when YOLOX is enabled but the cache is empty after
        // detection, inject a synthetic bbox covering the central 60% of
        // the frame so the downstream RTMPose stage always has work. Use
        // only for pipeline-ceiling measurement without a subject in view.
        bool fake_bbox_if_empty = false;
    };

    // Yolox can be nullptr -> decode-only (no detection).
    // rtmpose_opts is optional: if provided, the per-camera worker also
    // pre-bakes the RTMPose input (warp + normalize + HWC->CHW) so the
    // central inference thread only has to memcpy + GPU + decode.
    FrameSource(std::unique_ptr<V4l2Capture> capture,
                std::unique_ptr<infer::Yolox> yolox,
                Options opts,
                const infer::RtmPose::Options* rtmpose_opts = nullptr);
    ~FrameSource();

    FrameSource(const FrameSource&) = delete;
    FrameSource& operator=(const FrameSource&) = delete;

    void start();
    void stop();

    bool try_pop_latest_decoded(DecodedFrame& out);

    // Block until a new decoded frame is available (without consuming it),
    // `consumer_stop` is set, or `timeout` elapses. Returns true if a fresh
    // frame is waiting. The caller then consumes it via try_pop_latest_decoded
    // -- splitting wait-from-consume keeps the central loop's existing Pass-1
    // poll structure intact. Used by the single-camera event-driven path.
    bool wait_available(std::atomic<bool>& consumer_stop,
                        std::chrono::milliseconds timeout);

    // Wake any thread parked in wait_available so a consumer stop flag is
    // observed immediately. Notifies under the slot lock.
    void wake();

    V4l2Capture&  capture()             { return *capture_; }
    const V4l2Options& options() const  { return capture_->options(); }
    double recv_fps() const             { return capture_->recv_fps(); }
    std::uint64_t total_received() const{ return capture_->total_received(); }
    bool prebakes_pose() const          { return rtmpose_enabled_; }

private:
    void decode_loop();

    std::unique_ptr<V4l2Capture>  capture_;
    std::unique_ptr<infer::Yolox> yolox_;
    Options                       opts_;
    bool                          rtmpose_enabled_ = false;
    infer::RtmPose::Options       rtmpose_opts_{};

    JpegDecoder            decoder_;
    std::thread            worker_;
    std::atomic<bool>      stop_{false};

    // Det-frequency state (touched only by decode_loop).
    int                      frame_idx_ = 0;
    std::vector<infer::Bbox> cached_bboxes_;

    mutable std::mutex          slot_mu_;
    std::condition_variable     slot_cv_;
    std::optional<DecodedFrame> latest_;
    std::uint64_t               last_returned_seq_ = 0;
};

}  // namespace fitra::camera
