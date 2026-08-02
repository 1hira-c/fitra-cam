#pragma once
//
// Raw V4L2 capture for a single camera (MJPEG or YUYV).
//
// Bypasses OpenCV's VideoCapture so we can choose pixel format, buffer
// count, decode path, and synchronization explicitly:
//   - MJPEG (default) or YUYV (uncompressed) per V4l2Options::pixel_format
//   - configurable mmap ring depth (default 4; min 2)
//   - blocking VIDIOC_DQBUF in a worker thread
//   - latest-frame-wins semantics (drop older frames if the consumer is
//     behind), mirroring python/scripts/pose_pipeline.py::CameraReader
//
// The Frame holds a COPY of the raw payload bytes (compressed JPEG for
// MJPEG, packed YUV422 for YUYV) so the V4L2 buffer can be re-queued
// immediately. The consumer branches on `pixel_format` to decode.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace fitra::camera {

// Raw payload encoding of a captured Frame.
enum class PixFmt {
    Mjpeg,   // compressed JPEG; decode via cv::imdecode (CPU)
    Yuyv,    // packed YUV422 (V4L2_PIX_FMT_YUYV); decode via cv::cvtColor
    Nvjpeg,  // captured as MJPEG, decoded on the Jetson HW NVJPEG block (GPU/VIC)
};

struct Frame {
    std::vector<std::uint8_t> data;  // raw payload (JPEG bytes or packed YUYV)
    std::uint64_t seq{0};
    std::chrono::steady_clock::time_point captured_at{};
    // Exact host observation timestamp from CLOCK_MONOTONIC at DQBUF return.
    // This is the content time for fusion-facing samples; it is not wall-clock.
    std::uint64_t captured_mono_ns{0};
};

struct V4l2Options {
    std::string device_path;
    // Output (effective) resolution -- what every downstream consumer sees.
    // When cap_width/cap_height override the capture resolution, frames are
    // downscaled to width/height in FrameSource::decode_loop. Unchanged
    // semantics for the no-override case (cap_* == 0).
    int width  = 640;
    int height = 480;
    // Actual V4L2 capture resolution. 0 => same as width/height (no override,
    // no downscale). Set higher than width/height for cameras whose low-res
    // modes center-crop instead of downscaling (so the full sensor FOV is kept,
    // then resized down to the common runtime resolution).
    int cap_width  = 0;
    int cap_height = 0;
    int fps    = 30;
    int n_buffers = 4;
    PixFmt pixel_format = PixFmt::Mjpeg;

    // --- Exposure / gain control (anti-blur + steady 60fps pacing) -------
    // Some UVC cameras' firmware auto-exposure lets the exposure TIME run long
    // enough to (a) motion-blur a moving subject and (b) overrun the per-frame
    // budget (16.7ms @60fps), producing irregular frame pacing (jitter) that
    // appears on any host. We replace it:
    //   Auto   = leave the camera's own controls untouched (default; no regress)
    //   Manual = set fixed manual exposure + gain at start, then freeze
    //   Assist = Manual initial set, then a slow software AE (FrameSource) nudges
    //            gain (and exposure, capped at the fps-safe budget) toward a
    //            brightness target -- keeps exposure short (low blur, steady fps).
    // See docs/design/core-pipeline-camera-exposure-control.md.
    enum class ExposureMode { Auto, Manual, Assist };
    ExposureMode exposure_mode = ExposureMode::Auto;
    int exposure_us100 = 0;    // V4L2_CID_EXPOSURE_ABSOLUTE (100us units); 0 = leave
    int gain           = -1;   // V4L2_CID_GAIN; <0 = leave at camera default
    int ae_target      = 110;  // Assist: target mean luma (0-255)

    int capture_w() const { return cap_width  > 0 ? cap_width  : width;  }
    int capture_h() const { return cap_height > 0 ? cap_height : height; }
    bool downscaling() const {
        return capture_w() != width || capture_h() != height;
    }
};

// Map a config exposure-mode string to the enum. Shared by camera_builder (the
// live pipeline) and setup_camera_manager (the wizard preview) so both interpret
// the same config token identically.
inline V4l2Options::ExposureMode parse_exposure_mode(const std::string& s) {
    if (s == "manual") return V4l2Options::ExposureMode::Manual;
    if (s == "assist") return V4l2Options::ExposureMode::Assist;
    return V4l2Options::ExposureMode::Auto;
}

class V4l2Capture {
public:
    explicit V4l2Capture(V4l2Options opts);
    ~V4l2Capture();

    V4l2Capture(const V4l2Capture&) = delete;
    V4l2Capture& operator=(const V4l2Capture&) = delete;

    // Open device, set format, request buffers, start streaming, start the
    // worker thread. Throws on failure.
    void start();

    // Stop streaming, join thread, close device.
    void stop();

    // If a new frame has arrived since the last call, fill `out` and return
    // true. Otherwise return false (no copy). Thread-safe.
    bool try_pop_latest(Frame& out);

    // Block until a new frame arrives (then fill `out` + return true),
    // `consumer_stop` is set, or `timeout` elapses (return false). Event-driven
    // replacement for poll-sleep loops; preserves latest-frame-wins semantics.
    // `consumer_stop` lets the caller's own stop flag break the wait; pair with
    // wake() so stop() doesn't have to wait out the timeout.
    bool wait_pop_latest(Frame& out, std::atomic<bool>& consumer_stop,
                         std::chrono::milliseconds timeout);

    // Wake any thread parked in wait_pop_latest (e.g. so a consumer's stop
    // flag is observed immediately). Notifies under the slot lock.
    void wake();

    // Receive rate over the last ~60 frames (instantaneous-ish), Hz.
    double recv_fps() const { return recv_fps_.load(); }

    // Monotonically increasing capture count.
    std::uint64_t total_received() const { return total_received_.load(); }

    const V4l2Options& options() const { return opts_; }

    // Live exposure/gain setters for the software-AE assist (called from the
    // FrameSource decode thread). VIDIOC_S_CTRL is independent of the streaming
    // DQBUF/QBUF on the worker thread. Return false on ioctl failure.
    bool set_exposure_us100(int v);
    bool set_gain(int v);
    // Gain control range, queried from the driver at start() (defaults if the
    // control is absent). Used by the assist controller to clamp.
    int gain_min() const { return gain_min_; }
    int gain_max() const { return gain_max_; }

private:
    struct MmapBuf {
        void*       ptr     = nullptr;
        std::size_t length  = 0;
    };

    void worker_loop();
    void update_recv_fps(std::chrono::steady_clock::time_point now);
    // Set a single V4L2 control (logs + returns false on failure).
    bool set_ctrl(unsigned int id, int value, const char* name);
    // Query gain range + apply the configured initial exposure/gain/focus
    // controls (no-op in Auto mode). Called from start() after format setup.
    void apply_exposure_controls();
    int gain_min_ = 0;
    int gain_max_ = 255;

    // Diagnostic (env FITRA_CAPTURE_DEBUG): track driver-side frame drops via
    // v4l2_buffer.sequence gaps. A gap means the driver captured frames we
    // failed to DQBUF in time (our capture thread was descheduled / too slow),
    // distinguishing a scheduling stall from the camera simply not delivering.
    bool          capture_debug_   = false;
    std::uint32_t last_buf_seq_     = 0;
    bool          have_last_seq_    = false;
    std::uint64_t driver_dropped_   = 0;
    std::uint64_t dbg_frames_       = 0;

    V4l2Options opts_;
    int fd_ = -1;
    std::vector<MmapBuf> bufs_;
    std::thread worker_;
    std::atomic<bool> stop_{false};

    // Capture-thread-owned scratch for the next frame's payload. Reused every
    // frame (swapped with latest_->data under the slot lock) so the large
    // uncompressed YUYV payload (e.g. 2.46MB at 1280x960) is NOT heap-allocated
    // per frame. glibc malloc serves >128KB via mmap/munmap, so a fresh
    // per-frame vector for YUYV meant an mmap + page-fault-zeroing + munmap each
    // frame (~ms on Jetson), capping capture at ~53fps; the camera itself
    // delivers 60 (measured with v4l2-ctl). Ping-ponging two buffers removes it.
    std::vector<std::uint8_t> spare_data_;

    // latest-frame slot
    mutable std::mutex slot_mu_;
    std::condition_variable slot_cv_;
    std::optional<Frame> latest_;
    std::uint64_t last_returned_seq_ = 0;
    std::atomic<std::uint64_t> total_received_{0};

    // fps EMA
    mutable std::mutex fps_mu_;
    std::deque<std::chrono::steady_clock::time_point> recv_times_;
    std::atomic<double> recv_fps_{0.0};
};

}  // namespace fitra::camera
