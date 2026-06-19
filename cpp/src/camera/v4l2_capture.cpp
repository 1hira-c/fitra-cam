#include "camera/v4l2_capture.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "util/logging.hpp"

namespace fitra::camera {

namespace {

int xioctl(int fd, unsigned long req, void* arg) {
    int r;
    do {
        r = ::ioctl(fd, req, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

[[noreturn]] void throw_errno(const std::string& what) {
    std::ostringstream oss;
    oss << what << ": errno=" << errno << " (" << std::strerror(errno) << ")";
    throw std::runtime_error(oss.str());
}

}  // namespace

V4l2Capture::V4l2Capture(V4l2Options opts) : opts_{std::move(opts)} {}

bool V4l2Capture::set_ctrl(unsigned int id, int value, const char* name) {
    v4l2_control c{};
    c.id    = id;
    c.value = value;
    if (xioctl(fd_, VIDIOC_S_CTRL, &c) < 0) {
        FITRA_LOG_WARN("v4l2 {}: set {}={} failed: errno={} ({})",
                       opts_.device_path, name, value, errno, std::strerror(errno));
        return false;
    }
    return true;
}

bool V4l2Capture::set_exposure_us100(int v) {
    return set_ctrl(V4L2_CID_EXPOSURE_ABSOLUTE, v, "exposure_absolute");
}

bool V4l2Capture::set_gain(int v) { return set_ctrl(V4L2_CID_GAIN, v, "gain"); }

void V4l2Capture::apply_exposure_controls() {
    // Query the gain range (the assist controller clamps to it). Harmless if
    // the control is absent — keep the [0,255] default.
    v4l2_queryctrl qc{};
    qc.id = V4L2_CID_GAIN;
    if (xioctl(fd_, VIDIOC_QUERYCTRL, &qc) == 0 && !(qc.flags & V4L2_CTRL_FLAG_DISABLED)) {
        gain_min_ = qc.minimum;
        gain_max_ = qc.maximum;
    }
    if (opts_.exposure_mode == V4l2Options::ExposureMode::Auto) return;

    // Manual & Assist: turn the camera's own auto-exposure OFF (so exposure
    // time can't run long -> no motion blur, no fps-budget overrun) and fix
    // focus (autofocus hunting also stalls frames). Then set the initial
    // exposure / gain. Assist drives them live afterwards from FrameSource.
    set_ctrl(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, "auto_exposure=manual");
    set_ctrl(V4L2_CID_FOCUS_AUTO, 0, "focus_auto=off");  // ok if camera has no AF
    if (opts_.exposure_us100 > 0)
        set_ctrl(V4L2_CID_EXPOSURE_ABSOLUTE, opts_.exposure_us100, "exposure_absolute");
    if (opts_.gain >= 0)
        set_ctrl(V4L2_CID_GAIN, opts_.gain, "gain");
    FITRA_LOG_INFO("v4l2 {}: exposure_mode={} exposure={}x100us gain={} (gain range [{},{}])",
                   opts_.device_path,
                   opts_.exposure_mode == V4l2Options::ExposureMode::Manual ? "manual" : "assist",
                   opts_.exposure_us100, opts_.gain, gain_min_, gain_max_);
}

V4l2Capture::~V4l2Capture() {
    try { stop(); } catch (...) {}
}

void V4l2Capture::start() {
    fd_ = ::open(opts_.device_path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) throw_errno("open(" + opts_.device_path + ")");

    // Set pixel format. Nvjpeg captures the same MJPEG stream as Mjpeg; only
    // the downstream decode differs (HW NVJPEG vs CPU cv::imdecode).
    const bool want_yuyv = (opts_.pixel_format == PixFmt::Yuyv);
    const __u32 want_fourcc = want_yuyv ? V4L2_PIX_FMT_YUYV : V4L2_PIX_FMT_MJPEG;
    const char* fmt_name    = want_yuyv ? "YUYV"
                            : (opts_.pixel_format == PixFmt::Nvjpeg ? "MJPG(HW)" : "MJPG");
    // Negotiate the *capture* resolution (>= output resolution). When the
    // camera is downscaling (cap_* set), the captured frame is resized to
    // width/height in FrameSource; the V4L2 format must match the larger
    // capture dims so the driver fills full sensor frames.
    const int req_w = opts_.capture_w();
    const int req_h = opts_.capture_h();
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = req_w;
    fmt.fmt.pix.height      = req_h;
    fmt.fmt.pix.pixelformat = want_fourcc;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;
    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) throw_errno("VIDIOC_S_FMT");
    if (fmt.fmt.pix.pixelformat != want_fourcc) {
        throw std::runtime_error(std::string("driver did not accept ") + fmt_name + " format");
    }
    if (static_cast<int>(fmt.fmt.pix.width)  != req_w
     || static_cast<int>(fmt.fmt.pix.height) != req_h) {
        FITRA_LOG_WARN("driver returned {}x{} (requested {}x{})",
                       fmt.fmt.pix.width, fmt.fmt.pix.height, req_w, req_h);
        const int acc_w = static_cast<int>(fmt.fmt.pix.width);
        const int acc_h = static_cast<int>(fmt.fmt.pix.height);
        // Write the accepted dims back to whichever field drove the request:
        // the capture override (downscaling) or the output resolution.
        if (opts_.cap_width > 0)  opts_.cap_width  = acc_w;
        else                      opts_.width      = acc_w;
        if (opts_.cap_height > 0) opts_.cap_height = acc_h;
        else                      opts_.height     = acc_h;
    }

    // Frame interval (best-effort).
    v4l2_streamparm sp{};
    sp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    sp.parm.capture.timeperframe.numerator   = 1;
    sp.parm.capture.timeperframe.denominator = opts_.fps;
    if (xioctl(fd_, VIDIOC_S_PARM, &sp) < 0) {
        FITRA_LOG_WARN("VIDIOC_S_PARM failed; continuing with driver default fps");
    }

    // Request N mmap buffers.
    v4l2_requestbuffers req{};
    req.count  = static_cast<__u32>(opts_.n_buffers);
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) throw_errno("VIDIOC_REQBUFS");
    if (req.count < 2) {
        throw std::runtime_error("driver granted <2 buffers; cannot pipeline");
    }
    bufs_.resize(req.count);

    // Query and mmap each buffer.
    for (std::size_t i = 0; i < bufs_.size(); ++i) {
        v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = static_cast<__u32>(i);
        if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) throw_errno("VIDIOC_QUERYBUF");
        void* p = ::mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                         fd_, buf.m.offset);
        if (p == MAP_FAILED) throw_errno("mmap");
        bufs_[i].ptr    = p;
        bufs_[i].length = buf.length;
    }

    // Query gain range + apply manual exposure / gain / focus controls (no-op
    // in Auto mode). Done before STREAMON so the first frames already reflect
    // the configured exposure.
    apply_exposure_controls();

    // Queue all buffers.
    for (std::size_t i = 0; i < bufs_.size(); ++i) {
        v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = static_cast<__u32>(i);
        if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) throw_errno("VIDIOC_QBUF (initial)");
    }

    if (const char* e = std::getenv("FITRA_CAPTURE_DEBUG"); e && *e && std::string(e) != "0")
        capture_debug_ = true;

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) throw_errno("VIDIOC_STREAMON");

    if (opts_.downscaling()) {
        FITRA_LOG_INFO("v4l2: {} opened (capture {}x{} -> output {}x{}, {}, {} buffers, requested {} fps)",
                       opts_.device_path, opts_.capture_w(), opts_.capture_h(),
                       opts_.width, opts_.height, fmt_name, bufs_.size(), opts_.fps);
    } else {
        FITRA_LOG_INFO("v4l2: {} opened ({}x{}, {}, {} buffers, requested {} fps)",
                       opts_.device_path, opts_.width, opts_.height,
                       fmt_name, bufs_.size(), opts_.fps);
    }

    stop_.store(false);
    worker_ = std::thread{&V4l2Capture::worker_loop, this};
}

void V4l2Capture::stop() {
    if (fd_ < 0 && !worker_.joinable()) return;
    stop_.store(true);
    if (worker_.joinable()) worker_.join();

    if (fd_ >= 0) {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(fd_, VIDIOC_STREAMOFF, &type);
    }
    for (auto& b : bufs_) {
        if (b.ptr && b.length) ::munmap(b.ptr, b.length);
        b.ptr = nullptr;
        b.length = 0;
    }
    bufs_.clear();
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void V4l2Capture::worker_loop() {
    while (!stop_.load()) {
        pollfd pfd{};
        pfd.fd     = fd_;
        pfd.events = POLLIN;
        int pr = ::poll(&pfd, 1, 200);  // 200ms tick to let stop_ propagate
        if (pr < 0) {
            if (errno == EINTR) continue;
            FITRA_LOG_ERROR("v4l2 poll: {}", std::strerror(errno));
            return;
        }
        if (pr == 0) continue;
        if (!(pfd.revents & POLLIN)) continue;

        v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) continue;
            FITRA_LOG_ERROR("v4l2 VIDIOC_DQBUF: {}", std::strerror(errno));
            return;
        }
        auto now = std::chrono::steady_clock::now();
        std::uint64_t seq = total_received_.fetch_add(1) + 1;

        // Driver-side drop detection: a jump in buf.sequence > 1 means the
        // driver captured frames we didn't dequeue in time (capture thread
        // descheduled). Distinguishes a scheduling stall from under-delivery.
        if (capture_debug_) {
            if (have_last_seq_ && buf.sequence > last_buf_seq_ + 1)
                driver_dropped_ += buf.sequence - last_buf_seq_ - 1;
            last_buf_seq_  = buf.sequence;
            have_last_seq_ = true;
            if (++dbg_frames_ % 120 == 0) {
                FITRA_LOG_INFO("v4l2 dbg {}: recv_fps={} driver_dropped={} "
                               "(buf.seq={})", opts_.device_path, recv_fps_.load(),
                               driver_dropped_, buf.sequence);
            }
        }

        // Copy out so we can re-queue the V4L2 buffer immediately.
        // YUYV is fixed-size; some drivers report bytesused=0 for uncompressed
        // formats, so fall back to the full mmap buffer length in that case.
        std::size_t nbytes = buf.bytesused;
        if (nbytes == 0) nbytes = bufs_[buf.index].length;
        // Fill the reusable capture-thread scratch (reuses its capacity after
        // warmup -> no per-frame heap alloc; critical for large YUYV payloads,
        // see spare_data_ doc). Swapped into latest_ under the slot lock below.
        spare_data_.assign(static_cast<std::uint8_t*>(bufs_[buf.index].ptr),
                           static_cast<std::uint8_t*>(bufs_[buf.index].ptr) + nbytes);

        // Re-queue
        v4l2_buffer qb{};
        qb.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        qb.memory = V4L2_MEMORY_MMAP;
        qb.index  = buf.index;
        if (xioctl(fd_, VIDIOC_QBUF, &qb) < 0) {
            FITRA_LOG_ERROR("v4l2 VIDIOC_QBUF: {}", std::strerror(errno));
            return;
        }

        {
            std::lock_guard<std::mutex> lk{slot_mu_};
            if (!latest_) latest_.emplace();
            // Swap: latest_ takes the freshly-filled bytes; spare_data_ reclaims
            // the previous frame's buffer (capacity retained) for the next fill.
            std::swap(latest_->data, spare_data_);
            latest_->seq         = seq;
            latest_->captured_at = now;
            slot_cv_.notify_one();  // wake a consumer parked in wait_pop_latest
        }
        update_recv_fps(now);
    }
}

void V4l2Capture::update_recv_fps(std::chrono::steady_clock::time_point now) {
    std::lock_guard<std::mutex> lk{fps_mu_};
    recv_times_.push_back(now);
    while (recv_times_.size() > 60) recv_times_.pop_front();
    if (recv_times_.size() >= 2) {
        auto span = std::chrono::duration<double>(recv_times_.back() - recv_times_.front()).count();
        if (span > 0) {
            recv_fps_.store((recv_times_.size() - 1) / span);
        }
    }
}

bool V4l2Capture::try_pop_latest(Frame& out) {
    std::lock_guard<std::mutex> lk{slot_mu_};
    if (!latest_) return false;
    if (latest_->seq == last_returned_seq_) return false;
    last_returned_seq_ = latest_->seq;
    out = *latest_;
    return true;
}

bool V4l2Capture::wait_pop_latest(Frame& out, std::atomic<bool>& consumer_stop,
                                  std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lk{slot_mu_};
    slot_cv_.wait_for(lk, timeout, [&] {
        return consumer_stop.load(std::memory_order_relaxed)
            || (latest_ && latest_->seq != last_returned_seq_);
    });
    if (!latest_ || latest_->seq == last_returned_seq_) return false;
    last_returned_seq_ = latest_->seq;
    out = *latest_;
    return true;
}

void V4l2Capture::wake() {
    std::lock_guard<std::mutex> lk{slot_mu_};
    slot_cv_.notify_all();
}

}  // namespace fitra::camera
