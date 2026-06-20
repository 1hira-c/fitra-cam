#include "camera/setup_camera_manager.hpp"

#include <exception>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "camera/v4l2_capture.hpp"
#include "util/logging.hpp"

namespace fitra::camera {

struct SetupCameraManager::Stream {
    std::unique_ptr<V4l2Capture> cap;
    PixFmt pixfmt = PixFmt::Mjpeg;
    int width = 0;
    int height = 0;
    Frame frame;                          // reused scratch for try_pop_latest
    std::vector<std::uint8_t> last_jpeg;  // cached most-recent encoded frame
};

namespace {
PixFmt parse_pixfmt(const std::string& s) {
    if (s == "yuyv") return PixFmt::Yuyv;
    // nvjpeg streams MJPEG on the wire; preview decodes it on the CPU like mjpeg.
    return PixFmt::Mjpeg;
}
}  // namespace

SetupCameraManager::SetupCameraManager() = default;

SetupCameraManager::~SetupCameraManager() { stop_all(); }

bool SetupCameraManager::start(const PreviewRequest& req, std::string& err) {
    std::lock_guard<std::mutex> lk{mu_};
    streams_.erase(req.device);  // restart if already previewing

    auto s = std::make_unique<Stream>();
    s->pixfmt = parse_pixfmt(req.pixel_format);
    s->width  = req.width;
    s->height = req.height;

    V4l2Options opts;
    opts.device_path  = req.device;
    opts.width        = req.width;
    opts.height       = req.height;
    opts.fps          = req.fps;
    opts.pixel_format = s->pixfmt;
    opts.n_buffers    = 4;
    // Exposure override (same string->enum mapping as app/camera_builder.cpp).
    opts.exposure_mode = (req.exposure_mode == "manual")
                             ? V4l2Options::ExposureMode::Manual
                             : (req.exposure_mode == "assist")
                                   ? V4l2Options::ExposureMode::Assist
                                   : V4l2Options::ExposureMode::Auto;
    opts.exposure_us100 = req.exposure;
    opts.gain           = req.gain;
    opts.ae_target      = req.ae_target;
    try {
        s->cap = std::make_unique<V4l2Capture>(std::move(opts));
        s->cap->start();
    } catch (const std::exception& e) {
        err = e.what();
        return false;
    }
    streams_[req.device] = std::move(s);
    FITRA_LOG_INFO("setup-preview: streaming {} {}x{}@{} {}", req.device,
                   req.width, req.height, req.fps, req.pixel_format);
    return true;
}

void SetupCameraManager::stop(const std::string& device) {
    std::lock_guard<std::mutex> lk{mu_};
    streams_.erase(device);  // V4l2Capture dtor stops + joins
}

void SetupCameraManager::stop_all() {
    std::lock_guard<std::mutex> lk{mu_};
    streams_.clear();
}

bool SetupCameraManager::latest_jpeg(const std::string& device,
                                     std::vector<std::uint8_t>& out) {
    std::lock_guard<std::mutex> lk{mu_};
    auto it = streams_.find(device);
    if (it == streams_.end()) return false;
    Stream& s = *it->second;

    if (s.cap->try_pop_latest(s.frame)) {
        if (s.pixfmt == PixFmt::Yuyv) {
            // Packed YUV422 -> BGR -> JPEG. capture_h() x capture_w() is the
            // actual decoded buffer geometry.
            const int w = s.cap->options().capture_w();
            const int h = s.cap->options().capture_h();
            if (static_cast<std::size_t>(w) * h * 2 == s.frame.data.size()) {
                cv::Mat yuyv(h, w, CV_8UC2,
                             const_cast<std::uint8_t*>(s.frame.data.data()));
                cv::Mat bgr;
                cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);
                std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 80};
                cv::imencode(".jpg", bgr, s.last_jpeg, params);
            }
        } else {
            // MJPEG/nvjpeg: payload is already a JPEG.
            s.last_jpeg = s.frame.data;
        }
    }
    if (s.last_jpeg.empty()) return false;
    out = s.last_jpeg;
    return true;
}

}  // namespace fitra::camera
