#pragma once
//
// SetupCameraManager — single-shot JPEG preview for the setup wizard
// (docs/design/core-pipeline-setup-mode.md). Owns a V4l2Capture per previewed
// device; the browser polls GET /api/cameras/preview.jpg at a low rate. MJPEG
// frames are returned verbatim (zero re-encode); YUYV is decoded + JPEG-encoded
// via OpenCV. No background thread — latest_jpeg() pulls + caches on the caller
// thread (Crow worker). Crow can't cleanly stream multipart/x-mixed-replace
// from a handler, hence the poll-a-snapshot model.

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace fitra::camera {

class V4l2Capture;

struct PreviewRequest {
    std::string device;        // /dev/v4l/by-path/... (or /dev/videoN)
    int width  = 640;
    int height = 480;
    int fps    = 30;
    std::string pixel_format = "mjpeg";   // mjpeg | yuyv | nvjpeg
    // Per-camera exposure controls so the preview reflects an override before
    // it is committed. exposure_mode "" / "auto" leaves the camera untouched.
    std::string exposure_mode;            // "" | "auto" | "manual" | "assist"
    int exposure  = 0;                    // V4L2 exposure_absolute, 100us units; 0 = leave
    int gain      = -1;                   // V4L2 gain; <0 = leave at camera default
    int ae_target = 110;                  // assist target mean luma
};

class SetupCameraManager {
public:
    SetupCameraManager();
    ~SetupCameraManager();

    SetupCameraManager(const SetupCameraManager&) = delete;
    SetupCameraManager& operator=(const SetupCameraManager&) = delete;

    // Open + stream `req.device`. Restarts the stream if the device is already
    // previewing. Returns false + a reason in `err` on failure.
    bool start(const PreviewRequest& req, std::string& err);
    void stop(const std::string& device);
    void stop_all();

    // Fill `out` with the latest JPEG for `device`. Returns false if the device
    // is not streaming or no frame has arrived yet.
    bool latest_jpeg(const std::string& device, std::vector<std::uint8_t>& out);

private:
    struct Stream;
    std::mutex mu_;
    std::map<std::string, std::unique_ptr<Stream>> streams_;
};

}  // namespace fitra::camera
