#include "app/camera_builder.hpp"

#include "camera/v4l2_capture.hpp"
#include "infer/yolox.hpp"

namespace fitra::app {

CameraSet make_frame_sources(const config::MainOptions& opts,
                             TrtStack* trt,
                             std::shared_ptr<std::atomic<bool>> recording_flag) {
    CameraSet set;
    for (std::size_t i = 0; i < opts.cam_paths.size(); ++i) {
        const auto& path = opts.cam_paths[i];
        if (path.empty()) continue;
        camera::V4l2Options o;
        o.device_path = path;
        o.width  = opts.width;
        o.height = opts.height;
        // Per-camera capture-resolution override (0 = no override/downscale).
        // FrameSource resizes the capture to width/height when these are set.
        o.cap_width  = opts.cam_cap_width[i];
        o.cap_height = opts.cam_cap_height[i];
        o.fps    = opts.fps;
        o.n_buffers = opts.n_buffers;
        o.pixel_format = (opts.pixel_format == "yuyv")
                             ? camera::PixFmt::Yuyv
                             : (opts.pixel_format == "nvjpeg")
                                   ? camera::PixFmt::Nvjpeg
                                   : camera::PixFmt::Mjpeg;
        auto cap = std::make_unique<camera::V4l2Capture>(o);

        if (!trt) {
            // Decode-only: no detection, no prebake; DecodedFrame::bgr is
            // retained for the CPU consumer (AprilTag detection).
            set.sources.push_back(std::make_unique<camera::FrameSource>(
                std::move(cap), nullptr, camera::FrameSource::Options{}));
            continue;
        }

        // One Yolox (per-camera IExecutionContext) per cam, wrapping the
        // shared ICudaEngine. FrameSource runs its own decode + YOLOX thread,
        // so all N cameras run capture/decode/YOLOX in parallel.
        auto yolox_eng = infer::TrtEngine::from_shared(trt->yolox_shared);
        infer::Yolox::Options yolo_opts;
        yolo_opts.score_thr = opts.det_score;
        auto yolox = std::make_unique<infer::Yolox>(*yolox_eng, yolo_opts);
        set.yolox_engines.push_back(std::move(yolox_eng));

        camera::FrameSource::Options src_opts;
        src_opts.det_frequency = opts.det_frequency;
        src_opts.single_person = !opts.multi_person;
        src_opts.fake_bbox_if_empty = opts.bench_fake_bbox;
        // Subject-calibration recording taps MultiCameraDriver's frame tap.
        // The same flag pauses YOLOX + RTMPose pre-bake while recording so
        // disk I/O has the CPU/GPU headroom and we don't burn cycles on a
        // pose feed nobody is watching.
        src_opts.calib_recording_flag = recording_flag;
        // Have the per-camera worker pre-bake the RTMPose input so the
        // central inference thread only does memcpy + GPU + decode.
        const auto& rtmpose_opts = trt->rtmpose->options();
        set.sources.push_back(std::make_unique<camera::FrameSource>(
            std::move(cap), std::move(yolox), src_opts, &rtmpose_opts));
    }
    return set;
}

}  // namespace fitra::app
