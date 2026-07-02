// record_3cam — synchronized N-camera (2 or 3) raw MP4 recorder.
//
// Records the rig's cameras to one MP4 per camera (raw_cam0.mp4, raw_cam1.mp4,
// [raw_cam2.mp4]) for the spatial-filtering verification harness
// (docs/design/pose-3d-spatial-filtering.md, milestone M-infra). The clips feed
// tools/dump_keypoints_3d, which pairs the cameras BY FRAME INDEX, so this tool
// produces index-aligned, equal-length clips:
//
//   * cameras are built from the SAME session config the live `run` uses
//     (--config session.yaml) via the same per-camera V4l2Options that
//     app/camera_builder builds (resolution / capture-resolution override /
//     pixel format / exposure). Geometry (resolution / crop / exposure) matches
//     the live pose pipeline; the JPEG DECODE does not — this tool always uses
//     CPU cv::imdecode, whereas a live `pixel_format: nvjpeg` camera decodes on
//     the Jetson HW NVJPEG block, so decoded pixels can differ slightly. The
//     offline dump therefore reproduces live *closely* (and is internally
//     self-consistent, so its jitter numbers are valid), not bit-for-bit.
//   * pacing is "write one frame to every camera only when ALL cameras have a
//     fresh (un-written) latest frame", so the slowest camera gates the rate
//     and no duplicate frames are emitted. This mirrors the live triangulation
//     input (latest-frame-wins snapshot per camera within a sync window).
//
// Standalone: no TensorRT execution, no MultiCameraDriver, no Crow. MJPEG/NVJPEG
// payloads decode on the CPU via cv::imdecode (HW NVJPEG is not needed for a
// recorder); YUYV via cv::cvtColor. A downscaling camera (cap_* override) is
// resized to the output resolution with INTER_AREA, exactly like FrameSource.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "camera/v4l2_capture.hpp"
#include "config/main_config.hpp"

namespace fs = std::filesystem;

namespace {

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

struct Args {
    std::string config;
    std::string out;
    std::string name;          // pose name; "" => raw_cam{i}.mp4
    double      seconds = 0.0;  // 0 = until Ctrl-C
    double      warmup_s = 1.5;  // fps measurement window before opening writers
};

void usage(const char* argv0) {
    std::fprintf(stderr,
        "usage: %s --config session.yaml --out DIR [options]\n"
        "\n"
        "Record N (2 or 3) synchronized raw MP4 clips for the spatial-filtering\n"
        "harness. Cameras are built from the session config exactly like the\n"
        "live `run` (resolution / pixel format / exposure), so the recorded\n"
        "frames match what the live pose pipeline sees.\n"
        "\n"
        "  --config PATH    session YAML (same one `run` uses; cameras section)\n"
        "  --out DIR        output directory (created; must not already exist)\n"
        "  --name NAME      pose name -> raw_<NAME>_cam{i}.mp4 (default raw_cam{i}.mp4)\n"
        "  --seconds X      stop after X seconds (default: until Ctrl-C)\n"
        "  --warmup-s X     fps-measurement warmup before recording (default 1.5)\n",
        argv0);
}

bool parse_args(int argc, char** argv, Args& a) {
    auto need = [&](int& i) -> const char* {
        if (i + 1 >= argc) { std::fprintf(stderr, "missing value for %s\n", argv[i]); return nullptr; }
        return argv[++i];
    };
    for (int i = 1; i < argc; ++i) {
        std::string_view k{argv[i]};
        const char* v = nullptr;
        if (k == "--config")        { if (!(v = need(i))) return false; a.config = v; }
        else if (k == "--out")      { if (!(v = need(i))) return false; a.out = v; }
        else if (k == "--name")     { if (!(v = need(i))) return false; a.name = v; }
        else if (k == "--seconds")  { if (!(v = need(i))) return false; a.seconds = std::atof(v); }
        else if (k == "--warmup-s") { if (!(v = need(i))) return false; a.warmup_s = std::atof(v); }
        else if (k == "--help" || k == "-h") { usage(argv[0]); std::exit(EXIT_SUCCESS); }
        else { std::fprintf(stderr, "unknown argument: %s\n", argv[i]); return false; }
    }
    if (a.config.empty()) { std::fprintf(stderr, "--config is required\n"); return false; }
    if (a.out.empty())    { std::fprintf(stderr, "--out is required\n");    return false; }
    return true;
}

// Build a single camera's V4l2Options from the session config, mirroring
// app/camera_builder::make_frame_sources (decode-only branch). Kept in sync by
// hand: this tool links fitra_camera + fitra_config, not fitra_app.
fitra::camera::V4l2Options camera_options(const fitra::config::MainOptions& cfg,
                                          std::size_t i) {
    fitra::camera::V4l2Options o;
    o.device_path = cfg.cam_paths[i];
    o.width       = cfg.width;
    o.height      = cfg.height;
    o.cap_width   = cfg.cam_cap_width[i];
    o.cap_height  = cfg.cam_cap_height[i];
    o.fps         = cfg.fps;
    o.n_buffers   = cfg.n_buffers;
    const std::string& pf = cfg.cam_pixel_format[i].empty()
                                ? cfg.pixel_format
                                : cfg.cam_pixel_format[i];
    o.pixel_format = (pf == "yuyv")   ? fitra::camera::PixFmt::Yuyv
                   : (pf == "nvjpeg") ? fitra::camera::PixFmt::Nvjpeg
                                      : fitra::camera::PixFmt::Mjpeg;
    o.exposure_mode  = fitra::camera::parse_exposure_mode(cfg.cam_exposure_mode[i]);
    o.exposure_us100 = cfg.cam_exposure[i];
    o.gain           = cfg.cam_gain[i];
    o.ae_target      = cfg.cam_ae_target[i];
    return o;
}

// Decode a raw V4L2 payload to BGR at the output resolution. Mirrors
// FrameSource::decode_loop: cv::imdecode for MJPEG/NVJPEG payloads (both are
// JPEG bytes), cv::cvtColor for YUYV, then INTER_AREA downscale to the output
// dims for a capture-resolution-override (center-crop) camera. Returns false on
// a corrupt/short frame (skip it, like the live path).
bool decode_to_bgr(const fitra::camera::Frame& raw,
                   const fitra::camera::V4l2Options& o,
                   cv::Mat& out) {
    if (o.pixel_format == fitra::camera::PixFmt::Yuyv) {
        const int cw = o.capture_w(), ch = o.capture_h();
        if (static_cast<int>(raw.data.size()) < cw * ch * 2) return false;
        cv::Mat yuy2(ch, cw, CV_8UC2, const_cast<std::uint8_t*>(raw.data.data()));
        cv::cvtColor(yuy2, out, cv::COLOR_YUV2BGR_YUYV);
    } else {
        out = cv::imdecode(raw.data, cv::IMREAD_COLOR);
        if (out.empty()) return false;
    }
    if (o.downscaling() && (out.cols != o.width || out.rows != o.height)) {
        cv::resize(out, out, cv::Size(o.width, o.height), 0, 0, cv::INTER_AREA);
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    if (!parse_args(argc, argv, args)) { usage(argv[0]); return EXIT_FAILURE; }

    fitra::config::MainOptions cfg;
    try {
        fitra::config::load_main_config(args.config, cfg);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "failed to load config %s: %s\n", args.config.c_str(), e.what());
        return EXIT_FAILURE;
    }

    // Active cameras in slot order (cam0..). camera_builder skips empty paths
    // but keeps slot order; the harness expects cam0,cam1[,cam2] contiguous.
    std::vector<std::size_t> slots;
    for (std::size_t i = 0; i < cfg.cam_paths.size(); ++i)
        if (!cfg.cam_paths[i].empty()) slots.push_back(i);
    const std::size_t n = slots.size();
    if (n < 2 || n > 3) {
        std::fprintf(stderr, "config has %zu active cameras; the harness needs 2 or 3\n", n);
        return EXIT_FAILURE;
    }

    const fs::path out_dir{args.out};
    if (fs::exists(out_dir)) {
        std::fprintf(stderr, "output dir already exists: %s\n", args.out.c_str());
        return EXIT_FAILURE;
    }
    // Defer creating out_dir until after the cameras start + warm up, so a
    // camera failure does not leave a stale empty directory that would then
    // trip the "already exists" guard on retry.

    // Open + start every camera.
    std::vector<fitra::camera::V4l2Options> opts(n);
    std::vector<std::unique_ptr<fitra::camera::V4l2Capture>> caps;
    caps.reserve(n);
    for (std::size_t k = 0; k < n; ++k) {
        opts[k] = camera_options(cfg, slots[k]);
        caps.push_back(std::make_unique<fitra::camera::V4l2Capture>(opts[k]));
    }
    try {
        for (auto& c : caps) c->start();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "camera start failed: %s\n", e.what());
        return EXIT_FAILURE;
    }
    // The driver may have adjusted the accepted resolution at start(); re-read
    // so the writers and downscale target use the real output dims.
    for (std::size_t k = 0; k < n; ++k) opts[k] = caps[k]->options();

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    using clock = std::chrono::steady_clock;

    // --- Warmup: let capture spin up, measure the slowest delivery rate so the
    // MP4 fps (and the analyzer's --fps) reflects reality. ------------------
    std::printf("warming up %.1fs (measuring camera delivery rate)...\n", args.warmup_s);
    const auto warm_start = clock::now();
    while (!g_stop.load() &&
           std::chrono::duration<double>(clock::now() - warm_start).count() < args.warmup_s) {
        fitra::camera::Frame fr;
        for (auto& c : caps) c->try_pop_latest(fr);  // drain; keeps recv_fps current
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (g_stop.load()) { for (auto& c : caps) c->stop(); return EXIT_SUCCESS; }

    // Every camera must deliver: the sync loop only writes a frame set when ALL
    // cameras are fresh, so one stalled camera (e.g. failed to enumerate on the
    // shared USB bus) would otherwise pass warmup here and then silently produce
    // empty clips until the time limit. Gate per-camera, not on the live subset.
    double min_fps = 1e9;
    bool all_live = true;
    for (std::size_t k = 0; k < n; ++k) {
        const double f = caps[k]->recv_fps();
        if (f < 1.0) {
            std::fprintf(stderr,
                         "cam%zu (%s) is not delivering frames (recv_fps=%.2f); aborting\n",
                         k, opts[k].device_path.c_str(), f);
            all_live = false;
        }
        min_fps = std::min(min_fps, f);
    }
    if (!all_live) {
        for (auto& c : caps) c->stop();
        return EXIT_FAILURE;
    }
    const int out_fps = std::max(1, static_cast<int>(min_fps + 0.5));

    // Cameras are alive and delivering — now it's safe to create the output dir.
    std::error_code ec;
    fs::create_directories(out_dir, ec);
    if (ec) {
        std::fprintf(stderr, "cannot create %s: %s\n", args.out.c_str(), ec.message().c_str());
        for (auto& c : caps) c->stop();
        return EXIT_FAILURE;
    }

    // --- Open one MP4 writer per camera at the measured output fps. --------
    std::vector<cv::VideoWriter> writers(n);
    std::vector<std::string> filenames(n);
    for (std::size_t k = 0; k < n; ++k) {
        filenames[k] = args.name.empty()
                           ? ("raw_cam" + std::to_string(k) + ".mp4")
                           : ("raw_" + args.name + "_cam" + std::to_string(k) + ".mp4");
        const fs::path p = out_dir / filenames[k];
        writers[k].open(p.string(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                        static_cast<double>(out_fps),
                        cv::Size(opts[k].width, opts[k].height));
        if (!writers[k].isOpened()) {
            std::fprintf(stderr, "cannot open writer %s\n", p.string().c_str());
            for (auto& c : caps) c->stop();
            return EXIT_FAILURE;
        }
    }

    std::printf("recording %zu camera(s) @ %dx%d, out_fps=%d -> %s  (Ctrl-C to stop%s)\n",
                n, opts[0].width, opts[0].height, out_fps, args.out.c_str(),
                args.seconds > 0 ? "" : ", no time limit");

    // --- Synchronized record loop. Write a frame to every camera only once
    // ALL have a fresh (un-written) latest frame, so clips stay index-aligned
    // and the slowest camera paces the rate without emitting duplicates. -----
    std::vector<cv::Mat> latest_bgr(n);
    std::vector<bool> fresh(n, false);
    std::vector<std::uint64_t> written(n, 0);
    const auto rec_start = clock::now();
    auto last_status = rec_start;
    auto last_write = rec_start;
    bool stalled_warned = false;

    while (!g_stop.load()) {
        for (std::size_t k = 0; k < n; ++k) {
            fitra::camera::Frame fr;
            if (!caps[k]->try_pop_latest(fr)) continue;
            cv::Mat bgr;
            if (!decode_to_bgr(fr, opts[k], bgr)) continue;  // skip corrupt frame
            latest_bgr[k] = std::move(bgr);
            fresh[k] = true;
        }

        bool all_fresh = true;
        for (std::size_t k = 0; k < n; ++k) all_fresh = all_fresh && fresh[k];

        const auto now = clock::now();
        if (all_fresh) {
            for (std::size_t k = 0; k < n; ++k) {
                writers[k].write(latest_bgr[k]);
                ++written[k];
                fresh[k] = false;
            }
            last_write = now;
            stalled_warned = false;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            if (!stalled_warned &&
                std::chrono::duration<double>(now - last_write).count() > 3.0) {
                std::printf("\nWARNING: no synchronized frame set in 3s — a camera "
                            "may have stalled (check the bus / device)\n");
                stalled_warned = true;
            }
        }

        const double elapsed = std::chrono::duration<double>(now - rec_start).count();
        if (args.seconds > 0 && elapsed >= args.seconds) break;
        if (now - last_status >= std::chrono::seconds(1)) {
            last_status = now;
            std::string s;
            for (std::size_t k = 0; k < n; ++k) {
                char b[80];
                std::snprintf(b, sizeof(b), "cam%zu %5.1ffps(%" PRIu64 ") ", k,
                              caps[k]->recv_fps(), written[k]);
                s += b;
            }
            std::printf("\r%s| t=%.0fs   ", s.c_str(), elapsed);
            std::fflush(stdout);
        }
    }
    std::printf("\n");

    for (auto& w : writers) w.release();
    for (auto& c : caps) c->stop();

    const double duration = std::chrono::duration<double>(clock::now() - rec_start).count();

    // meta.json — settings + measured per-camera fps, mirroring excal_record.
    std::FILE* meta = std::fopen((out_dir / "meta.json").c_str(), "w");
    if (meta) {
        std::fprintf(meta,
                     "{\n  \"version\": 1,\n  \"tool\": \"record_3cam\",\n"
                     "  \"config\": \"%s\",\n  \"width\": %d,\n  \"height\": %d,\n"
                     "  \"out_fps\": %d,\n  \"duration_s\": %.3f,\n"
                     "  \"num_cameras\": %zu,\n  \"cameras\": [\n",
                     args.config.c_str(), opts[0].width, opts[0].height, out_fps,
                     duration, n);
        for (std::size_t k = 0; k < n; ++k) {
            std::fprintf(meta,
                         "    {\"file\": \"%s\", \"device\": \"%s\", \"frames\": %" PRIu64
                         ", \"fps_written\": %.2f}%s\n",
                         filenames[k].c_str(), opts[k].device_path.c_str(), written[k],
                         duration > 0 ? written[k] / duration : 0.0,
                         k + 1 < n ? "," : "");
        }
        std::fprintf(meta, "  ]\n}\n");
        std::fclose(meta);
    }

    for (std::size_t k = 0; k < n; ++k)
        std::printf("cam%zu: %" PRIu64 " frames (%.2f fps) -> %s\n", k, written[k],
                    duration > 0 ? written[k] / duration : 0.0, filenames[k].c_str());

    // Print a ready-to-run dump_keypoints_3d command (the next harness step).
    // Bake in the true measured fps (frames actually written / duration), NOT
    // the header out_fps: when the record loop is encode-bound they diverge, and
    // dump uses --fps for the Kalman dt. written[] are equal (lockstep writes).
    const double fps_written = (duration > 0.0 && written[0] > 0)
                                   ? static_cast<double>(written[0]) / duration
                                   : static_cast<double>(out_fps);
    std::printf("\nnext: analyze with dump_keypoints_3d, e.g.\n  dump_keypoints_3d");
    for (std::size_t k = 0; k < n; ++k)
        std::printf(" --video %s/%s", args.out.c_str(), filenames[k].c_str());
    std::printf(" \\\n    --calib <extrinsics.yaml> --det-engine <yolox.engine>"
                " --pose-engine <rtmpose.engine> \\\n"
                "    --keypoint-format %s --fps %.2f --out %s/dump.jsonl\n",
                cfg.keypoint_format.c_str(), fps_written, args.out.c_str());
    // jitter is fps-independent (position std-dev) and takes no --fps; lag does.
    std::printf("(then: analyze_3d_jitter_lag.py jitter %s/dump.jsonl        "
                "# lag needs --fps %.2f)\n",
                args.out.c_str(), fps_written);
    return EXIT_SUCCESS;
}
