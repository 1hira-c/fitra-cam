// record_3cam — synchronized N-camera (2 or 3) recorder (MP4 or MJPEG passthrough).
//
// Records the rig's cameras — one MP4 per camera (raw_cam0.mp4, ...) or, with
// --format mjpeg, a per-camera JPEG sequence (cam0/000001.jpg, ...) — for the
// spatial-filtering / spatiotemporal-filter verification harness
// (docs/design/pose-3d-spatial-filtering.md M-infra, pose-3d-spatiotemporal-filter.md
// M-C4). The clips feed tools/dump_keypoints_3d, which pairs the cameras BY FRAME
// INDEX, so this tool produces index-aligned, equal-length clips:
//
// FORMATS:
//   * mp4 (default): decode each payload (cv::imdecode / cvtColor) + downscale to
//     the output resolution, then mp4v-encode. Encode-bound — on 1280x960x3cam
//     this caps at ~8.6 fps (imdecode ~23ms + mp4v encode ~30ms, ×3 serial), too
//     coarse to resolve fast motion / lag (see the M-infra changelog).
//   * mjpeg (recorder v2): write the camera's MJPEG payload VERBATIM — no decode,
//     no re-encode — to cam{k}/NNNNNN.jpg. Costs almost no CPU, so 3-cam capture
//     runs at the true camera rate (~30 fps), which is what the spatiotemporal
//     filter's motion / lag tuning (M-C4) needs. dump_keypoints_3d reads each
//     camera as an image sequence via a printf pattern (--video cam{k}/%06d.jpg);
//     no harness change needed. RESOLUTION: passthrough stores the NATIVE capture
//     resolution (verbatim), NOT a downscaled output res — the calibration fed to
//     the harness must match the stored (capture) resolution. A downscaling camera
//     (cap_* override) is flagged with a warning in this mode.
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
    std::string format = "mp4"; // "mp4" | "mjpeg" (verbatim JPEG passthrough)
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
        "  --format FMT     mp4 (default) | mjpeg (verbatim JPEG passthrough, ~30fps,\n"
        "                   cam{i}/NNNNNN.jpg; harness reads --video cam{i}/%%06d.jpg)\n"
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
        else if (k == "--format")   { if (!(v = need(i))) return false; a.format = v; }
        else if (k == "--seconds")  { if (!(v = need(i))) return false; a.seconds = std::atof(v); }
        else if (k == "--warmup-s") { if (!(v = need(i))) return false; a.warmup_s = std::atof(v); }
        else if (k == "--help" || k == "-h") { usage(argv[0]); std::exit(EXIT_SUCCESS); }
        else { std::fprintf(stderr, "unknown argument: %s\n", argv[i]); return false; }
    }
    if (a.config.empty()) { std::fprintf(stderr, "--config is required\n"); return false; }
    if (a.out.empty())    { std::fprintf(stderr, "--out is required\n");    return false; }
    if (a.format != "mp4" && a.format != "mjpeg") {
        std::fprintf(stderr, "--format must be mp4 or mjpeg (got '%s')\n", a.format.c_str());
        return false;
    }
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

// A quick JPEG sanity check for the passthrough path: a valid JFIF/EXIF frame
// starts with the SOI marker 0xFFD8. Skips a short/corrupt V4L2 dequeue so the
// harness never has to imdecode garbage (the mp4 path gets the same protection
// from decode_to_bgr returning false).
bool looks_like_jpeg(const std::vector<std::uint8_t>& data) {
    return data.size() > 2 && data[0] == 0xFF && data[1] == 0xD8;
}

// Streaming verbatim MJPEG -> AVI muxer. Writes the camera's JPEG payloads into
// '00dc' chunks of a standard MJPG AVI — NO decode, NO re-encode — so recording
// costs almost no CPU and the stored frames are byte-identical to what the live
// CPU decode path would imdecode. cv::VideoCapture reads the result natively
// (harness: --video raw_camK.avi, no pattern). Sizes / frame count / fps are
// back-filled in finalize(). The byte layout was validated by a round-trip
// probe (write -> cv::VideoCapture reads back the exact frame count + dims).
class AviMjpegWriter {
public:
    bool open(const std::string& path, int w, int h) {
        f_.open(path, std::ios::binary | std::ios::trunc);
        if (!f_.is_open()) return false;
        w_ = w; h_ = h;
        fourcc("RIFF"); riff_size_pos_ = f_.tellp(); u32(0); fourcc("AVI ");
        // LIST hdrl — fixed size (avih + LIST strl(strh + strf)).
        const std::uint32_t strl_bytes = 4 + (8 + 56) + (8 + 40);
        const std::uint32_t hdrl_bytes = 4 + (8 + 56) + (8 + strl_bytes);
        fourcc("LIST"); u32(hdrl_bytes); fourcc("hdrl");
        fourcc("avih"); u32(56);
        usec_avih_ = f_.tellp(); u32(33333);   // dwMicroSecPerFrame (patched)
        u32(0); u32(0); u32(0x10);              // maxbytes, pad, flags=HASINDEX
        total_avih_ = f_.tellp(); u32(0);       // dwTotalFrames (patched)
        u32(0); u32(1); u32(0);                 // initial, streams=1, bufsize
        u32(static_cast<std::uint32_t>(w_)); u32(static_cast<std::uint32_t>(h_));
        u32(0); u32(0); u32(0); u32(0);         // reserved[4]
        fourcc("LIST"); u32(strl_bytes); fourcc("strl");
        fourcc("strh"); u32(56);
        fourcc("vids"); fourcc("MJPG"); u32(0); u16(0); u16(0); u32(0);
        scale_strh_ = f_.tellp(); u32(33333);   // dwScale = usec (patched)
        u32(1000000);                           // dwRate -> rate/scale = fps
        u32(0);
        length_strh_ = f_.tellp(); u32(0);      // dwLength = frames (patched)
        u32(0); u32(0xFFFFFFFF); u32(0);
        u16(0); u16(0); u16(static_cast<std::uint16_t>(w_)); u16(static_cast<std::uint16_t>(h_));
        fourcc("strf"); u32(40);
        u32(40); u32(static_cast<std::uint32_t>(w_)); u32(static_cast<std::uint32_t>(h_));
        u16(1); u16(24); fourcc("MJPG");
        u32(static_cast<std::uint32_t>(w_ * h_ * 3));
        u32(0); u32(0); u32(0); u32(0);
        fourcc("LIST"); movi_size_pos_ = f_.tellp(); u32(0);
        movi_start_ = f_.tellp();               // position of the 'movi' fourcc
        fourcc("movi");
        return f_.good();
    }

    void add(const std::uint8_t* data, std::size_t n) {
        const std::streamoff chunk = f_.tellp();
        fourcc("00dc"); u32(static_cast<std::uint32_t>(n));
        f_.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(n));
        if (n & 1) { char z = 0; f_.write(&z, 1); }  // pad chunks to even length
        idx_.push_back({static_cast<std::uint32_t>(chunk - static_cast<std::streamoff>(movi_start_)),
                        static_cast<std::uint32_t>(n)});
        ++count_;
    }

    bool finalize(double fps) {
        if (!f_.is_open()) return false;
        const std::streamoff movi_end = f_.tellp();
        fourcc("idx1"); u32(static_cast<std::uint32_t>(idx_.size() * 16));
        for (const auto& e : idx_) { fourcc("00dc"); u32(0x10); u32(e.first); u32(e.second); }
        const std::streamoff file_end = f_.tellp();
        const std::uint32_t usec = static_cast<std::uint32_t>(1.0e6 / (fps > 0 ? fps : 30.0) + 0.5);
        patch(riff_size_pos_, static_cast<std::uint32_t>(file_end - 8));
        patch(usec_avih_,  usec);
        patch(total_avih_, static_cast<std::uint32_t>(count_));
        patch(scale_strh_, usec);
        patch(length_strh_, static_cast<std::uint32_t>(count_));
        patch(movi_size_pos_, static_cast<std::uint32_t>(movi_end - static_cast<std::streamoff>(movi_start_)));
        f_.flush();
        const bool ok = f_.good();
        f_.close();
        return ok;
    }

    std::uint64_t count() const { return count_; }

private:
    void u16(std::uint16_t v) { char b[2] = {char(v & 0xFF), char((v >> 8) & 0xFF)}; f_.write(b, 2); }
    void u32(std::uint32_t v) {
        char b[4] = {char(v & 0xFF), char((v >> 8) & 0xFF), char((v >> 16) & 0xFF), char((v >> 24) & 0xFF)};
        f_.write(b, 4);
    }
    void fourcc(const char* s) { f_.write(s, 4); }
    void patch(std::streampos pos, std::uint32_t v) {
        const std::streampos cur = f_.tellp();
        f_.seekp(pos); u32(v); f_.seekp(cur);
    }

    std::ofstream f_;
    int w_ = 0, h_ = 0;
    std::uint64_t count_ = 0;
    std::streampos riff_size_pos_, usec_avih_, total_avih_, scale_strh_,
                   length_strh_, movi_size_pos_, movi_start_;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> idx_;
};

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

    const bool mjpeg = (args.format == "mjpeg");
    if (mjpeg) {
        for (std::size_t k = 0; k < n; ++k) {
            if (opts[k].pixel_format == fitra::camera::PixFmt::Yuyv) {
                std::fprintf(stderr,
                    "--format mjpeg needs a JPEG payload, but cam%zu (%s) is YUYV; "
                    "use --format mp4 or an mjpeg/nvjpeg pixel_format\n",
                    k, opts[k].device_path.c_str());
                for (auto& c : caps) c->stop();
                return EXIT_FAILURE;
            }
            if (opts[k].downscaling()) {
                std::fprintf(stderr,
                    "WARNING: cam%zu downscales (%dx%d capture -> %dx%d output); "
                    "mjpeg passthrough stores the CAPTURE resolution %dx%d verbatim "
                    "-- feed the harness a calibration at THAT resolution.\n",
                    k, opts[k].capture_w(), opts[k].capture_h(),
                    opts[k].width, opts[k].height,
                    opts[k].capture_w(), opts[k].capture_h());
            }
        }
    }

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

    // --- Set up outputs: one MP4 writer per camera (mp4, decode+re-encode), or
    // one verbatim MJPEG AVI per camera (mjpeg passthrough). `filenames[k]` is
    // the per-camera output file used for meta / the harness command. ----------
    std::vector<cv::VideoWriter> writers;   // mp4 only
    std::vector<AviMjpegWriter>  avi(n);     // mjpeg only (default-constructed no-ops otherwise)
    std::vector<std::string> filenames(n);
    // Stored (per-frame) resolution: the output dims for mp4 (decoded+downscaled),
    // the verbatim capture dims for mjpeg passthrough.
    const int store_w = mjpeg ? opts[0].capture_w() : opts[0].width;
    const int store_h = mjpeg ? opts[0].capture_h() : opts[0].height;
    if (mjpeg) {
        for (std::size_t k = 0; k < n; ++k) {
            filenames[k] = args.name.empty()
                               ? ("raw_cam" + std::to_string(k) + ".avi")
                               : ("raw_" + args.name + "_cam" + std::to_string(k) + ".avi");
            const fs::path p = out_dir / filenames[k];
            if (!avi[k].open(p.string(), opts[k].capture_w(), opts[k].capture_h())) {
                std::fprintf(stderr, "cannot open AVI writer %s\n", p.string().c_str());
                for (auto& c : caps) c->stop();
                return EXIT_FAILURE;
            }
        }
    } else {
        writers.resize(n);
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
    }

    std::printf("recording %zu camera(s) @ %dx%d (%s), out_fps=%d -> %s  (Ctrl-C to stop%s)\n",
                n, store_w, store_h, args.format.c_str(), out_fps, args.out.c_str(),
                args.seconds > 0 ? "" : ", no time limit");

    // --- Synchronized record loop. Write a frame to every camera only once
    // ALL have a fresh (un-written) latest frame, so clips stay index-aligned
    // and the slowest camera paces the rate without emitting duplicates. -----
    std::vector<cv::Mat> latest_bgr(n);                    // mp4 path
    std::vector<std::vector<std::uint8_t>> latest_raw(n);  // mjpeg passthrough path
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
            if (mjpeg) {
                // Verbatim: keep the raw JPEG payload, no decode. Skip a
                // short/corrupt dequeue so the AVI never holds garbage.
                if (!looks_like_jpeg(fr.data)) continue;
                latest_raw[k] = fr.data;  // copy (fr is reused next iteration)
            } else {
                cv::Mat bgr;
                if (!decode_to_bgr(fr, opts[k], bgr)) continue;  // skip corrupt frame
                latest_bgr[k] = std::move(bgr);
            }
            fresh[k] = true;
        }

        bool all_fresh = true;
        for (std::size_t k = 0; k < n; ++k) all_fresh = all_fresh && fresh[k];

        const auto now = clock::now();
        if (all_fresh) {
            for (std::size_t k = 0; k < n; ++k) {
                if (mjpeg) avi[k].add(latest_raw[k].data(), latest_raw[k].size());
                else       writers[k].write(latest_bgr[k]);
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

    for (auto& c : caps) c->stop();
    // Measure duration at loop exit (before the potentially-slow mp4 flush) so
    // the reported fps is the true capture rate. mjpeg passthrough is not
    // encode-bound, so this ≈ the camera rate.
    const double duration = std::chrono::duration<double>(clock::now() - rec_start).count();
    const double fps_written = (duration > 0.0 && written[0] > 0)
                                   ? static_cast<double>(written[0]) / duration
                                   : static_cast<double>(out_fps);
    if (mjpeg) {
        for (std::size_t k = 0; k < n; ++k) {
            if (!avi[k].finalize(fps_written))
                std::fprintf(stderr, "WARNING: failed to finalize %s\n", filenames[k].c_str());
        }
    } else {
        for (auto& w : writers) w.release();
    }

    // meta.json — settings + measured per-camera fps, mirroring excal_record.
    std::FILE* meta = std::fopen((out_dir / "meta.json").c_str(), "w");
    if (meta) {
        std::fprintf(meta,
                     "{\n  \"version\": 1,\n  \"tool\": \"record_3cam\",\n"
                     "  \"format\": \"%s\",\n"
                     "  \"config\": \"%s\",\n  \"width\": %d,\n  \"height\": %d,\n"
                     "  \"out_fps\": %d,\n  \"duration_s\": %.3f,\n"
                     "  \"num_cameras\": %zu,\n  \"cameras\": [\n",
                     args.format.c_str(), args.config.c_str(), store_w, store_h, out_fps,
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
    // fps_written (frames / duration) is the true rate baked into --fps for the
    // Kalman dt; for mp4 it can diverge from the header out_fps when the record
    // loop is encode-bound. For the .avi passthrough clips cv::VideoCapture reads
    // them natively (no %06d pattern) exactly like the mp4 clips.
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
