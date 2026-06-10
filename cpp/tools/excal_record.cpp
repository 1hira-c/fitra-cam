// Sample-data recorder for the controller-marker extrinsic calibration
// offline replay (docs/design/pose-3d-calib-mode-separation.md, オフライン
// replay). Standalone tool — no TensorRT execution, no MultiCameraDriver,
// no Crow.
//
// Captures N USB cameras and pairs every frame with the latest VR controller
// pose from the VMT pose relay, writing a session directory:
//
//   <out>/
//     meta.json        recorder settings + measured fps per camera
//     frames.jsonl     one line per saved frame (see below)
//     cam0/000001.jpg  raw MJPEG payloads, byte-for-byte as dequeued
//     cam1/...
//
// frames.jsonl line:
//   {"cam":0,"seq":42,"file":"cam0/000001.jpg","ts_ms":345.6,
//    "ctrl":{"running_ok":true,"x":..,"y":..,"z":..,"qx":..,"qy":..,"qz":..,
//            "qw":..,"stale":false,"age_ms":8.2,"tracking_result":200,
//            "timestamp_s":55.2}}
//
// Design notes (mirror the design doc):
//   - JPEG passthrough: the V4L2 MJPEG payload is written verbatim — no
//     decode/re-encode, so recording costs almost no CPU and replay feeds
//     cv::imdecode the exact bytes the live CPU decode path would see.
//   - frame<->pose pairing is fixed HERE, at record time, exactly like the
//     live tap (bus snapshot taken when the frame is popped). Replay must
//     not re-pair by timestamp: the bus stale logic is wall-clock dependent
//     and would not reproduce.
//   - ts_ms matches the live tap semantics: captured_at relative to the
//     session t0 (first saved frame), monotonic milliseconds.
//   - ctrl.running_ok folds snapshot.stale && pose.running_ok(), the same
//     expression main.cpp uses to build ControllerObservation.

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

#include "camera/v4l2_capture.hpp"
#include "vmt/controller_pose_receiver.hpp"

namespace fs = std::filesystem;

namespace {

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

struct Args {
    std::vector<std::string> cameras;
    int           width    = 1920;
    int           height   = 1200;
    int           fps      = 30;
    int           n_buffers = 4;
    std::uint16_t port     = 39572;   // VMT pose relay controller channel
    double        stale_ms = 200.0;
    double        seconds  = 0.0;     // 0 = run until Ctrl-C
    std::string   out;
};

void usage(const char* argv0) {
    std::fprintf(stderr,
        "usage: %s --camera PATH [--camera PATH ...] --out DIR [options]\n"
        "\n"
        "Record excal replay sample data: raw MJPEG frames (JPEG passthrough)\n"
        "paired per-frame with the latest VR controller pose (VMT pose relay).\n"
        "\n"
        "  --camera PATH    V4L2 device (repeatable, e.g. /dev/v4l/by-path/...)\n"
        "  --out DIR        session directory to create (must not exist)\n"
        "  --width N        capture width (default 1920)\n"
        "  --height N       capture height (default 1200)\n"
        "  --fps N          requested fps (default 30)\n"
        "  --port N         controller pose relay UDP port (default 39572)\n"
        "  --stale-ms X     pose staleness threshold ms (default 200)\n"
        "  --seconds X      stop after X seconds (default: until Ctrl-C)\n",
        argv0);
}

bool parse_args(int argc, char** argv, Args& a) {
    auto need = [&](int& i) -> const char* {
        if (i + 1 >= argc) {
            std::fprintf(stderr, "missing value for %s\n", argv[i]);
            return nullptr;
        }
        return argv[++i];
    };
    for (int i = 1; i < argc; ++i) {
        std::string_view k{argv[i]};
        const char* v = nullptr;
        if (k == "--camera")        { if (!(v = need(i))) return false; a.cameras.emplace_back(v); }
        else if (k == "--out")      { if (!(v = need(i))) return false; a.out = v; }
        else if (k == "--width")    { if (!(v = need(i))) return false; a.width = std::atoi(v); }
        else if (k == "--height")   { if (!(v = need(i))) return false; a.height = std::atoi(v); }
        else if (k == "--fps")      { if (!(v = need(i))) return false; a.fps = std::atoi(v); }
        else if (k == "--port")     { if (!(v = need(i))) return false; a.port = static_cast<std::uint16_t>(std::atoi(v)); }
        else if (k == "--stale-ms") { if (!(v = need(i))) return false; a.stale_ms = std::atof(v); }
        else if (k == "--seconds")  { if (!(v = need(i))) return false; a.seconds = std::atof(v); }
        else if (k == "--help" || k == "-h") { usage(argv[0]); std::exit(EXIT_SUCCESS); }
        else {
            std::fprintf(stderr, "unknown argument: %s\n", argv[i]);
            return false;
        }
    }
    if (a.cameras.empty()) { std::fprintf(stderr, "--camera is required\n"); return false; }
    if (a.out.empty())     { std::fprintf(stderr, "--out is required\n");    return false; }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    if (!parse_args(argc, argv, args)) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    const fs::path out_dir{args.out};
    if (fs::exists(out_dir)) {
        std::fprintf(stderr, "output dir already exists: %s\n", args.out.c_str());
        return EXIT_FAILURE;
    }
    std::error_code ec;
    fs::create_directories(out_dir, ec);
    if (ec) {
        std::fprintf(stderr, "cannot create %s: %s\n", args.out.c_str(),
                     ec.message().c_str());
        return EXIT_FAILURE;
    }
    for (std::size_t i = 0; i < args.cameras.size(); ++i)
        fs::create_directories(out_dir / ("cam" + std::to_string(i)));

    // Controller pose relay receiver (same channel main's excal mode uses).
    fitra::vmt::ControllerPoseBus bus;
    fitra::vmt::ControllerPoseReceiverOptions ropts;
    ropts.port     = args.port;
    ropts.stale_ms = args.stale_ms;
    fitra::vmt::ControllerPoseReceiver receiver{bus, ropts};
    if (!receiver.start()) {
        std::fprintf(stderr, "cannot bind controller pose receiver on port %u\n",
                     static_cast<unsigned>(args.port));
        return EXIT_FAILURE;
    }

    // Cameras: raw V4L2, MJPEG only (passthrough writes the JPEG payload).
    std::vector<std::unique_ptr<fitra::camera::V4l2Capture>> caps;
    for (const auto& path : args.cameras) {
        fitra::camera::V4l2Options o;
        o.device_path  = path;
        o.width        = args.width;
        o.height       = args.height;
        o.fps          = args.fps;
        o.n_buffers    = args.n_buffers;
        o.pixel_format = fitra::camera::PixFmt::Mjpeg;
        caps.push_back(std::make_unique<fitra::camera::V4l2Capture>(o));
    }
    try {
        for (auto& c : caps) c->start();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "camera start failed: %s\n", e.what());
        return EXIT_FAILURE;
    }

    std::FILE* jsonl = std::fopen((out_dir / "frames.jsonl").c_str(), "w");
    if (!jsonl) {
        std::fprintf(stderr, "cannot open frames.jsonl for writing\n");
        return EXIT_FAILURE;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::printf("recording %zu camera(s) -> %s  (Ctrl-C to stop%s)\n",
                caps.size(), args.out.c_str(),
                args.seconds > 0 ? "" : ", no time limit");

    using clock = std::chrono::steady_clock;
    const auto wall_start = clock::now();
    clock::time_point t0{};          // captured_at of the first saved frame
    bool have_t0 = false;
    std::vector<std::uint64_t> saved(caps.size(), 0);
    auto last_status = wall_start;

    while (!g_stop.load()) {
        bool any = false;
        for (std::size_t i = 0; i < caps.size(); ++i) {
            fitra::camera::Frame fr;
            if (!caps[i]->try_pop_latest(fr)) continue;
            any = true;

            if (!have_t0) { t0 = fr.captured_at; have_t0 = true; }
            const double ts_ms =
                std::chrono::duration<double, std::milli>(fr.captured_at - t0)
                    .count();

            // Pair with the pose NOW — this pairing is the recording contract.
            const auto snap = bus.snapshot(args.stale_ms);
            const bool running_ok = !snap.stale && snap.pose.running_ok();

            char rel[64];
            std::snprintf(rel, sizeof(rel), "cam%zu/%06" PRIu64 ".jpg", i,
                          saved[i] + 1);
            {
                std::ofstream jf{out_dir / rel, std::ios::binary};
                if (!jf) {
                    std::fprintf(stderr, "write failed: %s\n", rel);
                    g_stop.store(true);
                    break;
                }
                jf.write(reinterpret_cast<const char*>(fr.data.data()),
                         static_cast<std::streamsize>(fr.data.size()));
            }
            ++saved[i];

            std::fprintf(
                jsonl,
                "{\"cam\":%zu,\"seq\":%" PRIu64 ",\"file\":\"%s\","
                "\"ts_ms\":%.3f,\"ctrl\":{\"running_ok\":%s,"
                "\"x\":%.9g,\"y\":%.9g,\"z\":%.9g,"
                "\"qx\":%.9g,\"qy\":%.9g,\"qz\":%.9g,\"qw\":%.9g,"
                "\"stale\":%s,\"age_ms\":%.1f,\"tracking_result\":%d,"
                "\"timestamp_s\":%.6f}}\n",
                i, fr.seq, rel, ts_ms, running_ok ? "true" : "false",
                static_cast<double>(snap.pose.x), static_cast<double>(snap.pose.y),
                static_cast<double>(snap.pose.z), static_cast<double>(snap.pose.qx),
                static_cast<double>(snap.pose.qy), static_cast<double>(snap.pose.qz),
                static_cast<double>(snap.pose.qw), snap.stale ? "true" : "false",
                snap.have_any ? snap.age_ms : -1.0, snap.pose.tracking_result,
                static_cast<double>(snap.pose.timestamp_s));
        }
        if (!any)
            std::this_thread::sleep_for(std::chrono::milliseconds(2));

        const auto now = clock::now();
        const double elapsed =
            std::chrono::duration<double>(now - wall_start).count();
        if (args.seconds > 0 && elapsed >= args.seconds) break;
        if (now - last_status >= std::chrono::seconds(1)) {
            last_status = now;
            const auto snap = bus.snapshot(args.stale_ms);
            std::string cams;
            for (std::size_t i = 0; i < caps.size(); ++i) {
                char b[64];
                std::snprintf(b, sizeof(b), "cam%zu %5.1ffps(%" PRIu64 ") ", i,
                              caps[i]->recv_fps(), saved[i]);
                cams += b;
            }
            std::printf("\r%s| ctrl %s age=%.0fms  t=%.0fs   ", cams.c_str(),
                        !snap.have_any           ? "none"
                        : snap.stale             ? "STALE"
                        : snap.pose.running_ok() ? "ok"
                                                 : "degraded",
                        snap.have_any ? snap.age_ms : -1.0, elapsed);
            std::fflush(stdout);
            std::fflush(jsonl);
        }
    }
    std::printf("\n");

    for (auto& c : caps) c->stop();
    receiver.stop();
    std::fclose(jsonl);

    const double duration =
        std::chrono::duration<double>(clock::now() - wall_start).count();
    const auto rstats = receiver.stats();

    std::FILE* meta = std::fopen((out_dir / "meta.json").c_str(), "w");
    if (meta) {
        std::fprintf(meta,
                     "{\n  \"version\": 1,\n  \"pixel_format\": \"mjpeg\",\n"
                     "  \"width\": %d,\n  \"height\": %d,\n"
                     "  \"fps_requested\": %d,\n  \"duration_s\": %.3f,\n"
                     "  \"controller_port\": %u,\n  \"stale_ms\": %.1f,\n"
                     "  \"pose_packets\": %" PRIu64 ",\n  \"cameras\": [\n",
                     args.width, args.height, args.fps, duration,
                     static_cast<unsigned>(args.port), args.stale_ms,
                     rstats.packets_total);
        for (std::size_t i = 0; i < caps.size(); ++i) {
            std::fprintf(meta,
                         "    {\"device\": \"%s\", \"frames\": %" PRIu64
                         ", \"fps_measured\": %.2f}%s\n",
                         args.cameras[i].c_str(), saved[i],
                         duration > 0 ? saved[i] / duration : 0.0,
                         i + 1 < caps.size() ? "," : "");
        }
        std::fprintf(meta, "  ]\n}\n");
        std::fclose(meta);
    }

    std::uint64_t total = 0;
    for (std::size_t i = 0; i < caps.size(); ++i) {
        std::printf("cam%zu: %" PRIu64 " frames (%.2f fps measured)\n", i,
                    saved[i], duration > 0 ? saved[i] / duration : 0.0);
        total += saved[i];
    }
    std::printf("pose packets: %" PRIu64 " (%" PRIu64 " rejected)\n",
                rstats.packets_total, rstats.packets_rejected);
    std::printf("wrote %" PRIu64 " frames + frames.jsonl + meta.json -> %s\n",
                total, args.out.c_str());
    if (rstats.packets_total == 0)
        std::printf("WARNING: no controller pose packets received — replay "
                    "will have no usable samples\n");
    return EXIT_SUCCESS;
}
