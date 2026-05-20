// fitra-cam main — Phase 3 driver.
//
// Runs N V4L2 USB cameras through the shared YOLOX + RTMPose TRT
// pipeline and exposes the result via Crow (HTTP + WebSocket).
//
// Usage (one line, no shell continuation):
//   fitra-cam --cam0 PATH [--cam1 PATH] [--cam2 PATH] --det-engine PATH --pose-engine PATH
//             [--port 8000] [--host 0.0.0.0] [--static DIR] [--no-web]
//             [--width 640] [--height 480] [--fps 30]
//             [--det-frequency 10] [--multi-person] [--enable-3d --calib PATH] [--probe]
//
// `--probe` keeps the Phase 0 diagnostic (CUDA device + TRT runtime sanity check).

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <cuda_runtime_api.h>
#include <NvInfer.h>
#include <NvInferVersion.h>

#include "camera/v4l2_capture.hpp"
#include "infer/rtmpose.hpp"
#include "infer/trt_engine.hpp"
#include "infer/yolox.hpp"
#include "lift/calib_io.hpp"
#include "lift/subject_profile.hpp"
#include "lift/triangulator.hpp"
#include "pipeline/calibration_session.hpp"
#include "pipeline/multi_pipeline.hpp"
#include "pipeline/snapshot.hpp"
#include "util/cuda_check.hpp"
#include "util/logging.hpp"
#include "web/crow_server.hpp"

namespace {

class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity > Severity::kWARNING) return;
        using S = Severity;
        switch (severity) {
            case S::kINTERNAL_ERROR: FITRA_LOG_ERROR("[trt] INTERNAL: {}", msg); return;
            case S::kERROR:          FITRA_LOG_ERROR("[trt] {}",          msg); return;
            case S::kWARNING:        FITRA_LOG_WARN ("[trt] {}",          msg); return;
            case S::kINFO:           FITRA_LOG_INFO ("[trt] {}",          msg); return;
            case S::kVERBOSE:        FITRA_LOG_TRACE("[trt] {}",          msg); return;
        }
    }
};

void print_help() {
    std::puts(
        "fitra-cam (C++) — Phase 3 driver\n"
        "\n"
        "Required:\n"
        "  --cam0 PATH               first V4L2 device (e.g. /dev/v4l/by-path/...index0)\n"
        "  --det-engine PATH         YOLOX .engine\n"
        "  --pose-engine PATH        RTMPose .engine\n"
        "\n"
        "Additional cameras:\n"
        "  --cam1 PATH               second camera\n"
        "  --cam2 PATH               third camera\n"
        "\n"
        "Optional:\n"
        "  --port N                  HTTP/WS port (default 8000)\n"
        "  --host ADDR               bind address (default 0.0.0.0)\n"
        "  --static DIR              web frontend dir (default <repo>/web/dual_rtmpose)\n"
        "  --no-web                  do not start Crow (driver only, for bench)\n"
        "  --width N / --height N    capture size per camera (default 640x480)\n"
        "  --fps N                   requested capture fps (default 30)\n"
        "  --det-frequency N         run YOLOX every N frames (default 10)\n"
        "  --multi-person            process all bboxes per camera (default: largest only)\n"
        "  --bench-fake-bbox         inject synthetic bbox when detections are empty (bench only)\n"
        "  --det-score F             detection score threshold (default 0.5)\n"
        "  --log-every-s F           stats interval in seconds (default 2.0)\n"
        "  --enable-3d               enable live 2D -> 3D lifting and /ws3d\n"
        "  --calib PATH              calibration YAML for --enable-3d (ids must be cam0..camN)\n"
        "  --kp-conf-thresh F        3D triangulation keypoint threshold (default 0.3)\n"
        "  --max-reproj-px F         3D reprojection outlier threshold (default 6.0)\n"
        "  --sync-window-ms F        max camera timestamp gap for 3D (default 15.0)\n"
        "  --bone-calib-frames N     frames used to lock IK bone lengths (default 150)\n"
        "  --subject-height-m F      lock IK bone lengths from Japanese anthropometry and height\n"
        "  --subject-id ID           load calibrations/subjects/<ID>/latest_profile.yaml for IK\n"
        "  --subjects-dir DIR        subject profile root (default calibrations/subjects)\n"
        "  --subject-profile PATH    direct subject profile YAML path for IK\n"
        "  --no-3d-kalman            disable 3D Kalman smoothing\n"
        "  --no-3d-ik                disable 3D IK projection\n"
        "\n"
        "Phase 8 — Subject calibration wizard (requires --enable-3d):\n"
        "  --calibrate                 auto-start calibration session at boot\n"
        "  --calib-subject-id ID       required with --calibrate (subject identifier)\n"
        "  --calib-subject-height-m F  required with --calibrate (1.0 .. 2.3 m)\n"
        "  --calib-frames-per-cam N    frames per camera per pose (default 75 ≈ 5s @ 15fps)\n"
        "  --calib-hold-sec F          stability seconds before recording (default 1.5)\n"
        "  --calib-auto-approve        auto approve when quality=pass (warn/fail stay manual)\n"
        "  --calib-auto-exit           exit main after a successful approval\n"
        "  --calib-static-dir DIR      override web/subject_calibration static path\n"
        "  --calib-dump-tool PATH      override dump_keypoints_3d path used by analysis\n"
        "  --subjects-dir DIR          subject profile root (also used by --calibrate)\n"
        "\n"
        "  --probe                   Phase 0 sanity check and exit\n"
        "  --help                    show this help\n");
}

int probe() {
    int device_count = 0;
    CUDA_CHECK(cudaGetDeviceCount(&device_count));
    FITRA_LOG_INFO("CUDA device count = {}", device_count);
    for (int i = 0; i < device_count; ++i) {
        cudaDeviceProp prop{};
        CUDA_CHECK(cudaGetDeviceProperties(&prop, i));
        FITRA_LOG_INFO("  [{}] {} (sm_{}{}, {} MB)",
                       i, prop.name, prop.major, prop.minor,
                       static_cast<unsigned long long>(prop.totalGlobalMem) / (1024ULL * 1024ULL));
    }
    FITRA_LOG_INFO("TensorRT headers: {}.{}.{}",
                   NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH);
    TrtLogger trt_logger;
    std::unique_ptr<nvinfer1::IRuntime> rt{nvinfer1::createInferRuntime(trt_logger)};
    TRT_CHECK(rt != nullptr);
    FITRA_LOG_INFO("nvinfer1::IRuntime ok (lib build: {})", getInferLibVersion());
    return EXIT_SUCCESS;
}

std::filesystem::path guess_static_dir() {
    auto exe = std::filesystem::canonical("/proc/self/exe");
    // build/main lives at <repo>/cpp/build/main; we want <repo>/web/dual_rtmpose
    auto repo = exe.parent_path().parent_path().parent_path();
    return repo / "web" / "dual_rtmpose";
}

std::filesystem::path guess_subject_calib_static_dir() {
    auto exe = std::filesystem::canonical("/proc/self/exe");
    auto repo = exe.parent_path().parent_path().parent_path();
    return repo / "web" / "subject_calibration";
}

std::filesystem::path guess_dump_tool_path() {
    auto exe = std::filesystem::canonical("/proc/self/exe");
    // main is at <repo>/cpp/build/main; dump tool is <repo>/cpp/build/tools/dump_keypoints_3d
    auto build_dir = exe.parent_path();
    return build_dir / "tools" / "dump_keypoints_3d";
}

std::vector<std::string> expected_camera_ids(std::size_t count) {
    std::vector<std::string> ids;
    ids.reserve(count);
    for (std::size_t i = 0; i < count; ++i) ids.push_back("cam" + std::to_string(i));
    return ids;
}

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> cam_paths;
    cam_paths.resize(3);  // slots for cam0..cam2
    std::string det_engine_path;
    std::string pose_engine_path;
    int   port = 8000;
    std::string host = "0.0.0.0";
    std::string static_dir;
    bool  no_web = false;
    int   width = 640, height = 480, fps = 30;
    int   det_frequency = 10;
    bool  multi_person = false;
    bool  bench_fake_bbox = false;
    float det_score = 0.5f;
    double log_every_s = 2.0;
    bool  enable_3d = false;
    std::string calib_path;
    float kp_conf_thresh = 0.3f;
    float max_reproj_px = 6.0f;
    double sync_window_ms = 15.0;
    int bone_calib_frames = 150;
    double subject_height_m = 0.0;
    std::string subject_id;
    std::string subjects_dir = "calibrations/subjects";
    std::string subject_profile_path;
    bool kalman_3d = true;
    bool ik_3d = true;
    bool  want_probe = false;

    // Phase 8 — calibration wizard.
    bool calibrate_on_boot = false;
    std::string calib_subject_id;
    double calib_subject_height_m = 0.0;
    int calib_frames_per_cam = 75;
    double calib_hold_sec = 1.5;
    bool calib_auto_approve = false;
    bool calib_auto_exit = false;
    std::string calib_static_dir;
    std::string calib_dump_tool;

    for (int i = 1; i < argc; ++i) {
        std::string_view a{argv[i]};
        auto need = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing argument for %s\n", flag);
                std::exit(EXIT_FAILURE);
            }
            return argv[++i];
        };
        if      (a == "--help" || a == "-h") { print_help(); return EXIT_SUCCESS; }
        else if (a == "--probe")             { want_probe = true; }
        else if (a == "--cam0")              { cam_paths[0] = need("--cam0"); }
        else if (a == "--cam1")              { cam_paths[1] = need("--cam1"); }
        else if (a == "--cam2")              { cam_paths[2] = need("--cam2"); }
        else if (a == "--det-engine")        { det_engine_path  = need("--det-engine"); }
        else if (a == "--pose-engine")       { pose_engine_path = need("--pose-engine"); }
        else if (a == "--port")              { port = std::atoi(need("--port")); }
        else if (a == "--host")              { host = need("--host"); }
        else if (a == "--static")            { static_dir = need("--static"); }
        else if (a == "--no-web")            { no_web = true; }
        else if (a == "--width")             { width  = std::atoi(need("--width")); }
        else if (a == "--height")            { height = std::atoi(need("--height")); }
        else if (a == "--fps")               { fps    = std::atoi(need("--fps")); }
        else if (a == "--det-frequency")     { det_frequency = std::atoi(need("--det-frequency")); }
        else if (a == "--multi-person")      { multi_person  = true; }
        else if (a == "--bench-fake-bbox")   { bench_fake_bbox = true; }
        else if (a == "--det-score")         { det_score = std::stof(need("--det-score")); }
        else if (a == "--log-every-s")       { log_every_s = std::stod(need("--log-every-s")); }
        else if (a == "--enable-3d")         { enable_3d = true; }
        else if (a == "--calib")             { calib_path = need("--calib"); }
        else if (a == "--kp-conf-thresh")    { kp_conf_thresh = std::stof(need("--kp-conf-thresh")); }
        else if (a == "--max-reproj-px")     { max_reproj_px = std::stof(need("--max-reproj-px")); }
        else if (a == "--sync-window-ms")    { sync_window_ms = std::stod(need("--sync-window-ms")); }
        else if (a == "--bone-calib-frames") { bone_calib_frames = std::atoi(need("--bone-calib-frames")); }
        else if (a == "--subject-height-m")  { subject_height_m = std::stod(need("--subject-height-m")); }
        else if (a == "--subject-id")         { subject_id = need("--subject-id"); }
        else if (a == "--subjects-dir")       { subjects_dir = need("--subjects-dir"); }
        else if (a == "--subject-profile")    { subject_profile_path = need("--subject-profile"); }
        else if (a == "--no-3d-kalman")      { kalman_3d = false; }
        else if (a == "--no-3d-ik")          { ik_3d = false; }
        else if (a == "--calibrate")             { calibrate_on_boot = true; }
        else if (a == "--calib-subject-id")      { calib_subject_id = need("--calib-subject-id"); }
        else if (a == "--calib-subject-height-m"){ calib_subject_height_m = std::stod(need("--calib-subject-height-m")); }
        else if (a == "--calib-frames-per-cam")  { calib_frames_per_cam = std::atoi(need("--calib-frames-per-cam")); }
        else if (a == "--calib-hold-sec")        { calib_hold_sec = std::stod(need("--calib-hold-sec")); }
        else if (a == "--calib-auto-approve")    { calib_auto_approve = true; }
        else if (a == "--calib-auto-exit")       { calib_auto_exit = true; }
        else if (a == "--calib-static-dir")      { calib_static_dir = need("--calib-static-dir"); }
        else if (a == "--calib-dump-tool")       { calib_dump_tool = need("--calib-dump-tool"); }
        else {
            std::fprintf(stderr, "unknown arg: %s\n", argv[i]);
            print_help();
            return EXIT_FAILURE;
        }
    }

    try {
        if (want_probe) return probe();
        if (cam_paths[0].empty() || det_engine_path.empty() || pose_engine_path.empty()) {
            print_help();
            return EXIT_FAILURE;
        }
        if (enable_3d && calib_path.empty()) {
            std::fprintf(stderr, "--enable-3d requires --calib PATH\n");
            return EXIT_FAILURE;
        }
        if (subject_height_m < 0.0 || subject_height_m > 2.5) {
            std::fprintf(stderr, "--subject-height-m must be 0 or a plausible meter value <= 2.5\n");
            return EXIT_FAILURE;
        }
        if (!enable_3d && (!subject_id.empty() || !subject_profile_path.empty())) {
            std::fprintf(stderr, "--subject-id/--subject-profile require --enable-3d\n");
            return EXIT_FAILURE;
        }
        if (calibrate_on_boot && !enable_3d) {
            std::fprintf(stderr, "--calibrate requires --enable-3d\n");
            return EXIT_FAILURE;
        }
        if (calibrate_on_boot
            && (calib_subject_id.empty() || calib_subject_height_m <= 0.0)) {
            std::fprintf(stderr,
                "--calibrate requires --calib-subject-id and --calib-subject-height-m\n");
            return EXIT_FAILURE;
        }
        // For the headless --calibrate path, prime the live IkSolver with the
        // calibration height up-front. Without this, the IK is unlocked at
        // boot and the 3D angle recognizer would have to wait for ~150 frames
        // of observational locking before it can judge pose holds. If the
        // user already passed --subject-height-m we honor that instead.
        if (calibrate_on_boot && subject_height_m <= 0.0
            && subject_profile_path.empty() && subject_id.empty()) {
            subject_height_m = calib_subject_height_m;
        }
        std::size_t requested_cam_count = 0;
        for (const auto& path : cam_paths) {
            if (!path.empty()) ++requested_cam_count;
        }
        const bool calib_frame_recording_possible =
            enable_3d && requested_cam_count == 2;
        // Shared "calibration is recording" flag. When true:
        //   - FrameSource skips YOLOX + RTMPose pre-bake and retains BGR
        //   - CalibrationSession collects raw frames into the per-pose buffer
        // Wired from CalibrationSession::set_on_recording_active below.
        auto calib_recording_flag =
            std::make_shared<std::atomic<bool>>(false);
        std::signal(SIGINT, on_signal);

        TrtLogger tlog;
        std::unique_ptr<nvinfer1::IRuntime> rt{nvinfer1::createInferRuntime(tlog)};
        TRT_CHECK(rt != nullptr);

        FITRA_LOG_INFO("loading YOLOX engine (shared): {}", det_engine_path);
        auto yolox_shared = fitra::infer::TrtEngine::load_shared(*rt, det_engine_path);
        FITRA_LOG_INFO("loading RTMPose engine: {}", pose_engine_path);
        auto rtmpose_eng  = fitra::infer::TrtEngine::from_file(*rt, pose_engine_path, tlog);
        // RTMPose stays as a single shared instance — batching across cameras
        // requires one execution context fed serially from the main thread.
        fitra::infer::RtmPose rtmpose{*rtmpose_eng};

        // One V4l2Capture + one Yolox (per-camera IExecutionContext) per cam.
        // Engines wrap into FrameSource which runs its own decode + YOLOX
        // thread, so all N cameras run capture/decode/YOLOX in parallel.
        std::vector<std::unique_ptr<fitra::infer::TrtEngine>> yolox_engines;
        std::vector<std::unique_ptr<fitra::camera::FrameSource>> sources;
        for (auto& path : cam_paths) {
            if (path.empty()) continue;
            fitra::camera::V4l2Options o;
            o.device_path = path;
            o.width  = width;
            o.height = height;
            o.fps    = fps;
            auto cap = std::make_unique<fitra::camera::V4l2Capture>(o);

            auto yolox_eng = fitra::infer::TrtEngine::from_shared(yolox_shared);
            fitra::infer::Yolox::Options yolo_opts;
            yolo_opts.score_thr = det_score;
            auto yolox = std::make_unique<fitra::infer::Yolox>(*yolox_eng, yolo_opts);
            yolox_engines.push_back(std::move(yolox_eng));

            fitra::camera::FrameSource::Options src_opts;
            src_opts.det_frequency = det_frequency;
            src_opts.single_person = !multi_person;
            src_opts.fake_bbox_if_empty = bench_fake_bbox;
            // Phase 8 records raw per-camera clips through MultiCameraDriver's
            // frame tap. The same flag pauses YOLOX + RTMPose pre-bake while
            // recording so disk I/O has the CPU/GPU headroom and we don't burn
            // cycles on a pose feed nobody is watching.
            if (calib_frame_recording_possible) {
                src_opts.calib_recording_flag = calib_recording_flag;
            }
            // Have the per-camera worker pre-bake the RTMPose input so the
            // central inference thread only does memcpy + GPU + decode.
            const auto& rtmpose_opts = rtmpose.options();
            sources.push_back(std::make_unique<fitra::camera::FrameSource>(
                std::move(cap), std::move(yolox), src_opts, &rtmpose_opts));
        }
        std::size_t n_cams = sources.size();
        if (enable_3d && n_cams < 2) {
            std::fprintf(stderr, "--enable-3d requires at least two cameras\n");
            return EXIT_FAILURE;
        }

        std::unique_ptr<fitra::lift::Triangulator> triangulator;
        std::unique_ptr<fitra::pipeline::Skeleton3DBus> bus3d;
        fitra::lift::SubjectProfile subject_profile;
        bool has_subject_profile = false;
        if (enable_3d) {
            FITRA_LOG_INFO("loading calibration: {}", calib_path);
            auto calib = fitra::lift::load_calibration(calib_path);
            fitra::lift::Triangulator::Options tri_opts;
            tri_opts.kp_conf_thresh = kp_conf_thresh;
            tri_opts.max_reproj_px = max_reproj_px;
            triangulator = std::make_unique<fitra::lift::Triangulator>(calib, tri_opts);
            triangulator->require_camera_ids(expected_camera_ids(n_cams));
            bus3d = std::make_unique<fitra::pipeline::Skeleton3DBus>();
            FITRA_LOG_INFO("3D lifting enabled ({} calibrated cameras, sync_window={}ms)",
                           triangulator->camera_count(), sync_window_ms);
            if (subject_height_m > 0.0) {
                FITRA_LOG_INFO("3D IK subject height prior enabled: {} m", subject_height_m);
            }
            if (subject_profile_path.empty() && !subject_id.empty()) {
                subject_profile_path = fitra::lift::default_subject_profile_path(subjects_dir, subject_id);
            }
            if (!subject_profile_path.empty()) {
                FITRA_LOG_INFO("loading subject profile: {}", subject_profile_path);
                subject_profile = fitra::lift::load_subject_profile(subject_profile_path);
                if (subject_profile.subject_height_m > 0.0) {
                    subject_height_m = subject_profile.subject_height_m;
                } else if (subject_height_m > 0.0) {
                    subject_profile.subject_height_m = subject_height_m;
                }
                has_subject_profile = true;
                FITRA_LOG_INFO("3D IK subject profile enabled: id={} quality={}",
                               subject_profile.subject_id, subject_profile.quality_status);
            }
        }

        fitra::pipeline::SnapshotBus bus{n_cams};
        std::unique_ptr<fitra::pipeline::MultiCameraDriver> driver;
        if (enable_3d) {
            fitra::pipeline::MultiCameraDriver::ThreeDConfig cfg;
            cfg.triangulator = triangulator.get();
            cfg.bus = bus3d.get();
            cfg.sync_window_ms = sync_window_ms;
            cfg.kalman_enabled = kalman_3d;
            cfg.ik_enabled = ik_3d;
            cfg.bone_calib_frames = bone_calib_frames;
            cfg.subject_height_m = subject_height_m;
            cfg.has_subject_profile = has_subject_profile;
            cfg.subject_profile = subject_profile;
            driver = std::make_unique<fitra::pipeline::MultiCameraDriver>(
                std::move(sources), rtmpose, bus, cfg);
        } else {
            driver = std::make_unique<fitra::pipeline::MultiCameraDriver>(
                std::move(sources), rtmpose, bus);
        }
        driver->start();

        // Phase 8 calibration session: only set up when 3D is enabled AND
        // exactly 2 cameras are attached. The session orchestrator and
        // dump_keypoints_3d both assume cam0/cam1 (Phase 7 spec); refuse to
        // attach for 1- or 3-camera runs rather than silently dropping cam2.
        std::unique_ptr<fitra::pipeline::CalibrationSession> calib_session;
        fitra::pipeline::CalibPreflight calib_defaults;
        const bool calib_available = enable_3d && n_cams == 2;
        if (calibrate_on_boot && !calib_available) {
            std::fprintf(stderr,
                "--calibrate currently requires exactly 2 cameras (got %zu)\n",
                n_cams);
            return EXIT_FAILURE;
        }
        if (enable_3d && n_cams != 2) {
            FITRA_LOG_WARN("calibration wizard disabled: needs exactly 2 cameras (got {})",
                           n_cams);
        }
        if (calib_available) {
            calib_session = std::make_unique<fitra::pipeline::CalibrationSession>();
            calib_session->set_fps_hint(static_cast<double>(fps));
            calib_session->set_auto_approve(calib_auto_approve);
            calib_session->set_auto_exit(calib_auto_exit);
            calib_session->set_log([](const std::string& l) {
                std::fprintf(stderr, "[calib] %s\n", l.c_str());
            });
            calib_session->set_on_approved([&](const fitra::lift::SubjectProfile& p) {
                FITRA_LOG_INFO("hot-reloading IK from approved profile (id={})",
                               p.subject_id);
                driver->ik().reload_from_profile(p);
            });
            // Prime the live IK with the subject's height the moment preflight
            // succeeds, so the 3D angle recognizer has a sensible bone-length
            // lock from the very first frame of capture.
            calib_session->set_on_preflight(
                [&](const fitra::pipeline::CalibPreflight& p) {
                    FITRA_LOG_INFO("priming IK with calibration height: {} m",
                                   p.subject_height_m);
                    driver->ik().apply_subject_height(p.subject_height_m);
                });
            calib_session->set_on_recording_active(
                [calib_recording_flag](bool active) {
                    calib_recording_flag->store(active, std::memory_order_relaxed);
                });
            calib_session->set_on_exit_requested([&]() {
                g_stop.store(true);
            });

            calib_defaults.subject_id = "";
            calib_defaults.subjects_dir = subjects_dir;
            calib_defaults.calib_yaml   = calib_path;
            calib_defaults.det_engine   = det_engine_path;
            calib_defaults.pose_engine  = pose_engine_path;
            calib_defaults.recording_frames_per_cam = calib_frames_per_cam;
            calib_defaults.required_hold_sec        = calib_hold_sec;
            calib_defaults.dump_tool_path = calib_dump_tool.empty()
                                            ? guess_dump_tool_path().string()
                                            : calib_dump_tool;

            driver->set_frame_tap(
                [s = calib_session.get()](std::size_t cam, const cv::Mat& bgr, double ts) {
                    s->on_frame(cam, bgr, ts);
                });
            driver->set_skeleton3d_tap(
                [s = calib_session.get()](const fitra::infer::Skeleton3D& skel, double drift) {
                    s->on_skeleton3d(skel, drift);
                });
        }

        std::unique_ptr<fitra::web::CrowServer> server;
        if (!no_web) {
            fitra::web::ServerOptions sopts;
            sopts.host = host;
            sopts.port = port;
            sopts.static_dir = static_dir.empty()
                                ? guess_static_dir().string()
                                : static_dir;
            sopts.calib_static_dir = calib_static_dir.empty()
                                ? guess_subject_calib_static_dir().string()
                                : calib_static_dir;
            server = std::make_unique<fitra::web::CrowServer>(
                bus, enable_3d ? bus3d.get() : nullptr, sopts);
            if (calib_session) {
                server->set_calibration_session(calib_session.get(), calib_defaults);
            }
            server->start();
        }

        // Boot-time auto preflight + start if --calibrate is set.
        if (calib_session && calibrate_on_boot) {
            fitra::pipeline::CalibPreflight in = calib_defaults;
            in.subject_id = calib_subject_id;
            in.subject_height_m = calib_subject_height_m;
            std::string err;
            if (!calib_session->preflight(in, err)) {
                std::fprintf(stderr, "calibrate preflight failed: %s\n", err.c_str());
                return EXIT_FAILURE;
            }
            if (!calib_session->start(err)) {
                std::fprintf(stderr, "calibrate start failed: %s\n", err.c_str());
                return EXIT_FAILURE;
            }
            FITRA_LOG_INFO("calibration auto-start: subject={} height={} m",
                           calib_subject_id, calib_subject_height_m);
        }

        auto last_log = std::chrono::steady_clock::now();
        while (!g_stop.load()) {
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_log).count();
            if (dt >= log_every_s) {
                for (std::size_t i = 0; i < n_cams; ++i) {
                    const auto& s = driver->stats_for(i);
                    char buf[256];
                    std::snprintf(buf, sizeof(buf),
                                  "cam%zu: recv=%5.2f avg_pose=%5.2f recent_pose=%5.2f "
                                  "stage_ms=%6.1f processed=%llu pending=%llu",
                                  i, driver->recv_fps_for(i),
                                  s.avg_pose_fps, s.recent_pose_fps,
                                  s.last_stage_ms,
                                  static_cast<unsigned long long>(s.processed_count),
                                  static_cast<unsigned long long>(driver->pending_for(i)));
                    FITRA_LOG_INFO("{}", buf);
                }
                last_log = now;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (server) server->stop();
        driver->stop();
        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        FITRA_LOG_ERROR("fatal: {}", e.what());
        return EXIT_FAILURE;
    }
}
