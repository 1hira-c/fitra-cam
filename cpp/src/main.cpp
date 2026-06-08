// fitra-cam main — N-camera driver.
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
// `--probe` is a CUDA device + TRT runtime sanity check that exits.

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
#include "config/main_config.hpp"
#include "infer/rtmpose.hpp"
#include "infer/trt_engine.hpp"
#include "infer/yolox.hpp"
#include "lift/calib_io.hpp"
#include "lift/keypoint_format.hpp"
#include "lift/subject_profile.hpp"
#include "lift/triangulator.hpp"
#include "pipeline/calibration_session.hpp"
#include "pipeline/extrinsic_calib_session.hpp"
#include "pipeline/multi_pipeline.hpp"
#include "pipeline/snapshot.hpp"
#include "slimevr/native_publisher.hpp"
#include "slimevr/slime_tracker_bus.hpp"
#include "slimevr/tracker_extractor.hpp"
#include "util/cuda_check.hpp"
#include "util/logging.hpp"
#include "vmt/vmt_publisher.hpp"
#include "vmt/continuous_aligner.hpp"
#include "vmt/hmd_pose_receiver.hpp"
#include "vmt/controller_pose_receiver.hpp"
#include "vmt/tracked_pose_receiver.hpp"
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
        "fitra-cam (C++)\n"
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
        "  --pixel-format FMT        mjpeg (default,CPU) | yuyv | nvjpeg (Jetson HW decode)\n"
        "  --n-buffers N             v4l2 mmap ring depth (default 4, min 2)\n"
        "  --det-frequency N         run YOLOX every N frames (default 10)\n"
        "  --keypoint-format FMT     pose topology: coco17 (17 kpts, default) or halpe26 (26 kpts).\n"
        "                            Must match the K of the supplied --pose-engine.\n"
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
        "  --vr-extract-event-driven react to each 3D frame (lower VR latency)\n"
        "  --vr-no-one-euro          use fixed-alpha EMA instead of One Euro smoothing\n"
        "  --vr-pos-mincutoff F      One Euro position at-rest cutoff Hz (default 1.0; lower=smoother)\n"
        "  --vr-pos-beta F           One Euro position motion responsiveness (default 4.0)\n"
        "  --vr-pos-dcutoff F        One Euro position speed-estimate cutoff Hz (default 1.0)\n"
        "  --vr-quat-mincutoff F     One Euro rotation at-rest cutoff Hz (default 1.5)\n"
        "  --vr-quat-beta F          One Euro rotation motion responsiveness (default 1.5)\n"
        "  --vr-quat-dcutoff F       One Euro rotation speed-estimate cutoff Hz (default 1.0)\n"
        "\n"
        "SlimeVR native Firmware UDP output (requires --enable-3d + --keypoint-format=halpe26):\n"
        "  --slimevr-out             enable the native Firmware UDP publisher (10 trackers)\n"
        "  --slimevr-host ADDR       SlimeVR Server host (default 127.0.0.1; typically the Windows IP)\n"
        "  --slimevr-port N          UDP port (default 6969 — SlimeVR firmware port)\n"
        "  --slimevr-rate-hz F       RotationData send rate (default 60.0)\n"
        "  --slimevr-quat-smooth F   per-tracker slerp alpha 0..1 (default 0.5)\n"
        "  --slimevr-preview-no-reset  pre-cancel SlimeVR default mounting so GUI preview\n"
        "                              works before reset\n"
        "\n"
        "Virtual Motion Tracker (VMT) → SteamVR direct (requires --enable-3d + --keypoint-format=halpe26):\n"
        "  --vmt-out                 enable the VMT OSC publisher (10 trackers, /VMT/Room/Driver)\n"
        "  --vmt-host ADDR           VMT Manager host (default 127.0.0.1; typically the Windows IP)\n"
        "  --vmt-port N              UDP port (default 39570 — VMT receive port)\n"
        "  --vmt-rate-hz F           send rate (default 60.0)\n"
        "  --vmt-index-base N        first VMT device index (default 10 -> VMT_10..VMT_19)\n"
        "  --vmt-pos-smooth F        position EMA alpha 0..1 (default 0.5; wired in M3)\n"
        "  --vmt-degeneracy-mode S   what to do for invalid trackers: hold|disable|skip (default hold)\n"
        "  --vmt-disable-below-floor disable trackers whose pos.z < 0 (room-matrix sanity, default off)\n"
        "\n"
        "HMD pose receiver from vmt_hmd_pose_sender.exe (Windows side):\n"
        "  --hmd-listen-enabled      bind a UDP socket and accept /fitra/hmd_pose datagrams\n"
        "  --hmd-listen-port N       UDP port to listen on (default 39571)\n"
        "  --hmd-listen-bind ADDR    bind address (default 0.0.0.0)\n"
        "  --hmd-stale-ms F          milliseconds without a packet → snapshot.stale=true (default 200)\n"
        "\n"
        "Continuous HMD-driven alignment (needs --vmt-out + --hmd-listen-enabled + --enable-3d):\n"
        "  --vmt-continuous-align       always-on background alignment refinement (default ON)\n"
        "  --no-vmt-continuous-align    disable the background refiner\n"
        "  --vmt-continuous-sample-hz F poll rate for HMD/head samples (default 15, [5,120])\n"
        "  --vmt-continuous-resolve-s F re-solve cadence in seconds (default 2, [0.2,30])\n"
        "  --vmt-continuous-blend F     EMA weight applied per solve (default 0.2, (0,1])\n"
        "\n"
        "Subject calibration wizard (requires --enable-3d):\n"
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
        "Controller-marker extrinsic calibration (mutually exclusive with --calibrate;\n"
        "see docs/design/pose-3d-controller-marker-extrinsic.md):\n"
        "  --extrinsic-calib           collect controller-marker samples; solve+write at exit\n"
        "  --excal-intrinsics PATH     intrinsics-only calibration YAML (else reuses --calib)\n"
        "  --excal-out PATH            output extrinsics YAML (default calibrations/extrinsics.yaml)\n"
        "  --excal-faces \"0,1,2\"       AprilTag 36h11 face IDs on the marker\n"
        "  --excal-tag-size-m F        physical tag side length, metres (default 0.10)\n"
        "  --excal-lin-vel-max F       motion gate, m/s (default 0.03)\n"
        "  --excal-ang-vel-max F       motion gate, deg/s (default 8)\n"
        "  --excal-burst-min N         frames averaged per accepted pose (default 5)\n"
        "  --excal-min-samples N       min samples per (cam,face) for the solve (default 8)\n"
        "  --excal-controller-role S   controller role to consume: left|right (default right)\n"
        "  --excal-controller-port N   deprecated legacy controller UDP port (default 39572)\n"
        "  --excal-controller-bind ADDR  deprecated legacy bind address (default 0.0.0.0)\n"
        "  --excal-controller-stale-ms F  controller pose staleness threshold (default 200)\n"
        "\n"
        "  --config PATH             runtime YAML config (see docs/backlog-main-yaml-config.md).\n"
        "                            Precedence (low -> high): code defaults < --config < CLI flags.\n"
        "                            CLI flags on the same invocation always override the YAML value.\n"
        "                            If --probe is also passed, --probe wins and the config is not read.\n"
        "\n"
        "  --probe                   CUDA + TRT runtime sanity check and exit\n"
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

std::filesystem::path guess_extrinsic_calib_static_dir() {
    auto exe = std::filesystem::canonical("/proc/self/exe");
    auto repo = exe.parent_path().parent_path().parent_path();
    return repo / "web" / "extrinsic_calibration";
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

// Split a comma-separated list, trimming surrounding ASCII whitespace from
// each token. Used to parse --excal-faces "0, 1, 2".
std::vector<std::string> split_csv(const std::string& s) {
    std::vector<std::string> out;
    std::size_t start = 0;
    while (start <= s.size()) {
        std::size_t comma = s.find(',', start);
        std::size_t end = (comma == std::string::npos) ? s.size() : comma;
        std::size_t b = start, e = end;
        while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
        while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
        out.push_back(s.substr(b, e - b));
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return out;
}

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

}  // namespace

int main(int argc, char** argv) {
    // --help / --probe / --config are pulled out of argv first so the YAML
    // loader doesn't need to know about meta-flags and so --probe can exit
    // without ever touching the config file (docs/backlog-main-yaml-config.md).
    fitra::config::EarlyArgs early;
    try {
        early = fitra::config::scan_early_args(argc - 1, argv + 1);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return EXIT_FAILURE;
    }
    if (early.want_help) { print_help(); return EXIT_SUCCESS; }

    fitra::config::MainOptions opts;

    try {
        // --probe wins over --config; a sanity run shouldn't depend
        // on a runtime YAML being valid.
        if (early.want_probe) return probe();

        if (!early.config_path.empty()) {
            fitra::config::load_main_config(early.config_path, opts);
        }
        fitra::config::apply_cli_overrides(opts, argc - 1, argv + 1);

        // Lock the process-wide keypoint topology before any pipeline thread
        // starts. RTMPose validates --pose-engine K against this format.
        {
            fitra::lift::KeypointFormat fmt;
            if (!fitra::lift::parse_keypoint_format(opts.keypoint_format, fmt)) {
                std::fprintf(stderr,
                    "unknown --keypoint-format %s (use coco17 or halpe26)\n",
                    opts.keypoint_format.c_str());
                return EXIT_FAILURE;
            }
            fitra::lift::set_active_keypoint_format(fmt);
            FITRA_LOG_INFO("[fitra] kp_format={} ({} keypoints)",
                           fitra::lift::keypoint_format_name(fmt),
                           static_cast<int>(fitra::lift::active_kp_count()));
        }

        try {
            fitra::config::validate_options(opts);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "%s\n", e.what());
            // Mirror the historical behavior of printing help on the
            // "required flags missing" path so a bare `./main` still shows
            // usage. Other failures (range checks etc.) come with a clear
            // message above and don't need the full help dump.
            if (opts.cam_paths[0].empty()
                || opts.det_engine.empty()
                || opts.pose_engine.empty()) {
                print_help();
            }
            return EXIT_FAILURE;
        }

        // Binding aliases keep the downstream runtime-construction code
        // (originally written against ~40 local variables) unchanged. The
        // compiler folds these references away.
        auto& cam_paths              = opts.cam_paths;
        auto& det_engine_path        = opts.det_engine;
        auto& pose_engine_path       = opts.pose_engine;
        auto& port                   = opts.port;
        auto& host                   = opts.host;
        auto& static_dir             = opts.static_dir;
        auto& no_web                 = opts.no_web;
        auto& width                  = opts.width;
        auto& height                 = opts.height;
        auto& fps                    = opts.fps;
        auto& det_frequency          = opts.det_frequency;
        auto& multi_person           = opts.multi_person;
        auto& bench_fake_bbox        = opts.bench_fake_bbox;
        auto& det_score              = opts.det_score;
        auto& log_every_s            = opts.log_every_s;
        auto& enable_3d              = opts.enable_3d;
        auto& calib_path             = opts.calib;
        auto& kp_conf_thresh         = opts.kp_conf_thresh;
        auto& max_reproj_px          = opts.max_reproj_px;
        auto& sync_window_ms         = opts.sync_window_ms;
        auto& bone_calib_frames      = opts.bone_calib_frames;
        auto& subject_height_m       = opts.subject_height_m;
        auto& subject_id             = opts.subject_id;
        auto& subjects_dir           = opts.subjects_dir;
        auto& subject_profile_path   = opts.subject_profile;
        auto& kalman_3d              = opts.kalman_3d;
        auto& ik_3d                  = opts.ik_3d;
        auto& slimevr_out            = opts.slimevr_out;
        auto& slimevr_host           = opts.slimevr_host;
        auto& slimevr_port           = opts.slimevr_port;
        auto& slimevr_rate_hz        = opts.slimevr_rate_hz;
        auto& slimevr_quat_smooth    = opts.slimevr_quat_smooth;
        auto& slimevr_preview_no_reset =
            opts.slimevr_preview_no_reset;
        auto& vmt_out                = opts.vmt_out;
        auto& vmt_host               = opts.vmt_host;
        auto& vmt_port               = opts.vmt_port;
        auto& vmt_rate_hz            = opts.vmt_rate_hz;
        auto& vmt_index_base         = opts.vmt_index_base;
        auto& vmt_degeneracy_mode    = opts.vmt_degeneracy_mode;
        auto& vmt_disable_below_floor= opts.vmt_disable_below_floor;
        auto& hmd_listen_enabled     = opts.hmd_listen_enabled;
        auto& hmd_listen_port        = opts.hmd_listen_port;
        auto& hmd_listen_bind        = opts.hmd_listen_bind;
        auto& hmd_stale_ms           = opts.hmd_stale_ms;
        auto& calibrate_on_boot      = opts.calibrate;
        auto& calib_subject_id       = opts.calib_subject_id;
        auto& calib_subject_height_m = opts.calib_subject_height_m;
        auto& calib_frames_per_cam   = opts.calib_frames_per_cam;
        auto& calib_hold_sec         = opts.calib_hold_sec;
        auto& calib_auto_approve     = opts.calib_auto_approve;
        auto& calib_auto_exit        = opts.calib_auto_exit;
        auto& calib_static_dir       = opts.calib_static_dir;
        auto& calib_dump_tool        = opts.calib_dump_tool;
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
            o.n_buffers = opts.n_buffers;
            o.pixel_format = (opts.pixel_format == "yuyv")
                                 ? fitra::camera::PixFmt::Yuyv
                                 : (opts.pixel_format == "nvjpeg")
                                       ? fitra::camera::PixFmt::Nvjpeg
                                       : fitra::camera::PixFmt::Mjpeg;
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
            // Controller-marker extrinsic calibration runs AprilTag detection
            // from the frame tap, so it needs a CPU BGR image even on the
            // all-GPU nvjpeg path.
            src_opts.retain_bgr = opts.excal_enabled;
            // Subject-calibration recording taps MultiCameraDriver's frame tap.
            // The same flag pauses YOLOX + RTMPose pre-bake while recording so
            // disk I/O has the CPU/GPU headroom and we don't burn cycles on a
            // pose feed nobody is watching.
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
        // SlimeVR tracker snapshot bus + extractor thread. Always alive when
        // enable_3d so the WebUI orientation viz works without --slimevr-out.
        std::unique_ptr<fitra::slimevr::SlimeTrackerBus>   slime_tracker_bus;
        std::unique_ptr<fitra::slimevr::TrackerExtractor>  tracker_extractor;
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
            slime_tracker_bus = std::make_unique<fitra::slimevr::SlimeTrackerBus>();
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

        // Subject calibration session: only set up when 3D is enabled AND
        // exactly 2 cameras are attached. The session orchestrator and
        // dump_keypoints_3d both assume cam0/cam1; refuse to attach for 1-
        // or 3-camera runs rather than silently dropping cam2.
        std::unique_ptr<fitra::pipeline::CalibrationSession> calib_session;
        fitra::pipeline::CalibPreflight calib_defaults;
        // The subject wizard and the extrinsic-calib collector both claim the
        // single frame tap; --extrinsic-calib (gated mutually exclusive with
        // --calibrate in validate_options) takes the tap, so suppress the
        // subject wizard entirely while extrinsic calibration is active.
        const bool calib_available = enable_3d && n_cams == 2 && !opts.excal_enabled;
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

        // Controller-marker extrinsic calibration (parallel to the subject
        // wizard; mutually exclusive with it). Receives the VR controller pose
        // from the unified VMT pose relay, taps camera frames into the
        // collection session, and solves + writes extrinsics at shutdown.
        // See docs/design/pose-3d-controller-marker-extrinsic.md.
        auto controller_pose_bus = std::make_unique<fitra::vmt::ControllerPoseBus>();
        std::unique_ptr<fitra::pipeline::ExtrinsicCalibSession> excal_session;
        if (opts.excal_enabled) {
            const std::string intr_path = opts.excal_intrinsics.empty()
                                          ? opts.calib : opts.excal_intrinsics;
            FITRA_LOG_INFO("extrinsic-calib: loading intrinsics from {}", intr_path);
            fitra::pipeline::ExtrinsicCalibConfig ec;
            ec.intrinsics = fitra::lift::load_calibration(intr_path);
            if (ec.intrinsics.cameras.size() < n_cams) {
                std::fprintf(stderr,
                    "extrinsic-calib: intrinsics file has %zu cameras, need >= %zu\n",
                    ec.intrinsics.cameras.size(), n_cams);
                return EXIT_FAILURE;
            }
            // Parse "0,1,2" face ids; uniform tag size for the skeleton.
            for (const auto& tok : split_csv(opts.excal_faces)) {
                if (tok.empty()) continue;
                fitra::lift::MarkerFace f;
                f.face_id = std::atoi(tok.c_str());
                f.tag_size_m = opts.excal_tag_size_m;
                ec.board.faces.push_back(f);
            }
            ec.lin_vel_max_mps        = opts.excal_lin_vel_max;
            ec.ang_vel_max_dps        = opts.excal_ang_vel_max;
            ec.burst_min              = opts.excal_burst_min;
            ec.min_samples_per_group  = opts.excal_min_samples;
            ec.out_path               = opts.excal_out;
            excal_session = std::make_unique<fitra::pipeline::ExtrinsicCalibSession>(
                std::move(ec));

            const double stale_ms = opts.excal_controller_stale_ms;
            driver->set_frame_tap(
                [s = excal_session.get(), bus = controller_pose_bus.get(), stale_ms]
                (std::size_t cam, const cv::Mat& bgr, double ts) {
                    auto snap = bus->snapshot(stale_ms);
                    fitra::pipeline::ControllerObservation c;
                    c.running_ok = !snap.stale && snap.pose.running_ok();
                    c.x = snap.pose.x; c.y = snap.pose.y; c.z = snap.pose.z;
                    c.qx = snap.pose.qx; c.qy = snap.pose.qy;
                    c.qz = snap.pose.qz; c.qw = snap.pose.qw;
                    c.ts_ms = ts;
                    s->on_frame(cam, bgr, c);
                });
            excal_session->start();
            FITRA_LOG_INFO("extrinsic-calib: collecting (faces={}, controller_role={}, out={}). "
                           "Stop the process to solve + write.",
                           opts.excal_faces, opts.excal_controller_role, opts.excal_out);
        }

        // Stop the driver worker before any session it taps into goes out of
        // scope. Declared *after* both calib_session and excal_session so that
        // on unwind (early return / exception path) this destructor — which
        // calls driver->stop() — runs first, quiescing the frame-tap callbacks
        // before the session objects they reference are destroyed. Normal
        // shutdown calls driver->stop() explicitly below; this guard handles
        // the exception path.
        struct DriverStop {
            fitra::pipeline::MultiCameraDriver* d;
            ~DriverStop() { if (d) d->stop(); }
        } driver_stop{driver.get()};

        // Start the TrackerExtractor before any consumer so the SlimeVR
        // Firmware UDP publisher and the WebUI both see the same smoothed
        // tracker stream. Always running when enable_3d, regardless of
        // --slimevr-out — the WebUI orientation viz needs trackers even when
        // the Firmware UDP path is off.
        if (bus3d && slime_tracker_bus) {
            fitra::slimevr::TrackerExtractorOptions tex_opts;
            tex_opts.extract_rate_hz = slimevr_rate_hz;
            tex_opts.quat_smooth     = static_cast<float>(slimevr_quat_smooth);
            // pos EMA alpha is sourced from --vmt-pos-smooth. The WebUI viz
            // also benefits from pos smoothing (AxesHelper jitter), so this
            // runs regardless of --vmt-out / --slimevr-out toggles — same
            // architecture as quat_smooth.
            tex_opts.pos_smooth      = static_cast<float>(opts.vmt_pos_smooth);
            tex_opts.event_driven    = opts.vr_extract_event_driven;
            // One Euro (speed-adaptive) smoothing — default path. When on,
            // quat_smooth/pos_smooth above are ignored. Feeds both outputs +
            // WebUI (single producer).
            tex_opts.one_euro        = opts.vr_one_euro;
            tex_opts.pos_one_euro    = {static_cast<float>(opts.vr_pos_mincutoff),
                                        static_cast<float>(opts.vr_pos_beta),
                                        static_cast<float>(opts.vr_pos_dcutoff)};
            tex_opts.quat_one_euro   = {static_cast<float>(opts.vr_quat_mincutoff),
                                        static_cast<float>(opts.vr_quat_beta),
                                        static_cast<float>(opts.vr_quat_dcutoff)};
            tracker_extractor = std::make_unique<fitra::slimevr::TrackerExtractor>(
                *bus3d, *slime_tracker_bus, tex_opts);
            tracker_extractor->start();
        }

        // Spin up the native SlimeVR Firmware UDP publisher BEFORE the Crow
        // server so /stats3d can hand out the slimevr stats block.
        // bus3d / slime_tracker_bus are guaranteed non-null at this point
        // (gated by enable_3d).
        std::unique_ptr<fitra::slimevr::NativePublisher> slime_pub;
        if (slimevr_out) {
            fitra::slimevr::NativePublisherOptions opts;
            opts.host         = slimevr_host;
            opts.port         = static_cast<std::uint16_t>(slimevr_port);
            opts.send_rate_hz = slimevr_rate_hz;
            opts.quat_smooth  = static_cast<float>(slimevr_quat_smooth);
            opts.preview_no_reset = slimevr_preview_no_reset;
            slime_pub = std::make_unique<fitra::slimevr::NativePublisher>(
                *bus3d, *slime_tracker_bus, opts);
            if (!slime_pub->start()) {
                // Socket setup or handshake failed; warn and continue without
                // publisher (pose pipeline is unaffected).
                slime_pub.reset();
            }
        }

        // Spin up the VMT publisher BEFORE the Crow server so /stats3d can
        // hand out the vmt stats block. Independent of slimevr; both
        // publishers can be enabled simultaneously and share the same
        // TrackerExtractor state (single-producer invariant).
        std::unique_ptr<fitra::vmt::VmtPublisher> vmt_pub;
        if (vmt_out) {
            fitra::vmt::VmtPublisherOptions vopts;
            vopts.host         = vmt_host;
            vopts.port         = static_cast<std::uint16_t>(vmt_port);
            vopts.send_rate_hz = vmt_rate_hz;
            vopts.index_base   = vmt_index_base;
            vopts.disable_below_floor = vmt_disable_below_floor;
            if (!fitra::vmt::parse_degen_mode(vmt_degeneracy_mode, vopts.degeneracy_mode)) {
                // validate_options should have caught this, but defend in depth.
                vopts.degeneracy_mode = fitra::vmt::DegenMode::Hold;
            }
            vmt_pub = std::make_unique<fitra::vmt::VmtPublisher>(
                *bus3d, *slime_tracker_bus, vopts);
            if (!vmt_pub->start()) {
                vmt_pub.reset();
            }
        }

        // Optional unified VMT pose relay receiver. Standalone from vmt_pub —
        // it feeds the HMD bus for alignment and the selected controller bus
        // for extrinsic calibration. It also accepts the legacy /fitra/hmd_pose
        // and /fitra/controller_pose messages on the same port during migration.
        auto hmd_pose_bus = std::make_unique<fitra::vmt::HmdPoseBus>();
        std::unique_ptr<fitra::vmt::TrackedPoseReceiver> tracked_pose_recv;
        const bool pose_relay_enabled = hmd_listen_enabled || opts.excal_enabled;
        fitra::vmt::TrackedPoseRole excal_controller_role =
            fitra::vmt::TrackedPoseRole::RightController;
        if (!fitra::vmt::parse_tracked_pose_role(opts.excal_controller_role,
                                                 excal_controller_role)) {
            // validate_options should have caught this, but keep the runtime
            // path deterministic if a caller constructed MainOptions manually.
            excal_controller_role = fitra::vmt::TrackedPoseRole::RightController;
        }
        if (pose_relay_enabled) {
            fitra::vmt::TrackedPoseReceiverOptions popts;
            popts.bind = hmd_listen_bind;
            popts.port = static_cast<std::uint16_t>(hmd_listen_port);
            popts.stale_ms = hmd_stale_ms;
            popts.controller_role = excal_controller_role;
            tracked_pose_recv = std::make_unique<fitra::vmt::TrackedPoseReceiver>(
                *hmd_pose_bus, *controller_pose_bus, popts);
            if (!tracked_pose_recv->start()) {
                tracked_pose_recv.reset();
            }
        }

        // Continuous (always-on) HMD-driven alignment refinement. Read-only
        // consumer of bus3d + the HMD bus; nudges vmt_pub's alignment over time.
        // Inert unless vmt_out + hmd_listen_enabled + enable_3d are all on.
        std::unique_ptr<fitra::vmt::ContinuousAligner> continuous_aligner;
        if (vmt_pub && bus3d && hmd_listen_enabled && opts.vmt_continuous_align) {
            fitra::vmt::ContinuousAlignerConfig cacfg;
            cacfg.enabled          = true;
            cacfg.sample_hz        = opts.vmt_continuous_sample_hz;
            cacfg.resolve_period_s = opts.vmt_continuous_resolve_s;
            cacfg.blend_alpha      = static_cast<float>(opts.vmt_continuous_blend);
            continuous_aligner = std::make_unique<fitra::vmt::ContinuousAligner>(
                *bus3d, *hmd_pose_bus, *vmt_pub, hmd_stale_ms, cacfg);
            continuous_aligner->start();
            FITRA_LOG_INFO("continuous HMD alignment: enabled "
                           "(sample {} Hz, resolve {} s, blend {})",
                           opts.vmt_continuous_sample_hz,
                           opts.vmt_continuous_resolve_s,
                           opts.vmt_continuous_blend);
        }

        // Stop the publishers + tracker extractor on any scope exit. Must
        // outlive the server (so /stats3d never reads a dead pointer / a
        // dead bus) and the driver (the publishers and extractor all read
        // buses the driver feeds). The aligner is stopped first because it
        // reads vmt_pub + the HMD/3D buses.
        struct SlimeStop {
            fitra::vmt::ContinuousAligner*    aligner;
            fitra::slimevr::NativePublisher*  pub;
            fitra::vmt::VmtPublisher*         vmt_pub;
            fitra::vmt::TrackedPoseReceiver*  pose_recv;
            fitra::slimevr::TrackerExtractor* tex;
            ~SlimeStop() {
                if (aligner)  aligner->stop();
                if (pub)      pub->stop();
                if (vmt_pub)  vmt_pub->stop();
                if (pose_recv) pose_recv->stop();
                if (tex)      tex->stop();
            }
        } slime_stop{continuous_aligner.get(), slime_pub.get(), vmt_pub.get(),
                     tracked_pose_recv.get(), tracker_extractor.get()};

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
            sopts.excal_static_dir = guess_extrinsic_calib_static_dir().string();
            server = std::make_unique<fitra::web::CrowServer>(
                bus, enable_3d ? bus3d.get() : nullptr, sopts);
            if (calib_session) {
                server->set_calibration_session(calib_session.get(), calib_defaults);
            }
            if (excal_session) {
                server->set_extrinsic_calib_session(excal_session.get());
            }
            if (slime_pub) {
                server->set_native_publisher(slime_pub.get());
            }
            if (vmt_pub) {
                server->set_vmt_publisher(vmt_pub.get());
            }
            if (slime_tracker_bus) {
                server->set_tracker_bus(slime_tracker_bus.get());
            }
            // Attach the HMD pose bus for either VMT alignment or extrinsic
            // calibration. The excal 3D scene uses the same Standing/VMT pose
            // relay frame as the selected controller marker.
            if (pose_relay_enabled) {
                server->set_hmd_pose_bus(hmd_pose_bus.get(), hmd_stale_ms);
            }
            if (excal_session) {
                server->set_extrinsic_calib_pose_bus(
                    controller_pose_bus.get(),
                    fitra::vmt::tracked_pose_role_name(excal_controller_role),
                    opts.excal_controller_stale_ms);
            }
            if (continuous_aligner) {
                server->set_continuous_aligner(continuous_aligner.get());
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
        if (slime_pub) slime_pub->stop();   // explicit stop; SlimeStop guard is the fallback
        driver->stop();   // taps quiesce here — safe to read/solve the session

        if (excal_session) {
            FITRA_LOG_INFO("extrinsic-calib: collected {} samples; solving...",
                           excal_session->sample_count());
            std::string err;
            if (excal_session->solve_and_write(err)) {
                FITRA_LOG_INFO("extrinsic-calib: wrote extrinsics to {}", opts.excal_out);
            } else {
                FITRA_LOG_ERROR("extrinsic-calib: solve/write failed: {}", err);
            }
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        FITRA_LOG_ERROR("fatal: {}", e.what());
        return EXIT_FAILURE;
    }
}
