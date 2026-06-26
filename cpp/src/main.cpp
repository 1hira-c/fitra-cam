// fitra-cam main — N-camera driver.
//
// Thin composition root: parses --config/CLI into MainOptions, validates,
// derives the exclusive RunMode (run / calib-subject / calib-extrinsic), and
// dispatches to the mode runner in app/. Each mode builds only what it needs;
// the only thing crossing a mode boundary is YAML on disk
// (docs/design/pose-3d-calib-mode-separation.md).
//
// Usage (one line, no shell continuation):
//   fitra-cam --cam0 PATH [--cam1 PATH] [--cam2 PATH] --det-engine PATH --pose-engine PATH
//             [--port 8000] [--host 0.0.0.0] [--static DIR] [--no-web]
//             [--width 640] [--height 480] [--fps 30]
//             [--det-frequency 10] [--multi-person] [--enable-3d --calib PATH] [--probe]
//
// `--probe` is a CUDA device + TRT runtime sanity check that exits.

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <memory>

#include <cuda_runtime_api.h>
#include <NvInfer.h>
#include <NvInferVersion.h>

#include "app/daemon.hpp"
#include "app/flow.hpp"
#include "app/mode_calib_extrinsic.hpp"
#include "app/mode_calib_extrinsic_floor.hpp"
#include "app/mode_calib_intrinsic.hpp"
#include "app/mode_calib_subject.hpp"
#include "app/mode_run.hpp"
#include "app/mode_setup.hpp"
#include "app/trt_stack.hpp"
#include "config/main_config.hpp"
#include "lift/keypoint_format.hpp"
#include "util/cuda_check.hpp"
#include "util/logging.hpp"

namespace {

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
        "  --static DIR              web frontend dir (default <repo>/web-ui/dist)\n"
        "  --no-web                  do not start Crow (driver only, for bench)\n"
        "  --no-idle                 disable auto standby (idle on by default)\n"
        "  --idle-enter-after-s S    consumer-zero dwell before standby (default 10)\n"
        "  --idle-tick-hz HZ         driver loop rate while idle (default 2)\n"
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
        "  --foot-tracker-pos S      foot tracker position: ankle|midpoint (default ankle)\n"
        "  --chest-height-frac F     chest tracker height up the spine, 0=hip 1=neck (default 0.65)\n"
        "  --waist-height-frac F     waist/hip tracker height up the spine, 0=hip 1=neck (default 0.15)\n"
        "\n"
        "SlimeVR native Firmware UDP output (requires --enable-3d + --keypoint-format=halpe26):\n"
        "  --slimevr-out             enable the native Firmware UDP publisher (10 trackers)\n"
        "  --no-slimevr-out          negate slimevr.slimevr_out from --config (calib spawns)\n"
        "  --slimevr-host ADDR       SlimeVR Server host (default 127.0.0.1; typically the Windows IP)\n"
        "  --slimevr-port N          UDP port (default 6969 — SlimeVR firmware port)\n"
        "  --slimevr-rate-hz F       RotationData send rate (default 60.0)\n"
        "  --slimevr-quat-smooth F   per-tracker slerp alpha 0..1 (default 0.5)\n"
        "  --slimevr-preview-no-reset  pre-cancel SlimeVR default mounting so GUI preview\n"
        "                              works before reset\n"
        "\n"
        "Virtual Motion Tracker (VMT) → SteamVR direct (requires --enable-3d + --keypoint-format=halpe26):\n"
        "  --vmt-out                 enable the VMT OSC publisher (/VMT/Room/Driver)\n"
        "  --no-vmt-out              negate vmt.vmt_out from --config (calib spawns)\n"
        "  --vmt-host ADDR           VMT Manager host (default empty = zeroconf discovery; set to pin the Windows IP)\n"
        "  --vmt-port N              UDP port (default 39570 — VMT receive port)\n"
        "  --vmt-rate-hz F           send rate (default 60.0)\n"
        "  --vmt-index-base N        first VMT device index (default 10 -> VMT_10..VMT_19)\n"
        "  --vmt-preset S            tracker set: p3|p6|p8|full (default p8 = VRChat std 8-point;\n"
        "                            p3=hip+feet, p6=+chest+knees, p8=+elbows, full=+shins/10)\n"
        "  --vmt-pos-smooth F        position EMA alpha 0..1 (default 0.5; wired in M3)\n"
        "  --vmt-degeneracy-mode S   what to do for invalid trackers: hold|disable|skip (default hold)\n"
        "  --vmt-disable-below-floor disable trackers whose pos.z < 0 (room-matrix sanity, default off)\n"
        "\n"
        "VMT zeroconf discovery (auto-resolves --vmt-host on the LAN; --vmt-host overrides):\n"
        "  --vmt-discovery           enable discovery beacon (default ON)\n"
        "  --no-vmt-discovery        disable discovery (then --vmt-host is required for --vmt-out)\n"
        "  --vmt-pair-id ID          pin a specific peer by its instance_id\n"
        "  --vmt-pairing-token TOK   only pair with peers advertising the same token (cross-rig isolation)\n"
        "  --vmt-discovery-group ADDR multicast group (default 239.255.42.99)\n"
        "  --vmt-discovery-port N    discovery UDP port (default 39580)\n"
        "  --vmt-instance-name NAME  human-readable label advertised to peers\n"
        "  --vmt-peer-timeout-s F    drop a peer after this many seconds without an announce (default 5)\n"
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
        "Subject calibration wizard (dedicated calib-subject mode, requires --enable-3d;\n"
        "VR publishers are unavailable while calibrating):\n"
        "  --calibrate                 run in subject-calibration mode (auto-start at boot)\n"
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
        "Controller-marker extrinsic calibration (dedicated calib-extrinsic mode, mutually\n"
        "exclusive with --calibrate; decode-only — --det-engine/--pose-engine not required;\n"
        "see docs/design/pose-3d-controller-marker-extrinsic.md):\n"
        "  --extrinsic-calib           collect controller-marker samples; auto-exits after a\n"
        "                              successful solve+write (also solves on Ctrl-C)\n"
        "  --excal-replay DIR          replay a tools/excal_record session unattended\n"
        "                              (collect->solve->write; no camera/SteamVR/web needed)\n"
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
        "Flow daemon (spawns one mode module at a time and chains them via exit codes;\n"
        "all module settings come from --config — other CLI flags are not forwarded):\n"
        "  --daemon                  run as the flow daemon: calib-extrinsic -> calib-subject\n"
        "                            -> run auto-chain + /api/flow/switch mode switching;\n"
        "                            crashes restart run mode (3 consecutive failures stop)\n"
        "  --daemon-initial MODE     first module: auto (default; picks the first stage whose\n"
        "                            artifact is missing, or setup when no cameras are configured)\n"
        "                            | setup | run | calib-subject | calib-extrinsic | ...\n"
        "  --flow-managed            mark this process as flow-daemon-spawned: enables\n"
        "                            POST /api/flow/switch and the calib auto-chain exit codes\n"
        "                            (set by the daemon; not meant for manual use)\n"
        "  --setup                   first-run setup module: GPU-less Crow server that\n"
        "                            enumerates cameras + composes the config, then hands off\n"
        "                            (docs/design/core-pipeline-setup-mode.md)\n"
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
    fitra::app::TrtLogger trt_logger;
    std::unique_ptr<nvinfer1::IRuntime> rt{nvinfer1::createInferRuntime(trt_logger)};
    TRT_CHECK(rt != nullptr);
    FITRA_LOG_INFO("nvinfer1::IRuntime ok (lib build: {})", getInferLibVersion());
    return EXIT_SUCCESS;
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
            // Bootstrap: if the --config target doesn't exist yet, seed it from
            // the tracked example template so a first run has a writable config
            // the Setup module can compose into. (Refusing to overwrite the
            // template itself lives in SetupConfigStore::write_union, so users
            // point --config at e.g. configs/session.yaml, not the .example.)
            std::error_code fs_ec;
            if (!std::filesystem::exists(early.config_path, fs_ec)) {
                const std::filesystem::path tmpl =
                    "configs/setup_first.yaml.example";
                if (std::filesystem::exists(tmpl, fs_ec) && !fs_ec) {
                    const auto parent =
                        std::filesystem::path{early.config_path}.parent_path();
                    if (!parent.empty())
                        std::filesystem::create_directories(parent, fs_ec);
                    std::filesystem::copy_file(tmpl, early.config_path, fs_ec);
                    if (fs_ec) {
                        std::fprintf(stderr,
                            "bootstrap: cannot seed config %s from %s: %s\n",
                            early.config_path.c_str(), tmpl.string().c_str(),
                            fs_ec.message().c_str());
                        return EXIT_FAILURE;
                    }
                    std::fprintf(stderr, "bootstrap: seeded %s from %s\n",
                                 early.config_path.c_str(),
                                 tmpl.string().c_str());
                }
                // else: template missing -> fall through; load_main_config
                // below reports the missing file clearly.
            }
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

        // Flow daemon: spawn one mode module at a time and chain via exit
        // codes. Dispatched before anything heavy — the daemon process never
        // touches CUDA/TRT/sockets (docs/design/pose-3d-flow-daemon.md). It
        // installs its own SIGINT/SIGTERM handlers (forward-to-child + stop),
        // so we don't pre-arm on_signal here.
        if (opts.daemon) {
            return fitra::app::run_daemon(opts, early.config_path, argv[0],
                                          g_stop);
        }

        // Exclusive run mode, derived from the validated flags. Each mode
        // builds only what it needs; the contract between modes is the YAML
        // artifacts on disk (docs/design/pose-3d-calib-mode-separation.md).
        const auto mode = fitra::config::run_mode(opts);
        FITRA_LOG_INFO("[fitra] mode={}{}", fitra::config::run_mode_name(mode),
                       opts.flow_managed ? " (flow-managed)" : "");
        std::signal(SIGINT, on_signal);

        // Daemon-managed modules report a requested mode switch through
        // their exit code; standalone runs never set next_mode and keep the
        // conventional 0/1 exits (docs/design/pose-3d-flow-daemon.md).
        fitra::app::FlowControl flow{g_stop, opts.flow_managed};
        int rc = EXIT_FAILURE;
        switch (mode) {
            case fitra::config::RunMode::Setup:
                rc = fitra::app::run_mode_setup(opts, early.config_path, flow);
                break;
            case fitra::config::RunMode::CalibExtrinsic:
                rc = fitra::app::run_mode_calib_extrinsic(opts, flow);
                break;
            case fitra::config::RunMode::CalibExtrinsicFloor:
                rc = fitra::app::run_mode_calib_extrinsic_floor(opts, flow);
                break;
            case fitra::config::RunMode::CalibIntrinsic:
                rc = fitra::app::run_mode_calib_intrinsic(opts, flow);
                break;
            case fitra::config::RunMode::CalibSubject:
                rc = fitra::app::run_mode_calib_subject(opts, flow);
                break;
            case fitra::config::RunMode::Run:
                rc = fitra::app::run_mode_run(opts, flow);
                break;
        }
        const int next = flow.next_mode.load();
        if (rc == EXIT_SUCCESS && next >= 0) {
            const auto next_mode = static_cast<fitra::config::RunMode>(next);
            FITRA_LOG_INFO("[fitra] flow: requesting next mode {}",
                           fitra::config::run_mode_name(next_mode));
            return fitra::app::flow_exit_code(next_mode);
        }
        return rc;
    } catch (const std::exception& e) {
        FITRA_LOG_ERROR("fatal: {}", e.what());
        return EXIT_FAILURE;
    }
}
