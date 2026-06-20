// fitra_config — runtime options for `cpp/build/main`.
//
// `MainOptions` is the single source of truth for every CLI flag and YAML
// config key consumed by main.cpp. The doc lives in
// `docs/backlog-main-yaml-config.md`; this header should track that schema 1:1.
//
// Precedence (low -> high):
//   1. defaults baked into MainOptions field initializers
//   2. values loaded from --config PATH (load_main_config)
//   3. CLI flags applied via apply_cli_overrides
//
// `validate_options` consolidates the start-up sanity checks previously
// scattered inline at the top of main(); failures throw std::runtime_error
// which main() catches and turns into EXIT_FAILURE.

#pragma once

#include <array>
#include <stdexcept>
#include <string>

namespace fitra::config {

struct MainOptions {
    // cameras
    std::array<std::string, 3> cam_paths{};
    int width  = 640;
    int height = 480;
    // Per-camera V4L2 capture resolution override (0 = use width/height, no
    // downscale). For cameras whose low-res modes center-crop instead of
    // downscaling: capture at full-sensor dims here, then FrameSource resizes
    // to width/height. See docs/design/core-pipeline-per-camera-capture-downscale.md.
    std::array<int, 3> cam_cap_width{};
    std::array<int, 3> cam_cap_height{};
    int fps    = 30;
    // V4L2 pixel format: "mjpeg" (default; camera-side compression, CPU decode)
    // or "yuyv" (uncompressed; skips decode + camera encode latency but costs
    // USB bandwidth -> caps resolution/fps). See docs/design/core-pipeline-e2e-latency.md.
    std::string pixel_format = "mjpeg";
    // Per-camera pixel_format override (empty = use the global pixel_format).
    // Use to mix decode paths: e.g. nvjpeg (HW) for a camera on a clean bus that
    // must keep up at full rate, mjpeg (CPU, tolerant of corrupt frames) for
    // cameras on a saturated USB2.0 bus where HW NVJPEG would crash on truncated
    // MJPEG. See docs/design/core-pipeline-per-camera-capture-downscale.md.
    std::array<std::string, 3> cam_pixel_format{};
    // Per-camera exposure control (anti-blur + steady fps). exposure_mode:
    // "" / "auto" = leave camera controls untouched; "manual" = fixed exposure
    // + gain, frozen; "assist" = manual initial + slow software AE (gain-first,
    // fps-capped exposure). exposure is V4L2_CID_EXPOSURE_ABSOLUTE in 100us
    // units (0 = leave); gain is V4L2_CID_GAIN (<0 = leave); ae_target is the
    // assist target mean luma. See docs/design/core-pipeline-camera-exposure-control.md.
    std::array<std::string, 3> cam_exposure_mode{};
    std::array<int, 3>         cam_exposure{};
    std::array<int, 3>         cam_gain{-1, -1, -1};
    std::array<int, 3>         cam_ae_target{110, 110, 110};
    // V4L2 mmap ring depth. Fewer buffers = lower worst-case ring staleness;
    // driver-enforced minimum is 2.
    int n_buffers = 4;

    // inference
    std::string det_engine;
    std::string pose_engine;
    int   det_frequency      = 10;
    float det_score          = 0.5f;
    std::string keypoint_format = "coco17";
    bool  multi_person       = false;
    bool  bench_fake_bbox    = false;

    // web
    std::string host = "0.0.0.0";
    int  port = 8000;
    std::string static_dir;
    bool no_web = false;

    // three_d
    bool   enable_3d = false;
    std::string calib;
    float  kp_conf_thresh  = 0.3f;
    float  max_reproj_px   = 6.0f;
    double sync_window_ms  = 15.0;
    int    bone_calib_frames = 150;
    // YAML keys `three_d.no_3d_kalman` / `three_d.no_3d_ik` mirror the CLI
    // negated flags; the runtime predicate stays positive (kalman_3d / ik_3d).
    bool   kalman_3d = true;
    bool   ik_3d     = true;
    // VR tracker extraction: react to each new 3D frame (event-driven) instead
    // of resampling at a fixed cadence. Cuts the extractor's contribution to
    // capture->VR-send latency. Feeds both SlimeVR and VMT. Default off.
    bool   vr_extract_event_driven = false;
    // One Euro (speed-adaptive) tracker smoothing. Feeds both SlimeVR and VMT
    // (single producer). Default on: kills at-rest jitter that a fixed-alpha
    // EMA cannot, staying lag-free in motion. When off, the fixed-alpha EMA
    // (slimevr.quat_smooth / vmt.pos_smooth) is used. Position params are
    // per-axis (m/s); rotation params on geodesic angular speed (rad/s). beta=0
    // → fixed-cutoff low-pass without leaving the One Euro path.
    bool   vr_one_euro       = true;
    double vr_pos_mincutoff  = 1.0;   // Hz, at-rest smoothness (lower = smoother)
    double vr_pos_beta       = 4.0;   // motion responsiveness (higher = less lag)
    double vr_pos_dcutoff    = 1.0;   // Hz, speed-estimate low-pass
    double vr_quat_mincutoff = 1.5;
    double vr_quat_beta      = 1.5;
    double vr_quat_dcutoff   = 1.0;

    // subject — identity (used by both run and subject calibration). Lives in
    // the YAML `subject:` block; subject_id + subject_height_m are the single
    // source of truth (no separate calib_subject_* anymore).
    std::string subjects_dir = "calibrations/subjects";
    std::string subject_id;
    std::string subject_profile;
    double subject_height_m = 0.0;

    // subject calibration — process knobs only (YAML `subject_calib:` block,
    // de-prefixed keys; the legacy `calibration:` block + calib_* keys still
    // load as deprecated aliases). The subject id/height come from `subject:`.
    bool   calibrate = false;  // run-mode-deriving; CLI/--calibrate only
    int    calib_frames_per_cam = 75;
    double calib_hold_sec = 1.5;
    bool   calib_auto_approve = false;
    bool   calib_auto_exit    = false;
    std::string calib_static_dir;
    std::string calib_dump_tool;

    // logging
    double log_every_s = 2.0;

    // slimevr native Firmware UDP publisher
    bool   slimevr_out = false;
    std::string slimevr_host = "127.0.0.1";
    int    slimevr_port = 6969;
    double slimevr_rate_hz = 60.0;
    double slimevr_quat_smooth = 0.5;
    bool   slimevr_preview_no_reset = false;

    // VMT (Virtual Motion Tracker) publisher, SteamVR Driver direct.
    // Independent of slimevr; both can be enabled simultaneously and share
    // the same TrackerExtractor state (single-producer invariant).
    bool   vmt_out = false;
    std::string vmt_host = "127.0.0.1";
    int    vmt_port = 39570;
    double vmt_rate_hz = 60.0;
    int    vmt_index_base = 10;
    double vmt_pos_smooth = 0.5;             // Position EMA alpha
    // "hold" (default) | "disable" | "skip" — see vmt_publisher.hpp DegenMode
    std::string vmt_degeneracy_mode = "hold";
    bool   vmt_disable_below_floor = false;
    // Receive HMD pose from the Windows-side vmt_hmd_pose_sender.
    // Independent of vmt_out (sender path) — receiver can run alone for
    // diagnostics, but auto-alignment requires both.
    bool        hmd_listen_enabled = false;
    int         hmd_listen_port    = 39571;
    std::string hmd_listen_bind    = "0.0.0.0";
    double      hmd_stale_ms       = 200.0;

    // Continuous (always-on) HMD-driven alignment refinement. Requires vmt_out
    // + hmd_listen_enabled + enable_3d to do anything; silently inert otherwise.
    // Defaults on so the rig self-aligns from start-up without a manual T-pose.
    bool   vmt_continuous_align     = true;
    double vmt_continuous_sample_hz = 15.0;
    double vmt_continuous_resolve_s = 2.0;
    double vmt_continuous_blend     = 0.2;   // EMA weight per resolve, in (0, 1]

    // Controller-marker extrinsic calibration (see
    // docs/design/pose-3d-controller-marker-extrinsic.md). When enabled, main
    // receives the controller pose on a parallel UDP channel, taps camera
    // frames into the collection session, and writes extrinsics on solve.
    bool        excal_enabled        = false;
    // Offline replay: a tools/excal_record session directory (JPEG sequence +
    // frames.jsonl). Non-empty implies calib-extrinsic mode and runs
    // collect→solve unattended — no cameras, no SteamVR, no web.
    std::string excal_replay;
    // Intrinsics-only calibration YAML (extrinsics ignored). Empty → reuse
    // three_d.calib.
    std::string excal_intrinsics;
    std::string excal_out            = "calibrations/extrinsics.yaml";
    // AprilTag 36h11 face IDs, comma-separated (e.g. "0,1,2").
    std::string excal_faces          = "0,1,2";
    double      excal_tag_size_m     = 0.10;     // uniform face size, metres
    double      excal_lin_vel_max    = 0.03;     // motion gate, m/s
    double      excal_ang_vel_max    = 8.0;      // motion gate, deg/s
    int         excal_burst_min      = 5;
    int         excal_min_samples    = 8;        // per (cam, face) group
    // Controller pose role consumed from the unified VMT pose relay:
    // "left" | "right" (default). The old dedicated controller port below is
    // kept for config compatibility during migration.
    std::string excal_controller_role = "right";
    // Deprecated legacy controller pose receiver (parallel to the HMD channel).
    int         excal_controller_port = 39572;
    std::string excal_controller_bind = "0.0.0.0";
    double      excal_controller_stale_ms = 200.0;

    // Floor-AprilTag extrinsic calibration (see
    // docs/design/pose-3d-floor-apriltag-extrinsic.md). A VR-free path: each
    // camera localises against a known floor tag map via multi-tag PnP. Selected
    // by --floor-calib (live) or --floor-replay (offline). Distinct from the
    // controller-marker path above (different solver / world frame).
    bool        floor_calib_enabled  = false;
    // Daemon-only extrinsic stage selector ("controller" | "floor"), loaded from
    // extrinsic_calib.method. Distinct from floor_calib_enabled (the run_mode
    // flag, set only by --floor-calib): the shared daemon config must NOT set a
    // run_mode-deriving flag, or the parent + run child derive a calib mode and
    // the flow stalls. The daemon reads excal_method to pick which extrinsic
    // stage to enter / spawn (--floor-calib injected per-child by module_argv).
    std::string excal_method = "controller";
    // Known tag layout YAML (lift::FloorTagMap). Required for the floor path.
    std::string floor_map;
    // Offline replay session directory (tools/excal_record format). Non-empty
    // implies calib-extrinsic-floor and runs collect→solve unattended.
    std::string floor_replay;
    // Calibration-resolution intrinsics used for detection + PnP. Empty → reuse
    // three_d.calib. The recovered T_cw is resolution-independent, so the
    // written YAML carries the runtime intrinsics (floor_out_intrinsics / calib).
    std::string floor_intrinsics;
    // Runtime-resolution intrinsics to embed in the output YAML. Empty → write
    // the same intrinsics used for PnP.
    std::string floor_out_intrinsics;
    std::string floor_out            = "calibrations/extrinsics.yaml";
    int         floor_burst_min      = 10;
    double      floor_max_reproj_px  = 3.0;     // per-frame detection filter (px)
    bool        floor_fisheye        = false;   // intrinsics use the fisheye model

    // Intrinsic (per-camera K + distortion) calibration (see
    // docs/design/pose-3d-intrinsic-calibration.md). ChArUco board capture →
    // cv::calibrateCamera (pinhole) or cv::fisheye::calibrate (fisheye). The
    // setup step before extrinsic calibration. Selected by --calib-intrinsic
    // (live) or --intrinsic-replay (offline).
    bool        intrinsic_calib_enabled = false;
    // Daemon-only: include the intrinsic step (step 0) in the setup chain,
    // loaded from intrinsic_calib.enabled. Distinct from intrinsic_calib_enabled
    // (the run_mode flag, set only by --calib-intrinsic) for the same reason as
    // excal_method above — keeping the shared daemon config free of run_mode
    // flags. The daemon enters calib-intrinsic first when this is set and the
    // intrinsics YAML is missing.
    bool        intrinsic_step_enabled = false;
    std::string intrinsic_replay;
    std::string intrinsic_out      = "calibrations/intrinsics.yaml";
    std::string intrinsic_model    = "pinhole";  // "pinhole" | "fisheye"
    int         charuco_squares_x  = 5;
    int         charuco_squares_y  = 7;
    double      charuco_square_len_m = 0.04;
    double      charuco_marker_len_m = 0.03;
    int         charuco_dict       = -1;   // -1 → DICT_4X4_50
    int         intrinsic_min_views   = 12;
    int         intrinsic_min_corners = 8;
    // Acceptance gate: reject a solve above this rms (a transposed/ wrong board
    // still "solves" with a huge rms). 0 disables. See IntrinsicCalibConfig.
    double      intrinsic_max_rms_px  = 1.5;

    // Flow daemon (docs/design/pose-3d-flow-daemon.md). CLI-only — how the
    // process is launched is not part of the YAML schema. `flow_managed` is
    // set by the daemon on spawned mode modules; it enables the
    // /api/flow/switch route and the calib auto-chain exit codes.
    bool        daemon         = false;   // --daemon: spawn mode modules
    std::string daemon_initial = "auto";  // auto|setup|run|calib-subject|calib-extrinsic|...
    bool        flow_managed   = false;   // --flow-managed (daemon-spawned)

    // --setup: the first-run setup module (RunMode::Setup). A GPU-less Crow
    // server that enumerates V4L2 cameras, composes/writes the union config,
    // and hands off to the next stage via a flow exit code. CLI-only (the
    // launch form is the caller's responsibility, like the other daemon flags).
    // See docs/design/core-pipeline-setup-mode.md.
    bool        setup_mode     = false;
};

// Exclusive run mode, derived from the calibration flags (invocation stays
// flag-compatible; see docs/design/pose-3d-calib-mode-separation.md). Each
// mode builds only what it needs; the only thing crossing a mode boundary is
// YAML on disk. Derive after validate_options() — it enforces the flags'
// mutual exclusivity.
enum class RunMode {
    Run, Setup, CalibSubject, CalibExtrinsic, CalibExtrinsicFloor, CalibIntrinsic
};

RunMode run_mode(const MainOptions& opts);

// Stable label for logs and /api/state: "run" / "calib-subject" /
// "calib-extrinsic".
const char* run_mode_name(RunMode mode);

// Inverse of run_mode_name(): parse a mode label (e.g. from
// /api/flow/switch or --daemon-initial). Returns false on an unknown label.
bool parse_run_mode_name(const std::string& name, RunMode& out);

// Pre-flight check before a flow-switch respawns into `target`: verifies the
// shared config carries everything that mode needs (required files present /
// readable), so the /api/flow/switch handler can refuse with a clear reason
// instead of respawning a child that dies at validate_options() and silently
// falls the daemon back to run. Returns true if `target` should be reachable;
// on false, `err` holds a human-facing reason. Checks config preconditions
// only — not runtime resources (cameras, pose relay).
bool precheck_mode_switch(const MainOptions& opts, RunMode target,
                          std::string& err);

// Schema version embedded in every YAML config. Bump only when a
// non-backwards-compatible change to the YAML layout is required.
inline constexpr const char* kMainConfigSchema = "fitra_main_config_v1";

// Serialize `opts` back to a `fitra_main_config_v1` YAML document. The inverse
// of load_main_config: every key the loader reads round-trips (emit -> load is
// the identity on the loader-visible fields). Only non-default values are
// emitted (keeps files clean). Run-mode-deriving flags (calibration.calibrate,
// extrinsic_calib.enabled, *.replay_dir) and launch-form flags (daemon /
// flow_managed / setup_mode — which have no YAML key) are NEVER emitted: a
// written config is always a union config the daemon can consume.
std::string emit_main_config(const MainOptions& opts);

// emit_main_config(opts) written to `path` atomically (path.tmp + rename).
// Throws std::runtime_error on an I/O failure.
void save_main_config(const std::string& path, const MainOptions& opts);

// Rewrite the CWD-relative path fields (engines, calib artifacts, subject dir,
// ...) to absolute, resolved against the current working directory. Engine /
// calib paths are opened CWD-relative at runtime (std::ifstream, no base dir);
// the daemon spawns every mode child with the daemon's CWD, so absolutizing in
// the (CWD-sharing) setup module removes the ambiguity of a relative path
// resolving differently later. Idempotent for already-absolute paths; empty
// fields are left empty. Output paths are absolutized too (they need not exist).
void absolutize_config_paths(MainOptions& opts);

// Load a YAML config from `path` into `out`. Unknown keys, type mismatches,
// missing `schema`, or wrong `schema` value all throw std::runtime_error
// with a key-path-qualified message (e.g. "config: unknown key three_d.foo").
//
// Only keys explicitly present in the YAML are overwritten; absent keys keep
// whatever value `out` already has (defaults or prior overlay). Empty strings
// stay empty so that subsequent CLI overrides can fill them in.
void load_main_config(const std::string& path, MainOptions& out);

// Apply CLI flags (the same set that main.cpp historically parsed) on top of
// `out`. `--config PATH`, `--help`, `--probe` are pre-extracted by the caller
// and ignored here. Unknown flags and missing argument values throw
// std::runtime_error.
//
// Pass `argv + 1` (i.e. skip the program name) and the matching argc.
void apply_cli_overrides(MainOptions& out, int argc, char** argv);

// Convenience: report whether the user asked for --probe, --help, or supplied
// a --config PATH, without otherwise mutating `out`. `config_path` is set to
// empty if no --config flag was present. Throws on `--config` missing arg.
struct EarlyArgs {
    bool want_help  = false;
    bool want_probe = false;
    std::string config_path;
};
EarlyArgs scan_early_args(int argc, char** argv);

// Verify the post-overlay options are runnable. Mirrors the historical
// start-up checks in main.cpp (required cam0 + engines, --enable-3d gating,
// subject-height range, slimevr gating, calibrate gating, etc.).
//
// Throws std::runtime_error with a user-facing message on the first failure.
void validate_options(const MainOptions& opts);

}  // namespace fitra::config
