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
    int fps    = 30;
    // V4L2 pixel format: "mjpeg" (default; camera-side compression, CPU decode)
    // or "yuyv" (uncompressed; skips decode + camera encode latency but costs
    // USB bandwidth -> caps resolution/fps). See docs/design/core-pipeline-e2e-latency.md.
    std::string pixel_format = "mjpeg";
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

    // subject
    std::string subjects_dir = "calibrations/subjects";
    std::string subject_id;
    std::string subject_profile;
    double subject_height_m = 0.0;

    // calibration (wizard)
    bool   calibrate = false;
    std::string calib_subject_id;
    double calib_subject_height_m = 0.0;
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
    // Controller pose receiver (parallel to the HMD channel).
    int         excal_controller_port = 39572;
    std::string excal_controller_bind = "0.0.0.0";
    double      excal_controller_stale_ms = 200.0;
};

// Schema version embedded in every YAML config. Bump only when a
// non-backwards-compatible change to the YAML layout is required.
inline constexpr const char* kMainConfigSchema = "fitra_main_config_v1";

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
