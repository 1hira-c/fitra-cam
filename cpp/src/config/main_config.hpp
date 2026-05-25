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

    // slimevr (Phase 11 native Firmware UDP publisher)
    bool   slimevr_out = false;
    std::string slimevr_host = "127.0.0.1";
    int    slimevr_port = 6969;
    double slimevr_rate_hz = 60.0;
    double slimevr_quat_smooth = 0.5;
    bool   slimevr_preview_no_reset = false;
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
