// test_main_config — unit tests for fitra::config::MainOptions YAML loader,
// CLI overlay, and validation. No TensorRT / CUDA dependencies.
//
// Style follows the other tests in this dir (test_firmware_protocol.cpp etc.):
// hand-rolled assertions that throw std::runtime_error on failure; main()
// catches and reports.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "config/main_config.hpp"
#include "config/setup_config_store.hpp"

namespace {

using fitra::config::EarlyArgs;
using fitra::config::MainOptions;
using fitra::config::apply_cli_overrides;
using fitra::config::load_main_config;
using fitra::config::scan_early_args;
using fitra::config::validate_options;

void check(bool cond, const std::string& msg) {
    if (!cond) throw std::runtime_error("ASSERT FAILED: " + msg);
}

void check_contains(const std::string& haystack, const std::string& needle,
                    const std::string& label) {
    if (haystack.find(needle) == std::string::npos) {
        throw std::runtime_error(label + ": expected substring \"" + needle
                                 + "\" in \"" + haystack + "\"");
    }
}

std::filesystem::path write_tmp(const std::string& name, const std::string& body) {
    auto dir = std::filesystem::temp_directory_path() / "fitra_test_main_config";
    std::filesystem::create_directories(dir);
    auto p = dir / name;
    std::ofstream f(p);
    if (!f) throw std::runtime_error("cannot write " + p.string());
    f << body;
    f.close();
    return p;
}

// Helper for argv simulation. argv_buf must outlive the returned vector.
std::vector<char*> make_argv(std::vector<std::string>& argv_buf) {
    std::vector<char*> argv;
    argv.reserve(argv_buf.size());
    for (auto& s : argv_buf) argv.push_back(s.data());
    return argv;
}

void test_minimum_config_loads_and_validates() {
    auto p = write_tmp("minimum.yaml", R"(schema: fitra_main_config_v1
cameras:
  cam0: /dev/v4l/by-path/cam-A
  cam1: /dev/v4l/by-path/cam-B
inference:
  det_engine: /tmp/yolox.engine
  pose_engine: /tmp/rtmpose.engine
web:
  port: 9000
)");
    MainOptions opts;
    load_main_config(p.string(), opts);
    check(opts.cam_paths[0] == "/dev/v4l/by-path/cam-A", "cam0 loaded");
    check(opts.cam_paths[1] == "/dev/v4l/by-path/cam-B", "cam1 loaded");
    check(opts.cam_paths[2].empty(),                     "cam2 untouched");
    check(opts.det_engine  == "/tmp/yolox.engine",       "det_engine loaded");
    check(opts.pose_engine == "/tmp/rtmpose.engine",     "pose_engine loaded");
    check(opts.port == 9000,                             "port loaded");
    check(opts.keypoint_format == "coco17",              "default keypoint_format kept");
    check(opts.host == "0.0.0.0",                        "default host kept");
    validate_options(opts);  // must not throw
}

void test_unknown_top_level_key_fails() {
    auto p = write_tmp("unknown_top.yaml", R"(schema: fitra_main_config_v1
cameras:
  cam0: /tmp/a
inference:
  det_engine: /tmp/d
  pose_engine: /tmp/p
flunky:
  irrelevant: true
)");
    MainOptions opts;
    bool threw = false;
    try {
        load_main_config(p.string(), opts);
    } catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "unknown key flunky", "unknown_top_level msg");
    }
    check(threw, "unknown top-level key must throw");
}

void test_unknown_nested_key_fails() {
    auto p = write_tmp("unknown_nested.yaml", R"(schema: fitra_main_config_v1
three_d:
  no_3d_ikx: true
)");
    MainOptions opts;
    bool threw = false;
    try {
        load_main_config(p.string(), opts);
    } catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "three_d.no_3d_ikx", "unknown_nested msg");
    }
    check(threw, "unknown nested key must throw");
}

void test_type_mismatch_fails() {
    auto p = write_tmp("badtype.yaml", R"(schema: fitra_main_config_v1
web:
  port: "abc"
)");
    MainOptions opts;
    bool threw = false;
    try {
        load_main_config(p.string(), opts);
    } catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "web.port", "type_mismatch msg");
    }
    check(threw, "type-mismatched value must throw");
}

void test_missing_schema_fails() {
    auto p = write_tmp("no_schema.yaml", R"(cameras:
  cam0: /tmp/x
)");
    MainOptions opts;
    bool threw = false;
    try {
        load_main_config(p.string(), opts);
    } catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "schema", "missing_schema msg");
    }
    check(threw, "missing schema must throw");
}

void test_wrong_schema_fails() {
    auto p = write_tmp("wrong_schema.yaml", R"(schema: fitra_main_config_v9999
cameras:
  cam0: /tmp/x
)");
    MainOptions opts;
    bool threw = false;
    try {
        load_main_config(p.string(), opts);
    } catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "fitra_main_config_v9999", "wrong_schema msg");
    }
    check(threw, "wrong schema must throw");
}

void test_cli_overrides_yaml() {
    // YAML sets port 8000; CLI bumps it to 8010. Code-default port is also
    // 8000, so we explicitly write YAML to make the override path observable.
    auto p = write_tmp("port_8000.yaml", R"(schema: fitra_main_config_v1
cameras:
  cam0: /dev/v4l/by-path/cam-A
  cam1: /dev/v4l/by-path/cam-B
inference:
  det_engine: /tmp/y.engine
  pose_engine: /tmp/r.engine
web:
  port: 8000
)");
    MainOptions opts;
    load_main_config(p.string(), opts);
    check(opts.port == 8000, "yaml set port=8000");
    std::vector<std::string> argv_buf{"--port", "8010"};
    auto argv = make_argv(argv_buf);
    apply_cli_overrides(opts, static_cast<int>(argv.size()), argv.data());
    check(opts.port == 8010, "CLI --port overrides YAML");
}

void test_negated_three_d_keys_invert_runtime_bools() {
    auto p = write_tmp("no_3d_keys.yaml", R"(schema: fitra_main_config_v1
three_d:
  no_3d_kalman: true
  no_3d_ik: true
)");
    MainOptions opts;
    load_main_config(p.string(), opts);
    check(opts.kalman_3d == false, "no_3d_kalman=true -> kalman_3d=false");
    check(opts.ik_3d     == false, "no_3d_ik=true     -> ik_3d=false");
}

void test_slimevr_preview_no_reset_yaml_and_cli() {
    auto p = write_tmp("slimevr_preview_no_reset.yaml", R"(schema: fitra_main_config_v1
slimevr:
  preview_no_reset: true
)");
    MainOptions opts;
    load_main_config(p.string(), opts);
    check(opts.slimevr_preview_no_reset == true,
          "slimevr.preview_no_reset loads");

    opts.slimevr_preview_no_reset = false;
    std::vector<std::string> argv_buf{"--slimevr-preview-no-reset"};
    auto argv = make_argv(argv_buf);
    apply_cli_overrides(opts, static_cast<int>(argv.size()), argv.data());
    check(opts.slimevr_preview_no_reset == true,
          "--slimevr-preview-no-reset CLI sets option");
}

void test_vmt_index_base_yaml_cli_and_validate() {
    auto p = write_tmp("vmt_index_base.yaml", R"(schema: fitra_main_config_v1
vmt:
  index_base: 20
)");
    MainOptions opts;
    load_main_config(p.string(), opts);
    check(opts.vmt_index_base == 20, "vmt.index_base loads");

    std::vector<std::string> argv_buf{"--vmt-index-base", "10"};
    auto argv = make_argv(argv_buf);
    apply_cli_overrides(opts, static_cast<int>(argv.size()), argv.data());
    check(opts.vmt_index_base == 10, "--vmt-index-base CLI overrides YAML");

    opts.cam_paths[0] = "/tmp/a";
    opts.det_engine   = "/tmp/y";
    opts.pose_engine  = "/tmp/r";
    opts.enable_3d    = true;
    opts.calib        = "/tmp/cam.yaml";
    opts.vmt_out      = true;
    opts.keypoint_format = "halpe26";
    validate_options(opts);

    opts.vmt_index_base = 49;
    bool threw = false;
    try {
        validate_options(opts);
    } catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "--vmt-index-base", "vmt_index_base range msg");
    }
    check(threw, "vmt_index_base 49 must throw");
}

void test_vmt_discovery_yaml_cli_and_validate() {
    auto p = write_tmp("vmt_discovery.yaml", R"(schema: fitra_main_config_v1
vmt:
  discovery: true
  pair_id: living-rig
  pairing_token: lab7
  discovery_group: 239.255.42.50
  discovery_port: 39581
  instance_name: Living Rig
  peer_timeout_s: 7.5
)");
    MainOptions opts;
    load_main_config(p.string(), opts);
    check(opts.vmt_discovery, "vmt.discovery loads");
    check(opts.vmt_pair_id == "living-rig", "vmt.pair_id loads");
    check(opts.vmt_pairing_token == "lab7", "vmt.pairing_token loads");
    check(opts.vmt_discovery_group == "239.255.42.50", "vmt.discovery_group loads");
    check(opts.vmt_discovery_port == 39581, "vmt.discovery_port loads");
    check(opts.vmt_instance_name == "Living Rig", "vmt.instance_name loads");
    check(std::abs(opts.vmt_peer_timeout_s - 7.5) < 1e-9, "vmt.peer_timeout_s loads");

    std::vector<std::string> argv_buf{"--no-vmt-discovery",
                                      "--vmt-pair-id", "other",
                                      "--vmt-discovery-port", "40000"};
    auto argv = make_argv(argv_buf);
    apply_cli_overrides(opts, static_cast<int>(argv.size()), argv.data());
    check(!opts.vmt_discovery, "--no-vmt-discovery overrides YAML");
    check(opts.vmt_pair_id == "other", "--vmt-pair-id CLI overrides YAML");
    check(opts.vmt_discovery_port == 40000, "--vmt-discovery-port CLI overrides YAML");

    // vmt_out with discovery off and no manual host must be rejected.
    MainOptions bad;
    bad.cam_paths[0] = "/tmp/a";
    bad.det_engine   = "/tmp/y";
    bad.pose_engine  = "/tmp/r";
    bad.enable_3d    = true;
    bad.calib        = "/tmp/cam.yaml";
    bad.keypoint_format = "halpe26";
    bad.vmt_out      = true;
    bad.vmt_discovery = false;
    bad.vmt_host     = "";   // no destination at all
    bool threw = false;
    try { validate_options(bad); }
    catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "vmt.host", "discovery-off needs host msg");
    }
    check(threw, "vmt_out + discovery off + empty host must throw");

    // Same but with discovery left on -> valid (the beacon resolves the host).
    bad.vmt_discovery = true;
    validate_options(bad);  // must not throw

    // An out-of-range discovery port is rejected when discovery resolves host.
    bad.vmt_discovery_port = 70000;
    threw = false;
    try { validate_options(bad); }
    catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "--vmt-discovery-port", "discovery port range msg");
    }
    check(threw, "discovery port 70000 must throw");
}

void test_validate_required_missing() {
    MainOptions opts;
    bool threw = false;
    try {
        validate_options(opts);
    } catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "missing required", "validate missing-required msg");
    }
    check(threw, "validate must throw when required flags missing");
}

void test_validate_enable_3d_needs_calib() {
    MainOptions opts;
    opts.cam_paths[0] = "/tmp/a";
    opts.det_engine   = "/tmp/y";
    opts.pose_engine  = "/tmp/r";
    opts.enable_3d    = true;  // calib still empty
    bool threw = false;
    try {
        validate_options(opts);
    } catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "--enable-3d requires --calib", "enable_3d msg");
    }
    check(threw, "enable_3d without calib must throw");
}

void test_validate_slimevr_requires_halpe26() {
    MainOptions opts;
    opts.cam_paths[0] = "/tmp/a";
    opts.det_engine   = "/tmp/y";
    opts.pose_engine  = "/tmp/r";
    opts.enable_3d    = true;
    opts.calib        = "/tmp/cam.yaml";
    opts.slimevr_out  = true;
    opts.keypoint_format = "coco17";   // wrong topology for slimevr
    bool threw = false;
    try {
        validate_options(opts);
    } catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "halpe26", "slimevr halpe26 msg");
    }
    check(threw, "slimevr_out + coco17 must throw");
}

void test_one_euro_yaml_cli_and_validate() {
    auto p = write_tmp("one_euro.yaml", R"(schema: fitra_main_config_v1
three_d:
  vr_one_euro: false
  vr_pos_mincutoff: 0.5
  vr_pos_beta: 0.2
  vr_quat_mincutoff: 1.5
)");
    MainOptions opts;
    load_main_config(p.string(), opts);
    check(opts.vr_one_euro == false,        "three_d.vr_one_euro loads");
    check(opts.vr_pos_mincutoff == 0.5,     "three_d.vr_pos_mincutoff loads");
    check(opts.vr_pos_beta == 0.2,          "three_d.vr_pos_beta loads");
    check(opts.vr_quat_mincutoff == 1.5,    "three_d.vr_quat_mincutoff loads");

    // CLI numeric overrides take precedence over the YAML-loaded values.
    std::vector<std::string> argv_buf{
        "--vr-pos-mincutoff", "0.9", "--vr-quat-beta", "0.4"};
    auto argv = make_argv(argv_buf);
    apply_cli_overrides(opts, static_cast<int>(argv.size()), argv.data());
    check(opts.vr_pos_mincutoff == 0.9, "--vr-pos-mincutoff CLI overrides YAML");
    check(opts.vr_quat_beta == 0.4,     "--vr-quat-beta CLI sets value");

    // dcutoff <= 0 must fail validation.
    opts.cam_paths[0] = "/tmp/a";
    opts.det_engine   = "/tmp/y";
    opts.pose_engine  = "/tmp/r";
    opts.vr_pos_dcutoff = 0.0;
    bool threw = false;
    try {
        validate_options(opts);
    } catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "dcutoff", "dcutoff validation msg");
    }
    check(threw, "vr_pos_dcutoff <= 0 must throw");
}

void test_early_args_extracts_config_and_probe() {
    std::vector<std::string> argv_buf{"--config", "/tmp/x.yaml", "--probe"};
    auto argv = make_argv(argv_buf);
    auto ea = scan_early_args(static_cast<int>(argv.size()), argv.data());
    check(ea.want_probe,                              "scan picks up --probe");
    check(ea.config_path == "/tmp/x.yaml",            "scan picks up --config value");
    check(!ea.want_help,                              "scan does not flag --help");
}

void test_early_args_missing_config_value_throws() {
    std::vector<std::string> argv_buf{"--config"};
    auto argv = make_argv(argv_buf);
    bool threw = false;
    try {
        (void)scan_early_args(static_cast<int>(argv.size()), argv.data());
    } catch (const std::exception&) {
        threw = true;
    }
    check(threw, "scan_early_args must throw on dangling --config");
}

void test_apply_unknown_flag_throws() {
    MainOptions opts;
    std::vector<std::string> argv_buf{"--whoops"};
    auto argv = make_argv(argv_buf);
    bool threw = false;
    try {
        apply_cli_overrides(opts, static_cast<int>(argv.size()), argv.data());
    } catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "--whoops", "unknown flag msg");
    }
    check(threw, "apply_cli_overrides must reject unknown flags");
}

void test_extrinsic_calib_yaml_cli_and_validate() {
    auto p = write_tmp("excal.yaml", R"(schema: fitra_main_config_v1
extrinsic_calib:
  enabled: true
  intrinsics: /tmp/intr.yaml
  faces: "3,4,5"
  tag_size_m: 0.08
  min_samples: 10
  controller_role: left
  controller_port: 40000
)");
    MainOptions opts;
    load_main_config(p.string(), opts);
    check(opts.excal_enabled, "extrinsic_calib.enabled loads");
    check(opts.excal_intrinsics == "/tmp/intr.yaml", "extrinsic_calib.intrinsics loads");
    check(opts.excal_faces == "3,4,5", "extrinsic_calib.faces loads");
    check(opts.excal_min_samples == 10, "extrinsic_calib.min_samples loads");
    check(opts.excal_controller_role == "left", "extrinsic_calib.controller_role loads");
    check(opts.excal_controller_port == 40000, "extrinsic_calib.controller_port loads");

    std::vector<std::string> argv_buf{
        "--excal-tag-size-m", "0.12",
        "--excal-faces", "0,1",
        "--excal-controller-role", "right",
    };
    auto argv = make_argv(argv_buf);
    apply_cli_overrides(opts, static_cast<int>(argv.size()), argv.data());
    check(opts.excal_faces == "0,1", "--excal-faces CLI overrides YAML");
    check(opts.excal_controller_role == "right", "--excal-controller-role CLI overrides YAML");

    // Valid: has intrinsics. Required cam0/engines must be present too.
    opts.cam_paths[0] = "/tmp/a";
    opts.det_engine   = "/tmp/y";
    opts.pose_engine  = "/tmp/r";
    validate_options(opts);

    // Mutually exclusive with --calibrate. Satisfy --calibrate's own
    // prerequisites so validation reaches the exclusivity check.
    opts.calibrate = true;
    opts.enable_3d = true;
    opts.calib = "/tmp/cam.yaml";
    opts.subject_id = "subj";
    opts.subject_height_m = 1.7;
    bool threw = false;
    try { validate_options(opts); }
    catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "--extrinsic-calib", "excal+calibrate exclusivity msg");
    }
    check(threw, "--extrinsic-calib + --calibrate must throw");
}

void test_run_mode_derivation_and_publisher_exclusivity() {
    using fitra::config::RunMode;
    using fitra::config::run_mode;
    using fitra::config::run_mode_name;

    MainOptions opts;
    opts.cam_paths[0] = "/tmp/a";
    opts.det_engine   = "/tmp/y";
    opts.pose_engine  = "/tmp/r";
    validate_options(opts);
    check(run_mode(opts) == RunMode::Run, "no calib flags -> run mode");
    check(std::string(run_mode_name(RunMode::Run)) == "run", "run label");

    MainOptions subj = opts;
    subj.calibrate = true;
    subj.enable_3d = true;
    subj.calib = "/tmp/cam.yaml";
    subj.subject_id = "subj";
    subj.subject_height_m = 1.7;
    validate_options(subj);
    check(run_mode(subj) == RunMode::CalibSubject,
          "--calibrate -> calib-subject mode");
    check(std::string(run_mode_name(RunMode::CalibSubject)) == "calib-subject",
          "calib-subject label");

    MainOptions excal = opts;
    excal.excal_enabled = true;
    excal.excal_intrinsics = "/tmp/intr.yaml";
    validate_options(excal);
    check(run_mode(excal) == RunMode::CalibExtrinsic,
          "--extrinsic-calib -> calib-extrinsic mode");
    check(std::string(run_mode_name(RunMode::CalibExtrinsic)) == "calib-extrinsic",
          "calib-extrinsic label");

    // Setup modes never construct publishers; validate rejects the combos.
    MainOptions bad = excal;
    bad.enable_3d = true;
    bad.calib = "/tmp/cam.yaml";
    bad.keypoint_format = "halpe26";
    bad.slimevr_out = true;
    bool threw = false;
    try { validate_options(bad); }
    catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "--slimevr-out cannot be combined with a calibration mode",
                       "slimevr+excal exclusivity msg");
    }
    check(threw, "--slimevr-out + --extrinsic-calib must throw");

    bad.slimevr_out = false;
    bad.vmt_out = true;
    threw = false;
    try { validate_options(bad); }
    catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "--vmt-out cannot be combined with a calibration mode",
                       "vmt+excal exclusivity msg");
    }
    check(threw, "--vmt-out + --extrinsic-calib must throw");

    // The exclusivity now covers the floor + intrinsic calib modes too.
    MainOptions floorbad;
    floorbad.cam_paths[0] = "/dev/null";
    floorbad.floor_calib_enabled = true;
    floorbad.floor_map = "/tmp/m.yaml";
    floorbad.calib = "/tmp/cam.yaml";
    floorbad.enable_3d = true;
    floorbad.keypoint_format = "halpe26";
    floorbad.vmt_out = true;
    threw = false;
    try { validate_options(floorbad); }
    catch (const std::exception&) { threw = true; }
    check(threw, "--vmt-out + --floor-calib must throw");
}

void test_excal_replay_yaml_cli_and_mode() {
    using fitra::config::RunMode;
    using fitra::config::run_mode;

    auto p = write_tmp("excal_replay.yaml", R"(schema: fitra_main_config_v1
extrinsic_calib:
  replay_dir: /tmp/session
  intrinsics: /tmp/intr.yaml
)");
    MainOptions opts;
    load_main_config(p.string(), opts);
    check(opts.excal_replay == "/tmp/session", "extrinsic_calib.replay_dir loads");

    // CLI flag + mode derivation. Replay validates with no cameras and no
    // TRT engines — the session brings its own frames.
    MainOptions o2;
    std::vector<std::string> argv_buf{"--excal-replay", "/tmp/sess2",
                                      "--excal-intrinsics", "/tmp/intr.yaml"};
    auto argv = make_argv(argv_buf);
    apply_cli_overrides(o2, static_cast<int>(argv.size()), argv.data());
    check(o2.excal_replay == "/tmp/sess2", "--excal-replay CLI sets value");
    validate_options(o2);  // must not throw
    check(run_mode(o2) == RunMode::CalibExtrinsic,
          "--excal-replay alone -> calib-extrinsic mode");

    // Still exclusive with --calibrate.
    o2.calibrate = true;
    o2.enable_3d = true;
    o2.calib = "/tmp/cam.yaml";
    o2.subject_id = "s";
    o2.subject_height_m = 1.7;
    bool threw = false;
    try { validate_options(o2); }
    catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "--excal-replay", "replay+calibrate exclusivity msg");
    }
    check(threw, "--excal-replay + --calibrate must throw");
}

void test_floor_calib_yaml_cli_and_mode() {
    using fitra::config::RunMode;
    using fitra::config::parse_run_mode_name;
    using fitra::config::run_mode;
    using fitra::config::run_mode_name;

    // YAML: method: floor sets the daemon-only selector (excal_method), NOT the
    // run_mode flag — so a shared daemon config does not derive a calib mode in
    // the parent / run child. floor_* keys load.
    auto p = write_tmp("floor_calib.yaml", R"(schema: fitra_main_config_v1
extrinsic_calib:
  method: floor
  floor_map: /tmp/floor_map.yaml
  floor_intrinsics: /tmp/intr.yaml
  floor_fisheye: true
  out: /tmp/extr.yaml
)");
    MainOptions opts;
    load_main_config(p.string(), opts);
    check(opts.excal_method == "floor", "method: floor sets excal_method");
    check(!opts.floor_calib_enabled, "method: floor does NOT set the run_mode flag");
    check(opts.floor_map == "/tmp/floor_map.yaml", "floor_map loads");
    check(opts.floor_fisheye, "floor_fisheye loads");
    check(opts.floor_out == "/tmp/extr.yaml", "floor shares extrinsic_calib.out");
    check(run_mode(opts) == RunMode::Run,
          "method: floor alone stays run-mode (daemon injects --floor-calib)");
    // --floor-calib (CLI / module_argv) is what selects the floor run mode.
    opts.floor_calib_enabled = true;
    check(run_mode(opts) == RunMode::CalibExtrinsicFloor,
          "--floor-calib -> calib-extrinsic-floor mode");

    // CLI replay path: validates with no cameras (replay brings frames), but
    // still needs --floor-map and intrinsics.
    MainOptions o2;
    std::vector<std::string> argv_buf{"--floor-replay", "/tmp/sess",
                                      "--floor-map", "/tmp/m.yaml",
                                      "--floor-intrinsics", "/tmp/intr.yaml"};
    auto argv = make_argv(argv_buf);
    apply_cli_overrides(o2, static_cast<int>(argv.size()), argv.data());
    check(run_mode(o2) == RunMode::CalibExtrinsicFloor,
          "--floor-replay -> calib-extrinsic-floor mode");
    validate_options(o2);  // must not throw

    check(std::string(run_mode_name(RunMode::CalibExtrinsicFloor)) ==
              "calib-extrinsic-floor", "calib-extrinsic-floor label");
    RunMode m;
    check(parse_run_mode_name("calib-extrinsic-floor", m) &&
              m == RunMode::CalibExtrinsicFloor, "parse calib-extrinsic-floor");

    // --floor-map missing must throw.
    MainOptions o3;
    o3.floor_calib_enabled = true;
    o3.cam_paths[0] = "/dev/null";
    o3.floor_intrinsics = "/tmp/intr.yaml";
    bool threw = false;
    try { validate_options(o3); }
    catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "--floor-map", "floor missing map msg");
    }
    check(threw, "floor without --floor-map must throw");
}

void test_intrinsic_calib_yaml_cli_and_mode() {
    using fitra::config::RunMode;
    using fitra::config::run_mode;
    using fitra::config::run_mode_name;
    using fitra::config::parse_run_mode_name;

    auto p = write_tmp("intrinsic_calib.yaml", R"(schema: fitra_main_config_v1
intrinsic_calib:
  model: fisheye
  out: /tmp/intr_hires.yaml
  squares_x: 5
  squares_y: 7
  square_len_m: 0.04
  marker_len_m: 0.03
  enabled: true
  min_views: 15
  max_rms_px: 2.0
)");
    MainOptions opts;
    load_main_config(p.string(), opts);
    check(opts.intrinsic_model == "fisheye", "intrinsic_calib.model loads");
    check(opts.intrinsic_out == "/tmp/intr_hires.yaml", "intrinsic_calib.out loads");
    check(opts.intrinsic_min_views == 15, "intrinsic_calib.min_views loads");
    // Regression: max_rms_px must be in the check_keys allow-list, else a config
    // using the documented gate key fails with "unknown key" before parsing.
    check(opts.intrinsic_max_rms_px == 2.0, "intrinsic_calib.max_rms_px loads");
    // enabled sets the daemon-only step selector, NOT the run_mode flag.
    check(opts.intrinsic_step_enabled, "intrinsic_calib.enabled sets intrinsic_step_enabled");
    check(!opts.intrinsic_calib_enabled, "intrinsic_calib.enabled does NOT set the run_mode flag");
    check(run_mode(opts) == RunMode::Run, "intrinsic enabled alone stays run-mode");
    // CLI --calib-intrinsic selects the mode; replay validates without cameras.
    MainOptions o2;
    std::vector<std::string> argv_buf{"--intrinsic-replay", "/tmp/isess",
                                      "--intrinsic-model", "pinhole"};
    auto argv = make_argv(argv_buf);
    apply_cli_overrides(o2, static_cast<int>(argv.size()), argv.data());
    check(run_mode(o2) == RunMode::CalibIntrinsic,
          "--intrinsic-replay -> calib-intrinsic mode");
    validate_options(o2);  // must not throw

    check(std::string(run_mode_name(RunMode::CalibIntrinsic)) == "calib-intrinsic",
          "calib-intrinsic label");
    RunMode m;
    check(parse_run_mode_name("calib-intrinsic", m) && m == RunMode::CalibIntrinsic,
          "parse calib-intrinsic");

    // Bad board (marker >= square) must throw.
    MainOptions o3;
    o3.intrinsic_calib_enabled = true;
    o3.cam_paths[0] = "/dev/null";
    o3.charuco_marker_len_m = 0.05;
    o3.charuco_square_len_m = 0.04;
    bool threw = false;
    try { validate_options(o3); }
    catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "marker-len", "intrinsic board validation msg");
    }
    check(threw, "calib-intrinsic bad board must throw");

    // Intrinsic takes precedence over extrinsic when both run_mode flags set.
    MainOptions o4;
    o4.intrinsic_calib_enabled = true;
    o4.floor_calib_enabled = true;
    check(run_mode(o4) == RunMode::CalibIntrinsic,
          "intrinsic precedence over floor");

    // Regression (Codex P1): a shared daemon config that selects floor + the
    // intrinsic step must NOT make --daemon fail. The selectors are excal_method
    // / intrinsic_step_enabled, which leave run_mode == Run so validate passes.
    MainOptions d;
    d.cam_paths[0] = "/tmp/a";
    d.det_engine = "/tmp/y";
    d.pose_engine = "/tmp/r";
    d.daemon = true;
    d.excal_method = "floor";
    d.intrinsic_step_enabled = true;
    check(run_mode(d) == RunMode::Run, "daemon parent with floor+intrinsic stays run-mode");
    validate_options(d);  // must not throw (--daemon + Run)
}

void test_precheck_mode_switch() {
    using fitra::config::MainOptions;
    using fitra::config::RunMode;
    using fitra::config::precheck_mode_switch;

    auto intr = write_tmp("precheck_intr.yaml", "schema: x\n");
    auto mapf = write_tmp("precheck_map.yaml", "%YAML:1.0\n");
    std::string err;

    // Floor: missing floor_map → refused with a floor_map reason.
    MainOptions o;
    o.calib = intr.string();
    err.clear();
    check(!precheck_mode_switch(o, RunMode::CalibExtrinsicFloor, err),
          "floor switch refused without floor_map");
    check_contains(err, "floor_map", "floor precheck names floor_map");

    // Floor: floor_map points at a missing file → refused with "not found".
    o.floor_map = "/no/such/floor_map.yaml";
    err.clear();
    check(!precheck_mode_switch(o, RunMode::CalibExtrinsicFloor, err),
          "floor switch refused on missing map file");
    check_contains(err, "not found", "floor precheck reports missing file");

    // Floor: map present + PnP intrinsics via three_d.calib → allowed.
    o.floor_map = mapf.string();
    err.clear();
    check(precheck_mode_switch(o, RunMode::CalibExtrinsicFloor, err),
          "floor switch allowed with map + calib intrinsics");

    // Floor: floor_intrinsics set but missing → refused.
    o.floor_intrinsics = "/no/such/intr.yaml";
    err.clear();
    check(!precheck_mode_switch(o, RunMode::CalibExtrinsicFloor, err),
          "floor switch refused on missing floor_intrinsics");

    // Controller: needs intrinsics (excal_intrinsics or calib).
    MainOptions c;
    err.clear();
    check(!precheck_mode_switch(c, RunMode::CalibExtrinsic, err),
          "controller switch refused without intrinsics");
    c.calib = intr.string();
    err.clear();
    check(precheck_mode_switch(c, RunMode::CalibExtrinsic, err),
          "controller switch allowed with calib");

    // Run is always reachable (safe fallback, tolerates missing calib).
    MainOptions r;
    err.clear();
    check(precheck_mode_switch(r, RunMode::Run, err), "run switch always allowed");

    std::remove(intr.string().c_str());
    std::remove(mapf.string().c_str());
}

void test_flow_managed_and_publisher_negation() {
    using fitra::config::RunMode;
    using fitra::config::parse_run_mode_name;

    // The daemon's calib-spawn scenario: a union --config YAML carries the
    // run-mode publisher settings; the spawn argv negates them so the setup
    // mode passes the publisher-exclusivity validation.
    auto p = write_tmp("flow_union.yaml", R"(schema: fitra_main_config_v1
cameras:
  cam0: /dev/v4l/by-path/cam-A
  cam1: /dev/v4l/by-path/cam-B
inference:
  det_engine: /tmp/yolox.engine
  pose_engine: /tmp/rtmpose.engine
  keypoint_format: halpe26
three_d:
  enable_3d: true
  calib: /tmp/extrinsics.yaml
vmt:
  vmt_out: true
slimevr:
  slimevr_out: true
)");
    MainOptions opts;
    load_main_config(p.string(), opts);
    std::vector<std::string> argv_buf{
        "--flow-managed", "--calibrate", "--calib-auto-exit",
        "--no-vmt-out", "--no-slimevr-out",
        "--calib-subject-id", "subj", "--calib-subject-height-m", "1.7",
    };
    auto argv = make_argv(argv_buf);
    apply_cli_overrides(opts, static_cast<int>(argv.size()), argv.data());
    check(opts.flow_managed,  "--flow-managed sets flow_managed");
    check(!opts.vmt_out,      "--no-vmt-out negates vmt.vmt_out from YAML");
    check(!opts.slimevr_out,  "--no-slimevr-out negates slimevr.slimevr_out from YAML");
    validate_options(opts);   // must not throw (publishers negated)
    check(fitra::config::run_mode(opts) == RunMode::CalibSubject,
          "union YAML + --calibrate -> calib-subject mode");

    // parse_run_mode_name is the /api/flow/switch + --daemon-initial parser.
    RunMode m;
    check(parse_run_mode_name("run", m) && m == RunMode::Run, "parse run");
    check(parse_run_mode_name("calib-subject", m) && m == RunMode::CalibSubject,
          "parse calib-subject");
    check(parse_run_mode_name("calib-extrinsic", m) && m == RunMode::CalibExtrinsic,
          "parse calib-extrinsic");
    check(!parse_run_mode_name("bogus", m), "parse rejects unknown label");
    check(!parse_run_mode_name("", m),      "parse rejects empty label");
}

void test_daemon_flags_and_validate() {
    // Run-shaped base opts (the daemon validates the union YAML up front).
    MainOptions opts;
    opts.cam_paths[0] = "/tmp/a";
    opts.det_engine   = "/tmp/y";
    opts.pose_engine  = "/tmp/r";

    std::vector<std::string> argv_buf{"--daemon", "--daemon-initial",
                                      "calib-extrinsic"};
    auto argv = make_argv(argv_buf);
    apply_cli_overrides(opts, static_cast<int>(argv.size()), argv.data());
    check(opts.daemon, "--daemon sets daemon");
    check(opts.daemon_initial == "calib-extrinsic", "--daemon-initial sets value");
    validate_options(opts);  // must not throw

    // The daemon picks modes itself — explicit mode flags are rejected.
    MainOptions bad = opts;
    bad.excal_enabled = true;
    bad.excal_intrinsics = "/tmp/intr.yaml";
    bool threw = false;
    try { validate_options(bad); }
    catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "--daemon", "daemon+excal exclusivity msg");
    }
    check(threw, "--daemon + --extrinsic-calib must throw");

    bad = opts;
    bad.flow_managed = true;
    threw = false;
    try { validate_options(bad); }
    catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "--flow-managed", "daemon+managed exclusivity msg");
    }
    check(threw, "--daemon + --flow-managed must throw");

    bad = opts;
    bad.daemon_initial = "bogus";
    threw = false;
    try { validate_options(bad); }
    catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "--daemon-initial", "daemon-initial range msg");
    }
    check(threw, "--daemon-initial bogus must throw");
}

void test_setup_mode_and_daemon_blank_config() {
    using fitra::config::RunMode;
    using fitra::config::run_mode;
    using fitra::config::run_mode_name;
    using fitra::config::parse_run_mode_name;

    // --setup -> RunMode::Setup, validates with NO cameras/engines (it is the
    // module that configures them). Only the port must be sane.
    MainOptions opts;
    std::vector<std::string> argv_buf{"--setup"};
    auto argv = make_argv(argv_buf);
    apply_cli_overrides(opts, static_cast<int>(argv.size()), argv.data());
    check(opts.setup_mode, "--setup sets setup_mode");
    check(run_mode(opts) == RunMode::Setup, "--setup -> Setup mode");
    validate_options(opts);  // must not throw despite empty cam_paths/engines

    // Label round-trips both ways.
    check(std::string(run_mode_name(RunMode::Setup)) == "setup", "setup label");
    RunMode m;
    check(parse_run_mode_name("setup", m) && m == RunMode::Setup, "parse setup label");

    // A bad port is still rejected in setup mode.
    MainOptions badport = opts;
    badport.port = 0;
    bool threw = false;
    try { validate_options(badport); }
    catch (const std::exception&) { threw = true; }
    check(threw, "setup mode rejects an invalid port");

    // The daemon parent validates its union config in run form, but a first-run
    // config legitimately has no cameras/engines yet (the Setup module fills
    // them in). --daemon must NOT reject a blank config.
    MainOptions daemon_blank;
    std::vector<std::string> dargv_buf{"--daemon"};
    auto dargv = make_argv(dargv_buf);
    apply_cli_overrides(daemon_blank, static_cast<int>(dargv.size()), dargv.data());
    check(daemon_blank.daemon && daemon_blank.cam_paths[0].empty(),
          "daemon set, no cameras");
    validate_options(daemon_blank);  // must not throw

    // But a NON-daemon, non-setup blank config (a bare ./main) still fails the
    // run-form requirement.
    MainOptions bare;
    threw = false;
    try { validate_options(bare); }
    catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "required option", "bare run-form msg");
    }
    check(threw, "bare ./main still requires --cam0 + engines");

    // --daemon-initial setup is accepted.
    MainOptions di;
    std::vector<std::string> diargv_buf{"--daemon", "--daemon-initial", "setup"};
    auto diargv = make_argv(diargv_buf);
    apply_cli_overrides(di, static_cast<int>(diargv.size()), diargv.data());
    validate_options(di);  // must not throw
    check(di.daemon_initial == "setup", "--daemon-initial setup accepted");
}

void test_emit_load_round_trip() {
    using fitra::config::emit_main_config;
    using fitra::config::save_main_config;

    // A representative union config: varied non-default values across every
    // section, including the tricky ones (negated kalman/ik, bare slimevr/vmt
    // host/port, excal_method, intrinsic step selector, per-camera arrays).
    // excal_enabled / replay dirs stay at defaults — those are deliberately not
    // emitted (run-mode-deriving). calibrate is set true below to prove it is
    // dropped on emit while the persistent calibration knobs round-trip.
    MainOptions o;
    o.cam_paths[0] = "/dev/v4l/by-path/cam-A";
    o.cam_paths[2] = "/dev/v4l/by-path/cam-C";
    o.width = 1280; o.height = 960; o.fps = 60;
    o.pixel_format = "yuyv"; o.n_buffers = 6;
    o.cam_cap_width[1] = 1280; o.cam_cap_height[1] = 960;
    o.cam_pixel_format[2] = "nvjpeg";
    o.cam_exposure_mode[0] = "manual"; o.cam_exposure[0] = 120; o.cam_gain[0] = 8;
    o.cam_ae_target[1] = 95;
    o.det_engine = "/tmp/yolox.engine"; o.pose_engine = "/tmp/rtmpose.engine";
    o.det_frequency = 5; o.det_score = 0.42f; o.keypoint_format = "halpe26";
    o.multi_person = true;
    o.host = "127.0.0.1"; o.port = 8123;
    o.enable_3d = true; o.calib = "calibrations/extrinsics.yaml";
    o.kp_conf_thresh = 0.25f; o.max_reproj_px = 5.5f; o.sync_window_ms = 12.0;
    o.bone_calib_frames = 200;
    o.kalman_3d = false;   // -> no_3d_kalman: true
    o.ik_3d = false;       // -> no_3d_ik: true
    o.vr_extract_event_driven = true; o.vr_one_euro = false;
    o.vr_pos_mincutoff = 2.0; o.vr_pos_beta = 6.0; o.vr_quat_beta = 2.5;
    o.subjects_dir = "calibrations/subjects"; o.subject_id = "alice";
    o.subject_height_m = 1.72;
    o.log_every_s = 1.0;
    o.slimevr_out = true; o.slimevr_host = "172.34.1.9"; o.slimevr_port = 6970;
    o.slimevr_rate_hz = 75.0; o.slimevr_quat_smooth = 0.4;
    o.vmt_out = true; o.vmt_host = "172.34.1.9"; o.vmt_port = 39571;
    o.vmt_index_base = 12; o.vmt_degeneracy_mode = "disable";
    o.vmt_discovery = false; o.vmt_pair_id = "rig-A"; o.vmt_pairing_token = "tok9";
    o.vmt_discovery_group = "239.255.42.7"; o.vmt_discovery_port = 39581;
    o.vmt_instance_name = "Bench Rig"; o.vmt_peer_timeout_s = 8.0;
    o.hmd_listen_enabled = true; o.hmd_listen_port = 39572;
    o.vmt_continuous_align = false; o.vmt_continuous_blend = 0.3;
    o.excal_intrinsics = "calibrations/intrinsics.yaml";
    o.excal_out = "calibrations/extrinsics.yaml";
    o.excal_method = "floor"; o.excal_tag_size_m = 0.12; o.excal_min_samples = 10;
    o.floor_map = "configs/floor_tag_map.yaml"; o.floor_fisheye = true;
    o.floor_out = o.excal_out;  // loader couples floor_out := excal_out via `out`
    o.intrinsic_step_enabled = true; o.intrinsic_out = "calibrations/intrinsics.yaml";
    o.intrinsic_model = "fisheye"; o.charuco_squares_x = 7; o.charuco_squares_y = 5;
    o.charuco_square_len_m = 0.035; o.charuco_marker_len_m = 0.026;
    o.intrinsic_min_views = 15; o.intrinsic_max_rms_px = 1.2;
    // subject_calib block: calibrate is run-mode-deriving (must NOT emit), the
    // process knobs must round-trip (subject id/height live in `subject:` above).
    o.calibrate = true;
    o.calib_frames_per_cam = 90; o.calib_hold_sec = 2.0;
    o.calib_auto_approve = true; o.calib_auto_exit = true;
    o.calib_static_dir = "web-ui/dist/subject-calib";
    o.calib_dump_tool = "/tmp/dump_keypoints_3d";

    auto p = write_tmp("round_trip.yaml", "");
    save_main_config(p.string(), o);

    MainOptions r;  // fresh defaults
    load_main_config(p.string(), r);

    // The emitted document must itself be valid + accepted by the loader.
    // Compare every loader-visible field.
    auto eq_s = [](const std::string& a, const std::string& b, const char* k) {
        check(a == b, std::string("round-trip string ") + k + ": '" + a + "' vs '" + b + "'");
    };
    auto eq_i = [](long a, long b, const char* k) {
        check(a == b, std::string("round-trip int ") + k);
    };
    auto eq_b = [](bool a, bool b, const char* k) {
        check(a == b, std::string("round-trip bool ") + k);
    };
    auto eq_f = [](double a, double b, const char* k) {
        check(std::abs(a - b) < 1e-6, std::string("round-trip float ") + k);
    };

    for (int i = 0; i < 3; ++i) {
        eq_s(o.cam_paths[i], r.cam_paths[i], "cam_path");
        eq_i(o.cam_cap_width[i],  r.cam_cap_width[i],  "cam_cap_width");
        eq_i(o.cam_cap_height[i], r.cam_cap_height[i], "cam_cap_height");
        eq_s(o.cam_pixel_format[i],  r.cam_pixel_format[i],  "cam_pixel_format");
        eq_s(o.cam_exposure_mode[i], r.cam_exposure_mode[i], "cam_exposure_mode");
        eq_i(o.cam_exposure[i],  r.cam_exposure[i],  "cam_exposure");
        eq_i(o.cam_gain[i],      r.cam_gain[i],      "cam_gain");
        eq_i(o.cam_ae_target[i], r.cam_ae_target[i], "cam_ae_target");
    }
    eq_i(o.width, r.width, "width"); eq_i(o.height, r.height, "height");
    eq_i(o.fps, r.fps, "fps"); eq_s(o.pixel_format, r.pixel_format, "pixel_format");
    eq_i(o.n_buffers, r.n_buffers, "n_buffers");
    eq_s(o.det_engine, r.det_engine, "det_engine");
    eq_s(o.pose_engine, r.pose_engine, "pose_engine");
    eq_i(o.det_frequency, r.det_frequency, "det_frequency");
    eq_f(o.det_score, r.det_score, "det_score");
    eq_s(o.keypoint_format, r.keypoint_format, "keypoint_format");
    eq_b(o.multi_person, r.multi_person, "multi_person");
    eq_s(o.host, r.host, "host"); eq_i(o.port, r.port, "port");
    eq_b(o.enable_3d, r.enable_3d, "enable_3d"); eq_s(o.calib, r.calib, "calib");
    eq_f(o.kp_conf_thresh, r.kp_conf_thresh, "kp_conf_thresh");
    eq_f(o.max_reproj_px, r.max_reproj_px, "max_reproj_px");
    eq_f(o.sync_window_ms, r.sync_window_ms, "sync_window_ms");
    eq_i(o.bone_calib_frames, r.bone_calib_frames, "bone_calib_frames");
    eq_b(o.kalman_3d, r.kalman_3d, "kalman_3d (negated key)");
    eq_b(o.ik_3d, r.ik_3d, "ik_3d (negated key)");
    eq_b(o.vr_extract_event_driven, r.vr_extract_event_driven, "vr_extract_event_driven");
    eq_b(o.vr_one_euro, r.vr_one_euro, "vr_one_euro");
    eq_f(o.vr_pos_mincutoff, r.vr_pos_mincutoff, "vr_pos_mincutoff");
    eq_f(o.vr_pos_beta, r.vr_pos_beta, "vr_pos_beta");
    eq_f(o.vr_quat_beta, r.vr_quat_beta, "vr_quat_beta");
    eq_s(o.subjects_dir, r.subjects_dir, "subjects_dir");
    eq_s(o.subject_id, r.subject_id, "subject_id");
    eq_f(o.subject_height_m, r.subject_height_m, "subject_height_m");
    eq_f(o.log_every_s, r.log_every_s, "log_every_s");
    eq_b(o.slimevr_out, r.slimevr_out, "slimevr_out");
    eq_s(o.slimevr_host, r.slimevr_host, "slimevr_host");
    eq_i(o.slimevr_port, r.slimevr_port, "slimevr_port");
    eq_f(o.slimevr_rate_hz, r.slimevr_rate_hz, "slimevr_rate_hz");
    eq_f(o.slimevr_quat_smooth, r.slimevr_quat_smooth, "slimevr_quat_smooth");
    eq_b(o.vmt_out, r.vmt_out, "vmt_out");
    eq_s(o.vmt_host, r.vmt_host, "vmt_host");
    eq_i(o.vmt_port, r.vmt_port, "vmt_port");
    eq_i(o.vmt_index_base, r.vmt_index_base, "vmt_index_base");
    eq_s(o.vmt_degeneracy_mode, r.vmt_degeneracy_mode, "vmt_degeneracy_mode");
    eq_b(o.vmt_discovery, r.vmt_discovery, "vmt_discovery");
    eq_s(o.vmt_pair_id, r.vmt_pair_id, "vmt_pair_id");
    eq_s(o.vmt_pairing_token, r.vmt_pairing_token, "vmt_pairing_token");
    eq_s(o.vmt_discovery_group, r.vmt_discovery_group, "vmt_discovery_group");
    eq_i(o.vmt_discovery_port, r.vmt_discovery_port, "vmt_discovery_port");
    eq_s(o.vmt_instance_name, r.vmt_instance_name, "vmt_instance_name");
    eq_f(o.vmt_peer_timeout_s, r.vmt_peer_timeout_s, "vmt_peer_timeout_s");
    eq_b(o.hmd_listen_enabled, r.hmd_listen_enabled, "hmd_listen_enabled");
    eq_i(o.hmd_listen_port, r.hmd_listen_port, "hmd_listen_port");
    eq_b(o.vmt_continuous_align, r.vmt_continuous_align, "vmt_continuous_align");
    eq_f(o.vmt_continuous_blend, r.vmt_continuous_blend, "vmt_continuous_blend");
    eq_s(o.excal_intrinsics, r.excal_intrinsics, "excal_intrinsics");
    eq_s(o.excal_out, r.excal_out, "excal_out");
    eq_s(o.excal_method, r.excal_method, "excal_method");
    eq_f(o.excal_tag_size_m, r.excal_tag_size_m, "excal_tag_size_m");
    eq_i(o.excal_min_samples, r.excal_min_samples, "excal_min_samples");
    eq_s(o.floor_map, r.floor_map, "floor_map");
    eq_s(o.floor_out, r.floor_out, "floor_out (coupled to excal_out)");
    eq_b(o.floor_fisheye, r.floor_fisheye, "floor_fisheye");
    eq_b(o.intrinsic_step_enabled, r.intrinsic_step_enabled, "intrinsic_step_enabled");
    eq_s(o.intrinsic_out, r.intrinsic_out, "intrinsic_out");
    eq_s(o.intrinsic_model, r.intrinsic_model, "intrinsic_model");
    eq_i(o.charuco_squares_x, r.charuco_squares_x, "charuco_squares_x");
    eq_i(o.charuco_squares_y, r.charuco_squares_y, "charuco_squares_y");
    eq_f(o.charuco_square_len_m, r.charuco_square_len_m, "charuco_square_len_m");
    eq_f(o.charuco_marker_len_m, r.charuco_marker_len_m, "charuco_marker_len_m");
    eq_i(o.intrinsic_min_views, r.intrinsic_min_views, "intrinsic_min_views");
    eq_f(o.intrinsic_max_rms_px, r.intrinsic_max_rms_px, "intrinsic_max_rms_px");
    eq_i(o.calib_frames_per_cam, r.calib_frames_per_cam, "calib_frames_per_cam");
    eq_f(o.calib_hold_sec, r.calib_hold_sec, "calib_hold_sec");
    eq_b(o.calib_auto_approve, r.calib_auto_approve, "calib_auto_approve");
    eq_b(o.calib_auto_exit, r.calib_auto_exit, "calib_auto_exit");
    eq_s(o.calib_static_dir, r.calib_static_dir, "calib_static_dir");
    eq_s(o.calib_dump_tool, r.calib_dump_tool, "calib_dump_tool");

    // Run-mode-deriving flags must NOT have been emitted: the reloaded config is
    // a clean union config (no calib mode derived) even though o.calibrate=true.
    check(!r.calibrate && !r.excal_enabled && r.excal_replay.empty()
          && r.intrinsic_replay.empty(),
          "emitted config carries no run-mode-deriving flags");
    validate_options(r);  // emitted config must be runnable-as-loaded
}

void test_absolutize_paths() {
    using fitra::config::absolutize_config_paths;
    MainOptions o;
    o.det_engine  = "outputs/y.engine";   // relative -> absolutized
    o.pose_engine = "/abs/r.engine";      // already absolute -> unchanged
    o.calib       = "";                   // empty -> stays empty
    o.intrinsic_out = "calibrations/intrinsics.yaml";
    absolutize_config_paths(o);
    check(!o.det_engine.empty() && o.det_engine.front() == '/',
          "relative det_engine becomes absolute");
    check(o.det_engine.find("outputs/y.engine") != std::string::npos,
          "absolutized det_engine keeps the relative tail");
    check(o.pose_engine == "/abs/r.engine", "absolute pose_engine unchanged");
    check(o.calib.empty(), "empty calib stays empty");
    check(!o.intrinsic_out.empty() && o.intrinsic_out.front() == '/',
          "relative intrinsic_out becomes absolute");
}

void test_validate_rejects_duplicate_cameras() {
    MainOptions o;
    o.cam_paths[0] = "/dev/v4l/by-path/cam-A";
    o.cam_paths[1] = "/dev/v4l/by-path/cam-A";  // same device in two slots
    o.det_engine = "/tmp/y.engine";
    o.pose_engine = "/tmp/r.engine";
    bool threw = false;
    try { validate_options(o); } catch (const std::exception&) { threw = true; }
    check(threw, "duplicate cam_paths must be rejected");
    o.cam_paths[1] = "/dev/v4l/by-path/cam-B";  // distinct now
    validate_options(o);                        // must not throw
}

void test_setup_store_refuses_example_path() {
    using fitra::config::SetupConfigStore;
    MainOptions seed;
    seed.cam_paths[0] = "/dev/v4l/by-path/cam-A";
    seed.det_engine = "/tmp/y.engine";
    seed.pose_engine = "/tmp/r.engine";
    {
        SetupConfigStore store{seed, ""};  // launched without --config
        std::string err;
        check(!store.write_union(err), "write_union must refuse an empty path");
        check(err.find("--config") != std::string::npos,
              "empty-path error mentions --config");
    }
    {
        SetupConfigStore store{seed, "configs/foo.yaml.example"};
        std::string err;
        check(!store.write_union(err), "write_union must refuse a .example path");
        check(err.find(".example") != std::string::npos
              || err.find("template") != std::string::npos,
              "refusal error names the template");
    }
    {
        auto dir = std::filesystem::temp_directory_path() / "fitra_test_main_config";
        std::filesystem::create_directories(dir);
        const std::string out = (dir / "session.yaml").string();
        std::filesystem::remove(out);
        SetupConfigStore store{seed, out};
        std::string err;
        check(store.write_union(err), "write_union to a .yaml path succeeds: " + err);
        check(std::filesystem::exists(out), "write_union created the runtime config");
    }
}

void test_subject_calib_schema() {
    // Canonical: subject.* is the single source of identity; subject_calib: holds
    // the (de-prefixed) process knobs.
    auto p = write_tmp("subj_canonical.yaml", R"(schema: fitra_main_config_v1
subject:
  subject_id: subjectX
  subject_height_m: 1.66
subject_calib:
  frames_per_cam: 90
  hold_sec: 2.0
  auto_exit: true
)");
    MainOptions a;
    load_main_config(p.string(), a);
    check(a.subject_id == "subjectX", "subject.subject_id loads");
    check(std::abs(a.subject_height_m - 1.66) < 1e-6, "subject.subject_height_m loads");
    check(a.calib_frames_per_cam == 90, "subject_calib.frames_per_cam loads");
    check(std::abs(a.calib_hold_sec - 2.0) < 1e-6, "subject_calib.hold_sec loads");
    check(a.calib_auto_exit, "subject_calib.auto_exit loads");

    // Deprecated calibration: block still loads — calib_subject_* alias to
    // subject.*, the rest map to the same process knobs.
    auto q = write_tmp("subj_legacy.yaml", R"(schema: fitra_main_config_v1
calibration:
  calib_subject_id: subjectY
  calib_subject_height_m: 1.80
  calib_frames_per_cam: 50
)");
    MainOptions b;
    load_main_config(q.string(), b);
    check(b.subject_id == "subjectY", "deprecated calib_subject_id aliases to subject_id");
    check(std::abs(b.subject_height_m - 1.80) < 1e-6,
          "deprecated calib_subject_height_m aliases to subject_height_m");
    check(b.calib_frames_per_cam == 50, "deprecated calib_frames_per_cam still loads");

    // Explicit subject.* wins over the deprecated calibration alias.
    auto r = write_tmp("subj_wins.yaml", R"(schema: fitra_main_config_v1
subject:
  subject_id: realSubj
calibration:
  calib_subject_id: legacySubj
)");
    MainOptions c;
    load_main_config(r.string(), c);
    check(c.subject_id == "realSubj", "subject.subject_id wins over deprecated alias");

    // Round-trip emits subject_calib (not calibration) and no duplicate identity.
    MainOptions o;
    o.subject_id = "rtSubj"; o.subject_height_m = 1.72;
    o.calib_frames_per_cam = 42;
    auto rt = write_tmp("subj_rt.yaml", "");
    save_main_config(rt.string(), o);
    MainOptions back;
    load_main_config(rt.string(), back);
    check(back.subject_id == "rtSubj" && std::abs(back.subject_height_m - 1.72) < 1e-6,
          "subject identity round-trips");
    check(back.calib_frames_per_cam == 42, "subject_calib knob round-trips");
}

struct TestCase {
    const char* name;
    void (*fn)();
};

const TestCase kTests[] = {
    {"minimum_config_loads_and_validates",     test_minimum_config_loads_and_validates},
    {"unknown_top_level_key_fails",            test_unknown_top_level_key_fails},
    {"unknown_nested_key_fails",               test_unknown_nested_key_fails},
    {"type_mismatch_fails",                    test_type_mismatch_fails},
    {"missing_schema_fails",                   test_missing_schema_fails},
    {"wrong_schema_fails",                     test_wrong_schema_fails},
    {"cli_overrides_yaml",                     test_cli_overrides_yaml},
    {"negated_three_d_keys_invert_bools",      test_negated_three_d_keys_invert_runtime_bools},
    {"slimevr_preview_no_reset_yaml_and_cli",  test_slimevr_preview_no_reset_yaml_and_cli},
    {"vmt_index_base_yaml_cli_and_validate",   test_vmt_index_base_yaml_cli_and_validate},
    {"vmt_discovery_yaml_cli_and_validate",    test_vmt_discovery_yaml_cli_and_validate},
    {"extrinsic_calib_yaml_cli_and_validate",  test_extrinsic_calib_yaml_cli_and_validate},
    {"run_mode_derivation_and_publisher_exclusivity",
                                               test_run_mode_derivation_and_publisher_exclusivity},
    {"excal_replay_yaml_cli_and_mode",         test_excal_replay_yaml_cli_and_mode},
    {"floor_calib_yaml_cli_and_mode",          test_floor_calib_yaml_cli_and_mode},
    {"intrinsic_calib_yaml_cli_and_mode",      test_intrinsic_calib_yaml_cli_and_mode},
    {"precheck_mode_switch",                   test_precheck_mode_switch},
    {"flow_managed_and_publisher_negation",    test_flow_managed_and_publisher_negation},
    {"daemon_flags_and_validate",              test_daemon_flags_and_validate},
    {"setup_mode_and_daemon_blank_config",     test_setup_mode_and_daemon_blank_config},
    {"emit_load_round_trip",                   test_emit_load_round_trip},
    {"absolutize_paths",                       test_absolutize_paths},
    {"validate_rejects_duplicate_cameras",     test_validate_rejects_duplicate_cameras},
    {"setup_store_refuses_example_path",       test_setup_store_refuses_example_path},
    {"subject_calib_schema",                   test_subject_calib_schema},
    {"one_euro_yaml_cli_and_validate",         test_one_euro_yaml_cli_and_validate},
    {"validate_required_missing",              test_validate_required_missing},
    {"validate_enable_3d_needs_calib",         test_validate_enable_3d_needs_calib},
    {"validate_slimevr_requires_halpe26",      test_validate_slimevr_requires_halpe26},
    {"early_args_extracts_config_and_probe",   test_early_args_extracts_config_and_probe},
    {"early_args_missing_config_value_throws", test_early_args_missing_config_value_throws},
    {"apply_unknown_flag_throws",              test_apply_unknown_flag_throws},
};

}  // namespace

int main() {
    int failed = 0;
    for (const auto& t : kTests) {
        try {
            t.fn();
            std::printf("[PASS] %s\n", t.name);
        } catch (const std::exception& e) {
            std::printf("[FAIL] %s: %s\n", t.name, e.what());
            ++failed;
        }
    }
    if (failed) {
        std::printf("\n%d test(s) failed\n", failed);
        return EXIT_FAILURE;
    }
    std::printf("\nall %zu tests passed\n", sizeof(kTests) / sizeof(kTests[0]));
    return EXIT_SUCCESS;
}
