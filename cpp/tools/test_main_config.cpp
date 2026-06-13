// test_main_config — unit tests for fitra::config::MainOptions YAML loader,
// CLI overlay, and validation. No TensorRT / CUDA dependencies.
//
// Style follows the other tests in this dir (test_firmware_protocol.cpp etc.):
// hand-rolled assertions that throw std::runtime_error on failure; main()
// catches and reports.

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "config/main_config.hpp"

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
    opts.calib_subject_id = "subj";
    opts.calib_subject_height_m = 1.7;
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
    subj.calib_subject_id = "subj";
    subj.calib_subject_height_m = 1.7;
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
        check_contains(e.what(), "--slimevr-out cannot be combined with --extrinsic-calib",
                       "slimevr+excal exclusivity msg");
    }
    check(threw, "--slimevr-out + --extrinsic-calib must throw");

    bad.slimevr_out = false;
    bad.vmt_out = true;
    threw = false;
    try { validate_options(bad); }
    catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "--vmt-out cannot be combined with --extrinsic-calib",
                       "vmt+excal exclusivity msg");
    }
    check(threw, "--vmt-out + --extrinsic-calib must throw");
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
    o2.calib_subject_id = "s";
    o2.calib_subject_height_m = 1.7;
    bool threw = false;
    try { validate_options(o2); }
    catch (const std::exception& e) {
        threw = true;
        check_contains(e.what(), "--excal-replay", "replay+calibrate exclusivity msg");
    }
    check(threw, "--excal-replay + --calibrate must throw");
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
    {"extrinsic_calib_yaml_cli_and_validate",  test_extrinsic_calib_yaml_cli_and_validate},
    {"run_mode_derivation_and_publisher_exclusivity",
                                               test_run_mode_derivation_and_publisher_exclusivity},
    {"excal_replay_yaml_cli_and_mode",         test_excal_replay_yaml_cli_and_mode},
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
