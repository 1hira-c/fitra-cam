#include "config/main_config.hpp"

#include <cstdlib>
#include <cstring>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>

#include <yaml-cpp/yaml.h>

namespace fitra::config {

namespace {

[[noreturn]] void fail(const std::string& msg) {
    throw std::runtime_error("config: " + msg);
}

template <typename T>
T parse_scalar(const YAML::Node& node, const std::string& key_path) {
    try {
        return node.as<T>();
    } catch (const YAML::Exception&) {
        fail("invalid value for " + key_path + " (type mismatch)");
    }
}

void ensure_map(const YAML::Node& node, const std::string& key_path) {
    if (!node.IsMap()) {
        fail("section " + key_path + " must be a YAML map");
    }
}

void check_keys(const YAML::Node& section,
                const std::set<std::string>& allowed,
                const std::string& section_path) {
    for (auto it = section.begin(); it != section.end(); ++it) {
        const auto key = it->first.as<std::string>();
        if (allowed.count(key) == 0) {
            fail("unknown key " + section_path + "." + key);
        }
    }
}

void load_cameras(const YAML::Node& section, MainOptions& out) {
    ensure_map(section, "cameras");
    static const std::set<std::string> allowed{
        "cam0", "cam1", "cam2", "width", "height", "fps",
    };
    check_keys(section, allowed, "cameras");
    if (section["cam0"])   out.cam_paths[0] = parse_scalar<std::string>(section["cam0"],   "cameras.cam0");
    if (section["cam1"])   out.cam_paths[1] = parse_scalar<std::string>(section["cam1"],   "cameras.cam1");
    if (section["cam2"])   out.cam_paths[2] = parse_scalar<std::string>(section["cam2"],   "cameras.cam2");
    if (section["width"])  out.width  = parse_scalar<int>(section["width"],  "cameras.width");
    if (section["height"]) out.height = parse_scalar<int>(section["height"], "cameras.height");
    if (section["fps"])    out.fps    = parse_scalar<int>(section["fps"],    "cameras.fps");
}

void load_inference(const YAML::Node& section, MainOptions& out) {
    ensure_map(section, "inference");
    static const std::set<std::string> allowed{
        "det_engine", "pose_engine", "det_frequency", "det_score",
        "keypoint_format", "multi_person", "bench_fake_bbox",
    };
    check_keys(section, allowed, "inference");
    if (section["det_engine"])      out.det_engine      = parse_scalar<std::string>(section["det_engine"],      "inference.det_engine");
    if (section["pose_engine"])     out.pose_engine     = parse_scalar<std::string>(section["pose_engine"],     "inference.pose_engine");
    if (section["det_frequency"])   out.det_frequency   = parse_scalar<int>(section["det_frequency"],           "inference.det_frequency");
    if (section["det_score"])       out.det_score       = parse_scalar<float>(section["det_score"],             "inference.det_score");
    if (section["keypoint_format"]) out.keypoint_format = parse_scalar<std::string>(section["keypoint_format"], "inference.keypoint_format");
    if (section["multi_person"])    out.multi_person    = parse_scalar<bool>(section["multi_person"],           "inference.multi_person");
    if (section["bench_fake_bbox"]) out.bench_fake_bbox = parse_scalar<bool>(section["bench_fake_bbox"],        "inference.bench_fake_bbox");
}

void load_web(const YAML::Node& section, MainOptions& out) {
    ensure_map(section, "web");
    static const std::set<std::string> allowed{
        "host", "port", "static", "no_web",
    };
    check_keys(section, allowed, "web");
    if (section["host"])   out.host       = parse_scalar<std::string>(section["host"],   "web.host");
    if (section["port"])   out.port       = parse_scalar<int>(section["port"],           "web.port");
    if (section["static"]) out.static_dir = parse_scalar<std::string>(section["static"], "web.static");
    if (section["no_web"]) out.no_web     = parse_scalar<bool>(section["no_web"],        "web.no_web");
}

void load_three_d(const YAML::Node& section, MainOptions& out) {
    ensure_map(section, "three_d");
    static const std::set<std::string> allowed{
        "enable_3d", "calib", "kp_conf_thresh", "max_reproj_px",
        "sync_window_ms", "bone_calib_frames", "no_3d_kalman", "no_3d_ik",
    };
    check_keys(section, allowed, "three_d");
    if (section["enable_3d"])         out.enable_3d         = parse_scalar<bool>(section["enable_3d"],            "three_d.enable_3d");
    if (section["calib"])             out.calib             = parse_scalar<std::string>(section["calib"],         "three_d.calib");
    if (section["kp_conf_thresh"])    out.kp_conf_thresh    = parse_scalar<float>(section["kp_conf_thresh"],      "three_d.kp_conf_thresh");
    if (section["max_reproj_px"])     out.max_reproj_px     = parse_scalar<float>(section["max_reproj_px"],       "three_d.max_reproj_px");
    if (section["sync_window_ms"])    out.sync_window_ms    = parse_scalar<double>(section["sync_window_ms"],     "three_d.sync_window_ms");
    if (section["bone_calib_frames"]) out.bone_calib_frames = parse_scalar<int>(section["bone_calib_frames"],     "three_d.bone_calib_frames");
    // YAML uses CLI-flag-equivalent negated names; flip into the positive
    // runtime predicates kalman_3d / ik_3d.
    if (section["no_3d_kalman"]) {
        out.kalman_3d = !parse_scalar<bool>(section["no_3d_kalman"], "three_d.no_3d_kalman");
    }
    if (section["no_3d_ik"]) {
        out.ik_3d = !parse_scalar<bool>(section["no_3d_ik"], "three_d.no_3d_ik");
    }
}

void load_subject(const YAML::Node& section, MainOptions& out) {
    ensure_map(section, "subject");
    static const std::set<std::string> allowed{
        "subjects_dir", "subject_id", "subject_profile", "subject_height_m",
    };
    check_keys(section, allowed, "subject");
    if (section["subjects_dir"])     out.subjects_dir     = parse_scalar<std::string>(section["subjects_dir"],    "subject.subjects_dir");
    if (section["subject_id"])       out.subject_id       = parse_scalar<std::string>(section["subject_id"],      "subject.subject_id");
    if (section["subject_profile"])  out.subject_profile  = parse_scalar<std::string>(section["subject_profile"], "subject.subject_profile");
    if (section["subject_height_m"]) out.subject_height_m = parse_scalar<double>(section["subject_height_m"],     "subject.subject_height_m");
}

void load_calibration(const YAML::Node& section, MainOptions& out) {
    ensure_map(section, "calibration");
    static const std::set<std::string> allowed{
        "calibrate", "calib_subject_id", "calib_subject_height_m",
        "calib_frames_per_cam", "calib_hold_sec", "calib_auto_approve",
        "calib_auto_exit", "calib_static_dir", "calib_dump_tool",
    };
    check_keys(section, allowed, "calibration");
    if (section["calibrate"])              out.calibrate              = parse_scalar<bool>(section["calibrate"],                 "calibration.calibrate");
    if (section["calib_subject_id"])       out.calib_subject_id       = parse_scalar<std::string>(section["calib_subject_id"],  "calibration.calib_subject_id");
    if (section["calib_subject_height_m"]) out.calib_subject_height_m = parse_scalar<double>(section["calib_subject_height_m"], "calibration.calib_subject_height_m");
    if (section["calib_frames_per_cam"])   out.calib_frames_per_cam   = parse_scalar<int>(section["calib_frames_per_cam"],      "calibration.calib_frames_per_cam");
    if (section["calib_hold_sec"])         out.calib_hold_sec         = parse_scalar<double>(section["calib_hold_sec"],         "calibration.calib_hold_sec");
    if (section["calib_auto_approve"])     out.calib_auto_approve     = parse_scalar<bool>(section["calib_auto_approve"],       "calibration.calib_auto_approve");
    if (section["calib_auto_exit"])        out.calib_auto_exit        = parse_scalar<bool>(section["calib_auto_exit"],          "calibration.calib_auto_exit");
    if (section["calib_static_dir"])       out.calib_static_dir       = parse_scalar<std::string>(section["calib_static_dir"],  "calibration.calib_static_dir");
    if (section["calib_dump_tool"])        out.calib_dump_tool        = parse_scalar<std::string>(section["calib_dump_tool"],   "calibration.calib_dump_tool");
}

void load_logging(const YAML::Node& section, MainOptions& out) {
    ensure_map(section, "logging");
    static const std::set<std::string> allowed{ "log_every_s" };
    check_keys(section, allowed, "logging");
    if (section["log_every_s"]) out.log_every_s = parse_scalar<double>(section["log_every_s"], "logging.log_every_s");
}

void load_slimevr(const YAML::Node& section, MainOptions& out) {
    ensure_map(section, "slimevr");
    static const std::set<std::string> allowed{
        "slimevr_out", "host", "port", "rate_hz", "quat_smooth",
        "preview_no_reset",
    };
    check_keys(section, allowed, "slimevr");
    if (section["slimevr_out"]) {
        out.slimevr_out = parse_scalar<bool>(
            section["slimevr_out"], "slimevr.slimevr_out");
    }
    if (section["host"]) {
        out.slimevr_host = parse_scalar<std::string>(
            section["host"], "slimevr.host");
    }
    if (section["port"]) {
        out.slimevr_port = parse_scalar<int>(
            section["port"], "slimevr.port");
    }
    if (section["rate_hz"]) {
        out.slimevr_rate_hz = parse_scalar<double>(
            section["rate_hz"], "slimevr.rate_hz");
    }
    if (section["quat_smooth"]) {
        out.slimevr_quat_smooth = parse_scalar<double>(
            section["quat_smooth"], "slimevr.quat_smooth");
    }
    if (section["preview_no_reset"]) {
        out.slimevr_preview_no_reset = parse_scalar<bool>(
            section["preview_no_reset"], "slimevr.preview_no_reset");
    }
}

void load_vmt(const YAML::Node& section, MainOptions& out) {
    ensure_map(section, "vmt");
    static const std::set<std::string> allowed{
        "vmt_out", "host", "port", "rate_hz", "index_base", "pos_smooth",
        "degeneracy_mode", "disable_below_floor",
        // HMD pose receiver.
        "hmd_listen_enabled", "hmd_listen_port", "hmd_listen_bind", "hmd_stale_ms",
    };
    check_keys(section, allowed, "vmt");
    if (section["vmt_out"])             out.vmt_out                 = parse_scalar<bool>(section["vmt_out"],                       "vmt.vmt_out");
    if (section["host"])                out.vmt_host                = parse_scalar<std::string>(section["host"],                   "vmt.host");
    if (section["port"])                out.vmt_port                = parse_scalar<int>(section["port"],                           "vmt.port");
    if (section["rate_hz"])             out.vmt_rate_hz             = parse_scalar<double>(section["rate_hz"],                     "vmt.rate_hz");
    if (section["index_base"])          out.vmt_index_base          = parse_scalar<int>(section["index_base"],                    "vmt.index_base");
    if (section["pos_smooth"])          out.vmt_pos_smooth          = parse_scalar<double>(section["pos_smooth"],                  "vmt.pos_smooth");
    if (section["degeneracy_mode"])     out.vmt_degeneracy_mode     = parse_scalar<std::string>(section["degeneracy_mode"],        "vmt.degeneracy_mode");
    if (section["disable_below_floor"]) out.vmt_disable_below_floor = parse_scalar<bool>(section["disable_below_floor"],           "vmt.disable_below_floor");
    if (section["hmd_listen_enabled"])  out.hmd_listen_enabled      = parse_scalar<bool>(section["hmd_listen_enabled"],            "vmt.hmd_listen_enabled");
    if (section["hmd_listen_port"])     out.hmd_listen_port         = parse_scalar<int>(section["hmd_listen_port"],                "vmt.hmd_listen_port");
    if (section["hmd_listen_bind"])     out.hmd_listen_bind         = parse_scalar<std::string>(section["hmd_listen_bind"],        "vmt.hmd_listen_bind");
    if (section["hmd_stale_ms"])        out.hmd_stale_ms            = parse_scalar<double>(section["hmd_stale_ms"],                "vmt.hmd_stale_ms");
}

}  // namespace

void load_main_config(const std::string& path, MainOptions& out) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        fail(std::string("failed to load ") + path + ": " + e.what());
    }
    if (!root.IsMap()) {
        fail(path + " must be a YAML map at the top level");
    }

    // schema gate first — every other check is meaningless if the file is
    // not the version we know how to parse.
    if (!root["schema"]) {
        fail("missing top-level `schema` (expected `" + std::string(kMainConfigSchema) + "`)");
    }
    const auto schema = parse_scalar<std::string>(root["schema"], "schema");
    if (schema != kMainConfigSchema) {
        fail("unsupported schema `" + schema
             + "` (expected `" + std::string(kMainConfigSchema) + "`)");
    }

    static const std::set<std::string> top_allowed{
        "schema", "cameras", "inference", "web", "three_d",
        "subject", "calibration", "logging", "slimevr", "vmt",
    };
    for (auto it = root.begin(); it != root.end(); ++it) {
        const auto key = it->first.as<std::string>();
        if (top_allowed.count(key) == 0) {
            fail("unknown key " + key);
        }
    }

    if (root["cameras"])     load_cameras   (root["cameras"],     out);
    if (root["inference"])   load_inference (root["inference"],   out);
    if (root["web"])         load_web       (root["web"],         out);
    if (root["three_d"])     load_three_d   (root["three_d"],     out);
    if (root["subject"])     load_subject   (root["subject"],     out);
    if (root["calibration"]) load_calibration(root["calibration"], out);
    if (root["logging"])     load_logging   (root["logging"],     out);
    if (root["slimevr"])     load_slimevr   (root["slimevr"],     out);
    if (root["vmt"])         load_vmt       (root["vmt"],         out);
}

EarlyArgs scan_early_args(int argc, char** argv) {
    EarlyArgs ea;
    for (int i = 0; i < argc; ++i) {
        std::string_view a{argv[i]};
        if (a == "--help" || a == "-h") {
            ea.want_help = true;
        } else if (a == "--probe") {
            ea.want_probe = true;
        } else if (a == "--config") {
            if (i + 1 >= argc) {
                fail("missing argument for --config");
            }
            ea.config_path = argv[++i];
        }
    }
    return ea;
}

void apply_cli_overrides(MainOptions& out, int argc, char** argv) {
    auto need = [&](int& i, const char* flag) -> const char* {
        if (i + 1 >= argc) {
            fail(std::string("missing argument for ") + flag);
        }
        return argv[++i];
    };

    for (int i = 0; i < argc; ++i) {
        std::string_view a{argv[i]};
        // Meta flags handled by scan_early_args / main; skip cleanly here so
        // a second pass with the same argv vector is idempotent.
        if (a == "--help" || a == "-h" || a == "--probe") continue;
        if (a == "--config") { (void)need(i, "--config"); continue; }

        if      (a == "--cam0")              { out.cam_paths[0] = need(i, "--cam0"); }
        else if (a == "--cam1")              { out.cam_paths[1] = need(i, "--cam1"); }
        else if (a == "--cam2")              { out.cam_paths[2] = need(i, "--cam2"); }
        else if (a == "--det-engine")        { out.det_engine  = need(i, "--det-engine"); }
        else if (a == "--pose-engine")       { out.pose_engine = need(i, "--pose-engine"); }
        else if (a == "--port")              { out.port = std::atoi(need(i, "--port")); }
        else if (a == "--host")              { out.host = need(i, "--host"); }
        else if (a == "--static")            { out.static_dir = need(i, "--static"); }
        else if (a == "--no-web")            { out.no_web = true; }
        else if (a == "--width")             { out.width  = std::atoi(need(i, "--width")); }
        else if (a == "--height")            { out.height = std::atoi(need(i, "--height")); }
        else if (a == "--fps")               { out.fps    = std::atoi(need(i, "--fps")); }
        else if (a == "--det-frequency")     { out.det_frequency = std::atoi(need(i, "--det-frequency")); }
        else if (a == "--keypoint-format")   { out.keypoint_format = need(i, "--keypoint-format"); }
        else if (a == "--multi-person")      { out.multi_person  = true; }
        else if (a == "--bench-fake-bbox")   { out.bench_fake_bbox = true; }
        else if (a == "--det-score")         { out.det_score = std::stof(need(i, "--det-score")); }
        else if (a == "--log-every-s")       { out.log_every_s = std::stod(need(i, "--log-every-s")); }
        else if (a == "--enable-3d")         { out.enable_3d = true; }
        else if (a == "--calib")             { out.calib = need(i, "--calib"); }
        else if (a == "--kp-conf-thresh")    { out.kp_conf_thresh = std::stof(need(i, "--kp-conf-thresh")); }
        else if (a == "--max-reproj-px")     { out.max_reproj_px = std::stof(need(i, "--max-reproj-px")); }
        else if (a == "--sync-window-ms")    { out.sync_window_ms = std::stod(need(i, "--sync-window-ms")); }
        else if (a == "--bone-calib-frames") { out.bone_calib_frames = std::atoi(need(i, "--bone-calib-frames")); }
        else if (a == "--subject-height-m")  { out.subject_height_m = std::stod(need(i, "--subject-height-m")); }
        else if (a == "--subject-id")        { out.subject_id = need(i, "--subject-id"); }
        else if (a == "--subjects-dir")      { out.subjects_dir = need(i, "--subjects-dir"); }
        else if (a == "--subject-profile")   { out.subject_profile = need(i, "--subject-profile"); }
        else if (a == "--no-3d-kalman")      { out.kalman_3d = false; }
        else if (a == "--no-3d-ik")          { out.ik_3d = false; }
        else if (a == "--slimevr-out")       { out.slimevr_out = true; }
        else if (a == "--slimevr-host")      { out.slimevr_host = need(i, "--slimevr-host"); }
        else if (a == "--slimevr-port")      { out.slimevr_port = std::atoi(need(i, "--slimevr-port")); }
        else if (a == "--slimevr-rate-hz")   { out.slimevr_rate_hz = std::stod(need(i, "--slimevr-rate-hz")); }
        else if (a == "--slimevr-quat-smooth"){ out.slimevr_quat_smooth = std::stod(need(i, "--slimevr-quat-smooth")); }
        else if (a == "--slimevr-preview-no-reset") {
            out.slimevr_preview_no_reset = true;
        }
        else if (a == "--vmt-out")           { out.vmt_out = true; }
        else if (a == "--vmt-host")          { out.vmt_host = need(i, "--vmt-host"); }
        else if (a == "--vmt-port")          { out.vmt_port = std::atoi(need(i, "--vmt-port")); }
        else if (a == "--vmt-rate-hz")       { out.vmt_rate_hz = std::stod(need(i, "--vmt-rate-hz")); }
        else if (a == "--vmt-index-base")    { out.vmt_index_base = std::atoi(need(i, "--vmt-index-base")); }
        else if (a == "--vmt-pos-smooth")    { out.vmt_pos_smooth = std::stod(need(i, "--vmt-pos-smooth")); }
        else if (a == "--vmt-degeneracy-mode"){ out.vmt_degeneracy_mode = need(i, "--vmt-degeneracy-mode"); }
        else if (a == "--vmt-disable-below-floor"){ out.vmt_disable_below_floor = true; }
        else if (a == "--hmd-listen-enabled") { out.hmd_listen_enabled = true; }
        else if (a == "--hmd-listen-port")    { out.hmd_listen_port    = std::atoi(need(i, "--hmd-listen-port")); }
        else if (a == "--hmd-listen-bind")    { out.hmd_listen_bind    = need(i, "--hmd-listen-bind"); }
        else if (a == "--hmd-stale-ms")       { out.hmd_stale_ms       = std::stod(need(i, "--hmd-stale-ms")); }
        else if (a == "--calibrate")             { out.calibrate = true; }
        else if (a == "--calib-subject-id")      { out.calib_subject_id = need(i, "--calib-subject-id"); }
        else if (a == "--calib-subject-height-m"){ out.calib_subject_height_m = std::stod(need(i, "--calib-subject-height-m")); }
        else if (a == "--calib-frames-per-cam")  { out.calib_frames_per_cam = std::atoi(need(i, "--calib-frames-per-cam")); }
        else if (a == "--calib-hold-sec")        { out.calib_hold_sec = std::stod(need(i, "--calib-hold-sec")); }
        else if (a == "--calib-auto-approve")    { out.calib_auto_approve = true; }
        else if (a == "--calib-auto-exit")       { out.calib_auto_exit = true; }
        else if (a == "--calib-static-dir")      { out.calib_static_dir = need(i, "--calib-static-dir"); }
        else if (a == "--calib-dump-tool")       { out.calib_dump_tool = need(i, "--calib-dump-tool"); }
        else {
            fail(std::string("unknown arg: ") + argv[i]);
        }
    }
}

void validate_options(const MainOptions& opts) {
    if (opts.cam_paths[0].empty() || opts.det_engine.empty() || opts.pose_engine.empty()) {
        fail("missing required option (need --cam0 + --det-engine + --pose-engine)");
    }
    if (opts.enable_3d && opts.calib.empty()) {
        fail("--enable-3d requires --calib PATH");
    }
    if (opts.subject_height_m < 0.0 || opts.subject_height_m > 2.5) {
        fail("--subject-height-m must be 0 or a plausible meter value <= 2.5");
    }
    if (!opts.enable_3d && (!opts.subject_id.empty() || !opts.subject_profile.empty())) {
        fail("--subject-id/--subject-profile require --enable-3d");
    }
    if (opts.slimevr_out) {
        if (!opts.enable_3d) {
            fail("--slimevr-out requires --enable-3d");
        }
        if (opts.keypoint_format != "halpe26") {
            fail("--slimevr-out requires --keypoint-format=halpe26");
        }
        if (opts.calibrate) {
            fail("--slimevr-out cannot be combined with --calibrate");
        }
        if (opts.slimevr_port <= 0 || opts.slimevr_port > 65535) {
            fail("--slimevr-port must be in [1, 65535]");
        }
        if (opts.slimevr_rate_hz <= 0.0 || opts.slimevr_rate_hz > 240.0) {
            fail("--slimevr-rate-hz must be in (0, 240]");
        }
        if (opts.slimevr_quat_smooth < 0.0 || opts.slimevr_quat_smooth > 1.0) {
            fail("--slimevr-quat-smooth must be in [0, 1]");
        }
    }
    if (opts.vmt_out) {
        if (!opts.enable_3d) {
            fail("--vmt-out requires --enable-3d");
        }
        if (opts.keypoint_format != "halpe26") {
            fail("--vmt-out requires --keypoint-format=halpe26");
        }
        if (opts.calibrate) {
            fail("--vmt-out cannot be combined with --calibrate");
        }
        if (opts.vmt_port <= 0 || opts.vmt_port > 65535) {
            fail("--vmt-port must be in [1, 65535]");
        }
        if (opts.vmt_rate_hz <= 0.0 || opts.vmt_rate_hz > 240.0) {
            fail("--vmt-rate-hz must be in (0, 240]");
        }
        if (opts.vmt_index_base < 0 || opts.vmt_index_base > 48) {
            fail("--vmt-index-base must be in [0, 48] so 10 trackers fit VMT index 0..57");
        }
        if (opts.vmt_pos_smooth < 0.0 || opts.vmt_pos_smooth > 1.0) {
            fail("--vmt-pos-smooth must be in [0, 1]");
        }
        if (opts.vmt_degeneracy_mode != "hold"
            && opts.vmt_degeneracy_mode != "disable"
            && opts.vmt_degeneracy_mode != "skip") {
            fail("--vmt-degeneracy-mode must be one of hold|disable|skip");
        }
    }
    if (opts.hmd_listen_enabled) {
        if (opts.hmd_listen_port <= 0 || opts.hmd_listen_port > 65535) {
            fail("--hmd-listen-port must be in [1, 65535]");
        }
        if (opts.hmd_stale_ms <= 0.0 || opts.hmd_stale_ms > 10000.0) {
            fail("--hmd-stale-ms must be in (0, 10000]");
        }
    }
    if (opts.calibrate && !opts.enable_3d) {
        fail("--calibrate requires --enable-3d");
    }
    if (opts.calibrate
        && (opts.calib_subject_id.empty() || opts.calib_subject_height_m <= 0.0)) {
        fail("--calibrate requires --calib-subject-id and --calib-subject-height-m");
    }
}

}  // namespace fitra::config
