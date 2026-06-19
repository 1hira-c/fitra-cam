#include "config/main_config.hpp"

#include <cstdlib>
#include <cstring>
#include <filesystem>
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
        "pixel_format", "n_buffers",
        "cam0_capture_width", "cam0_capture_height",
        "cam1_capture_width", "cam1_capture_height",
        "cam2_capture_width", "cam2_capture_height",
        "cam0_pixel_format", "cam1_pixel_format", "cam2_pixel_format",
        "cam0_exposure_mode", "cam1_exposure_mode", "cam2_exposure_mode",
        "cam0_exposure", "cam1_exposure", "cam2_exposure",
        "cam0_gain", "cam1_gain", "cam2_gain",
        "cam0_ae_target", "cam1_ae_target", "cam2_ae_target",
    };
    check_keys(section, allowed, "cameras");
    if (section["cam0"])   out.cam_paths[0] = parse_scalar<std::string>(section["cam0"],   "cameras.cam0");
    if (section["cam1"])   out.cam_paths[1] = parse_scalar<std::string>(section["cam1"],   "cameras.cam1");
    if (section["cam2"])   out.cam_paths[2] = parse_scalar<std::string>(section["cam2"],   "cameras.cam2");
    if (section["width"])  out.width  = parse_scalar<int>(section["width"],  "cameras.width");
    if (section["height"]) out.height = parse_scalar<int>(section["height"], "cameras.height");
    for (int i = 0; i < 3; ++i) {
        const std::string wk = "cam" + std::to_string(i) + "_capture_width";
        const std::string hk = "cam" + std::to_string(i) + "_capture_height";
        if (section[wk]) out.cam_cap_width[i]  = parse_scalar<int>(section[wk], "cameras." + wk);
        if (section[hk]) out.cam_cap_height[i] = parse_scalar<int>(section[hk], "cameras." + hk);
        const std::string pk = "cam" + std::to_string(i) + "_pixel_format";
        if (section[pk]) out.cam_pixel_format[i] = parse_scalar<std::string>(section[pk], "cameras." + pk);
        const std::string em = "cam" + std::to_string(i) + "_exposure_mode";
        const std::string ek = "cam" + std::to_string(i) + "_exposure";
        const std::string gk = "cam" + std::to_string(i) + "_gain";
        const std::string tk = "cam" + std::to_string(i) + "_ae_target";
        if (section[em]) out.cam_exposure_mode[i] = parse_scalar<std::string>(section[em], "cameras." + em);
        if (section[ek]) out.cam_exposure[i]      = parse_scalar<int>(section[ek], "cameras." + ek);
        if (section[gk]) out.cam_gain[i]          = parse_scalar<int>(section[gk], "cameras." + gk);
        if (section[tk]) out.cam_ae_target[i]     = parse_scalar<int>(section[tk], "cameras." + tk);
    }
    if (section["fps"])    out.fps    = parse_scalar<int>(section["fps"],    "cameras.fps");
    if (section["pixel_format"]) out.pixel_format = parse_scalar<std::string>(section["pixel_format"], "cameras.pixel_format");
    if (section["n_buffers"])    out.n_buffers    = parse_scalar<int>(section["n_buffers"],    "cameras.n_buffers");
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
        "vr_extract_event_driven",
        "vr_one_euro", "vr_pos_mincutoff", "vr_pos_beta", "vr_pos_dcutoff",
        "vr_quat_mincutoff", "vr_quat_beta", "vr_quat_dcutoff",
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
    if (section["vr_extract_event_driven"]) {
        out.vr_extract_event_driven = parse_scalar<bool>(
            section["vr_extract_event_driven"], "three_d.vr_extract_event_driven");
    }
    if (section["vr_one_euro"])       out.vr_one_euro       = parse_scalar<bool>(section["vr_one_euro"],         "three_d.vr_one_euro");
    if (section["vr_pos_mincutoff"])  out.vr_pos_mincutoff  = parse_scalar<double>(section["vr_pos_mincutoff"],  "three_d.vr_pos_mincutoff");
    if (section["vr_pos_beta"])       out.vr_pos_beta       = parse_scalar<double>(section["vr_pos_beta"],       "three_d.vr_pos_beta");
    if (section["vr_pos_dcutoff"])    out.vr_pos_dcutoff    = parse_scalar<double>(section["vr_pos_dcutoff"],    "three_d.vr_pos_dcutoff");
    if (section["vr_quat_mincutoff"]) out.vr_quat_mincutoff = parse_scalar<double>(section["vr_quat_mincutoff"], "three_d.vr_quat_mincutoff");
    if (section["vr_quat_beta"])      out.vr_quat_beta      = parse_scalar<double>(section["vr_quat_beta"],      "three_d.vr_quat_beta");
    if (section["vr_quat_dcutoff"])   out.vr_quat_dcutoff   = parse_scalar<double>(section["vr_quat_dcutoff"],   "three_d.vr_quat_dcutoff");
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
        // Continuous HMD-driven alignment refinement.
        "continuous_align", "continuous_sample_hz", "continuous_resolve_s", "continuous_blend",
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
    if (section["continuous_align"])      out.vmt_continuous_align     = parse_scalar<bool>(section["continuous_align"],             "vmt.continuous_align");
    if (section["continuous_sample_hz"])  out.vmt_continuous_sample_hz = parse_scalar<double>(section["continuous_sample_hz"],        "vmt.continuous_sample_hz");
    if (section["continuous_resolve_s"])  out.vmt_continuous_resolve_s = parse_scalar<double>(section["continuous_resolve_s"],        "vmt.continuous_resolve_s");
    if (section["continuous_blend"])      out.vmt_continuous_blend     = parse_scalar<double>(section["continuous_blend"],            "vmt.continuous_blend");
}

void load_extrinsic_calib(const YAML::Node& section, MainOptions& out) {
    ensure_map(section, "extrinsic_calib");
    static const std::set<std::string> allowed{
        "enabled", "replay_dir", "intrinsics", "out", "faces", "tag_size_m",
        "lin_vel_max", "ang_vel_max", "burst_min", "min_samples",
        "controller_role",
        "controller_port", "controller_bind", "controller_stale_ms",
        // Method selector + floor-AprilTag path (案D).
        "method",
        "floor_map", "floor_replay_dir", "floor_intrinsics",
        "floor_out_intrinsics", "floor_burst_min", "floor_max_reproj_px",
        "floor_fisheye",
    };
    check_keys(section, allowed, "extrinsic_calib");
    if (section["enabled"])         out.excal_enabled        = parse_scalar<bool>(section["enabled"],               "extrinsic_calib.enabled");
    if (section["replay_dir"])      out.excal_replay         = parse_scalar<std::string>(section["replay_dir"],     "extrinsic_calib.replay_dir");
    if (section["intrinsics"])      out.excal_intrinsics     = parse_scalar<std::string>(section["intrinsics"],     "extrinsic_calib.intrinsics");
    if (section["out"])             out.excal_out            = parse_scalar<std::string>(section["out"],            "extrinsic_calib.out");
    if (section["faces"])           out.excal_faces          = parse_scalar<std::string>(section["faces"],          "extrinsic_calib.faces");
    if (section["tag_size_m"])      out.excal_tag_size_m     = parse_scalar<double>(section["tag_size_m"],          "extrinsic_calib.tag_size_m");
    if (section["lin_vel_max"])     out.excal_lin_vel_max    = parse_scalar<double>(section["lin_vel_max"],         "extrinsic_calib.lin_vel_max");
    if (section["ang_vel_max"])     out.excal_ang_vel_max    = parse_scalar<double>(section["ang_vel_max"],         "extrinsic_calib.ang_vel_max");
    if (section["burst_min"])       out.excal_burst_min      = parse_scalar<int>(section["burst_min"],              "extrinsic_calib.burst_min");
    if (section["min_samples"])     out.excal_min_samples    = parse_scalar<int>(section["min_samples"],            "extrinsic_calib.min_samples");
    if (section["controller_role"]) out.excal_controller_role = parse_scalar<std::string>(section["controller_role"], "extrinsic_calib.controller_role");
    if (section["controller_port"]) out.excal_controller_port = parse_scalar<int>(section["controller_port"],      "extrinsic_calib.controller_port");
    if (section["controller_bind"]) out.excal_controller_bind = parse_scalar<std::string>(section["controller_bind"], "extrinsic_calib.controller_bind");
    if (section["controller_stale_ms"]) out.excal_controller_stale_ms = parse_scalar<double>(section["controller_stale_ms"], "extrinsic_calib.controller_stale_ms");

    // Method selector: "controller" (default, 案C) | "floor" (案D). Sets the
    // daemon-only stage selector — NOT the run_mode flag (floor_calib_enabled).
    // The shared daemon config must not carry a run_mode-deriving flag (that
    // would make the parent + run child derive a calib mode); the daemon uses
    // excal_method to choose the extrinsic stage and module_argv injects
    // --floor-calib into that child. Standalone floor uses --floor-calib.
    if (section["method"]) {
        const std::string m = parse_scalar<std::string>(section["method"], "extrinsic_calib.method");
        if (m != "controller" && m != "floor") {
            throw std::runtime_error("extrinsic_calib.method must be 'controller' or 'floor'");
        }
        out.excal_method = m;
    }
    if (section["floor_map"])            out.floor_map            = parse_scalar<std::string>(section["floor_map"],            "extrinsic_calib.floor_map");
    if (section["floor_replay_dir"])     out.floor_replay         = parse_scalar<std::string>(section["floor_replay_dir"],     "extrinsic_calib.floor_replay_dir");
    if (section["floor_intrinsics"])     out.floor_intrinsics     = parse_scalar<std::string>(section["floor_intrinsics"],     "extrinsic_calib.floor_intrinsics");
    if (section["floor_out_intrinsics"]) out.floor_out_intrinsics = parse_scalar<std::string>(section["floor_out_intrinsics"], "extrinsic_calib.floor_out_intrinsics");
    if (section["floor_burst_min"])      out.floor_burst_min      = parse_scalar<int>(section["floor_burst_min"],              "extrinsic_calib.floor_burst_min");
    if (section["floor_max_reproj_px"])  out.floor_max_reproj_px  = parse_scalar<double>(section["floor_max_reproj_px"],       "extrinsic_calib.floor_max_reproj_px");
    if (section["floor_fisheye"])        out.floor_fisheye        = parse_scalar<bool>(section["floor_fisheye"],               "extrinsic_calib.floor_fisheye");
    // The floor path shares extrinsic_calib.out as its output target.
    if (section["out"])                  out.floor_out            = out.excal_out;
}

void load_intrinsic_calib(const YAML::Node& section, MainOptions& out) {
    ensure_map(section, "intrinsic_calib");
    static const std::set<std::string> allowed{
        "enabled", "replay_dir", "out", "model",
        "squares_x", "squares_y", "square_len_m", "marker_len_m", "dict",
        "min_views", "min_corners", "max_rms_px",
    };
    check_keys(section, allowed, "intrinsic_calib");
    // enabled sets the daemon-only step selector, NOT the run_mode flag
    // (intrinsic_calib_enabled, set only by --calib-intrinsic). See the field
    // docs: the shared daemon config stays free of run_mode-deriving flags.
    if (section["enabled"])      out.intrinsic_step_enabled = parse_scalar<bool>(section["enabled"], "intrinsic_calib.enabled");
    if (section["replay_dir"])   out.intrinsic_replay        = parse_scalar<std::string>(section["replay_dir"], "intrinsic_calib.replay_dir");
    if (section["out"])          out.intrinsic_out           = parse_scalar<std::string>(section["out"], "intrinsic_calib.out");
    if (section["model"]) {
        out.intrinsic_model = parse_scalar<std::string>(section["model"], "intrinsic_calib.model");
        if (out.intrinsic_model != "pinhole" && out.intrinsic_model != "fisheye") {
            throw std::runtime_error("intrinsic_calib.model must be 'pinhole' or 'fisheye'");
        }
    }
    if (section["squares_x"])    out.charuco_squares_x    = parse_scalar<int>(section["squares_x"], "intrinsic_calib.squares_x");
    if (section["squares_y"])    out.charuco_squares_y    = parse_scalar<int>(section["squares_y"], "intrinsic_calib.squares_y");
    if (section["square_len_m"]) out.charuco_square_len_m = parse_scalar<double>(section["square_len_m"], "intrinsic_calib.square_len_m");
    if (section["marker_len_m"]) out.charuco_marker_len_m = parse_scalar<double>(section["marker_len_m"], "intrinsic_calib.marker_len_m");
    if (section["dict"])         out.charuco_dict         = parse_scalar<int>(section["dict"], "intrinsic_calib.dict");
    if (section["min_views"])    out.intrinsic_min_views  = parse_scalar<int>(section["min_views"], "intrinsic_calib.min_views");
    if (section["min_corners"])  out.intrinsic_min_corners = parse_scalar<int>(section["min_corners"], "intrinsic_calib.min_corners");
    if (section["max_rms_px"])   out.intrinsic_max_rms_px = parse_scalar<double>(section["max_rms_px"], "intrinsic_calib.max_rms_px");
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
        "extrinsic_calib", "intrinsic_calib",
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
    if (root["extrinsic_calib"]) load_extrinsic_calib(root["extrinsic_calib"], out);
    if (root["intrinsic_calib"]) load_intrinsic_calib(root["intrinsic_calib"], out);
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
    // Parse "WxH" (e.g. "1280x960") into out.cam_cap_width/height[idx].
    auto parse_capture = [&](const char* s, const char* flag, int idx) {
        const std::string v{s};
        const auto x = v.find('x');
        if (x == std::string::npos || x == 0 || x + 1 >= v.size()) {
            fail(std::string(flag) + " expects WxH (e.g. 1280x960)");
        }
        out.cam_cap_width[idx]  = std::atoi(v.substr(0, x).c_str());
        out.cam_cap_height[idx] = std::atoi(v.substr(x + 1).c_str());
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
        else if (a == "--cam0-capture")      { parse_capture(need(i, "--cam0-capture"), "--cam0-capture", 0); }
        else if (a == "--cam1-capture")      { parse_capture(need(i, "--cam1-capture"), "--cam1-capture", 1); }
        else if (a == "--cam2-capture")      { parse_capture(need(i, "--cam2-capture"), "--cam2-capture", 2); }
        else if (a == "--det-engine")        { out.det_engine  = need(i, "--det-engine"); }
        else if (a == "--pose-engine")       { out.pose_engine = need(i, "--pose-engine"); }
        else if (a == "--port")              { out.port = std::atoi(need(i, "--port")); }
        else if (a == "--host")              { out.host = need(i, "--host"); }
        else if (a == "--static")            { out.static_dir = need(i, "--static"); }
        else if (a == "--no-web")            { out.no_web = true; }
        else if (a == "--width")             { out.width  = std::atoi(need(i, "--width")); }
        else if (a == "--height")            { out.height = std::atoi(need(i, "--height")); }
        else if (a == "--fps")               { out.fps    = std::atoi(need(i, "--fps")); }
        else if (a == "--pixel-format")      { out.pixel_format = need(i, "--pixel-format"); }
        else if (a == "--n-buffers")         { out.n_buffers = std::atoi(need(i, "--n-buffers")); }
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
        else if (a == "--vr-extract-event-driven") { out.vr_extract_event_driven = true; }
        else if (a == "--vr-no-one-euro")    { out.vr_one_euro = false; }
        else if (a == "--vr-pos-mincutoff")  { out.vr_pos_mincutoff  = std::stod(need(i, "--vr-pos-mincutoff")); }
        else if (a == "--vr-pos-beta")       { out.vr_pos_beta       = std::stod(need(i, "--vr-pos-beta")); }
        else if (a == "--vr-pos-dcutoff")    { out.vr_pos_dcutoff    = std::stod(need(i, "--vr-pos-dcutoff")); }
        else if (a == "--vr-quat-mincutoff") { out.vr_quat_mincutoff = std::stod(need(i, "--vr-quat-mincutoff")); }
        else if (a == "--vr-quat-beta")      { out.vr_quat_beta      = std::stod(need(i, "--vr-quat-beta")); }
        else if (a == "--vr-quat-dcutoff")   { out.vr_quat_dcutoff   = std::stod(need(i, "--vr-quat-dcutoff")); }
        else if (a == "--slimevr-out")       { out.slimevr_out = true; }
        else if (a == "--no-slimevr-out")    { out.slimevr_out = false; }
        else if (a == "--slimevr-host")      { out.slimevr_host = need(i, "--slimevr-host"); }
        else if (a == "--slimevr-port")      { out.slimevr_port = std::atoi(need(i, "--slimevr-port")); }
        else if (a == "--slimevr-rate-hz")   { out.slimevr_rate_hz = std::stod(need(i, "--slimevr-rate-hz")); }
        else if (a == "--slimevr-quat-smooth"){ out.slimevr_quat_smooth = std::stod(need(i, "--slimevr-quat-smooth")); }
        else if (a == "--slimevr-preview-no-reset") {
            out.slimevr_preview_no_reset = true;
        }
        else if (a == "--vmt-out")           { out.vmt_out = true; }
        else if (a == "--no-vmt-out")        { out.vmt_out = false; }
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
        else if (a == "--vmt-continuous-align")    { out.vmt_continuous_align = true; }
        else if (a == "--no-vmt-continuous-align") { out.vmt_continuous_align = false; }
        else if (a == "--vmt-continuous-sample-hz"){ out.vmt_continuous_sample_hz = std::stod(need(i, "--vmt-continuous-sample-hz")); }
        else if (a == "--vmt-continuous-resolve-s"){ out.vmt_continuous_resolve_s = std::stod(need(i, "--vmt-continuous-resolve-s")); }
        else if (a == "--vmt-continuous-blend")    { out.vmt_continuous_blend     = std::stod(need(i, "--vmt-continuous-blend")); }
        else if (a == "--calibrate")             { out.calibrate = true; }
        else if (a == "--calib-subject-id")      { out.calib_subject_id = need(i, "--calib-subject-id"); }
        else if (a == "--calib-subject-height-m"){ out.calib_subject_height_m = std::stod(need(i, "--calib-subject-height-m")); }
        else if (a == "--calib-frames-per-cam")  { out.calib_frames_per_cam = std::atoi(need(i, "--calib-frames-per-cam")); }
        else if (a == "--calib-hold-sec")        { out.calib_hold_sec = std::stod(need(i, "--calib-hold-sec")); }
        else if (a == "--calib-auto-approve")    { out.calib_auto_approve = true; }
        else if (a == "--calib-auto-exit")       { out.calib_auto_exit = true; }
        else if (a == "--calib-static-dir")      { out.calib_static_dir = need(i, "--calib-static-dir"); }
        else if (a == "--calib-dump-tool")       { out.calib_dump_tool = need(i, "--calib-dump-tool"); }
        else if (a == "--extrinsic-calib")         { out.excal_enabled = true; }
        else if (a == "--excal-replay")            { out.excal_replay = need(i, "--excal-replay"); }
        else if (a == "--excal-intrinsics")        { out.excal_intrinsics = need(i, "--excal-intrinsics"); }
        else if (a == "--excal-out")               { out.excal_out = need(i, "--excal-out"); }
        else if (a == "--excal-faces")             { out.excal_faces = need(i, "--excal-faces"); }
        else if (a == "--excal-tag-size-m")        { out.excal_tag_size_m = std::stod(need(i, "--excal-tag-size-m")); }
        else if (a == "--excal-lin-vel-max")       { out.excal_lin_vel_max = std::stod(need(i, "--excal-lin-vel-max")); }
        else if (a == "--excal-ang-vel-max")       { out.excal_ang_vel_max = std::stod(need(i, "--excal-ang-vel-max")); }
        else if (a == "--excal-burst-min")         { out.excal_burst_min = std::atoi(need(i, "--excal-burst-min")); }
        else if (a == "--excal-min-samples")       { out.excal_min_samples = std::atoi(need(i, "--excal-min-samples")); }
        else if (a == "--excal-controller-role")   { out.excal_controller_role = need(i, "--excal-controller-role"); }
        else if (a == "--excal-controller-port")   { out.excal_controller_port = std::atoi(need(i, "--excal-controller-port")); }
        else if (a == "--excal-controller-bind")   { out.excal_controller_bind = need(i, "--excal-controller-bind"); }
        else if (a == "--excal-controller-stale-ms"){ out.excal_controller_stale_ms = std::stod(need(i, "--excal-controller-stale-ms")); }
        else if (a == "--floor-calib")             { out.floor_calib_enabled = true; }
        else if (a == "--floor-map")               { out.floor_map = need(i, "--floor-map"); }
        else if (a == "--floor-replay")            { out.floor_replay = need(i, "--floor-replay"); }
        else if (a == "--floor-intrinsics")        { out.floor_intrinsics = need(i, "--floor-intrinsics"); }
        else if (a == "--floor-out-intrinsics")    { out.floor_out_intrinsics = need(i, "--floor-out-intrinsics"); }
        else if (a == "--floor-out")               { out.floor_out = need(i, "--floor-out"); }
        else if (a == "--floor-burst-min")         { out.floor_burst_min = std::atoi(need(i, "--floor-burst-min")); }
        else if (a == "--floor-max-reproj-px")     { out.floor_max_reproj_px = std::stod(need(i, "--floor-max-reproj-px")); }
        else if (a == "--floor-fisheye")           { out.floor_fisheye = true; }
        else if (a == "--calib-intrinsic")         { out.intrinsic_calib_enabled = true; }
        else if (a == "--intrinsic-replay")        { out.intrinsic_replay = need(i, "--intrinsic-replay"); }
        else if (a == "--intrinsic-out")           { out.intrinsic_out = need(i, "--intrinsic-out"); }
        else if (a == "--intrinsic-model")         { out.intrinsic_model = need(i, "--intrinsic-model"); }
        else if (a == "--charuco-squares-x")       { out.charuco_squares_x = std::atoi(need(i, "--charuco-squares-x")); }
        else if (a == "--charuco-squares-y")       { out.charuco_squares_y = std::atoi(need(i, "--charuco-squares-y")); }
        else if (a == "--charuco-square-len-m")    { out.charuco_square_len_m = std::stod(need(i, "--charuco-square-len-m")); }
        else if (a == "--charuco-marker-len-m")    { out.charuco_marker_len_m = std::stod(need(i, "--charuco-marker-len-m")); }
        else if (a == "--charuco-dict")            { out.charuco_dict = std::atoi(need(i, "--charuco-dict")); }
        else if (a == "--intrinsic-min-views")     { out.intrinsic_min_views = std::atoi(need(i, "--intrinsic-min-views")); }
        else if (a == "--intrinsic-min-corners")   { out.intrinsic_min_corners = std::atoi(need(i, "--intrinsic-min-corners")); }
        else if (a == "--intrinsic-max-rms")       { out.intrinsic_max_rms_px = std::stod(need(i, "--intrinsic-max-rms")); }
        else if (a == "--daemon")            { out.daemon = true; }
        else if (a == "--daemon-initial")    { out.daemon_initial = need(i, "--daemon-initial"); }
        else if (a == "--flow-managed")      { out.flow_managed = true; }
        else if (a == "--setup")             { out.setup_mode = true; }
        else {
            fail(std::string("unknown arg: ") + argv[i]);
        }
    }
}

RunMode run_mode(const MainOptions& opts) {
    if (opts.setup_mode) return RunMode::Setup;
    if (opts.intrinsic_calib_enabled || !opts.intrinsic_replay.empty()) {
        return RunMode::CalibIntrinsic;
    }
    if (opts.floor_calib_enabled || !opts.floor_replay.empty()) {
        return RunMode::CalibExtrinsicFloor;
    }
    if (opts.excal_enabled || !opts.excal_replay.empty()) {
        return RunMode::CalibExtrinsic;
    }
    if (opts.calibrate) return RunMode::CalibSubject;
    return RunMode::Run;
}

const char* run_mode_name(RunMode mode) {
    switch (mode) {
        case RunMode::Setup:               return "setup";
        case RunMode::CalibSubject:        return "calib-subject";
        case RunMode::CalibExtrinsic:      return "calib-extrinsic";
        case RunMode::CalibExtrinsicFloor: return "calib-extrinsic-floor";
        case RunMode::CalibIntrinsic:      return "calib-intrinsic";
        case RunMode::Run:                 break;
    }
    return "run";
}

bool parse_run_mode_name(const std::string& name, RunMode& out) {
    if (name == "run")                   { out = RunMode::Run;                 return true; }
    if (name == "setup")                 { out = RunMode::Setup;               return true; }
    if (name == "calib-subject")         { out = RunMode::CalibSubject;        return true; }
    if (name == "calib-extrinsic")       { out = RunMode::CalibExtrinsic;      return true; }
    if (name == "calib-extrinsic-floor") { out = RunMode::CalibExtrinsicFloor; return true; }
    if (name == "calib-intrinsic")       { out = RunMode::CalibIntrinsic;      return true; }
    return false;
}

namespace {
// A calibration source (intrinsics / extrinsics YAML) is usable if a path is
// set and the file exists. `label` names it for the error message.
bool source_ready(const std::string& path, const char* label, std::string& err) {
    if (path.empty()) {
        err = std::string("no ") + label + " configured";
        return false;
    }
    std::error_code ec;
    if (!std::filesystem::exists(path, ec) || ec) {
        err = std::string(label) + " not found: " + path;
        return false;
    }
    return true;
}
}  // namespace

bool precheck_mode_switch(const MainOptions& opts, RunMode target,
                          std::string& err) {
    switch (target) {
        case RunMode::CalibExtrinsicFloor: {
            if (opts.floor_map.empty()) {
                err = "floor calibration needs a known tag layout — set "
                      "extrinsic_calib.floor_map (or --floor-map) in the config";
                return false;
            }
            std::error_code ec;
            if (!std::filesystem::exists(opts.floor_map, ec) || ec) {
                err = "floor_map not found: " + opts.floor_map;
                return false;
            }
            // PnP intrinsics: floor_intrinsics, else three_d.calib.
            const std::string pnp =
                opts.floor_intrinsics.empty() ? opts.calib : opts.floor_intrinsics;
            return source_ready(pnp, "floor PnP intrinsics "
                                "(extrinsic_calib.floor_intrinsics or three_d.calib)", err);
        }
        case RunMode::CalibExtrinsic: {
            const std::string intr =
                opts.excal_intrinsics.empty() ? opts.calib : opts.excal_intrinsics;
            return source_ready(intr, "intrinsics "
                                "(extrinsic_calib.intrinsics or three_d.calib)", err);
        }
        case RunMode::CalibIntrinsic:
            // Produces the intrinsics YAML from scratch — no input file needed.
            // But the board params are only validated by validate_options when
            // the process IS in CalibIntrinsic mode; the daemon parent is Run,
            // so re-check them here, else a flow-switch respawns a child that
            // dies at validate and the daemon silently falls back to run.
            if (opts.charuco_squares_x < 2 || opts.charuco_squares_y < 2) {
                err = "calib-intrinsic needs charuco squares_x/y >= 2 (intrinsic_calib.*)";
                return false;
            }
            if (opts.charuco_marker_len_m <= 0.0 ||
                opts.charuco_marker_len_m >= opts.charuco_square_len_m) {
                err = "calib-intrinsic needs 0 < marker_len < square_len (intrinsic_calib.*)";
                return false;
            }
            if (opts.intrinsic_model != "pinhole" && opts.intrinsic_model != "fisheye") {
                err = "intrinsic_calib.model must be 'pinhole' or 'fisheye'";
                return false;
            }
            return true;
        case RunMode::CalibSubject:
            // Subject calibration triangulates, so it needs the extrinsics YAML.
            return source_ready(opts.calib, "extrinsics (three_d.calib)", err);
        case RunMode::Setup:
            // Setup produces the config from scratch — it needs no input
            // artifact, cameras, or engines (it is how those get configured).
            return true;
        case RunMode::Run:
            // Run is the safe fallback; it tolerates a missing calib (2D only).
            return true;
    }
    return true;
}

void validate_options(const MainOptions& opts) {
    const RunMode mode = run_mode(opts);
    if (mode == RunMode::Setup) {
        // Setup builds only a Crow server + setup routes (camera enumeration,
        // config composition) — no cameras, engines, or 3D graph. The only
        // thing it needs is a sane port (host defaults are fine).
        if (opts.port <= 0 || opts.port > 65535) {
            fail("--port must be in [1, 65535]");
        }
        return;
    }
    if (mode == RunMode::CalibExtrinsic) {
        // calib-extrinsic is decode-only (AprilTag detection on CPU) — no TRT
        // engines are loaded. Cameras are required for live collection only;
        // a replay session brings its own frames.
        if (opts.excal_replay.empty() && opts.cam_paths[0].empty()) {
            fail("missing required option (need --cam0)");
        }
    } else if (mode == RunMode::CalibExtrinsicFloor) {
        // Floor path: also decode-only. Live collection needs cameras; replay
        // brings its own. A known tag map and PnP intrinsics are mandatory.
        if (opts.floor_replay.empty() && opts.cam_paths[0].empty()) {
            fail("missing required option (need --cam0)");
        }
        if (opts.floor_map.empty()) {
            fail("calib-extrinsic-floor requires --floor-map PATH");
        }
        if (opts.floor_intrinsics.empty() && opts.calib.empty()) {
            fail("calib-extrinsic-floor requires --floor-intrinsics PATH (or --calib)");
        }
    } else if (mode == RunMode::CalibIntrinsic) {
        // Intrinsic path: decode-only, produces the intrinsics YAML. Live needs
        // cameras; replay brings its own. The ChArUco board must be sane.
        if (opts.intrinsic_replay.empty() && opts.cam_paths[0].empty()) {
            fail("missing required option (need --cam0)");
        }
        if (opts.charuco_squares_x < 2 || opts.charuco_squares_y < 2) {
            fail("calib-intrinsic needs --charuco-squares-x/y >= 2");
        }
        if (opts.charuco_marker_len_m <= 0.0 ||
            opts.charuco_marker_len_m >= opts.charuco_square_len_m) {
            fail("calib-intrinsic needs 0 < --charuco-marker-len-m < --charuco-square-len-m");
        }
        if (opts.intrinsic_model != "pinhole" && opts.intrinsic_model != "fisheye") {
            fail("--intrinsic-model must be 'pinhole' or 'fisheye'");
        }
    } else if (!opts.daemon && (opts.cam_paths[0].empty() || opts.det_engine.empty()
               || opts.pose_engine.empty())) {
        // The daemon parent validates its union config in run form, but a
        // first-run config legitimately has no cameras/engines yet (the Setup
        // module fills them in). Defer the run-form requirement to each spawned
        // child + the initial_mode precheck (app/daemon.cpp); the daemon already
        // warns on a bare config and falls back safely if a run child dies.
        fail("missing required option (need --cam0 + --det-engine + --pose-engine)");
    }
    if (opts.pixel_format != "mjpeg" && opts.pixel_format != "yuyv"
        && opts.pixel_format != "nvjpeg") {
        fail("--pixel-format must be \"mjpeg\", \"yuyv\", or \"nvjpeg\"");
    }
    // Per-camera pixel_format overrides: reject invalid non-empty values at
    // startup instead of silently falling back to mjpeg (a typo like "yuyv "
    // would otherwise quietly disable the intended decode-path split).
    for (int i = 0; i < 3; ++i) {
        const std::string& pf = opts.cam_pixel_format[i];
        if (!pf.empty() && pf != "mjpeg" && pf != "yuyv" && pf != "nvjpeg") {
            fail("cam" + std::to_string(i) +
                 "_pixel_format must be \"mjpeg\", \"yuyv\", or \"nvjpeg\"");
        }
    }
    if (opts.n_buffers < 2) {
        fail("--n-buffers must be >= 2 (driver needs at least 2 to pipeline)");
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
        if (mode != RunMode::Run) {
            fail("--slimevr-out cannot be combined with a calibration mode "
                 "(--extrinsic-calib/--floor-calib/--calib-intrinsic/--calibrate)");
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
        if (mode != RunMode::Run) {
            fail("--vmt-out cannot be combined with a calibration mode "
                 "(--extrinsic-calib/--floor-calib/--calib-intrinsic/--calibrate)");
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
    if (opts.vmt_continuous_align) {
        if (opts.vmt_continuous_sample_hz < 5.0 || opts.vmt_continuous_sample_hz > 120.0) {
            fail("--vmt-continuous-sample-hz must be in [5, 120]");
        }
        if (opts.vmt_continuous_resolve_s < 0.2 || opts.vmt_continuous_resolve_s > 30.0) {
            fail("--vmt-continuous-resolve-s must be in [0.2, 30]");
        }
        if (opts.vmt_continuous_blend <= 0.0 || opts.vmt_continuous_blend > 1.0) {
            fail("--vmt-continuous-blend must be in (0, 1]");
        }
    }
    // One Euro params drive the TrackerExtractor whenever 3D is on (feeds both
    // SlimeVR/VMT and the WebUI viz), so validate unconditionally. mincutoff and
    // beta may be 0 (0 mincutoff with 0 beta freezes — allowed but degenerate);
    // dcutoff must be > 0 (it is a cutoff, not a coefficient).
    if (opts.vr_pos_mincutoff < 0.0 || opts.vr_quat_mincutoff < 0.0) {
        fail("--vr-{pos,quat}-mincutoff must be >= 0");
    }
    if (opts.vr_pos_beta < 0.0 || opts.vr_quat_beta < 0.0) {
        fail("--vr-{pos,quat}-beta must be >= 0");
    }
    if (opts.vr_pos_dcutoff <= 0.0 || opts.vr_quat_dcutoff <= 0.0) {
        fail("--vr-{pos,quat}-dcutoff must be > 0");
    }
    if (opts.calibrate && !opts.enable_3d) {
        fail("--calibrate requires --enable-3d");
    }
    if (opts.calibrate
        && (opts.calib_subject_id.empty() || opts.calib_subject_height_m <= 0.0)) {
        fail("--calibrate requires --calib-subject-id and --calib-subject-height-m");
    }
    if (mode == RunMode::CalibExtrinsic) {
        // Dedicated mode — mutually exclusive with the subject wizard.
        if (opts.calibrate) {
            fail("--extrinsic-calib/--excal-replay cannot be combined with --calibrate");
        }
        // Needs per-camera intrinsics: a dedicated file or the three_d.calib.
        if (opts.excal_intrinsics.empty() && opts.calib.empty()) {
            fail("--extrinsic-calib requires --excal-intrinsics PATH (or --calib PATH)");
        }
        if (opts.excal_faces.empty()) {
            fail("--extrinsic-calib requires at least one face id (--excal-faces)");
        }
        if (opts.excal_tag_size_m <= 0.0 || opts.excal_tag_size_m > 2.0) {
            fail("--excal-tag-size-m must be in (0, 2]");
        }
        if (opts.excal_burst_min < 1) {
            fail("--excal-burst-min must be >= 1");
        }
        if (opts.excal_min_samples < 3) {
            fail("--excal-min-samples must be >= 3 (hand-eye needs >= 3 per group)");
        }
        if (opts.excal_controller_port <= 0 || opts.excal_controller_port > 65535) {
            fail("--excal-controller-port must be in [1, 65535]");
        }
        if (opts.excal_controller_role != "left"
            && opts.excal_controller_role != "right"
            && opts.excal_controller_role != "left_controller"
            && opts.excal_controller_role != "right_controller"
            && opts.excal_controller_role != "left-controller"
            && opts.excal_controller_role != "right-controller") {
            fail("--excal-controller-role must be one of left|right");
        }
        if (opts.excal_lin_vel_max <= 0.0 || opts.excal_ang_vel_max <= 0.0) {
            fail("--excal-lin-vel-max / --excal-ang-vel-max must be > 0");
        }
    }
    if (opts.daemon) {
        // The daemon owns mode selection — it spawns modules with the mode
        // flags itself (docs/design/pose-3d-flow-daemon.md).
        if (mode != RunMode::Run) {
            fail("--daemon cannot be combined with --calibrate/--extrinsic-calib"
                 "/--excal-replay (use --daemon-initial to pick the first mode)");
        }
        if (opts.flow_managed) {
            fail("--daemon cannot be combined with --flow-managed");
        }
        RunMode initial;
        if (opts.daemon_initial != "auto"
            && !parse_run_mode_name(opts.daemon_initial, initial)) {
            fail("--daemon-initial must be one of auto|setup|run|calib-subject"
                 "|calib-extrinsic|calib-extrinsic-floor|calib-intrinsic");
        }
    }
}

}  // namespace fitra::config
