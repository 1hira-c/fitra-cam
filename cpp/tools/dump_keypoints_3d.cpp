// dump_keypoints_3d — offline N-camera 3D dump (2 or 3 views).
//
// Runs YOLOX + RTMPose on synchronized recorded videos, triangulates either
// COCO17 (17 kpts, default) or Halpe26 (26 kpts) joints with a calibration
// YAML, then optionally applies 3D Kalman + IK. The camera count is the number
// of --video inputs (2 or 3); the calibration is trimmed to cam0..cam{n-1} in
// order (same normalization as make_threed / threed_builder), so a 3-camera
// extrinsics file is valid input for either a 2- or 3-view run.
//
// Used as the offline verification harness for the spatial-filtering track
// (docs/design/pose-3d-spatial-filtering.md): per-joint 3D positions, view
// counts and reprojection errors are emitted per frame so the companion
// analyzer (tools/analyze_3d_jitter_lag.py) can score stationary jitter and
// motion lag across stage on/off runs.
//
// With --dump-trackers (Halpe26 only) it additionally runs the SlimeVR 10-tracker
// extraction (slimevr::extract_trackers) on the final skeleton and emits a
// per-frame "trackers" array (pos + orientation quat + valid + roll_confidence).
// This is the spatiotemporal-filter track's tracker harness (M-C1,
// docs/design/pose-3d-spatiotemporal-filter.md): it captures the RAW (no
// temporal smoothing) tracker trajectories so the analyzer's `trackers`
// subcommand can score per-tracker position / roll jitter and per-bone
// relative angular velocity — the deterministic OFF baseline the spatiotemporal
// filter is measured against, plus the data for the angle-domain (案6) test.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <NvInfer.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "infer/rtmpose.hpp"
#include "infer/trt_engine.hpp"
#include "infer/yolox.hpp"
#include "lift/calib_io.hpp"
#include "lift/ik.hpp"
#include "lift/kalman.hpp"
#include "lift/floor_grounding.hpp"
#include "lift/keypoint_format.hpp"
#include "lift/rigid_fit.hpp"
#include "lift/skeleton_def.hpp"
#include "lift/subject_profile.hpp"
#include "lift/triangulator.hpp"
#include "slimevr/st_filter.hpp"
#include "slimevr/tracker_extract.hpp"
#include "util/cuda_check.hpp"
#include "util/logging.hpp"

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

struct Args {
    std::vector<std::string> videos;
    std::string calib;
    std::string det_engine;
    std::string pose_engine;
    std::string out;
    std::string summary;
    std::string overlay_dir;
    std::string pose_session;
    std::string subject_profile;      // input profile (IK lock + rigid template)
    std::string subject_profile_out;
    std::string quality_out;
    int max_frames = 0;
    int cam1_frame_offset = 0;
    int cam2_frame_offset = 0;
    int bone_calib_frames = 150;
    double fps_override = 0.0;
    double subject_height_m = 0.0;
    float det_score = 0.5f;
    float kp_conf_thresh = 0.3f;
    float max_reproj_px = 6.0f;
    bool multi_person = false;
    bool no_kalman = false;
    bool no_ik = false;
    bool rigid_pelvis = false;
    bool rigid_shoulders = false;
    bool floor_grounding = false;
    double floor_z_m = 0.0;
    double floor_snap_band_m = 0.03;
    double floor_stance_vel_mps = 0.15;
    bool dump_trackers = false;
    bool roll_hysteresis = false;
    std::string tracker_smoothing = "raw";  // raw | one_euro | st
    double st_kalman_weaken = 1.0;           // st mode: chain-Kalman process-noise scale (M-C4: ×100 harmful at rest)
    double spine_tol = 0.12;
    std::string keypoint_format_str = "coco17";
};

void print_help() {
    std::puts(
        "dump_keypoints_3d — offline N-camera (2 or 3) 3D triangulation dump\n"
        "\n"
        "Required:\n"
        "  --video PATH              input MP4, repeat 2 or 3 times in cam order\n"
        "                            (optional when --pose-session is provided)\n"
        "  --calib PATH              calibration YAML with intrinsics/extrinsics\n"
        "                            (trimmed to cam0..cam{n-1} for the chosen view count)\n"
        "  --det-engine PATH         YOLOX .engine\n"
        "  --pose-engine PATH        RTMPose .engine\n"
        "  --out PATH                output JSONL\n"
        "\n"
        "Optional:\n"
        "  --summary PATH            output summary JSON (default: <out>.summary.json)\n"
        "  --overlay-dir DIR         write reprojection overlay MP4s\n"
        "  --pose-session PATH       pose_session.json with per-pose clips\n"
        "  --subject-profile-out PATH write subject profile YAML\n"
        "  --quality-out PATH        write quality JSON\n"
        "  --max-frames N            stop after N synchronized frames\n"
        "  --cam1-frame-offset N     skip N frames on cam1 before pairing (negative skips cam0)\n"
        "  --cam2-frame-offset N     skip N frames on cam2 before pairing (negative skips cam0)\n"
        "  --fps F                   override clip fps for Kalman dt + overlays\n"
        "                            (use the recorder meta.json fps_written; the MP4\n"
        "                             header fps can be wrong for a record_3cam clip)\n"
        "  --det-score F             YOLOX score threshold (default 0.5)\n"
        "  --kp-conf-thresh F        2D keypoint threshold for triangulation (default 0.3)\n"
        "  --max-reproj-px F         one-pass outlier threshold (default 6)\n"
        "  --bone-calib-frames N     frames used to lock IK bone lengths (default 150)\n"
        "  --subject-height-m F      lock IK bone lengths from Japanese anthropometry and height\n"
        "  --multi-person            keep all bboxes, but MVP triangulates person 0 only\n"
        "  --no-kalman               disable 3D Kalman smoothing\n"
        "  --no-ik                   disable IK length/hinge projection\n"
        "  --subject-profile PATH    load a subject profile YAML (locks IK bone lengths;\n"
        "                            also supplies the rigid-fit segment template)\n"
        "  --rigid-pelvis            enable the spatial pelvis rigid fit (Halpe26 only):\n"
        "                            reorders to spatial-first tri->rigid->IK->Kalman and\n"
        "                            weighted-Kabsch fits {hip_center,l_hip,r_hip} to the\n"
        "                            subject template (needs --subject-profile). M-A.\n"
        "  --rigid-shoulders         enable the shoulder-girdle rigid fit + spine soft\n"
        "                            coupling (Halpe26 only): weighted-Kabsch fits\n"
        "                            {neck,l_shoulder,r_shoulder}, then bounds neck's\n"
        "                            distance from hip_center to the spine length +/-\n"
        "                            --spine-tol (needs --subject-profile). M-B.\n"
        "  --spine-tol F             spine soft-coupling tolerance fraction (default 0.12)\n"
        "  --floor-grounding         ground foot sole points to the floor (Halpe26 only,\n"
        "                            M-D): clamp below-floor toe/heel to Z=floor and snap\n"
        "                            near-floor low-speed (stance) points onto it. Runs\n"
        "                            after Kalman+IK. Fixes heel-sink/penetration.\n"
        "  --floor-z-m F             floor plane Z in world m (default 0)\n"
        "  --floor-snap-band-m F     stance snap zone height above floor (default 0.03)\n"
        "  --floor-stance-vel-mps F  max foot speed to be 'planted' (default 0.15)\n"
        "  --dump-trackers           also emit the SlimeVR 10-tracker extraction per\n"
        "                            frame (Halpe26 only): a \"trackers\" array of RAW\n"
        "                            (no temporal smoothing) pos + orientation quat +\n"
        "                            valid + roll_confidence. This is the spatiotemporal\n"
        "                            filter's tracker harness OFF baseline (M-C1); score\n"
        "                            it with analyze_3d_jitter_lag.py trackers.\n"
        "  --roll-hysteresis         arm/thigh roll gate-raise hysteresis (#2): hold the\n"
        "                            last confident roll through the noisy extension band\n"
        "                            instead of following mid-band roll noise. Affects the\n"
        "                            dumped trackers' orientation (needs --dump-trackers).\n"
        "  --tracker-smoothing MODE  smoothing applied to the dumped trackers (needs\n"
        "                            --dump-trackers): raw (default, no smoothing =\n"
        "                            M-C1 OFF baseline) | one_euro (One Euro only,\n"
        "                            the pre-M-C5 shipping path) | st (spatiotemporal\n"
        "                            filter, chain Kalman untouched = the shipped M-C5\n"
        "                            live path; sweep --st-kalman-weaken to deviate).\n"
        "                            Run raw/one_euro/st on the same clip for the A/B.\n"
        "  --st-kalman-weaken F      st mode: chain-Kalman process-noise scale (default\n"
        "                            1 = no weakening, the M-C4 data-driven setting; the\n"
        "                            M-C3 seed ×100 was found to inject jitter at rest.\n"
        "                            Sweep it (e.g. 10, 100) for the motion/lag study.\n"
        "  --keypoint-format FMT     pose topology: coco17 (default) or halpe26\n"
        "  --help                    show this help\n");
}

Args parse_args(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        std::string_view a{argv[i]};
        auto need = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing argument for %s\n", flag);
                std::exit(EXIT_FAILURE);
            }
            return argv[++i];
        };
        if (a == "--help" || a == "-h") { print_help(); std::exit(EXIT_SUCCESS); }
        else if (a == "--video")        { args.videos.push_back(need("--video")); }
        else if (a == "--calib")        { args.calib = need("--calib"); }
        else if (a == "--det-engine")   { args.det_engine = need("--det-engine"); }
        else if (a == "--pose-engine")  { args.pose_engine = need("--pose-engine"); }
        else if (a == "--out" || a == "--output") { args.out = need("--out"); }
        else if (a == "--summary")      { args.summary = need("--summary"); }
        else if (a == "--overlay-dir")  { args.overlay_dir = need("--overlay-dir"); }
        else if (a == "--pose-session") { args.pose_session = need("--pose-session"); }
        else if (a == "--subject-profile") { args.subject_profile = need("--subject-profile"); }
        else if (a == "--subject-profile-out") { args.subject_profile_out = need("--subject-profile-out"); }
        else if (a == "--quality-out")  { args.quality_out = need("--quality-out"); }
        else if (a == "--max-frames")   { args.max_frames = std::atoi(need("--max-frames")); }
        else if (a == "--cam1-frame-offset") { args.cam1_frame_offset = std::atoi(need("--cam1-frame-offset")); }
        else if (a == "--cam2-frame-offset") { args.cam2_frame_offset = std::atoi(need("--cam2-frame-offset")); }
        else if (a == "--fps")          { args.fps_override = std::stod(need("--fps")); }
        else if (a == "--det-score")    { args.det_score = std::stof(need("--det-score")); }
        else if (a == "--kp-conf-thresh") { args.kp_conf_thresh = std::stof(need("--kp-conf-thresh")); }
        else if (a == "--max-reproj-px") { args.max_reproj_px = std::stof(need("--max-reproj-px")); }
        else if (a == "--bone-calib-frames") { args.bone_calib_frames = std::atoi(need("--bone-calib-frames")); }
        else if (a == "--subject-height-m") { args.subject_height_m = std::stod(need("--subject-height-m")); }
        else if (a == "--multi-person") { args.multi_person = true; }
        else if (a == "--no-kalman")    { args.no_kalman = true; }
        else if (a == "--no-ik")        { args.no_ik = true; }
        else if (a == "--rigid-pelvis") { args.rigid_pelvis = true; }
        else if (a == "--rigid-shoulders") { args.rigid_shoulders = true; }
        else if (a == "--floor-grounding") { args.floor_grounding = true; }
        else if (a == "--floor-z-m")    { args.floor_z_m = std::stod(need("--floor-z-m")); }
        else if (a == "--floor-snap-band-m") { args.floor_snap_band_m = std::stod(need("--floor-snap-band-m")); }
        else if (a == "--floor-stance-vel-mps") { args.floor_stance_vel_mps = std::stod(need("--floor-stance-vel-mps")); }
        else if (a == "--dump-trackers") { args.dump_trackers = true; }
        else if (a == "--roll-hysteresis") { args.roll_hysteresis = true; }
        else if (a == "--tracker-smoothing") { args.tracker_smoothing = need("--tracker-smoothing"); }
        else if (a == "--st-kalman-weaken") { args.st_kalman_weaken = std::stod(need("--st-kalman-weaken")); }
        else if (a == "--spine-tol")    { args.spine_tol = std::stod(need("--spine-tol")); }
        else if (a == "--keypoint-format") { args.keypoint_format_str = need("--keypoint-format"); }
        else {
            std::fprintf(stderr, "unknown arg: %s\n", argv[i]);
            print_help();
            std::exit(EXIT_FAILURE);
        }
    }
    // The camera count is the number of --video inputs (2 or 3). When a
    // pose-session drives the clips, --video may be omitted (0) and the count
    // comes from the session's clip lists instead.
    const std::size_t nv = args.videos.size();
    const bool videos_count_ok = (nv == 2 || nv == 3);
    const bool inputs_ok = args.pose_session.empty()
                               ? videos_count_ok
                               : (nv == 0 || videos_count_ok);
    if (!inputs_ok || args.calib.empty() || args.det_engine.empty() ||
        args.pose_engine.empty() || args.out.empty()) {
        print_help();
        std::exit(EXIT_FAILURE);
    }
    if (args.subject_height_m < 0.0 || args.subject_height_m > 2.5) {
        std::fprintf(stderr, "--subject-height-m must be 0 or a plausible meter value <= 2.5\n");
        std::exit(EXIT_FAILURE);
    }
    if (args.summary.empty()) args.summary = args.out + ".summary.json";
    if (args.quality_out.empty() && !args.subject_profile_out.empty()) {
        args.quality_out = args.subject_profile_out + ".quality.json";
    }
    return args;
}

std::string fmt(double v, int precision = 6) {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%.*g", precision, v);
    return buf;
}

// Runtime camera ids for an n-camera stage: cam0..cam{n-1}, in order. Mirrors
// fitra::app::expected_camera_ids (not linked here — this tool depends only on
// fitra_infer/fitra_lift) so a 3-camera extrinsics file trims to exactly the
// chosen view count via select_calib_cameras / require_camera_ids.
std::vector<std::string> expected_camera_ids(std::size_t count) {
    std::vector<std::string> ids;
    ids.reserve(count);
    for (std::size_t i = 0; i < count; ++i) ids.push_back("cam" + std::to_string(i));
    return ids;
}

double median(std::vector<double> vals) {
    if (vals.empty()) return 0.0;
    auto mid = vals.begin() + static_cast<std::ptrdiff_t>(vals.size() / 2);
    std::nth_element(vals.begin(), mid, vals.end());
    double m = *mid;
    if (vals.size() % 2 == 0) {
        auto mid2 = vals.begin() + static_cast<std::ptrdiff_t>(vals.size() / 2 - 1);
        std::nth_element(vals.begin(), mid2, vals.end());
        m = 0.5 * (m + *mid2);
    }
    return m;
}

std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (char ch : s) {
        switch (ch) {
            case '\\': out += "\\\\"; break;
            case '"': out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out += ch; break;
        }
    }
    return out;
}

std::string now_iso_like() {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S%z", &tm);
    return buf;
}

double joint_distance(const fitra::infer::Joint3D& a,
                      const fitra::infer::Joint3D& b) {
    const double dx = static_cast<double>(a.x) - b.x;
    const double dy = static_cast<double>(a.y) - b.y;
    const double dz = static_cast<double>(a.z) - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

struct VideoPair {
    std::string pose = "video";
    std::vector<std::string> videos{};  // one entry per camera, in cam order
};

struct PoseSession {
    std::string subject_id = "unknown";
    double subject_height_m = 0.0;
    std::string source_session;
    std::vector<VideoPair> pairs;
};

std::string file_node_string(const cv::FileNode& node) {
    return node.empty() ? std::string{} : static_cast<std::string>(node);
}

double file_node_real(const cv::FileNode& node, double fallback = 0.0) {
    return node.empty() ? fallback : static_cast<double>(node);
}

std::string resolve_session_path(const std::filesystem::path& base,
                                 const std::string& value) {
    std::filesystem::path p{value};
    if (p.is_absolute()) return p.string();
    return (base / p).lexically_normal().string();
}

PoseSession load_pose_session(const std::string& path) {
    cv::FileStorage fs{path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON};
    if (!fs.isOpened()) {
        throw std::runtime_error("failed to open pose session: " + path);
    }
    std::filesystem::path base = std::filesystem::path(path).parent_path();
    PoseSession session;
    session.subject_id = file_node_string(fs["subject_id"]);
    if (session.subject_id.empty()) session.subject_id = "unknown";
    session.subject_height_m = file_node_real(fs["subject_height_m"]);
    session.source_session = std::filesystem::absolute(path).string();

    cv::FileNode poses = fs["poses"];
    if (poses.empty() || !poses.isSeq()) {
        throw std::runtime_error("pose_session must contain poses sequence");
    }
    for (auto it = poses.begin(); it != poses.end(); ++it) {
        cv::FileNode node = *it;
        VideoPair pair;
        pair.pose = file_node_string(node["name"]);
        if (pair.pose.empty()) pair.pose = "pose";
        cv::FileNode clips = node["clips"];
        if (!clips.empty() && clips.isSeq()) {
            for (auto cit = clips.begin(); cit != clips.end(); ++cit) {
                pair.videos.push_back(resolve_session_path(base, static_cast<std::string>(*cit)));
            }
        }
        // No explicit clips: fall back to the 2-camera raw/<pose>_cam{0,1}.mp4
        // naming convention (back-compat with the subject-calib analysis flow).
        if (pair.videos.empty()) {
            pair.videos.push_back(resolve_session_path(base, "raw/" + pair.pose + "_cam0.mp4"));
            pair.videos.push_back(resolve_session_path(base, "raw/" + pair.pose + "_cam1.mp4"));
        }
        session.pairs.push_back(std::move(pair));
    }
    if (session.pairs.empty()) {
        throw std::runtime_error("pose_session has no usable poses");
    }
    // Every pose in a session must use the same camera count so a single
    // Triangulator (built for cam0..cam{n-1}) can process all clips.
    const std::size_t n_cams = session.pairs.front().videos.size();
    for (const auto& pair : session.pairs) {
        if (pair.videos.size() != n_cams) {
            throw std::runtime_error(
                "pose_session poses must all use the same camera count (pose '" +
                pair.pose + "' has " + std::to_string(pair.videos.size()) +
                ", expected " + std::to_string(n_cams) + ")");
        }
    }
    if (n_cams != 2 && n_cams != 3) {
        throw std::runtime_error(
            "pose_session camera count must be 2 or 3, got " + std::to_string(n_cams));
    }
    return session;
}

struct PoseStats {
    int frames = 0;
    int frames_with_3d = 0;
    std::vector<double> reproj;
};

struct ProfileAccumulator {
    std::string subject_id = "unknown";
    std::string source_session;
    double subject_height_m = 0.0;
    int frames = 0;
    int frames_with_3d = 0;
    std::vector<double> reproj;
    std::vector<double> valid_joints;
    // Sized for the largest topology; only the leading active_kp_count slots
    // are written by observe(), and major-bone iteration uses the active
    // SkeletonDef so the trailing zero-entries are never inspected.
    std::array<std::vector<double>, fitra::infer::kMaxKeypoints> bone_samples;
    std::vector<double> shoulder_samples;
    std::vector<double> hip_samples;
    std::map<std::string, PoseStats> poses;

    void observe(const std::string& pose,
                 const fitra::infer::Skeleton3D& skel,
                 const fitra::lift::TriangulatedSkeleton& tri) {
        frames += 1;
        auto& ps = poses[pose];
        ps.frames += 1;
        if (tri.valid_joints <= 0) return;
        frames_with_3d += 1;
        ps.frames_with_3d += 1;
        reproj.push_back(tri.median_reproj_px);
        ps.reproj.push_back(tri.median_reproj_px);
        valid_joints.push_back(static_cast<double>(tri.valid_joints));

        const auto& def = fitra::lift::active_skeleton_def();
        for (std::size_t child = 0; child < def.parents.size(); ++child) {
            int parent = def.parents[child];
            if (parent < 0) continue;
            const auto& a = skel.joints[static_cast<std::size_t>(parent)];
            const auto& b = skel.joints[child];
            if (!a.valid || !b.valid) continue;
            double len = joint_distance(a, b);
            if (len > 0.03 && len < 2.0) bone_samples[child].push_back(len);
        }
        const auto& ls = skel.joints[5];
        const auto& rs = skel.joints[6];
        if (ls.valid && rs.valid) {
            double len = joint_distance(ls, rs);
            if (len > 0.03 && len < 1.0) shoulder_samples.push_back(len);
        }
        const auto& lh = skel.joints[11];
        const auto& rh = skel.joints[12];
        if (lh.valid && rh.valid) {
            double len = joint_distance(lh, rh);
            if (len > 0.03 && len < 1.0) hip_samples.push_back(len);
        }
    }

    fitra::lift::SubjectProfile build_profile() const {
        fitra::lift::SubjectProfile profile = fitra::lift::make_default_subject_profile();
        profile.loaded = true;
        profile.subject_id = subject_id;
        profile.created_at = now_iso_like();
        profile.source_session = source_session;
        profile.subject_height_m = subject_height_m;
        const std::size_t kp_count = fitra::lift::active_kp_count();
        for (std::size_t child = 0; child < kp_count; ++child) {
            profile.bone_lengths_m[child] = median(bone_samples[child]);
        }
        profile.shoulder_width_m = median(shoulder_samples);
        profile.hip_width_m = median(hip_samples);
        if (profile.hip_width_m > 1.0e-6 && profile.bone_lengths_m[12] <= 1.0e-6) {
            profile.bone_lengths_m[12] = profile.hip_width_m;
        }
        return profile;
    }

    double major_coverage(const fitra::lift::SubjectProfile& profile) const {
        const auto& def = fitra::lift::active_skeleton_def();
        int have = 0;
        int total = static_cast<int>(def.major_bone_children.size()) + 1;
        for (int child : def.major_bone_children) {
            if (profile.bone_lengths_m[static_cast<std::size_t>(child)] > 1.0e-6) ++have;
        }
        if (profile.shoulder_width_m > 1.0e-6) ++have;
        return static_cast<double>(have) / static_cast<double>(std::max(1, total));
    }

    double profile_drift_pct(const fitra::lift::SubjectProfile& profile) const {
        const auto& def = fitra::lift::active_skeleton_def();
        std::vector<double> vals;
        for (int child : def.major_bone_children) {
            double target = profile.bone_lengths_m[static_cast<std::size_t>(child)];
            if (target <= 1.0e-6) continue;
            for (double sample : bone_samples[static_cast<std::size_t>(child)]) {
                vals.push_back(std::abs(sample - target) / target * 100.0);
            }
        }
        return median(vals);
    }

    double left_right_diff_pct(const fitra::lift::SubjectProfile& profile) const {
        constexpr std::array<std::pair<int, int>, 4> pairs{{
            {7, 8}, {9, 10}, {13, 14}, {15, 16},
        }};
        std::vector<double> diffs;
        for (auto [left, right] : pairs) {
            double a = profile.bone_lengths_m[static_cast<std::size_t>(left)];
            double b = profile.bone_lengths_m[static_cast<std::size_t>(right)];
            double denom = 0.5 * (a + b);
            if (denom > 1.0e-6) diffs.push_back(std::abs(a - b) / denom * 100.0);
        }
        return median(diffs);
    }

    std::string quality_status(const fitra::lift::SubjectProfile& profile) const {
        const double reproj_med = median(reproj);
        const double joints_med = median(valid_joints);
        const double coverage = major_coverage(profile);
        const double drift = profile_drift_pct(profile);
        const double lr_diff = left_right_diff_pct(profile);
        if (reproj_med <= 4.0 && joints_med >= 13.0 && coverage >= 0.80 &&
            drift <= 5.0 && lr_diff <= 15.0) {
            return "pass";
        }
        if (reproj_med <= 6.0 && coverage >= 0.60) {
            return "warn";
        }
        return "fail";
    }

    void write_quality(const std::string& path,
                       const fitra::lift::SubjectProfile& profile,
                       const std::string& profile_path) const {
        if (path.empty()) return;
        std::filesystem::path outp{path};
        if (outp.has_parent_path()) std::filesystem::create_directories(outp.parent_path());
        std::ofstream out{path, std::ios::trunc};
        if (!out.is_open()) throw std::runtime_error("failed to write quality: " + path);
        out << "{\n";
        out << "  \"schema\":\"fitra_subject_profile_quality_v1\",\n";
        out << "  \"subject_id\":\"" << json_escape(subject_id) << "\",\n";
        out << "  \"status\":\"" << json_escape(profile.quality_status) << "\",\n";
        out << "  \"frames\":" << frames << ",\n";
        out << "  \"frames_with_3d\":" << frames_with_3d << ",\n";
        out << "  \"reproj_err_med_px\":" << fmt(median(reproj)) << ",\n";
        out << "  \"valid_joints_median\":" << fmt(median(valid_joints)) << ",\n";
        out << "  \"major_bone_coverage\":" << fmt(major_coverage(profile)) << ",\n";
        out << "  \"profile_bone_drift_pct\":" << fmt(profile_drift_pct(profile)) << ",\n";
        out << "  \"left_right_bone_diff_pct\":" << fmt(left_right_diff_pct(profile)) << ",\n";
        out << "  \"subject_profile\":\"" << json_escape(profile_path) << "\",\n";
        out << "  \"poses\":[";
        bool first = true;
        for (const auto& [name, ps] : poses) {
            if (!first) out << ",";
            first = false;
            out << "{\"name\":\"" << json_escape(name) << "\",";
            out << "\"frames\":" << ps.frames << ",";
            out << "\"frames_with_3d\":" << ps.frames_with_3d << ",";
            out << "\"reproj_err_med_px\":" << fmt(median(ps.reproj)) << "}";
        }
        out << "]\n";
        out << "}\n";
    }
};

std::vector<fitra::infer::Bbox> keep_largest_if_needed(
    std::vector<fitra::infer::Bbox> bboxes,
    bool multi_person) {
    if (multi_person || bboxes.size() <= 1) return bboxes;
    auto largest = std::max_element(
        bboxes.begin(), bboxes.end(),
        [](const auto& a, const auto& b) {
            float aa = (a.x2 - a.x1) * (a.y2 - a.y1);
            float bb = (b.x2 - b.x1) * (b.y2 - b.y1);
            return aa < bb;
        });
    fitra::infer::Bbox keep = *largest;
    return {keep};
}

void draw_2d(cv::Mat& frame, const std::vector<fitra::infer::Person>& persons) {
    const cv::Scalar color(0, 210, 0);
    const auto& def = fitra::lift::active_skeleton_def();
    for (const auto& p : persons) {
        cv::rectangle(frame,
                      {static_cast<int>(p.bbox.x1), static_cast<int>(p.bbox.y1)},
                      {static_cast<int>(p.bbox.x2), static_cast<int>(p.bbox.y2)},
                      cv::Scalar(80, 80, 80), 1, cv::LINE_AA);
        for (auto [a, b] : def.edges) {
            const auto& ka = p.kpts[static_cast<std::size_t>(a)];
            const auto& kb = p.kpts[static_cast<std::size_t>(b)];
            if (ka.score < 0.3f || kb.score < 0.3f) continue;
            cv::line(frame, {static_cast<int>(ka.x), static_cast<int>(ka.y)},
                     {static_cast<int>(kb.x), static_cast<int>(kb.y)},
                     color, 1, cv::LINE_AA);
        }
    }
}

void draw_reprojection(cv::Mat& frame,
                       const fitra::lift::Triangulator& triangulator,
                       int cam_index,
                       const fitra::infer::Skeleton3D& skel) {
    const cv::Scalar color(255, 0, 255);
    const auto& def = fitra::lift::active_skeleton_def();
    std::array<cv::Point2f, fitra::infer::kMaxKeypoints> pts{};
    std::array<bool, fitra::infer::kMaxKeypoints> ok{};
    const std::size_t kp_count = fitra::lift::active_kp_count();
    for (std::size_t k = 0; k < kp_count; ++k) {
        ok[k] = triangulator.project(cam_index, skel.joints[k], pts[k]);
        if (ok[k]) {
            cv::circle(frame, {static_cast<int>(pts[k].x), static_cast<int>(pts[k].y)},
                       4, color, -1, cv::LINE_AA);
        }
    }
    for (auto [a, b] : def.edges) {
        if (!ok[static_cast<std::size_t>(a)] || !ok[static_cast<std::size_t>(b)]) continue;
        cv::line(frame,
                 {static_cast<int>(pts[static_cast<std::size_t>(a)].x),
                  static_cast<int>(pts[static_cast<std::size_t>(a)].y)},
                 {static_cast<int>(pts[static_cast<std::size_t>(b)].x),
                  static_cast<int>(pts[static_cast<std::size_t>(b)].y)},
                 color, 2, cv::LINE_AA);
    }
}

void write_json_line(std::ofstream& out,
                     int frame,
                     const std::string& pose,
                     const fitra::infer::Skeleton3D& skel,
                     const fitra::lift::TriangulatedSkeleton& tri,
                     double bone_drift_before,
                     double bone_drift_after,
                     bool ik_locked,
                     const std::array<fitra::slimevr::SlimeTracker,
                                       fitra::slimevr::kTrackerCount>* trackers) {
    out << "{\"frame\":" << frame
        << ",\"kp_format\":\""
        << fitra::lift::keypoint_format_name(fitra::lift::active_keypoint_format())
        << "\",\"pose\":\"" << json_escape(pose)
        << "\",\"persons_3d\":[{\"id\":0,\"joints\":[";
    const std::size_t emit_n = std::min<std::size_t>(
        skel.kp_count, skel.joints.size());
    for (std::size_t k = 0; k < emit_n; ++k) {
        if (k) out << ",";
        const auto& j = skel.joints[k];
        out << "[" << fmt(j.x) << "," << fmt(j.y) << "," << fmt(j.z)
            << "," << fmt(j.score) << "," << (j.valid ? "true" : "false") << "]";
    }
    out << "]}],\"stats\":{";
    out << "\"valid_joints\":" << tri.valid_joints;
    out << ",\"reproj_err_med_px\":" << fmt(tri.median_reproj_px);
    out << ",\"bone_len_drift_before_pct\":" << fmt(bone_drift_before);
    out << ",\"bone_len_drift_pct\":" << fmt(bone_drift_after);
    out << ",\"ik_locked\":" << (ik_locked ? "true" : "false");
    out << ",\"joint_view_counts\":[";
    for (std::size_t k = 0; k < emit_n; ++k) {
        if (k) out << ",";
        out << tri.view_count[k];
    }
    out << "],\"joint_reproj_px\":[";
    for (std::size_t k = 0; k < emit_n; ++k) {
        if (k) out << ",";
        out << fmt(tri.reproj_error_px[k]);
    }
    out << "]}";  // close "stats"
    if (trackers != nullptr) {
        // RAW SlimeVR trackers (no temporal smoothing): one entry per role in
        // TrackerRole order, [role, px,py,pz, qw,qx,qy,qz, valid, roll_conf].
        // The analyzer's `trackers` subcommand consumes this.
        out << ",\"trackers\":[";
        for (std::size_t t = 0; t < fitra::slimevr::kTrackerCount; ++t) {
            if (t) out << ",";
            const auto& tr = (*trackers)[t];
            out << "[" << static_cast<int>(tr.role)
                << "," << fmt(tr.pos[0]) << "," << fmt(tr.pos[1]) << "," << fmt(tr.pos[2])
                << "," << fmt(tr.quat_wxyz[0]) << "," << fmt(tr.quat_wxyz[1])
                << "," << fmt(tr.quat_wxyz[2]) << "," << fmt(tr.quat_wxyz[3])
                << "," << (tr.valid ? "true" : "false")
                << "," << fmt(tr.roll_confidence) << "]";
        }
        out << "]";
    }
    out << "}\n";  // close record
}

// The spatial rigid-segment fit and spine soft coupling live in lift/rigid_fit
// (fitra::lift::apply_segment_rigid_fit / apply_spine_coupling) so this offline
// harness and the live pipeline denoise with identical code.

}  // namespace

int main(int argc, char** argv) {
    Args args = parse_args(argc, argv);

    try {
        {
            fitra::lift::KeypointFormat fmt;
            if (!fitra::lift::parse_keypoint_format(args.keypoint_format_str, fmt)) {
                std::fprintf(stderr,
                    "unknown --keypoint-format %s (use coco17 or halpe26)\n",
                    args.keypoint_format_str.c_str());
                return EXIT_FAILURE;
            }
            fitra::lift::set_active_keypoint_format(fmt);
        }
        auto calib = fitra::lift::load_calibration(args.calib);

        // Resolve the jobs (and thus the camera count) before building the
        // Triangulator: the count is the number of clips per pose-session entry,
        // or the number of --video inputs in the standalone case.
        PoseSession pose_session;
        std::vector<VideoPair> jobs;
        if (!args.pose_session.empty()) {
            pose_session = load_pose_session(args.pose_session);
            jobs = pose_session.pairs;
            if (args.subject_height_m <= 0.0) {
                args.subject_height_m = pose_session.subject_height_m;
            }
        } else {
            VideoPair pair;
            pair.pose = "video";
            pair.videos = args.videos;  // already validated as size 2 or 3
            jobs.push_back(pair);
            pose_session.source_session = "";
        }
        const std::size_t n_cams = jobs.front().videos.size();

        // A rig's extrinsics file may carry more cameras than this run uses, or
        // store them in a different order. Select cam0..cam{n-1} in order so a
        // 3-camera extrinsics is valid input for a 2- or 3-view run and the order
        // matches require_camera_ids (mirrors make_threed / threed_builder;
        // pose-3d-calib-latest-resolution.md).
        const std::vector<std::string> cam_ids = expected_camera_ids(n_cams);
        calib = fitra::lift::select_calib_cameras(calib, cam_ids);
        fitra::lift::Triangulator::Options tri_opts;
        tri_opts.kp_conf_thresh = args.kp_conf_thresh;
        tri_opts.max_reproj_px = args.max_reproj_px;
        fitra::lift::Triangulator triangulator{calib, tri_opts};
        triangulator.require_camera_ids(cam_ids);

        TrtLogger tlog;
        std::unique_ptr<nvinfer1::IRuntime> runtime{nvinfer1::createInferRuntime(tlog)};
        TRT_CHECK(runtime != nullptr);
        auto yolox_engine = fitra::infer::TrtEngine::from_file(*runtime, args.det_engine, tlog);
        auto rtmpose_engine = fitra::infer::TrtEngine::from_file(*runtime, args.pose_engine, tlog);
        fitra::infer::Yolox::Options yolo_opts;
        yolo_opts.score_thr = args.det_score;
        fitra::infer::Yolox yolox{*yolox_engine, yolo_opts};
        fitra::infer::RtmPose rtmpose{*rtmpose_engine};

        std::filesystem::path outp{args.out};
        if (outp.has_parent_path()) std::filesystem::create_directories(outp.parent_path());
        std::ofstream jout{args.out, std::ios::trunc};
        if (!jout.is_open()) throw std::runtime_error("failed to open output: " + args.out);

        // Subject profile (input): locks IK bone lengths and supplies the
        // rigid-fit template — same source the live pipeline uses.
        fitra::lift::SubjectProfile subject_profile;
        bool have_profile = false;
        if (!args.subject_profile.empty()) {
            subject_profile = fitra::lift::load_subject_profile(args.subject_profile);
            have_profile = true;
        }

        // Spatial rigid segment templates (Halpe26 only). valid=false unless the
        // matching flag is set and the profile triangle is usable.
        fitra::lift::RigidTemplate pelvis_template;   // {hip_center,l_hip,r_hip}
        fitra::lift::RigidTemplate girdle_template;   // {neck,l_shoulder,r_shoulder}
        double spine_len = 0.0;                        // hip_center -> neck
        const bool rigid_any = args.rigid_pelvis || args.rigid_shoulders;
        if (rigid_any) {
            if (fitra::lift::active_keypoint_format() != fitra::lift::KeypointFormat::Halpe26) {
                std::fprintf(stderr, "--rigid-pelvis/--rigid-shoulders require --keypoint-format halpe26\n");
                return EXIT_FAILURE;
            }
            if (!have_profile) {
                std::fprintf(stderr, "--rigid-pelvis/--rigid-shoulders require --subject-profile (template distances)\n");
                return EXIT_FAILURE;
            }
        }
        if (args.dump_trackers &&
            fitra::lift::active_keypoint_format() != fitra::lift::KeypointFormat::Halpe26) {
            std::fprintf(stderr, "--dump-trackers requires --keypoint-format halpe26 "
                                 "(extract_trackers needs neck/hip_center/toe joints)\n");
            return EXIT_FAILURE;
        }
        const bool tm_one_euro = args.tracker_smoothing == "one_euro";
        const bool tm_st       = args.tracker_smoothing == "st";
        if (args.tracker_smoothing != "raw" && !tm_one_euro && !tm_st) {
            std::fprintf(stderr, "--tracker-smoothing must be raw|one_euro|st (got %s)\n",
                         args.tracker_smoothing.c_str());
            return EXIT_FAILURE;
        }
        if ((tm_one_euro || tm_st) && !args.dump_trackers) {
            std::fprintf(stderr, "--tracker-smoothing %s requires --dump-trackers\n",
                         args.tracker_smoothing.c_str());
            return EXIT_FAILURE;
        }
        // Tracker-stage smoothing config (mirrors TrackerExtractorOptions defaults
        // + MainConfig One Euro params). One Euro drives the swing base for both
        // one_euro and st; st adds the regime twist override + pos filter.
        const auto& st_cfg = fitra::slimevr::default_st_config();
        const fitra::slimevr::OneEuroParams pos_euro{1.0f, 4.0f, 1.0f};
        const fitra::slimevr::OneEuroParams quat_euro{1.5f, 1.5f, 1.0f};
        constexpr float kNominalDt = 1.0f / 60.0f;  // live extract_rate_hz
        // Floor-contact grounding (M-D) — last 3D stage, after Kalman+IK.
        fitra::lift::FloorGroundingOptions fg_opts;
        fg_opts.floor_z_m      = args.floor_z_m;
        fg_opts.stance_vel_mps = args.floor_stance_vel_mps;
        fg_opts.snap_band_m    = args.floor_snap_band_m;
        if (args.rigid_pelvis) {
            pelvis_template = fitra::lift::RigidTemplate::from_distances(
                subject_profile.bone_lengths_m[11],   // hip_center -> l_hip
                subject_profile.bone_lengths_m[12],   // hip_center -> r_hip
                subject_profile.hip_width_m);          // l_hip <-> r_hip
            if (!pelvis_template.valid) {
                std::fprintf(stderr,
                    "--rigid-pelvis: profile pelvis triangle degenerate "
                    "(l_hip=%.4f r_hip=%.4f width=%.4f)\n",
                    subject_profile.bone_lengths_m[11], subject_profile.bone_lengths_m[12],
                    subject_profile.hip_width_m);
                return EXIT_FAILURE;
            }
            FITRA_LOG_INFO("rigid-pelvis: template d(hc,lh)={} d(hc,rh)={} hip_w={} (spatial-first)",
                           fmt(subject_profile.bone_lengths_m[11]),
                           fmt(subject_profile.bone_lengths_m[12]),
                           fmt(subject_profile.hip_width_m));
        }
        if (args.rigid_shoulders) {
            girdle_template = fitra::lift::RigidTemplate::from_distances(
                subject_profile.bone_lengths_m[5],    // neck -> l_shoulder
                subject_profile.bone_lengths_m[6],    // neck -> r_shoulder
                subject_profile.shoulder_width_m);     // l_shoulder <-> r_shoulder
            spine_len = subject_profile.bone_lengths_m[18];  // hip_center -> neck
            if (!girdle_template.valid) {
                std::fprintf(stderr,
                    "--rigid-shoulders: profile shoulder-girdle triangle degenerate "
                    "(l_sh=%.4f r_sh=%.4f width=%.4f)\n",
                    subject_profile.bone_lengths_m[5], subject_profile.bone_lengths_m[6],
                    subject_profile.shoulder_width_m);
                return EXIT_FAILURE;
            }
            if (spine_len <= 1.0e-6) {
                std::fprintf(stderr, "--rigid-shoulders: profile spine length (bone[18]) missing\n");
                return EXIT_FAILURE;
            }
            FITRA_LOG_INFO("rigid-shoulders: girdle d(nk,ls)={} d(nk,rs)={} sh_w={} spine={} tol=+/-{}%",
                           fmt(subject_profile.bone_lengths_m[5]),
                           fmt(subject_profile.bone_lengths_m[6]),
                           fmt(subject_profile.shoulder_width_m),
                           fmt(spine_len), fmt(args.spine_tol * 100.0));
        }

        fitra::lift::IkSolver::Options ik_opts;
        ik_opts.bone_calib_frames = std::max(1, args.bone_calib_frames);
        ik_opts.subject_height_m = args.subject_height_m;
        if (have_profile) {
            ik_opts.has_subject_profile = true;
            ik_opts.subject_profile = subject_profile;
        }
        fitra::lift::IkSolver ik{ik_opts};

        ProfileAccumulator profile_acc;
        profile_acc.subject_id = pose_session.subject_id.empty() ? "unknown" : pose_session.subject_id;
        profile_acc.source_session = pose_session.source_session;
        profile_acc.subject_height_m = args.subject_height_m;

        std::vector<double> med_reproj_values;
        std::vector<double> drift_values;
        int processed = 0;
        int global_frame = 0;
        int frames_with_3d = 0;
        auto start = std::chrono::steady_clock::now();

        // Per-camera start skips for pairing. --cam{K}-frame-offset > 0 skips
        // camK; < 0 skips cam0 (the shared reference) by |N|. Each offset
        // asserts the pairwise invariant start_skip[K] − start_skip[0] ==
        // offset_K, so a common cam0 skip (base) must be compensated on EVERY
        // other camera — including ones whose own offset is 0 (they must follow
        // cam0's skip to stay aligned with it).
        std::vector<int> start_skip(n_cams, 0);
        const int off1 = args.cam1_frame_offset;
        int off2 = 0;
        if (n_cams >= 3) {
            off2 = args.cam2_frame_offset;
        } else if (args.cam2_frame_offset != 0) {
            FITRA_LOG_WARN("--cam2-frame-offset ignored for a {}-camera run", n_cams);
        }
        const int base = std::max({0, -off1, -off2});
        start_skip[0] = base;
        start_skip[1] = base + off1;  // ≥ 0 by construction of base
        if (n_cams >= 3) start_skip[2] = base + off2;

        for (const auto& job : jobs) {
            if (args.max_frames > 0 && processed >= args.max_frames) break;
            std::vector<cv::VideoCapture> caps;
            caps.reserve(n_cams);
            for (const auto& path : job.videos) {
                caps.emplace_back(path);
                if (!caps.back().isOpened()) throw std::runtime_error("failed to open video: " + path);
            }

            for (std::size_t cam = 0; cam < n_cams; ++cam) {
                cv::Mat tmp;
                for (int i = 0; i < start_skip[cam]; ++i) caps[cam].read(tmp);
            }

            // record_3cam clips carry a nominal header fps that can differ from
            // the true per-frame interval (encode-bound sync rate). Prefer the
            // explicit --fps (recorder meta.json fps_written) so the Kalman dt is
            // real; else fall back to the MP4 header.
            double fps = args.fps_override > 0.0 ? args.fps_override
                                                 : caps[0].get(cv::CAP_PROP_FPS);
            if (fps <= 0.0) fps = 30.0;

            std::vector<std::unique_ptr<cv::VideoWriter>> overlays(n_cams);
            if (!args.overlay_dir.empty()) {
                std::filesystem::create_directories(args.overlay_dir);
                for (std::size_t i = 0; i < n_cams; ++i) {
                    // Size each writer from its own camera: a rig may pair cameras
                    // of different resolutions, and a writer sized to cam0 would
                    // drop every frame from a differently-sized camera.
                    int w = static_cast<int>(caps[i].get(cv::CAP_PROP_FRAME_WIDTH));
                    int h = static_cast<int>(caps[i].get(cv::CAP_PROP_FRAME_HEIGHT));
                    std::string name = jobs.size() > 1
                        ? (job.pose + "_cam" + std::to_string(i) + "_reproj.mp4")
                        : ("cam" + std::to_string(i) + "_reproj.mp4");
                    std::filesystem::path path = std::filesystem::path(args.overlay_dir) / name;
                    overlays[i] = std::make_unique<cv::VideoWriter>(
                        path.string(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                        fps, cv::Size(w, h));
                    if (!overlays[i]->isOpened()) {
                        throw std::runtime_error("failed to open overlay writer: " + path.string());
                    }
                }
            }

            // st mode: the chain Kalman is untouched by default (weaken = 1,
            // the M-C4 data-driven setting mirrored by the live pipeline's
            // kStWeaken; the M-C3 seed ×100 injected jitter at rest). A
            // non-1 --st-kalman-weaken scales the process noise for sweeps.
            fitra::lift::SkeletonKalman::Options kopts;
            if (tm_st && args.st_kalman_weaken != 1.0) {
                const double w = args.st_kalman_weaken;
                kopts.q_pos *= w; kopts.q_vel *= w;
                kopts.q_pos_offset *= w; kopts.q_vel_offset *= w;
            }
            fitra::lift::SkeletonKalman kalman{kopts};
            // Per-job foot-anchor state for extract_trackers' FK fallback,
            // mirroring TrackerExtractor::extract_ctx_. Declared inside the job
            // loop so it resets between poses (matches `kalman`).
            fitra::slimevr::ExtractContext tracker_ctx;
            fitra::lift::FloorGroundingState fg_state;  // per-job (resets between poses)
            // Per-job tracker-smoothing state (reset between poses like `kalman`).
            std::array<cv::Vec4f, fitra::slimevr::kTrackerCount> tk_prev_quat;
            tk_prev_quat.fill(cv::Vec4f{1.0f, 0.0f, 0.0f, 0.0f});
            std::array<cv::Vec3f, fitra::slimevr::kTrackerCount> tk_prev_pos{};
            fitra::slimevr::QuatSmoothingContext tk_quat_ctx{};
            fitra::slimevr::PosSmoothingContext tk_pos_ctx{};
            fitra::slimevr::StPosState  tk_st_pos{};
            fitra::slimevr::StTwistState tk_st_twist{};
            for (int frame_idx = 0;; ++frame_idx) {
                if (args.max_frames > 0 && processed >= args.max_frames) break;
                std::vector<cv::Mat> frames(n_cams);
                bool ok = true;
                for (std::size_t cam = 0; cam < n_cams; ++cam) {
                    ok = caps[cam].read(frames[cam]) && !frames[cam].empty();
                    if (!ok) break;
                }
                if (!ok) break;

                std::vector<std::vector<fitra::infer::Person>> persons_by_cam(n_cams);
                std::vector<fitra::lift::PerCameraObservation> observations;
                for (std::size_t cam = 0; cam < n_cams; ++cam) {
                    auto bboxes = keep_largest_if_needed(yolox.infer(frames[cam]),
                                                         args.multi_person);
                    persons_by_cam[cam] = rtmpose.infer(frames[cam], bboxes);
                    if (!persons_by_cam[cam].empty()) {
                        observations.push_back(
                            fitra::lift::PerCameraObservation{
                                static_cast<int>(cam), &persons_by_cam[cam][0]});
                    }
                }

                auto tri = triangulator.triangulate(observations);
                auto skel = tri.skeleton;
                double drift_before = 0.0;
                double drift_after = 0.0;
                // Run the enabled rigid stages; `applied` tracks whether any fit
                // actually modified the skeleton this frame. Order: 1) pelvis,
                // 2) shoulder girdle, 3) spine soft coupling (neck bounded to the
                // now-fitted hip_center). Each stage is a no-op (returns false /
                // self-guards) when its joints are missing/degenerate.
                bool applied = false;
                if (rigid_any) {
                    if (args.rigid_pelvis)
                        applied |= fitra::lift::apply_segment_rigid_fit(skel, tri, pelvis_template, {19, 11, 12});
                    if (args.rigid_shoulders) {
                        applied |= fitra::lift::apply_segment_rigid_fit(skel, tri, girdle_template, {18, 5, 6});
                        fitra::lift::apply_spine_coupling(skel, 19, 18, {18, 5, 6}, spine_len, args.spine_tol);
                    }
                }
                if (applied) {
                    // Spatial-first (design A): tri -> rigid fit(s) -> IK -> Kalman.
                    drift_before = ik.bone_drift_pct(skel);
                    // Observe the PRE-FIT triangulation, not `skel`: the rigid
                    // fit above has already forced the pelvis/girdle joints onto
                    // the *input* profile's template distances (a rigid transform
                    // preserves them exactly), so observing the fitted skeleton
                    // would make --subject-profile-out echo the input profile
                    // instead of the measured geometry — the same echo the
                    // pre-IK placement exists to avoid.
                    profile_acc.observe(job.pose, tri.skeleton, tri);
                    if (!args.no_ik) skel = ik.update(skel);
                    if (!args.no_kalman) skel = kalman.update(skel, 1.0 / fps);
                    drift_after = ik.bone_drift_pct(skel);
                } else {
                    // Baseline (design B): tri -> Kalman -> IK. Also the fallback
                    // when no rigid stage applied (flags off, or every enabled fit
                    // failed this frame) so such frames match the no-rigid path.
                    if (!args.no_kalman) skel = kalman.update(skel, 1.0 / fps);
                    profile_acc.observe(job.pose, skel, tri);
                    drift_before = ik.bone_drift_pct(skel);
                    if (!args.no_ik) skel = ik.update(skel);
                    drift_after = ik.bone_drift_pct(skel);
                }

                // Floor-contact grounding (M-D): last 3D stage (after Kalman+IK),
                // mirrors multi_pipeline. No-op on COCO17 / when off.
                if (args.floor_grounding) {
                    fitra::lift::apply_floor_grounding(skel, fg_state, 1.0 / fps, fg_opts);
                }

                // Tracker harness (M-C1): RAW SlimeVR trackers from the final
                // skeleton, no temporal smoothing (that lives downstream in
                // TrackerExtractor). Product placement defaults (foot=Ankle,
                // chest/waist spine fracs 0.65/0.15) so the geometry matches the
                // live pipeline. Emitted on every frame (all-invalid on a
                // dropout) so the trackers array stays frame-aligned with
                // persons_3d.
                std::array<fitra::slimevr::SlimeTracker,
                           fitra::slimevr::kTrackerCount> trackers{};
                if (args.dump_trackers) {
                    trackers = fitra::slimevr::extract_trackers(
                        skel, &tracker_ctx, fitra::slimevr::FootPosMode::Ankle,
                        0.65f, 0.15f, args.roll_hysteresis);
                    // Optional tracker-stage smoothing (one_euro / st), replaying
                    // the live TrackerExtractor per 3D frame at the clip cadence.
                    if (tm_one_euro || tm_st) {
                        const float dtf = static_cast<float>(1.0 / fps);
                        // Hip context for the position hold (skel[19]=hip_center).
                        const auto& hc = skel.joints[19];
                        tk_pos_ctx.hip_valid = hc.valid;
                        if (hc.valid) tk_pos_ctx.current_hip_pos = cv::Vec3f{hc.x, hc.y, hc.z};
                        tk_pos_ctx.dt_s = dtf;
                        if (tm_st) {
                            std::array<float, fitra::slimevr::kTrackerCount> tw;
                            fitra::slimevr::fill_st_twist_overrides(
                                trackers, tk_prev_quat, tk_st_twist, st_cfg, dtf, kNominalDt, tw);
                            fitra::slimevr::apply_quat_smoothing(
                                trackers, tk_prev_quat, tk_quat_ctx, quat_euro, dtf, kNominalDt, &tw);
                            const cv::Vec3f* waist_fallback =
                                tk_pos_ctx.hip_valid ? &tk_pos_ctx.current_hip_pos : nullptr;
                            fitra::slimevr::apply_pos_st_filter(
                                trackers, tk_st_pos, st_cfg, dtf, kNominalDt, waist_fallback);
                        } else {  // one_euro
                            fitra::slimevr::apply_quat_smoothing(
                                trackers, tk_prev_quat, tk_quat_ctx, quat_euro, dtf, kNominalDt);
                            fitra::slimevr::apply_pos_smoothing(
                                trackers, tk_prev_pos, tk_pos_ctx, pos_euro, kNominalDt);
                        }
                    }
                }
                write_json_line(jout, global_frame++, job.pose, skel, tri,
                                drift_before, drift_after, ik.locked(),
                                args.dump_trackers ? &trackers : nullptr);
                if (tri.valid_joints > 0) {
                    frames_with_3d += 1;
                    med_reproj_values.push_back(tri.median_reproj_px);
                    if (ik.locked()) drift_values.push_back(drift_after);
                }

                for (std::size_t cam = 0; cam < n_cams; ++cam) {
                    if (!overlays[cam]) continue;
                    draw_2d(frames[cam], persons_by_cam[cam]);
                    draw_reprojection(frames[cam], triangulator, static_cast<int>(cam), skel);
                    overlays[cam]->write(frames[cam]);
                }

                processed += 1;
                if (processed % 25 == 0) {
                    FITRA_LOG_INFO("processed {} frames; pose={} last_valid_joints={} reproj_med={}",
                                   processed, job.pose, tri.valid_joints, fmt(tri.median_reproj_px));
                }
            }

            for (auto& writer : overlays) {
                if (writer) writer->release();
            }
        }

        auto end = std::chrono::steady_clock::now();
        double secs = std::chrono::duration<double>(end - start).count();

        fitra::lift::SubjectProfile generated_profile;
        bool wrote_profile = false;
        if (!args.subject_profile_out.empty() || !args.quality_out.empty()) {
            generated_profile = profile_acc.build_profile();
            generated_profile.quality_status = profile_acc.quality_status(generated_profile);
            if (!args.subject_profile_out.empty()) {
                fitra::lift::write_subject_profile(args.subject_profile_out, generated_profile);
                wrote_profile = true;
            }
            profile_acc.write_quality(args.quality_out, generated_profile, args.subject_profile_out);
        }

        std::filesystem::path sump{args.summary};
        if (sump.has_parent_path()) std::filesystem::create_directories(sump.parent_path());
        std::ofstream sout{args.summary, std::ios::trunc};
        if (!sout.is_open()) throw std::runtime_error("failed to open summary: " + args.summary);
        sout << "{\n";
        sout << "  \"frames\":" << processed << ",\n";
        sout << "  \"frames_with_3d\":" << frames_with_3d << ",\n";
        sout << "  \"runtime_fps\":" << fmt(processed / std::max(secs, 1.0e-9)) << ",\n";
        sout << "  \"reproj_err_med_px\":" << fmt(median(med_reproj_values)) << ",\n";
        sout << "  \"bone_len_drift_pct\":" << fmt(median(drift_values)) << ",\n";
        sout << "  \"ik_locked\":" << (ik.locked() ? "true" : "false") << ",\n";
        sout << "  \"subject_height_m\":" << fmt(args.subject_height_m) << ",\n";
        if (!args.pose_session.empty()) {
            sout << "  \"pose_session\":\"" << json_escape(args.pose_session) << "\",\n";
        }
        if (wrote_profile) {
            sout << "  \"subject_profile\":\"" << json_escape(args.subject_profile_out) << "\",\n";
            sout << "  \"profile_quality_status\":\""
                 << json_escape(generated_profile.quality_status) << "\",\n";
        }
        sout << "  \"num_cameras\":" << n_cams << ",\n";
        sout << "  \"videos\":[";
        for (std::size_t i = 0; i < jobs.front().videos.size(); ++i) {
            if (i) sout << ",";
            sout << "\"" << json_escape(jobs.front().videos[i]) << "\"";
        }
        sout << "],\n";
        sout << "  \"calib\":\"" << json_escape(args.calib) << "\"\n";
        sout << "}\n";

        FITRA_LOG_INFO("done: {} frames, {} with 3D, median reproj={} px -> {}",
                       processed, frames_with_3d, fmt(median(med_reproj_values)), args.out);
        FITRA_LOG_INFO("summary -> {}", args.summary);
        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        FITRA_LOG_ERROR("fatal: {}", e.what());
        return EXIT_FAILURE;
    }
}
