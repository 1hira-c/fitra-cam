// dump_keypoints_3d — offline 2-camera 3D dump.
//
// Runs YOLOX + RTMPose on synchronized recorded videos, triangulates either
// COCO17 (17 kpts, default) or Halpe26 (26 kpts) joints with a calibration
// YAML, then optionally applies 3D Kalman + IK.

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
#include "lift/keypoint_format.hpp"
#include "lift/skeleton_def.hpp"
#include "lift/subject_profile.hpp"
#include "lift/triangulator.hpp"
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
    std::string subject_profile_out;
    std::string quality_out;
    int max_frames = 0;
    int cam1_frame_offset = 0;
    int bone_calib_frames = 150;
    double subject_height_m = 0.0;
    float det_score = 0.5f;
    float kp_conf_thresh = 0.3f;
    float max_reproj_px = 6.0f;
    bool multi_person = false;
    bool no_kalman = false;
    bool no_ik = false;
    std::string keypoint_format_str = "coco17";
};

void print_help() {
    std::puts(
        "dump_keypoints_3d — offline two-camera 3D triangulation dump\n"
        "\n"
        "Required:\n"
        "  --video PATH              input MP4, repeat twice in cam order\n"
        "                            (optional when --pose-session is provided)\n"
        "  --calib PATH              calibration YAML with intrinsics/extrinsics (ids must be cam0,cam1)\n"
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
        "  --det-score F             YOLOX score threshold (default 0.5)\n"
        "  --kp-conf-thresh F        2D keypoint threshold for triangulation (default 0.3)\n"
        "  --max-reproj-px F         one-pass outlier threshold (default 6)\n"
        "  --bone-calib-frames N     frames used to lock IK bone lengths (default 150)\n"
        "  --subject-height-m F      lock IK bone lengths from Japanese anthropometry and height\n"
        "  --multi-person            keep all bboxes, but MVP triangulates person 0 only\n"
        "  --no-kalman               disable 3D Kalman smoothing\n"
        "  --no-ik                   disable IK length/hinge projection\n"
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
        else if (a == "--subject-profile-out") { args.subject_profile_out = need("--subject-profile-out"); }
        else if (a == "--quality-out")  { args.quality_out = need("--quality-out"); }
        else if (a == "--max-frames")   { args.max_frames = std::atoi(need("--max-frames")); }
        else if (a == "--cam1-frame-offset") { args.cam1_frame_offset = std::atoi(need("--cam1-frame-offset")); }
        else if (a == "--det-score")    { args.det_score = std::stof(need("--det-score")); }
        else if (a == "--kp-conf-thresh") { args.kp_conf_thresh = std::stof(need("--kp-conf-thresh")); }
        else if (a == "--max-reproj-px") { args.max_reproj_px = std::stof(need("--max-reproj-px")); }
        else if (a == "--bone-calib-frames") { args.bone_calib_frames = std::atoi(need("--bone-calib-frames")); }
        else if (a == "--subject-height-m") { args.subject_height_m = std::stod(need("--subject-height-m")); }
        else if (a == "--multi-person") { args.multi_person = true; }
        else if (a == "--no-kalman")    { args.no_kalman = true; }
        else if (a == "--no-ik")        { args.no_ik = true; }
        else if (a == "--keypoint-format") { args.keypoint_format_str = need("--keypoint-format"); }
        else {
            std::fprintf(stderr, "unknown arg: %s\n", argv[i]);
            print_help();
            std::exit(EXIT_FAILURE);
        }
    }
    if (((args.pose_session.empty() && args.videos.size() != 2) ||
         (!args.pose_session.empty() && !args.videos.empty() && args.videos.size() != 2)) ||
        args.calib.empty() || args.det_engine.empty() ||
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
    std::array<std::string, 2> videos{};
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
            int idx = 0;
            for (auto cit = clips.begin(); cit != clips.end() && idx < 2; ++cit, ++idx) {
                pair.videos[static_cast<std::size_t>(idx)] =
                    resolve_session_path(base, static_cast<std::string>(*cit));
            }
        }
        if (pair.videos[0].empty()) {
            pair.videos[0] = resolve_session_path(base, "raw/" + pair.pose + "_cam0.mp4");
        }
        if (pair.videos[1].empty()) {
            pair.videos[1] = resolve_session_path(base, "raw/" + pair.pose + "_cam1.mp4");
        }
        session.pairs.push_back(std::move(pair));
    }
    if (session.pairs.empty()) {
        throw std::runtime_error("pose_session has no usable poses");
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
                     bool ik_locked) {
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
    out << "]}}\n";
}

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
        // This analyzer is two-camera (cam0,cam1); a rig's extrinsics file may
        // carry more cameras, or store them in a different order. Always select
        // cam0,cam1 in order so a 3-camera extrinsics is valid input and the
        // order matches require_camera_ids (mirrors make_threed;
        // pose-3d-calib-latest-resolution.md).
        calib = fitra::lift::select_calib_cameras(calib, {"cam0", "cam1"});
        fitra::lift::Triangulator::Options tri_opts;
        tri_opts.kp_conf_thresh = args.kp_conf_thresh;
        tri_opts.max_reproj_px = args.max_reproj_px;
        fitra::lift::Triangulator triangulator{calib, tri_opts};
        triangulator.require_camera_ids({"cam0", "cam1"});

        TrtLogger tlog;
        std::unique_ptr<nvinfer1::IRuntime> runtime{nvinfer1::createInferRuntime(tlog)};
        TRT_CHECK(runtime != nullptr);
        auto yolox_engine = fitra::infer::TrtEngine::from_file(*runtime, args.det_engine, tlog);
        auto rtmpose_engine = fitra::infer::TrtEngine::from_file(*runtime, args.pose_engine, tlog);
        fitra::infer::Yolox::Options yolo_opts;
        yolo_opts.score_thr = args.det_score;
        fitra::infer::Yolox yolox{*yolox_engine, yolo_opts};
        fitra::infer::RtmPose rtmpose{*rtmpose_engine};

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
            pair.videos = {args.videos[0], args.videos[1]};
            jobs.push_back(pair);
            pose_session.source_session = "";
        }

        std::filesystem::path outp{args.out};
        if (outp.has_parent_path()) std::filesystem::create_directories(outp.parent_path());
        std::ofstream jout{args.out, std::ios::trunc};
        if (!jout.is_open()) throw std::runtime_error("failed to open output: " + args.out);

        fitra::lift::IkSolver::Options ik_opts;
        ik_opts.bone_calib_frames = std::max(1, args.bone_calib_frames);
        ik_opts.subject_height_m = args.subject_height_m;
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

        for (const auto& job : jobs) {
            if (args.max_frames > 0 && processed >= args.max_frames) break;
            std::vector<cv::VideoCapture> caps;
            caps.reserve(2);
            for (const auto& path : job.videos) {
                caps.emplace_back(path);
                if (!caps.back().isOpened()) throw std::runtime_error("failed to open video: " + path);
            }

            if (args.cam1_frame_offset > 0) {
                cv::Mat tmp;
                for (int i = 0; i < args.cam1_frame_offset; ++i) caps[1].read(tmp);
            } else if (args.cam1_frame_offset < 0) {
                cv::Mat tmp;
                for (int i = 0; i < -args.cam1_frame_offset; ++i) caps[0].read(tmp);
            }

            double fps = caps[0].get(cv::CAP_PROP_FPS);
            if (fps <= 0.0) fps = 30.0;
            int w = static_cast<int>(caps[0].get(cv::CAP_PROP_FRAME_WIDTH));
            int h = static_cast<int>(caps[0].get(cv::CAP_PROP_FRAME_HEIGHT));

            std::vector<std::unique_ptr<cv::VideoWriter>> overlays(2);
            if (!args.overlay_dir.empty()) {
                std::filesystem::create_directories(args.overlay_dir);
                for (int i = 0; i < 2; ++i) {
                    std::string name = jobs.size() > 1
                        ? (job.pose + "_cam" + std::to_string(i) + "_reproj.mp4")
                        : ("cam" + std::to_string(i) + "_reproj.mp4");
                    std::filesystem::path path = std::filesystem::path(args.overlay_dir) / name;
                    overlays[static_cast<std::size_t>(i)] = std::make_unique<cv::VideoWriter>(
                        path.string(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                        fps, cv::Size(w, h));
                    if (!overlays[static_cast<std::size_t>(i)]->isOpened()) {
                        throw std::runtime_error("failed to open overlay writer: " + path.string());
                    }
                }
            }

            fitra::lift::SkeletonKalman kalman;
            for (int frame_idx = 0;; ++frame_idx) {
                if (args.max_frames > 0 && processed >= args.max_frames) break;
                std::vector<cv::Mat> frames(2);
                bool ok = true;
                for (int cam = 0; cam < 2; ++cam) {
                    ok = caps[static_cast<std::size_t>(cam)].read(frames[static_cast<std::size_t>(cam)])
                         && !frames[static_cast<std::size_t>(cam)].empty();
                    if (!ok) break;
                }
                if (!ok) break;

                std::vector<std::vector<fitra::infer::Person>> persons_by_cam(2);
                std::vector<fitra::lift::PerCameraObservation> observations;
                for (int cam = 0; cam < 2; ++cam) {
                    auto bboxes = keep_largest_if_needed(yolox.infer(frames[static_cast<std::size_t>(cam)]),
                                                         args.multi_person);
                    persons_by_cam[static_cast<std::size_t>(cam)] = rtmpose.infer(
                        frames[static_cast<std::size_t>(cam)], bboxes);
                    if (!persons_by_cam[static_cast<std::size_t>(cam)].empty()) {
                        observations.push_back(
                            fitra::lift::PerCameraObservation{
                                cam, &persons_by_cam[static_cast<std::size_t>(cam)][0]});
                    }
                }

                auto tri = triangulator.triangulate(observations);
                auto skel = tri.skeleton;
                if (!args.no_kalman) {
                    skel = kalman.update(skel, 1.0 / fps);
                }
                profile_acc.observe(job.pose, skel, tri);
                double drift_before = ik.bone_drift_pct(skel);
                if (!args.no_ik) {
                    skel = ik.update(skel);
                }
                double drift_after = ik.bone_drift_pct(skel);

                write_json_line(jout, global_frame++, job.pose, skel, tri,
                                drift_before, drift_after, ik.locked());
                if (tri.valid_joints > 0) {
                    frames_with_3d += 1;
                    med_reproj_values.push_back(tri.median_reproj_px);
                    if (ik.locked()) drift_values.push_back(drift_after);
                }

                for (int cam = 0; cam < 2; ++cam) {
                    if (!overlays[static_cast<std::size_t>(cam)]) continue;
                    draw_2d(frames[static_cast<std::size_t>(cam)],
                            persons_by_cam[static_cast<std::size_t>(cam)]);
                    draw_reprojection(frames[static_cast<std::size_t>(cam)], triangulator, cam, skel);
                    overlays[static_cast<std::size_t>(cam)]->write(frames[static_cast<std::size_t>(cam)]);
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
        sout << "  \"videos\":[\"" << json_escape(jobs.front().videos[0]) << "\",\""
             << json_escape(jobs.front().videos[1]) << "\"],\n";
        sout << "  \"calib\":\"" << args.calib << "\"\n";
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
