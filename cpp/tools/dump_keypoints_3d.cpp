// dump_keypoints_3d — Phase 7 offline 2-camera 3D MVP.
//
// Runs YOLOX + RTMPose on synchronized recorded videos, triangulates COCO-17
// joints with a calibration YAML, then optionally applies 3D Kalman + IK.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <NvInfer.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "infer/rtmpose.hpp"
#include "infer/trt_engine.hpp"
#include "infer/yolox.hpp"
#include "lift/calib_io.hpp"
#include "lift/ik.hpp"
#include "lift/kalman.hpp"
#include "lift/skeleton_def.hpp"
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
};

void print_help() {
    std::puts(
        "dump_keypoints_3d — offline two-camera 3D triangulation dump\n"
        "\n"
        "Required:\n"
        "  --video PATH              input MP4, repeat twice in cam order\n"
        "  --calib PATH              calibration YAML with intrinsics/extrinsics (ids must be cam0,cam1)\n"
        "  --det-engine PATH         YOLOX .engine\n"
        "  --pose-engine PATH        RTMPose .engine\n"
        "  --out PATH                output JSONL\n"
        "\n"
        "Optional:\n"
        "  --summary PATH            output summary JSON (default: <out>.summary.json)\n"
        "  --overlay-dir DIR         write reprojection overlay MP4s\n"
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
        else {
            std::fprintf(stderr, "unknown arg: %s\n", argv[i]);
            print_help();
            std::exit(EXIT_FAILURE);
        }
    }
    if (args.videos.size() != 2 || args.calib.empty() || args.det_engine.empty() ||
        args.pose_engine.empty() || args.out.empty()) {
        print_help();
        std::exit(EXIT_FAILURE);
    }
    if (args.subject_height_m < 0.0 || args.subject_height_m > 2.5) {
        std::fprintf(stderr, "--subject-height-m must be 0 or a plausible meter value <= 2.5\n");
        std::exit(EXIT_FAILURE);
    }
    if (args.summary.empty()) args.summary = args.out + ".summary.json";
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
    for (const auto& p : persons) {
        cv::rectangle(frame,
                      {static_cast<int>(p.bbox.x1), static_cast<int>(p.bbox.y1)},
                      {static_cast<int>(p.bbox.x2), static_cast<int>(p.bbox.y2)},
                      cv::Scalar(80, 80, 80), 1, cv::LINE_AA);
        for (auto [a, b] : fitra::lift::kCocoEdges) {
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
    std::array<cv::Point2f, fitra::infer::kNumKeypoints> pts{};
    std::array<bool, fitra::infer::kNumKeypoints> ok{};
    for (std::size_t k = 0; k < skel.joints.size(); ++k) {
        ok[k] = triangulator.project(cam_index, skel.joints[k], pts[k]);
        if (ok[k]) {
            cv::circle(frame, {static_cast<int>(pts[k].x), static_cast<int>(pts[k].y)},
                       4, color, -1, cv::LINE_AA);
        }
    }
    for (auto [a, b] : fitra::lift::kCocoEdges) {
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
                     const fitra::infer::Skeleton3D& skel,
                     const fitra::lift::TriangulatedSkeleton& tri,
                     double bone_drift_before,
                     double bone_drift_after,
                     bool ik_locked) {
    out << "{\"frame\":" << frame << ",\"persons_3d\":[{\"id\":0,\"joints\":[";
    for (std::size_t k = 0; k < skel.joints.size(); ++k) {
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
    for (std::size_t k = 0; k < tri.view_count.size(); ++k) {
        if (k) out << ",";
        out << tri.view_count[k];
    }
    out << "]}}\n";
}

}  // namespace

int main(int argc, char** argv) {
    Args args = parse_args(argc, argv);

    try {
        auto calib = fitra::lift::load_calibration(args.calib);
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

        std::vector<cv::VideoCapture> caps;
        caps.reserve(args.videos.size());
        for (const auto& path : args.videos) {
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

        std::filesystem::path outp{args.out};
        if (outp.has_parent_path()) std::filesystem::create_directories(outp.parent_path());
        std::ofstream jout{args.out, std::ios::trunc};
        if (!jout.is_open()) throw std::runtime_error("failed to open output: " + args.out);

        std::vector<std::unique_ptr<cv::VideoWriter>> overlays(2);
        if (!args.overlay_dir.empty()) {
            std::filesystem::create_directories(args.overlay_dir);
            for (int i = 0; i < 2; ++i) {
                std::filesystem::path path = std::filesystem::path(args.overlay_dir)
                                           / ("cam" + std::to_string(i) + "_reproj.mp4");
                overlays[static_cast<std::size_t>(i)] = std::make_unique<cv::VideoWriter>(
                    path.string(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                    fps, cv::Size(w, h));
                if (!overlays[static_cast<std::size_t>(i)]->isOpened()) {
                    throw std::runtime_error("failed to open overlay writer: " + path.string());
                }
            }
        }

        fitra::lift::SkeletonKalman kalman;
        fitra::lift::IkSolver::Options ik_opts;
        ik_opts.bone_calib_frames = std::max(1, args.bone_calib_frames);
        ik_opts.subject_height_m = args.subject_height_m;
        fitra::lift::IkSolver ik{ik_opts};

        std::vector<double> med_reproj_values;
        std::vector<double> drift_values;
        int processed = 0;
        int frames_with_3d = 0;
        auto start = std::chrono::steady_clock::now();

        for (int frame_idx = 0;; ++frame_idx) {
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
            double drift_before = ik.bone_drift_pct(skel);
            if (!args.no_ik) {
                skel = ik.update(skel);
            }
            double drift_after = ik.bone_drift_pct(skel);

            write_json_line(jout, frame_idx, skel, tri, drift_before, drift_after, ik.locked());
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
                FITRA_LOG_INFO("processed {} frames; last valid_joints={} reproj_med={}",
                               processed, tri.valid_joints, fmt(tri.median_reproj_px));
            }
            if (args.max_frames > 0 && processed >= args.max_frames) break;
        }

        auto end = std::chrono::steady_clock::now();
        double secs = std::chrono::duration<double>(end - start).count();
        for (auto& writer : overlays) {
            if (writer) writer->release();
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
        sout << "  \"videos\":[\"" << args.videos[0] << "\",\"" << args.videos[1] << "\"],\n";
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
