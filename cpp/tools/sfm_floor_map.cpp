// sfm_floor_map — build a floor-AprilTag map from a recorded video, no tape
// measure. A single moving camera (e.g. a phone, calibrated by
// charuco_intrinsic_video) sweeps the floor markers; we detect every tag per
// frame, run per-tag PnP (the known tag size fixes metric scale), chain the
// observations across frames into one consistent FloorTagMap
// (lift::build_floor_map_sfm), and write it for the floor extrinsic solver to
// localise the fixed rig against. A held-out fraction of frames re-localises
// against the built map to report reprojection error.
//
// See docs/design/pose-3d-smartphone-sfm-marker-map.md.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <string_view>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "lift/apriltag_marker.hpp"
#include "lift/calib_io.hpp"
#include "lift/floor_extrinsic_solver.hpp"
#include "lift/floor_map_sfm.hpp"
#include "lift/floor_tag_map.hpp"

namespace {

void print_help() {
    std::puts(
        "sfm_floor_map — build a floor-AprilTag map from a recorded video\n"
        "\n"
        "Required:\n"
        "  --video PATH            input video of the floor markers\n"
        "  --intrinsics PATH       camera intrinsics YAML (same camera/optics)\n"
        "\n"
        "Optional:\n"
        "  --out PATH              output map YAML (default configs/floor_tag_map.yaml)\n"
        "  --ids SPEC              tag ids, e.g. 20-27 or 20,21,22 (default 20-27)\n"
        "  --tag-size M            black-square side, metres (default 0.1145)\n"
        "  --dict ID               cv::aruco dict id (default -1 = DICT_APRILTAG_36h11)\n"
        "  --anchor ID             world-origin tag id (default = smallest id)\n"
        "  --stride N              process every Nth frame (default 1)\n"
        "  --max-rms PX            drop per-tag PnP above this (default 3.0)\n"
        "  --holdout-frac F        fraction of frames for validation (default 0.2)\n"
        "  --no-floor-fit          skip the floor-plane re-gauge\n"
        "  --no-refine             skip pose-averaging relaxation\n"
        "  --no-clahe              disable CLAHE contrast boost before detection\n"
        "  --help                  show this help\n");
}

// Parse "a-b" or "a,b,c" into a list of ids.
std::vector<int> parse_ids(const std::string& spec) {
    std::vector<int> ids;
    auto dash = spec.find('-');
    if (dash != std::string::npos && spec.find(',') == std::string::npos) {
        int lo = std::atoi(spec.substr(0, dash).c_str());
        int hi = std::atoi(spec.substr(dash + 1).c_str());
        for (int i = lo; i <= hi; ++i) ids.push_back(i);
        return ids;
    }
    std::size_t start = 0;
    while (start < spec.size()) {
        std::size_t comma = spec.find(',', start);
        std::string tok = spec.substr(start, comma == std::string::npos
                                                  ? std::string::npos
                                                  : comma - start);
        if (!tok.empty()) ids.push_back(std::atoi(tok.c_str()));
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return ids;
}

double median_of(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

}  // namespace

int main(int argc, char** argv) {
    std::string video, intrinsics;
    std::string out = "configs/floor_tag_map.yaml";
    std::string ids_spec = "20-27";
    double tag_size = 0.1145;
    int dict = -1;
    int anchor = -1;
    int stride = 1;
    double max_rms = 3.0;
    double holdout_frac = 0.2;
    bool floor_fit = true, refine = true, clahe = true;

    for (int i = 1; i < argc; ++i) {
        std::string_view a{argv[i]};
        auto need = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing argument for %s\n", flag);
                std::exit(EXIT_FAILURE);
            }
            return argv[++i];
        };
        if (a == "--help" || a == "-h")  { print_help(); return EXIT_SUCCESS; }
        else if (a == "--video")         { video = need("--video"); }
        else if (a == "--intrinsics")    { intrinsics = need("--intrinsics"); }
        else if (a == "--out")           { out = need("--out"); }
        else if (a == "--ids")           { ids_spec = need("--ids"); }
        else if (a == "--tag-size")      { tag_size = std::atof(need("--tag-size")); }
        else if (a == "--dict")          { dict = std::atoi(need("--dict")); }
        else if (a == "--anchor")        { anchor = std::atoi(need("--anchor")); }
        else if (a == "--stride")        { stride = std::max(1, std::atoi(need("--stride"))); }
        else if (a == "--max-rms")       { max_rms = std::atof(need("--max-rms")); }
        else if (a == "--holdout-frac")  { holdout_frac = std::atof(need("--holdout-frac")); }
        else if (a == "--no-floor-fit")  { floor_fit = false; }
        else if (a == "--no-refine")     { refine = false; }
        else if (a == "--no-clahe")      { clahe = false; }
        else {
            std::fprintf(stderr, "unknown arg: %s\n", argv[i]);
            print_help();
            return EXIT_FAILURE;
        }
    }
    if (video.empty() || intrinsics.empty()) { print_help(); return EXIT_FAILURE; }

    // --- intrinsics -----------------------------------------------------------
    cv::Mat K, dist;
    bool fisheye = false;
    int intr_w = 0, intr_h = 0;
    try {
        auto calib = fitra::lift::load_calibration(intrinsics);
        if (calib.cameras.empty()) {
            std::fprintf(stderr, "no cameras in %s\n", intrinsics.c_str());
            return EXIT_FAILURE;
        }
        const auto& in = calib.cameras.front().intrinsics;
        K = in.K; dist = in.dist; fisheye = in.is_fisheye();
        intr_w = in.width; intr_h = in.height;
        std::printf("intrinsics: %s %dx%d model=%s\n",
                    calib.cameras.front().id.c_str(), intr_w, intr_h,
                    in.distortion_model.c_str());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "failed to load intrinsics: %s\n", e.what());
        return EXIT_FAILURE;
    }

    // --- detector -------------------------------------------------------------
    std::vector<int> ids = parse_ids(ids_spec);
    if (ids.empty()) { std::fprintf(stderr, "no ids parsed from '%s'\n", ids_spec.c_str()); return EXIT_FAILURE; }
    fitra::lift::MarkerBoardConfig mcfg;
    mcfg.dictionary = dict;
    mcfg.use_clahe = clahe;
    for (int id : ids) mcfg.faces.push_back({id, tag_size});
    fitra::lift::AprilTagDetector detector{mcfg};

    cv::VideoCapture cap{video};
    if (!cap.isOpened()) {
        std::fprintf(stderr, "failed to open video: %s\n", video.c_str());
        return EXIT_FAILURE;
    }

    // --- detect per frame; split build vs holdout -----------------------------
    const int holdout_every =
        (holdout_frac > 0.0 && holdout_frac < 1.0)
            ? std::max(2, static_cast<int>(std::lround(1.0 / holdout_frac)))
            : 0;

    std::vector<fitra::lift::SfmFrame> build_frames, holdout_frames;
    long read = 0, frames_with_tags = 0, kept = 0;
    int idx = 0, kept_idx = 0;
    bool checked_res = false;
    cv::Mat frame;
    while (cap.read(frame)) {
        if ((idx++ % stride) != 0) continue;
        if (frame.empty()) continue;
        ++read;
        if (!checked_res) {
            checked_res = true;
            if (intr_w && (frame.cols != intr_w || frame.rows != intr_h)) {
                std::fprintf(stderr,
                    "WARNING: video %dx%d != intrinsics %dx%d — PnP scale will be wrong\n",
                    frame.cols, frame.rows, intr_w, intr_h);
            }
        }
        auto dets = detector.detect(frame, K, dist, fisheye);
        fitra::lift::SfmFrame fr;
        for (const auto& d : dets) {
            if (!d.pose_ok) continue;
            fitra::lift::SfmTagObs ob;
            ob.id = d.face_id;
            ob.T_cam_tag = d.T_cam_face;
            ob.reproj_rms_px = d.reproj_rms_px;
            ob.corners = d.corners;
            fr.tags.push_back(ob);
        }
        if (fr.tags.empty()) continue;
        ++frames_with_tags;
        if (holdout_every && (kept_idx % holdout_every) == 0)
            holdout_frames.push_back(std::move(fr));
        else
            build_frames.push_back(std::move(fr));
        ++kept_idx;
        ++kept;
        if (read % 100 == 0) {
            std::printf("\rframes=%ld with_tags=%ld build=%zu holdout=%zu",
                        read, frames_with_tags, build_frames.size(), holdout_frames.size());
            std::fflush(stdout);
        }
    }
    std::printf("\rframes=%ld with_tags=%ld build=%zu holdout=%zu\n",
                read, frames_with_tags, build_frames.size(), holdout_frames.size());

    // --- build the map --------------------------------------------------------
    fitra::lift::SfmMapOptions opt;
    opt.tag_size_m = tag_size;
    opt.anchor_tag_id = anchor;
    opt.max_pose_rms_px = max_rms;
    opt.fit_floor_plane = floor_fit;
    opt.rotation_refine = refine;

    fitra::lift::FloorTagMap map;
    fitra::lift::SfmMapReport rep;
    bool connected = fitra::lift::build_floor_map_sfm(build_frames, opt, map, rep);

    std::printf("map: anchor=%d tags_placed=%zu/%d edges=%d connected=%s plane_rms=%.4f m\n",
                rep.anchor_id, map.tags.size(), rep.n_tags, rep.n_edges,
                connected ? "yes" : "NO", rep.floor_plane_rms_m);
    if (!connected) {
        std::printf("  unreached:");
        for (int id : rep.unreached_ids) std::printf(" %d", id);
        std::printf("  (%s)\n", rep.message.c_str());
    }
    for (const auto& t : map.tags) {
        cv::Vec3d p = fitra::geom::trans_of(t.T_world_tag.raw());
        std::printf("  tag %d: pos=[% .4f % .4f % .4f] m\n", t.id, p[0], p[1], p[2]);
    }

    if (map.tags.empty()) {
        std::fprintf(stderr, "no tags placed; nothing to write\n");
        return EXIT_FAILURE;
    }
    try {
        fitra::lift::floor_tag_map_write(out, map);
        std::printf("wrote %s\n", out.c_str());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "write failed: %s\n", e.what());
        return EXIT_FAILURE;
    }

    // --- holdout validation: localise each holdout frame against the map ------
    if (!holdout_frames.empty()) {
        // Report the true reprojection distribution over every multi-tag holdout
        // frame: raise the solver's reject gate so geometry, not the 3 px cutoff,
        // decides what "solved". A separate under-3px count shows how many would
        // pass the rig's default acceptance.
        fitra::lift::FloorSolverOptions vopts;
        vopts.max_reproj_px = 1e9;  // report geometry, not the rig's 3px gate
        std::vector<double> rms2, rms3;  // by tag count: ==2 vs >=3
        for (const auto& fr : holdout_frames) {
            if (fr.tags.size() < 2) continue;  // single-tag PnP is trivial here
            fitra::lift::FloorCameraInput cam;
            cam.cam_index = 0; cam.K = K; cam.dist = dist; cam.fisheye = fisheye;
            for (const auto& ob : fr.tags) {
                if (!map.find(ob.id)) continue;
                fitra::lift::FloorTagObservation o;
                o.id = ob.id; o.corners = ob.corners;
                cam.obs.push_back(o);
            }
            if (cam.obs.size() < 2) continue;
            auto sol = fitra::lift::solve_floor_extrinsics({cam}, map, vopts);
            if (sol.cameras.empty() || !sol.cameras[0].solved) continue;
            (cam.obs.size() >= 3 ? rms3 : rms2).push_back(sol.cameras[0].reproj_rms_px);
        }
        auto pct = [](std::vector<double> v, double q) {
            if (v.empty()) return 0.0;
            std::sort(v.begin(), v.end());
            return v[std::min(v.size() - 1, (std::size_t)(q * v.size()))];
        };
        std::printf("holdout reproj_rms (px), median / p90:\n");
        std::printf("  >=3 tags (well-conditioned): n=%zu  median=%.2f  p90=%.2f\n",
                    rms3.size(), median_of(rms3), pct(rms3, 0.9));
        std::printf("  ==2 tags (coplanar PnP often ambiguous): n=%zu  median=%.2f  p90=%.2f\n",
                    rms2.size(), median_of(rms2), pct(rms2, 0.9));
        std::printf("  note: floor-only map is coplanar — a fixed rig camera needs "
                    "off-plane stand tags for height/tilt observability\n");
    }
    return EXIT_SUCCESS;
}
