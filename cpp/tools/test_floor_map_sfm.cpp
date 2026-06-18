// Synthetic-data unit test for the floor-AprilTag SfM map builder.
//
// We define a ground-truth FloorTagMap (8 floor tags, ids 20-27, each with a
// distinct yaw on z=0), synthesize a sequence of frames where each frame sees
// only a small subset of tags (mean ~1.8, never all 8) from a random camera
// pose, hand the per-tag GT T_cam←tag observations to build_floor_map_sfm, and
// check it chains them back into the GT layout (up to the anchor gauge). Covers:
// noise-free recovery, floor-plane re-gauge (z=0), metric-scale preservation,
// noisy robustness, split-graph reporting, and a round-trip through
// solve_floor_extrinsics (the built map must localise a camera).

#include "lift/floor_map_sfm.hpp"
#include "lift/floor_extrinsic_solver.hpp"
#include "lift/floor_tag_map.hpp"
#include "geom/frames.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <cstdio>
#include <map>
#include <random>
#include <vector>

namespace {

using fitra::geom::T_cam_marker;
using fitra::geom::T_world_marker;
using fitra::lift::FloorCameraInput;
using fitra::lift::FloorTag;
using fitra::lift::FloorTagMap;
using fitra::lift::FloorTagObservation;
using fitra::lift::SfmFrame;
using fitra::lift::SfmMapOptions;
using fitra::lift::SfmMapReport;
using fitra::lift::SfmTagObs;
using fitra::lift::build_floor_map_sfm;
using fitra::lift::solve_floor_extrinsics;

int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++g_fail; } \
} while (0)

#define CHECK_LT(a, b) do { \
    if (!((a) < (b))) { \
        std::fprintf(stderr, "FAIL %s:%d %s < %s (%g vs %g)\n", \
            __FILE__, __LINE__, #a, #b, double(a), double(b)); ++g_fail; \
    } \
} while (0)

constexpr double kPi = 3.14159265358979323846;

cv::Mat make_K(double f, double cx, double cy) {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = f;
    K.at<double>(1, 1) = f;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    return K;
}

cv::Matx33d rot_z(double deg) {
    double r = deg * kPi / 180.0, c = std::cos(r), s = std::sin(r);
    return cv::Matx33d(c, -s, 0, s, c, 0, 0, 0, 1);
}

double rot_angle_deg(const cv::Matx44d& a, const cv::Matx44d& b) {
    return fitra::geom::rotation_angle_deg(a, b);
}

FloorTag make_tag(int id, double size, const cv::Matx33d& R, const cv::Vec3d& t) {
    FloorTag tag;
    tag.id = id;
    tag.size_m = size;
    tag.T_world_tag = T_world_marker::from_raw(fitra::geom::compose(R, t));
    return tag;
}

// GT: 8 floor tags in a 4×2 grid on z=0, 0.6 m pitch, each yawed differently.
FloorTagMap gt_map() {
    FloorTagMap m;
    const double sz = 0.1145, pitch = 0.6;
    int id = 20;
    for (int row = 0; row < 2; ++row) {
        for (int col = 0; col < 4; ++col) {
            cv::Vec3d pos(col * pitch, row * pitch, 0.0);
            m.tags.push_back(make_tag(id, sz, rot_z(17.0 * (id - 20)), pos));
            ++id;
        }
    }
    return m;
}

// A random rigid camera pose (world→camera), kept well away from the floor.
cv::Matx44d random_cam_pose(std::mt19937& rng) {
    std::uniform_real_distribution<double> ang(-kPi, kPi);
    std::uniform_real_distribution<double> tilt(-0.6, 0.6);
    std::uniform_real_distribution<double> pos(-0.5, 0.5);
    cv::Vec3d rv(tilt(rng), tilt(rng), ang(rng));
    cv::Matx33d Rcw;
    cv::Rodrigues(rv, Rcw);
    cv::Vec3d C(pos(rng), pos(rng), 1.5 + pos(rng));
    cv::Vec3d t = -(Rcw * C);
    return fitra::geom::compose(Rcw, t);
}

cv::Matx44d perturb(const cv::Matx44d& T, double rot_deg, double trans_m,
                    std::mt19937& rng) {
    std::normal_distribution<double> g(0.0, 1.0);
    cv::Vec3d axis(g(rng), g(rng), g(rng));
    double n = cv::norm(axis);
    if (n < 1e-9) { axis = cv::Vec3d(1, 0, 0); n = 1; }
    axis *= 1.0 / n;
    cv::Vec3d rv = axis * (rot_deg * kPi / 180.0 * g(rng));
    cv::Matx33d dR;
    cv::Rodrigues(rv, dR);
    cv::Vec3d dt(g(rng) * trans_m, g(rng) * trans_m, g(rng) * trans_m);
    return T * fitra::geom::compose(dR, dt);
}

// Build frames: each `vis` entry lists the tag ids seen in that frame; observe
// them from a random camera pose with optional pose noise.
std::vector<SfmFrame> gen_frames(const FloorTagMap& gt,
                                 const std::vector<std::vector<int>>& vis,
                                 double rot_noise_deg, double trans_noise_m,
                                 std::mt19937& rng) {
    std::vector<SfmFrame> frames;
    for (const auto& ids : vis) {
        cv::Matx44d Tcw = random_cam_pose(rng);
        SfmFrame fr;
        for (int id : ids) {
            const FloorTag* tag = gt.find(id);
            if (!tag) continue;
            cv::Matx44d Tcam_tag = Tcw * tag->T_world_tag.raw();  // tag→camera
            if (rot_noise_deg > 0.0 || trans_noise_m > 0.0)
                Tcam_tag = perturb(Tcam_tag, rot_noise_deg, trans_noise_m, rng);
            SfmTagObs ob;
            ob.id = id;
            ob.T_cam_tag = T_cam_marker::from_raw(Tcam_tag);
            ob.reproj_rms_px = 0.1;
            fr.tags.push_back(ob);
        }
        frames.push_back(std::move(fr));
    }
    return frames;
}

double tag_dist(const FloorTagMap& m, int a, int b) {
    return cv::norm(fitra::geom::trans_of(m.find(a)->T_world_tag.raw()) -
                    fitra::geom::trans_of(m.find(b)->T_world_tag.raw()));
}

// Visibility pattern that keeps all 8 tags connected through overlapping pairs.
std::vector<std::vector<int>> connected_vis() {
    return {
        {20, 21}, {21, 22}, {22, 23}, {23, 24}, {24, 25}, {25, 26}, {26, 27},
        {20, 21, 22}, {23, 24, 25}, {25, 26, 27},
        {20}, {24}, {27},  // single-tag frames (no edges) must not break anything
    };
}

void test_recovery_noisefree() {
    FloorTagMap gt = gt_map();
    std::mt19937 rng(1);
    auto frames = gen_frames(gt, connected_vis(), 0.0, 0.0, rng);

    SfmMapOptions opt;
    opt.tag_size_m = 0.1145;
    opt.fit_floor_plane = false;   // compare directly in the anchor gauge
    opt.rotation_refine = false;
    FloorTagMap map;
    SfmMapReport rep;
    bool ok = build_floor_map_sfm(frames, opt, map, rep);

    CHECK(ok);
    CHECK(rep.connected);
    CHECK(rep.n_tags == 8);
    CHECK(rep.anchor_id == 20);
    CHECK(rep.unreached_ids.empty());
    CHECK(static_cast<int>(map.tags.size()) == 8);

    // Built world == anchor(20) tag frame; compare each tag to GT-relative-anchor.
    const cv::Matx44d gt_anchor_inv =
        fitra::geom::invert_rigid(gt.find(20)->T_world_tag.raw());
    for (const auto& t : map.tags) {
        cv::Matx44d gt_rel = gt_anchor_inv * gt.find(t.id)->T_world_tag.raw();
        const cv::Matx44d& got = t.T_world_tag.raw();
        CHECK_LT(rot_angle_deg(got, gt_rel), 1e-3);
        CHECK_LT(cv::norm(fitra::geom::trans_of(got) -
                          fitra::geom::trans_of(gt_rel)), 1e-4);
    }
    // Metric scale (gauge-invariant): pairwise distances match GT.
    CHECK_LT(std::abs(tag_dist(map, 20, 27) - tag_dist(gt, 20, 27)), 1e-4);
    CHECK_LT(std::abs(tag_dist(map, 20, 23) - tag_dist(gt, 20, 23)), 1e-4);
}

void test_floor_fit() {
    FloorTagMap gt = gt_map();
    std::mt19937 rng(7);
    auto frames = gen_frames(gt, connected_vis(), 0.0, 0.0, rng);

    SfmMapOptions opt;
    opt.fit_floor_plane = true;
    FloorTagMap map;
    SfmMapReport rep;
    CHECK(build_floor_map_sfm(frames, opt, map, rep));
    CHECK_LT(rep.floor_plane_rms_m, 1e-6);
    for (const auto& t : map.tags) {
        CHECK_LT(std::abs(fitra::geom::trans_of(t.T_world_tag.raw())[2]), 1e-6);
    }
    // Anchor at origin after re-gauge.
    CHECK_LT(cv::norm(fitra::geom::trans_of(map.find(20)->T_world_tag.raw())), 1e-6);
    // Scale still intact.
    CHECK_LT(std::abs(tag_dist(map, 20, 27) - tag_dist(gt, 20, 27)), 1e-4);
}

void test_noise() {
    FloorTagMap gt = gt_map();
    std::mt19937 rng(3);
    // More frames per edge so robust averaging has something to chew on.
    auto vis = connected_vis();
    auto more = vis;
    for (int k = 0; k < 4; ++k)
        for (auto& v : vis) more.push_back(v);
    auto frames = gen_frames(gt, more, 0.4, 0.004, rng);  // ~0.4° / 4 mm noise

    SfmMapOptions opt;
    opt.fit_floor_plane = false;
    opt.rotation_refine = true;
    FloorTagMap map;
    SfmMapReport rep;
    CHECK(build_floor_map_sfm(frames, opt, map, rep));
    CHECK(rep.connected);
    const cv::Matx44d gt_anchor_inv =
        fitra::geom::invert_rigid(gt.find(20)->T_world_tag.raw());
    for (const auto& t : map.tags) {
        cv::Matx44d gt_rel = gt_anchor_inv * gt.find(t.id)->T_world_tag.raw();
        CHECK_LT(rot_angle_deg(t.T_world_tag.raw(), gt_rel), 2.0);
    }
    CHECK_LT(std::abs(tag_dist(map, 20, 27) - tag_dist(gt, 20, 27)), 0.03);
}

void test_split_graph() {
    FloorTagMap gt = gt_map();
    std::mt19937 rng(5);
    // Two disconnected components: {20,21,22,23} and {24,25,26,27}.
    std::vector<std::vector<int>> vis = {
        {20, 21}, {21, 22}, {22, 23}, {20, 21, 22},
        {24, 25}, {25, 26}, {26, 27}, {24, 25, 26},
    };
    auto frames = gen_frames(gt, vis, 0.0, 0.0, rng);

    SfmMapOptions opt;
    FloorTagMap map;
    SfmMapReport rep;
    bool ok = build_floor_map_sfm(frames, opt, map, rep);
    CHECK(!ok);
    CHECK(!rep.connected);
    CHECK(rep.n_tags == 8);
    CHECK(rep.unreached_ids.size() == 4);  // 24,25,26,27 not reachable from 20
    for (int id : rep.unreached_ids) CHECK(id >= 24);
    // Reachable component (20-23) still emitted.
    CHECK(static_cast<int>(map.tags.size()) == 4);
}

// The built map must plug into the existing floor extrinsic solver: place a
// camera in the built world, project the map's tag corners, and confirm the
// solver localises it back with low reprojection error.
void test_roundtrip_solve() {
    FloorTagMap gt = gt_map();
    std::mt19937 rng(11);
    auto frames = gen_frames(gt, connected_vis(), 0.0, 0.0, rng);
    SfmMapOptions opt;
    opt.fit_floor_plane = true;
    FloorTagMap map;
    SfmMapReport rep;
    CHECK(build_floor_map_sfm(frames, opt, map, rep));

    cv::Mat K = make_K(900.0, 640.0, 360.0);
    cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);
    // Camera ~2 m above the floor, looking down with a slight tilt.
    const double tilt = 15.0 * kPi / 180.0;
    cv::Matx33d down(1, 0, 0, 0, -1, 0, 0, 0, -1);
    cv::Matx33d Rx(1, 0, 0, 0, std::cos(tilt), -std::sin(tilt),
                   0, std::sin(tilt), std::cos(tilt));
    cv::Matx33d Rcw = Rx * down;
    cv::Vec3d C(0.9, 0.3, 2.2);
    cv::Matx44d Tcw = fitra::geom::compose(Rcw, -(Rcw * C));

    cv::Mat rvec;
    cv::Rodrigues(cv::Mat(Rcw), rvec);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << Tcw(0, 3), Tcw(1, 3), Tcw(2, 3));

    FloorCameraInput cam;
    cam.cam_index = 0;
    cam.K = K;
    cam.dist = dist;
    cam.fisheye = false;
    for (const auto& tag : map.tags) {
        auto wc = map.world_corners(tag);
        std::vector<cv::Point3f> objp;
        for (int i = 0; i < 4; ++i)
            objp.emplace_back((float)wc[i].v[0], (float)wc[i].v[1], (float)wc[i].v[2]);
        std::vector<cv::Point2f> proj;
        cv::projectPoints(objp, rvec, tvec, K, dist, proj);
        FloorTagObservation ob;
        ob.id = tag.id;
        for (int i = 0; i < 4; ++i) ob.corners[i] = proj[i];
        cam.obs.push_back(ob);
    }

    auto sol = solve_floor_extrinsics({cam}, map);
    CHECK(sol.ok);
    CHECK(!sol.cameras.empty());
    if (!sol.cameras.empty()) {
        const auto& c = sol.cameras[0];
        CHECK(c.solved);
        CHECK_LT(c.reproj_rms_px, 1e-2);
        CHECK_LT(rot_angle_deg(c.T_cam_world.raw(), Tcw), 1e-2);
        CHECK_LT(cv::norm(fitra::geom::trans_of(c.T_cam_world.raw()) -
                          fitra::geom::trans_of(Tcw)), 1e-3);
    }
}

}  // namespace

int main() {
    test_recovery_noisefree();
    test_floor_fit();
    test_noise();
    test_split_graph();
    test_roundtrip_solve();
    if (g_fail) {
        std::fprintf(stderr, "test_floor_map_sfm: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_floor_map_sfm: OK\n");
    return 0;
}
