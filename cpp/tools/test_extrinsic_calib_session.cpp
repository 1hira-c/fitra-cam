// Tests for ExtrinsicCalibSession (collection loop) + calib_io write round-trip.
//
//  1) Motion gate + burst mechanics via ingest() (no images): stillness emits
//     a burst-averaged sample; motion rejects + flushes; burst_max auto-flush.
//  2) End-to-end: drive a synthetic multi-camera scene through ingest() with a
//     permissive gate, solve_and_write the extrinsics YAML, reload it, and
//     verify the recovered relative extrinsic matches ground truth.
//  3) calib_io write_calibration → load_calibration round-trip.

#include "pipeline/extrinsic_calib_session.hpp"
#include "lift/calib_io.hpp"
#include "lift/extrinsic_solver.hpp"

#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

using fitra::pipeline::ControllerObservation;
using fitra::pipeline::ExtrinsicCalibConfig;
using fitra::pipeline::ExtrinsicCalibSession;
using fitra::pipeline::ExtrinsicCalibState;

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

cv::Matx44d rigid(double rx, double ry, double rz, double tx, double ty, double tz) {
    double cx = std::cos(rx / 2), sx = std::sin(rx / 2);
    double cy = std::cos(ry / 2), sy = std::sin(ry / 2);
    double cz = std::cos(rz / 2), sz = std::sin(rz / 2);
    double w  = cx * cy * cz + sx * sy * sz;
    double qx = sx * cy * cz - cx * sy * sz;
    double qy = cx * sy * cz + sx * cy * sz;
    double qz = cx * cy * sz - sx * sy * cz;
    return fitra::lift::pose_from_pos_quat(tx, ty, tz, qx, qy, qz, w);
}

cv::Matx44d invert_rigid(const cv::Matx44d& T) {
    cv::Matx33d R(T(0,0),T(0,1),T(0,2), T(1,0),T(1,1),T(1,2), T(2,0),T(2,1),T(2,2));
    cv::Vec3d t(T(0,3), T(1,3), T(2,3));
    cv::Matx33d Rt = R.t();
    cv::Vec3d ti = -(Rt * t);
    cv::Matx44d out = cv::Matx44d::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) out(r, c) = Rt(r, c);
        out(r, 3) = ti(r);
    }
    return out;
}

double trans_dist(const cv::Matx44d& a, const cv::Matx44d& b) {
    return std::sqrt((a(0,3)-b(0,3))*(a(0,3)-b(0,3)) +
                     (a(1,3)-b(1,3))*(a(1,3)-b(1,3)) +
                     (a(2,3)-b(2,3))*(a(2,3)-b(2,3)));
}

ControllerObservation ctrl_at(const cv::Matx44d& B, double ts_ms) {
    // Extract pos + quat from a pose matrix (round-trip through helpers is
    // unnecessary; pose_from_pos_quat / our rigid() set these directly, but we
    // re-derive a quaternion-free representation by reusing pose math here).
    ControllerObservation c;
    c.running_ok = true;
    c.x = B(0, 3); c.y = B(1, 3); c.z = B(2, 3);
    // Recover a quaternion from the rotation (w,x,y,z) — Shepperd.
    double t = B(0,0) + B(1,1) + B(2,2), w, x, y, z;
    if (t > 0) { double s = std::sqrt(t + 1.0) * 2; w = 0.25*s; x=(B(2,1)-B(1,2))/s; y=(B(0,2)-B(2,0))/s; z=(B(1,0)-B(0,1))/s; }
    else if (B(0,0) > B(1,1) && B(0,0) > B(2,2)) { double s=std::sqrt(1+B(0,0)-B(1,1)-B(2,2))*2; w=(B(2,1)-B(1,2))/s; x=0.25*s; y=(B(0,1)+B(1,0))/s; z=(B(0,2)+B(2,0))/s; }
    else if (B(1,1) > B(2,2)) { double s=std::sqrt(1+B(1,1)-B(0,0)-B(2,2))*2; w=(B(0,2)-B(2,0))/s; x=(B(0,1)+B(1,0))/s; y=0.25*s; z=(B(1,2)+B(2,1))/s; }
    else { double s=std::sqrt(1+B(2,2)-B(0,0)-B(1,1))*2; w=(B(1,0)-B(0,1))/s; x=(B(0,2)+B(2,0))/s; y=(B(1,2)+B(2,1))/s; z=0.25*s; }
    c.qx = x; c.qy = y; c.qz = z; c.qw = w;
    c.ts_ms = ts_ms;
    return c;
}

fitra::lift::CalibrationSet make_intrinsics(int n_cams) {
    fitra::lift::CalibrationSet set;
    set.schema = "fitra_calibration_v1";
    set.unit = "m";
    set.coordinate_system = "vmt_standing";
    for (int i = 0; i < n_cams; ++i) {
        fitra::lift::CameraCalibration cam;
        cam.id = "cam" + std::to_string(i);
        cam.intrinsics.width = 640;
        cam.intrinsics.height = 480;
        cam.intrinsics.rms_px = 0.2;
        cam.intrinsics.source = "test";
        cam.intrinsics.K = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
        cam.intrinsics.dist = cv::Mat::zeros(1, 5, CV_64F);
        set.cameras.push_back(cam);
    }
    return set;
}

// 1) Motion gate + burst mechanics.
void test_gate_and_burst() {
    ExtrinsicCalibConfig cfg;
    cfg.intrinsics = make_intrinsics(1);
    cfg.lin_vel_max_mps = 0.03;
    cfg.ang_vel_max_dps = 8.0;
    cfg.burst_min = 5;
    cfg.burst_max = 40;
    cfg.burst_gap_ms = 250.0;
    cfg.min_samples_per_group = 3;

    ExtrinsicCalibSession s(cfg);
    s.start();

    cv::Matx44d still = rigid(0, 0, 0, 0.1, 1.0, 1.5);
    cv::Matx44d Tcf   = rigid(0.1, 0.2, 0.0, 0.0, 0.0, 1.0);
    double ts = 0.0;

    // First still frame primes velocity (no velocity yet → rejected).
    CHECK(!s.ingest(0, 0, Tcf, ctrl_at(still, ts))); ts += 10;
    // 7 more still frames → accepted into the burst.
    for (int i = 0; i < 7; ++i) {
        CHECK(s.ingest(0, 0, Tcf, ctrl_at(still, ts)));
        ts += 10;
    }
    CHECK(s.sample_count() == 0);  // not flushed yet

    // A fast move → gate fails, flushes the 7-frame burst (>= burst_min) → 1 sample.
    cv::Matx44d moved = rigid(0, 0, 0, 0.5, 1.0, 1.5);  // 40 cm jump in 10 ms
    CHECK(!s.ingest(0, 0, Tcf, ctrl_at(moved, ts))); ts += 10;
    CHECK(s.sample_count() == 1);

    // A short still burst below burst_min must NOT emit on flush.
    cv::Matx44d still2 = rigid(0, 0, 0, -0.1, 1.0, 1.5);
    // The jump from `moved` to `still2` makes this frame high-velocity → rejected.
    CHECK(!s.ingest(0, 0, Tcf, ctrl_at(still2, ts))); ts += 10;
    // 3 accepted still frames (below burst_min=5).
    for (int i = 0; i < 3; ++i) { CHECK(s.ingest(0, 0, Tcf, ctrl_at(still2, ts))); ts += 10; }
    cv::Matx44d moved2 = rigid(0, 0, 0, 0.6, 1.0, 1.5);
    s.ingest(0, 0, Tcf, ctrl_at(moved2, ts)); ts += 10;  // flush; burst (3) < 5 → no sample
    CHECK(s.sample_count() == 1);

    // Angular motion gate: still position but fast rotation → rejected.
    ExtrinsicCalibSession s2(cfg);
    s2.start();
    double t2 = 0.0;
    cv::Matx44d r0 = rigid(0, 0, 0, 0, 1, 1.5);
    s2.ingest(0, 0, Tcf, ctrl_at(r0, t2)); t2 += 10;  // prime
    cv::Matx44d r1 = rigid(0, 0, 0.5, 0, 1, 1.5);     // ~28.6° in 10 ms → 2865 dps
    CHECK(!s2.ingest(0, 0, Tcf, ctrl_at(r1, t2)));
}

// 2) End-to-end solve_and_write with a permissive gate.
void test_solve_and_write() {
    ExtrinsicCalibConfig cfg;
    cfg.intrinsics = make_intrinsics(2);
    cfg.lin_vel_max_mps = 1e9;   // accept any running_ok frame
    cfg.ang_vel_max_dps = 1e9;
    cfg.burst_min = 3;
    cfg.burst_max = 1000;
    cfg.burst_gap_ms = 100.0;    // separate poses by ts gaps
    cfg.min_samples_per_group = 3;
    cfg.out_path = std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp")
                   + "/fitra_excal_test.yaml";

    // Ground truth.
    std::vector<cv::Matx44d> Tcw = {
        rigid(0.0, 0.0, 0.0,  0.0, 0.1, 2.0),
        rigid(0.1, 1.2, -0.2, 0.5, 0.0, 2.3),
    };
    cv::Matx44d Tcf_off = rigid(0.4, -0.3, 0.0, 0.03, -0.02, 0.05);  // T_controller<-face

    ExtrinsicCalibSession s(cfg);
    s.start();

    double ts = 0.0;
    const int kPoses = 6;
    for (int p = 0; p < kPoses; ++p) {
        cv::Matx44d B = rigid(0.3 * p, -0.4 * p, 0.2 * p,
                              0.1 * p, 1.0, 1.5 + 0.05 * p);  // controller pose
        // New pose → big ts gap so the prior burst (same key) flushes on append.
        ts += 200.0;
        for (int c = 0; c < 2; ++c) {
            // forward chain: A = T_cam<-world · B · T_controller<-face
            cv::Matx44d A = Tcw[c] * B * Tcf_off;
            for (int k = 0; k < cfg.burst_min + 1; ++k) {
                s.ingest(c, 0, A, ctrl_at(B, ts));
                ts += 10.0;
            }
        }
    }

    std::string err;
    bool ok = s.solve_and_write(err);
    CHECK(ok);
    if (!ok) { std::fprintf(stderr, "  solve err: %s\n", err.c_str()); return; }
    CHECK(s.state() == ExtrinsicCalibState::kSolved);

    // Reload and verify the relative extrinsic matches ground truth.
    auto loaded = fitra::lift::load_calibration(cfg.out_path);
    CHECK(loaded.cameras.size() == 2);
    CHECK(loaded.cameras[0].has_extrinsics);
    CHECK(loaded.cameras[1].has_extrinsics);

    auto to_matx = [](const cv::Mat& m) {
        cv::Matx44d T;
        for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) T(r, c) = m.at<double>(r, c);
        return T;
    };
    cv::Matx44d e0 = to_matx(loaded.cameras[0].extrinsics.T_cw);
    cv::Matx44d e1 = to_matx(loaded.cameras[1].extrinsics.T_cw);
    cv::Matx44d rel_est = e0 * invert_rigid(e1);
    cv::Matx44d rel_gt  = Tcw[0] * invert_rigid(Tcw[1]);
    CHECK_LT(trans_dist(rel_est, rel_gt), 1e-6);
    CHECK_LT(fitra::lift::rotation_angle_deg(rel_est, rel_gt), 1e-4);

    // extrinsics_json carries the solved per-camera 6DoF for the 3D scene.
    std::string ex = s.extrinsics_json();
    CHECK(ex.find("\"solved\":true") != std::string::npos);
    CHECK(ex.find("\"id\":\"cam0\"") != std::string::npos);
    CHECK(ex.find("\"id\":\"cam1\"") != std::string::npos);
    CHECK(ex.find("\"T_cam_world\":[") != std::string::npos);
    CHECK(ex.find("\"center\":[") != std::string::npos);
}

// 3) calib_io write/read round-trip with extrinsics.
void test_calib_io_roundtrip() {
    auto set = make_intrinsics(2);
    set.cameras[0].has_extrinsics = true;
    set.cameras[0].extrinsics.method = "test";
    set.cameras[0].extrinsics.T_cw = cv::Mat(rigid(0.1, 0.2, 0.3, 1, 2, 3)).clone();
    set.cameras[0].extrinsics.camera_center_w = {1, 2, 3};

    std::string path = std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp")
                       + "/fitra_calibio_test.yaml";
    fitra::lift::write_calibration(path, set);
    auto back = fitra::lift::load_calibration(path);
    CHECK(back.cameras.size() == 2);
    CHECK(back.cameras[0].has_extrinsics);
    CHECK(!back.cameras[1].has_extrinsics);
    CHECK_LT(std::abs(back.cameras[0].intrinsics.K.at<double>(0, 0) - 600.0), 1e-9);
    CHECK_LT(std::abs(back.cameras[0].extrinsics.T_cw.at<double>(0, 3) - 1.0), 1e-9);
}

// 4) on_frame() full path: render a real 36h11 marker, push it through the
//    detector + live-state recording, and verify state_json carries the
//    detection and a sensible gate reason. No hardware.
void test_on_frame_live_detection() {
    ExtrinsicCalibConfig cfg;
    cfg.intrinsics = make_intrinsics(1);   // 640x480, cx=320 cy=240
    cfg.board.faces.push_back(fitra::lift::MarkerFace{0, 0.10});
    cfg.lin_vel_max_mps = 1e9;             // accept; we only test detection/gate
    cfg.ang_vel_max_dps = 1e9;
    ExtrinsicCalibSession s(cfg);
    s.start();

    // Render a fronto-parallel marker centred in a 640x480 frame.
    cv::aruco::Dictionary dict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Mat marker;
    dict.generateImageMarker(0, 240, marker, 1);
    cv::Mat frame(480, 640, CV_8UC1, cv::Scalar(255));
    marker.copyTo(frame(cv::Rect((640 - 240) / 2, (480 - 240) / 2, 240, 240)));

    ControllerObservation c;
    c.running_ok = true;
    c.x = 0; c.y = 1.0; c.z = 1.5; c.qw = 1.0;
    // Two still frames so a velocity estimate exists → gate reaches GOOD.
    c.ts_ms = 0.0;  s.on_frame(0, frame, c);
    c.ts_ms = 16.0; s.on_frame(0, frame, c);

    std::string js = s.state_json();
    CHECK(js.find("\"detections\":[") != std::string::npos);
    CHECK(js.find("\"cam\":0") != std::string::npos);
    CHECK(js.find("\"id\":0") != std::string::npos);
    CHECK(js.find("\"ok\":true") != std::string::npos);
    CHECK(js.find("\"gate_reason\":\"GOOD\"") != std::string::npos);

    // A frame with no marker + lost controller → NO_TAG / NO_POSE, never GOOD.
    ExtrinsicCalibSession s2(cfg);
    s2.start();
    cv::Mat blank(480, 640, CV_8UC1, cv::Scalar(255));
    ControllerObservation lost;
    lost.running_ok = false; lost.qw = 1.0; lost.y = 1.0; lost.z = 1.5;
    lost.ts_ms = 0.0;  s2.on_frame(0, blank, lost);
    lost.ts_ms = 16.0; s2.on_frame(0, blank, lost);
    std::string js2 = s2.state_json();
    CHECK(js2.find("\"gate_reason\":\"NO_TAG\"") != std::string::npos);
    CHECK(js2.find("\"gate_reason\":\"GOOD\"") == std::string::npos);
}

}  // namespace

int main() {
    test_gate_and_burst();
    test_solve_and_write();
    test_calib_io_roundtrip();
    test_on_frame_live_detection();
    if (g_fail) {
        std::fprintf(stderr, "test_extrinsic_calib_session: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_extrinsic_calib_session: OK\n");
    return 0;
}
