// Integration smoke test for the /api/excal/* routes: spins up a real
// CrowServer with an attached ExtrinsicCalibSession on a loopback port and
// drives state / start / stop / solve over HTTP via raw sockets. No cameras,
// no GPU — just the web wiring. Also locks the solve contract of the
// dedicated calib-extrinsic mode: a successful solve fires the session's
// on_solved hook (main wires it to auto-exit) and the response carries the
// "next_step" restart guidance (docs/design/pose-3d-calib-mode-separation.md).

#include "web/crow_server.hpp"
#include "config/main_config.hpp"
#include "pipeline/extrinsic_calib_session.hpp"
#include "pipeline/snapshot.hpp"
#include "lift/calib_io.hpp"
#include "lift/extrinsic_solver.hpp"
#include "vmt/controller_pose_receiver.hpp"
#include "vmt/hmd_pose_receiver.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

namespace {

int g_fail = 0;
#define CHECK(cond) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++g_fail; } \
} while (0)

constexpr int kPort = 18137;

// Minimal blocking HTTP/1.1 client. Returns false on connect/IO failure;
// otherwise sets status + body. Uses Connection: close so recv ends at EOF.
// `payload` (optional) is sent as a JSON request body.
bool http(const char* method, const char* path, int& status, std::string& body,
          const std::string& payload = std::string{}) {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return false;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(kPort);
    ::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return false;
    }
    std::string req = std::string(method) + " " + path + " HTTP/1.1\r\n"
                      "Host: 127.0.0.1\r\n"
                      "Content-Type: application/json\r\n"
                      "Content-Length: " + std::to_string(payload.size()) + "\r\n"
                      "Connection: close\r\n\r\n" + payload;
    ::send(fd, req.data(), req.size(), 0);
    std::string raw;
    char buf[2048];
    ssize_t n;
    while ((n = ::recv(fd, buf, sizeof(buf), 0)) > 0) raw.append(buf, n);
    ::close(fd);
    if (raw.empty()) return false;
    // Parse "HTTP/1.1 <status> ..."
    std::size_t sp = raw.find(' ');
    status = (sp != std::string::npos) ? std::atoi(raw.c_str() + sp + 1) : 0;
    std::size_t hdr_end = raw.find("\r\n\r\n");
    body = (hdr_end != std::string::npos) ? raw.substr(hdr_end + 4) : std::string{};
    return true;
}

fitra::lift::CalibrationSet make_intrinsics() {
    fitra::lift::CalibrationSet set;
    set.schema = "fitra_calibration_v1";
    set.unit = "m";
    fitra::lift::CameraCalibration cam;
    cam.id = "cam0";
    cam.intrinsics.width = 640;
    cam.intrinsics.height = 480;
    cam.intrinsics.K = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
    cam.intrinsics.dist = cv::Mat::zeros(1, 5, CV_64F);
    set.cameras.push_back(cam);
    return set;
}

// Synthetic-sample helpers, same pattern as test_extrinsic_calib_session.
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

fitra::pipeline::ControllerObservation ctrl_at(const cv::Matx44d& B, double ts_ms) {
    fitra::pipeline::ControllerObservation c;
    c.running_ok = true;
    c.x = B(0, 3); c.y = B(1, 3); c.z = B(2, 3);
    double t = B(0,0) + B(1,1) + B(2,2), w, x, y, z;
    if (t > 0) { double s = std::sqrt(t + 1.0) * 2; w = 0.25*s; x=(B(2,1)-B(1,2))/s; y=(B(0,2)-B(2,0))/s; z=(B(1,0)-B(0,1))/s; }
    else if (B(0,0) > B(1,1) && B(0,0) > B(2,2)) { double s=std::sqrt(1+B(0,0)-B(1,1)-B(2,2))*2; w=(B(2,1)-B(1,2))/s; x=0.25*s; y=(B(0,1)+B(1,0))/s; z=(B(0,2)+B(2,0))/s; }
    else if (B(1,1) > B(2,2)) { double s=std::sqrt(1+B(1,1)-B(0,0)-B(2,2))*2; w=(B(0,2)-B(2,0))/s; x=(B(0,1)+B(1,0))/s; y=0.25*s; z=(B(1,2)+B(2,1))/s; }
    else { double s=std::sqrt(1+B(2,2)-B(0,0)-B(1,1))*2; w=(B(1,0)-B(0,1))/s; x=(B(0,2)+B(2,0))/s; y=(B(1,2)+B(2,1))/s; z=0.25*s; }
    c.qx = x; c.qy = y; c.qz = z; c.qw = w;
    c.ts_ms = ts_ms;
    return c;
}

}  // namespace

int main() {
    fitra::pipeline::ExtrinsicCalibConfig cfg;
    cfg.intrinsics = make_intrinsics();
    cfg.board.faces.push_back(fitra::lift::MarkerFace{0, 0.10});
    cfg.out_path = "/tmp/fitra_excal_route_test.yaml";
    // Permissive gate so the success-path samples below pass straight through.
    cfg.lin_vel_max_mps = 1e9;
    cfg.ang_vel_max_dps = 1e9;
    cfg.burst_min = 3;
    cfg.burst_max = 1000;
    cfg.burst_gap_ms = 100.0;
    cfg.min_samples_per_group = 3;
    fitra::pipeline::ExtrinsicCalibSession session(cfg);

    // Auto-exit hook: main wires this to g_stop; here we only record it.
    std::atomic<int> solved_calls{0};
    session.set_on_solved([&solved_calls]() { ++solved_calls; });

    fitra::pipeline::SnapshotBus bus{1};
    fitra::vmt::HmdPoseBus hmd_bus;
    fitra::vmt::ControllerPoseBus controller_bus;

    fitra::vmt::HmdPose hmd;
    hmd.valid = true;
    hmd.timestamp_s = 12.5f;
    hmd.x = 1.25f;
    hmd.y = 2.5f;
    hmd.z = -3.75f;
    hmd.qx = 0.0f;
    hmd.qy = 0.38268343f;
    hmd.qz = 0.0f;
    hmd.qw = 0.9238795f;
    hmd_bus.publish(hmd);

    fitra::vmt::ControllerPose controller;
    controller.valid = true;
    controller.tracking_result = fitra::vmt::kTrackingResultRunningOk;
    controller.timestamp_s = 13.25f;
    controller.x = -0.5f;
    controller.y = 1.0f;
    controller.z = 2.25f;
    controller.qx = 0.1f;
    controller.qy = 0.2f;
    controller.qz = 0.3f;
    controller.qw = 0.9f;
    controller_bus.publish(controller);

    fitra::web::ServerOptions opts;
    opts.host = "127.0.0.1";
    opts.port = kPort;
    opts.publish_hz = 5.0;
    opts.mode_label = "calib-extrinsic";
#ifdef FITRA_EXCAL_WEB_DIR
    opts.excal_static_dir = FITRA_EXCAL_WEB_DIR;
#endif
#ifdef FITRA_SUBJECT_WEB_DIR
    opts.calib_static_dir = FITRA_SUBJECT_WEB_DIR;
#endif
    // Heap-allocated so it can be fully destroyed before the managed-server
    // block below: Crow's App keeps its Server (and the listening fd) alive
    // after stop(), so an in-process rebind of the same port needs the whole
    // CrowServer gone. Across processes (the flow daemon's module handover)
    // the fd closes with the process — no issue there.
    auto server = std::make_unique<fitra::web::CrowServer>(bus, nullptr, opts);
    server->set_extrinsic_calib_session(&session);
    server->set_extrinsic_calib_next_step(
        "restart: ./main --calibrate --enable-3d --calib /tmp/fitra_excal_route_test.yaml ...");
    server->set_hmd_pose_bus(&hmd_bus, 10000.0);
    server->set_extrinsic_calib_pose_bus(&controller_bus, "left", 10000.0);
    server->start();

    // Poll until the server accepts connections.
    int status = 0;
    std::string body;
    bool up = false;
    for (int i = 0; i < 100; ++i) {
        if (http("GET", "/api/excal/state", status, body)) { up = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    CHECK(up);

    if (up) {
        // Fresh session is idle; state_json carries the config the UI needs.
        CHECK(status == 200);
        CHECK(body.find("\"state\":\"idle\"") != std::string::npos);
        CHECK(body.find("\"num_cams\":1") != std::string::npos);
        CHECK(body.find("\"min_samples\":") != std::string::npos);
        CHECK(body.find("\"faces\":[0]") != std::string::npos);

        // Extrinsics route: valid JSON, no solution yet.
        CHECK(http("GET", "/api/excal/extrinsics", status, body));
        CHECK(status == 200);
        CHECK(body.find("\"solved\":false") != std::string::npos);
        CHECK(body.find("\"cameras\":[]") != std::string::npos);

        // Live pose route: HMD plus selected calibration controller only.
        CHECK(http("GET", "/api/excal/poses", status, body));
        CHECK(status == 200);
        CHECK(body.find("\"hmd\":{") != std::string::npos);
        CHECK(body.find("\"enabled\":true") != std::string::npos);
        CHECK(body.find("\"have_any\":true") != std::string::npos);
        CHECK(body.find("\"stale\":false") != std::string::npos);
        CHECK(body.find("\"valid\":true") != std::string::npos);
        CHECK(body.find("\"timestamp_s\":12.5") != std::string::npos);
        CHECK(body.find("\"pos\":[1.25,2.5,-3.75]") != std::string::npos);
        CHECK(body.find("\"quat_xyzw\":[0,0.382683") != std::string::npos);
        CHECK(body.find("\"controller\":{") != std::string::npos);
        CHECK(body.find("\"role\":\"left\"") != std::string::npos);
        CHECK(body.find("\"running_ok\":true") != std::string::npos);
        CHECK(body.find("\"tracking_result\":200") != std::string::npos);
        CHECK(body.find("\"timestamp_s\":13.25") != std::string::npos);
        CHECK(body.find("\"pos\":[-0.5,1,2.25]") != std::string::npos);

        // /api/state reports the mode label so the frontends can show the
        // right calibration entry points.
        CHECK(http("GET", "/api/state", status, body));
        CHECK(status == 200);
        CHECK(body.find("\"mode\":\"calib-extrinsic\"") != std::string::npos);
        CHECK(body.find("\"managed\":false") != std::string::npos);
        CHECK(body.find("\"enable_3d\":false") != std::string::npos);

        // /api/flow/switch only exists on daemon-managed modules. Standalone
        // runs never attach the handler, so the POST falls through to the
        // GET-only static catchall → 405 (docs/design/pose-3d-flow-daemon.md).
        CHECK(http("POST", "/api/flow/switch", status, body,
                   "{\"mode\":\"run\"}"));
        CHECK(status == 405);

        // The subject-calib group (static pages AND /api/calib/*) only exists
        // in calib-subject mode. With no CalibrationSession attached nothing
        // is registered, so requests fall through to the GET-only static
        // catchall: GET → 404 (no such file), POST → 405 (no POST rule).
        // No 503 JSON stub anymore
        // (docs/design/pose-3d-calib-mode-separation.md).
        CHECK(http("GET", "/subject-calib", status, body));
        CHECK(status == 404);
        CHECK(http("GET", "/api/calib/state", status, body));
        CHECK(status == 404);
        CHECK(http("POST", "/api/calib/start", status, body));
        CHECK(status == 405);

#ifdef FITRA_EXCAL_WEB_DIR
        // Static frontend is served (collect page + 3D scene).
        CHECK(http("GET", "/extrinsic-calib", status, body));
        CHECK(status == 200);
        CHECK(body.find("Extrinsic Calibration") != std::string::npos);
        CHECK(http("GET", "/extrinsic-calib/app.js", status, body));
        CHECK(status == 200);
        CHECK(body.find("/api/excal/state") != std::string::npos);
        CHECK(http("GET", "/extrinsic-calib/scene.html", status, body));
        CHECK(status == 200);
        CHECK(body.find("importmap") != std::string::npos);
        CHECK(http("GET", "/extrinsic-calib/scene.js", status, body));
        CHECK(status == 200);
        CHECK(body.find("/api/excal/extrinsics") != std::string::npos);
#endif

        // start → collecting.
        CHECK(http("POST", "/api/excal/start", status, body));
        CHECK(status == 200);
        CHECK(body.find("\"ok\":true") != std::string::npos);
        CHECK(http("GET", "/api/excal/state", status, body));
        CHECK(body.find("\"state\":\"collecting\"") != std::string::npos);
        CHECK(body.find("\"coverage\":[") != std::string::npos);

        // stop → reports sample count (0).
        CHECK(http("POST", "/api/excal/stop", status, body));
        CHECK(status == 200);
        CHECK(body.find("\"samples\":0") != std::string::npos);

        // solve with no samples → ok:false, state failed; no guidance, no
        // auto-exit hook fired.
        CHECK(http("POST", "/api/excal/solve", status, body));
        CHECK(status == 200);
        CHECK(body.find("\"ok\":false") != std::string::npos);
        CHECK(body.find("\"state\":\"failed\"") != std::string::npos);
        CHECK(body.find("\"next_step\"") == std::string::npos);
        CHECK(solved_calls.load() == 0);

        // Success path: refill with synthetic samples (same geometry as
        // test_extrinsic_calib_session::test_solve_and_write, 1 camera) and
        // solve again → ok:true + next_step guidance + on_solved fired once.
        CHECK(http("POST", "/api/excal/start", status, body));
        CHECK(status == 200);
        const cv::Matx44d Tcw     = rigid(0.0, 0.0, 0.0, 0.0, 0.1, 2.0);
        const cv::Matx44d Tcf_off = rigid(0.4, -0.3, 0.0, 0.03, -0.02, 0.05);
        double ts = 0.0;
        for (int p = 0; p < 6; ++p) {
            cv::Matx44d B = rigid(0.3 * p, -0.4 * p, 0.2 * p,
                                  0.1 * p, 1.0, 1.5 + 0.05 * p);
            ts += 200.0;
            auto A = fitra::geom::T_cam_marker::from_raw(Tcw * B * Tcf_off);
            for (int k = 0; k < cfg.burst_min + 1; ++k) {
                session.ingest(0, 0, A, ctrl_at(B, ts));
                ts += 10.0;
            }
        }
        CHECK(http("POST", "/api/excal/solve", status, body));
        CHECK(status == 200);
        CHECK(body.find("\"ok\":true") != std::string::npos);
        CHECK(body.find("\"state\":\"solved\"") != std::string::npos);
        CHECK(body.find("\"next_step\":\"restart: ./main --calibrate") != std::string::npos);
        CHECK(solved_calls.load() == 1);
    }

    server->stop();
    server.reset();  // release the listening fd (see the allocation comment)

    // Managed-module contract: /api/state advertises managed:true and
    // POST /api/flow/switch hands the parsed mode label to the handler
    // (server_builder wires it to FlowControl::request_switch). Binding the
    // same port right after stop() also smoke-tests the daemon's
    // module-to-module port handover.
    {
        fitra::web::ServerOptions mopts = opts;
        mopts.mode_label = "run";
        mopts.flow_managed = true;
        fitra::pipeline::SnapshotBus mbus{1};
        fitra::web::CrowServer managed(mbus, nullptr, mopts);
        std::atomic<int> switched{-1};
        managed.set_flow_switch_handler(
            [&switched](const std::string& mode, std::string& err) {
                fitra::config::RunMode m;
                if (!fitra::config::parse_run_mode_name(mode, m)) {
                    err = "unknown mode: " + mode;
                    return false;
                }
                switched.store(static_cast<int>(m));
                return true;
            });
        managed.start();

        bool up2 = false;
        for (int i = 0; i < 100; ++i) {
            if (http("GET", "/api/state", status, body)) { up2 = true; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        CHECK(up2);
        if (up2) {
            CHECK(status == 200);
            CHECK(body.find("\"mode\":\"run\"") != std::string::npos);
            CHECK(body.find("\"managed\":true") != std::string::npos);

            CHECK(http("POST", "/api/flow/switch", status, body,
                       "{\"mode\":\"calib-extrinsic\"}"));
            CHECK(status == 200);
            CHECK(body.find("\"ok\":true") != std::string::npos);
            CHECK(body.find("\"mode\":\"calib-extrinsic\"") != std::string::npos);
            CHECK(switched.load()
                  == static_cast<int>(fitra::config::RunMode::CalibExtrinsic));

            // Unknown label → handler refuses, nothing recorded.
            switched.store(-1);
            CHECK(http("POST", "/api/flow/switch", status, body,
                       "{\"mode\":\"bogus\"}"));
            CHECK(status == 200);
            CHECK(body.find("\"ok\":false") != std::string::npos);
            CHECK(body.find("unknown mode") != std::string::npos);
            CHECK(switched.load() == -1);

            // Missing/empty body → empty mode label → refused the same way.
            CHECK(http("POST", "/api/flow/switch", status, body));
            CHECK(status == 200);
            CHECK(body.find("\"ok\":false") != std::string::npos);
            CHECK(switched.load() == -1);
        }
        managed.stop();
    }

    if (g_fail) {
        std::fprintf(stderr, "test_crow_excal: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_crow_excal: OK\n");
    return 0;
}
