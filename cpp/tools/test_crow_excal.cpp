// Integration smoke test for the /api/excal/* routes: spins up a real
// CrowServer with an attached ExtrinsicCalibSession on a loopback port and
// drives state / start / stop / solve over HTTP via raw sockets. No cameras,
// no GPU — just the web wiring.

#include "web/crow_server.hpp"
#include "pipeline/extrinsic_calib_session.hpp"
#include "pipeline/snapshot.hpp"
#include "lift/calib_io.hpp"
#include "vmt/controller_pose_receiver.hpp"
#include "vmt/hmd_pose_receiver.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
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
bool http(const char* method, const char* path, int& status, std::string& body) {
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
                      "Host: 127.0.0.1\r\nContent-Length: 0\r\n"
                      "Connection: close\r\n\r\n";
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

}  // namespace

int main() {
    fitra::pipeline::ExtrinsicCalibConfig cfg;
    cfg.intrinsics = make_intrinsics();
    cfg.board.faces.push_back(fitra::lift::MarkerFace{0, 0.10});
    cfg.out_path = "/tmp/fitra_excal_route_test.yaml";
    fitra::pipeline::ExtrinsicCalibSession session(cfg);

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
#ifdef FITRA_EXCAL_WEB_DIR
    opts.excal_static_dir = FITRA_EXCAL_WEB_DIR;
#endif
#ifdef FITRA_SUBJECT_WEB_DIR
    opts.calib_static_dir = FITRA_SUBJECT_WEB_DIR;
#endif
    fitra::web::CrowServer server(bus, nullptr, opts);
    server.set_extrinsic_calib_session(&session);
    server.set_hmd_pose_bus(&hmd_bus, 10000.0);
    server.set_extrinsic_calib_pose_bus(&controller_bus, "left", 10000.0);
    server.start();

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

#ifdef FITRA_SUBJECT_WEB_DIR
        // Subject calibration UI is static and remains reachable even when
        // the CalibrationSession is not attached (for example non-3D runs).
        CHECK(http("GET", "/subject-calib", status, body));
        CHECK(status == 200);
        CHECK(body.find("Subject Profile Calibration") != std::string::npos);
        CHECK(http("GET", "/subject-calib/app.js", status, body));
        CHECK(status == 200);
        CHECK(body.find("/api/calib/state") != std::string::npos);
        // /api/calib/* only exist in calib-subject mode. With no session
        // attached the routes are never registered, so requests fall through
        // to the GET-only static catchall: GET → 404 (no such file), POST →
        // 405 (no POST rule). Either way there is no 503 JSON stub anymore
        // (docs/design/pose-3d-calib-mode-separation.md).
        CHECK(http("GET", "/api/calib/state", status, body));
        CHECK(status == 404);
        CHECK(http("POST", "/api/calib/start", status, body));
        CHECK(status == 405);
#endif

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

        // solve with no samples → ok:false, state failed.
        CHECK(http("POST", "/api/excal/solve", status, body));
        CHECK(status == 200);
        CHECK(body.find("\"ok\":false") != std::string::npos);
        CHECK(body.find("\"state\":\"failed\"") != std::string::npos);
    }

    server.stop();

    if (g_fail) {
        std::fprintf(stderr, "test_crow_excal: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_crow_excal: OK\n");
    return 0;
}
