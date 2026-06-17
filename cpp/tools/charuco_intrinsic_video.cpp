// charuco_intrinsic_video — solve a single camera's intrinsics from a recorded
// ChArUco-board VIDEO (offline), reusing the live IntrinsicCalibSession.
//
// The live intrinsic step reads USB cameras; for a phone you cannot point the
// rig pipeline at, record a ChArUco sweep with the camera in a FIXED optical
// state (focus/zoom locked, stabilisation OFF) and feed the mp4 here. The
// diversity gate, solve (cv::calibrateCamera / cv::fisheye::calibrate) and the
// calib_io YAML writer are exactly the session's — only the frame source differs
// (cv::VideoCapture instead of V4L2). See
// docs/design/pose-3d-smartphone-sfm-marker-map.md.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <string_view>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "lift/charuco_board.hpp"
#include "pipeline/intrinsic_calib_session.hpp"

namespace {

void print_help() {
    std::puts(
        "charuco_intrinsic_video — intrinsics from a recorded ChArUco video\n"
        "\n"
        "Required:\n"
        "  --video PATH            input video (mp4/mov/...)\n"
        "\n"
        "Optional:\n"
        "  --out PATH              output intrinsics YAML "
        "(default calibrations/iphone_intrinsics.yaml)\n"
        "  --cam-id NAME           camera id in the YAML (default cam0)\n"
        "  --model MODEL           pinhole | fisheye (default pinhole)\n"
        "  --squares-x N           board columns in squares (default 5)\n"
        "  --squares-y N           board rows in squares (default 7)\n"
        "  --square M              square side, metres (default 0.056)\n"
        "  --marker M              embedded marker side, metres (default 0.042)\n"
        "  --dict ID               cv::aruco dictionary id (default -1 = DICT_4X4_50)\n"
        "  --min-views N           min accepted views to solve (default 15)\n"
        "  --min-corners N         min charuco corners per view (default 8)\n"
        "  --stride N              process every Nth frame (default 1)\n"
        "  --help                  show this help\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::string video;
    std::string out = "calibrations/iphone_intrinsics.yaml";
    std::string cam_id = "cam0";
    std::string model = "pinhole";
    int squares_x = 5, squares_y = 7;
    double square = 0.056, marker = 0.042;
    int dict = -1;
    int min_views = 15, min_corners = 8;
    int stride = 1;

    for (int i = 1; i < argc; ++i) {
        std::string_view a{argv[i]};
        auto need = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing argument for %s\n", flag);
                std::exit(EXIT_FAILURE);
            }
            return argv[++i];
        };
        if (a == "--help" || a == "-h")    { print_help(); return EXIT_SUCCESS; }
        else if (a == "--video")           { video = need("--video"); }
        else if (a == "--out")             { out = need("--out"); }
        else if (a == "--cam-id")          { cam_id = need("--cam-id"); }
        else if (a == "--model")           { model = need("--model"); }
        else if (a == "--squares-x")       { squares_x = std::atoi(need("--squares-x")); }
        else if (a == "--squares-y")       { squares_y = std::atoi(need("--squares-y")); }
        else if (a == "--square")          { square = std::atof(need("--square")); }
        else if (a == "--marker")          { marker = std::atof(need("--marker")); }
        else if (a == "--dict")            { dict = std::atoi(need("--dict")); }
        else if (a == "--min-views")       { min_views = std::atoi(need("--min-views")); }
        else if (a == "--min-corners")     { min_corners = std::atoi(need("--min-corners")); }
        else if (a == "--stride")          { stride = std::max(1, std::atoi(need("--stride"))); }
        else {
            std::fprintf(stderr, "unknown arg: %s\n", argv[i]);
            print_help();
            return EXIT_FAILURE;
        }
    }

    if (video.empty()) { print_help(); return EXIT_FAILURE; }
    if (model != "pinhole" && model != "fisheye") {
        std::fprintf(stderr, "--model must be pinhole or fisheye\n");
        return EXIT_FAILURE;
    }

    cv::VideoCapture cap{video};
    if (!cap.isOpened()) {
        std::fprintf(stderr, "failed to open video: %s\n", video.c_str());
        return EXIT_FAILURE;
    }

    fitra::pipeline::IntrinsicCalibConfig cfg;
    cfg.board.squares_x = squares_x;
    cfg.board.squares_y = squares_y;
    cfg.board.square_len_m = square;
    cfg.board.marker_len_m = marker;
    cfg.board.dictionary = dict;
    cfg.distortion_model = model;
    cfg.num_cams = 1;
    cfg.cam_ids = {cam_id};
    cfg.min_views = min_views;
    cfg.min_corners = min_corners;
    cfg.max_views = 60;  // a video sweep yields many candidates; allow more
    cfg.out_path = out;

    fitra::lift::CharucoBoardDetector detector{cfg.board};
    fitra::pipeline::IntrinsicCalibSession session{cfg};
    session.start();

    cv::Mat frame;
    long read = 0, detected = 0;
    int idx = 0;
    while (cap.read(frame)) {
        if ((idx++ % stride) != 0) continue;
        if (frame.empty()) continue;
        ++read;
        fitra::lift::CharucoView v = detector.detect(frame);
        if (v.count() > 0) ++detected;
        session.ingest(0, v, frame.cols, frame.rows);
        if (read % 100 == 0) {
            std::printf("\rframes=%ld board_seen=%ld accepted_views=%zu",
                        read, detected, session.accepted_views(0));
            std::fflush(stdout);
        }
    }
    std::printf("\rframes=%ld board_seen=%ld accepted_views=%zu\n",
                read, detected, session.accepted_views(0));

    std::string err;
    bool ok = session.solve_and_write(err);
    std::printf("%s\n", session.state_json().c_str());
    if (!ok) {
        std::fprintf(stderr, "solve failed: %s\n", err.c_str());
        return EXIT_FAILURE;
    }
    std::printf("wrote %s\n", out.c_str());
    return EXIT_SUCCESS;
}
