// scale_intrinsics — rescale a calibration's intrinsics to a runtime resolution.
//
// Supports the "calibrate high, run low" split: markers/ChArUco need high
// resolution to detect, but the live pipeline wants a lower resolution for
// latency/fps. The triangulator does NOT rescale K, so the calibration handed
// to `run` must match the capture resolution. This loads a CalibrationSet,
// rescales every camera's intrinsics (fx,fy + principal point; distortion is
// scale-invariant) to (--width,--height), and writes a new file — extrinsics
// (T_cw, camera_center_w) are resolution-independent and pass through unchanged.
//
// Same-FOV resize only (uniform downscale, not a crop): a changed aspect ratio
// is rejected. See lift::scale_intrinsics.

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <string_view>

#include "lift/calib_io.hpp"

namespace {

void print_help() {
    std::puts(
        "scale_intrinsics — rescale calibration intrinsics to a new resolution\n"
        "\n"
        "Required:\n"
        "  --in PATH       input calibration YAML (intrinsics [+ extrinsics])\n"
        "  --width N       target width\n"
        "  --height N      target height\n"
        "  --out PATH      output calibration YAML\n"
        "\n"
        "Optional:\n"
        "  --help          show this help\n"
        "\n"
        "Extrinsics (T_cw) are resolution-independent and pass through unchanged.\n"
        "Valid only for a same-FOV resize (uniform downscale, not a crop).\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::string in_path, out_path;
    int width = 0, height = 0;

    for (int i = 1; i < argc; ++i) {
        std::string_view a{argv[i]};
        auto need = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing argument for %s\n", flag);
                std::exit(EXIT_FAILURE);
            }
            return argv[++i];
        };
        if (a == "--help" || a == "-h") { print_help(); return EXIT_SUCCESS; }
        else if (a == "--in")     { in_path = need("--in"); }
        else if (a == "--out")    { out_path = need("--out"); }
        else if (a == "--width")  { width = std::atoi(need("--width")); }
        else if (a == "--height") { height = std::atoi(need("--height")); }
        else {
            std::fprintf(stderr, "unknown arg: %s\n", argv[i]);
            print_help();
            return EXIT_FAILURE;
        }
    }
    if (in_path.empty() || out_path.empty() || width <= 0 || height <= 0) {
        print_help();
        return EXIT_FAILURE;
    }

    try {
        fitra::lift::CalibrationSet calib = fitra::lift::load_calibration(in_path);
        for (auto& cam : calib.cameras) {
            const int ow = cam.intrinsics.width, oh = cam.intrinsics.height;
            cam.intrinsics = fitra::lift::scale_intrinsics(cam.intrinsics, width, height);
            std::printf("%s: %dx%d -> %dx%d  fx=%.2f fy=%.2f cx=%.2f cy=%.2f%s\n",
                        cam.id.c_str(), ow, oh, width, height,
                        cam.intrinsics.K.at<double>(0, 0),
                        cam.intrinsics.K.at<double>(1, 1),
                        cam.intrinsics.K.at<double>(0, 2),
                        cam.intrinsics.K.at<double>(1, 2),
                        cam.has_extrinsics ? "  [T_cw preserved]" : "");
        }
        fitra::lift::write_calibration(out_path, calib);
        std::printf("wrote %s\n", out_path.c_str());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "scale_intrinsics failed: %s\n", e.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
