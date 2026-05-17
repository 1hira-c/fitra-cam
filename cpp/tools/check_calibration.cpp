#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <string_view>

#include "lift/calib_io.hpp"

namespace {

void print_help() {
    std::puts(
        "check_calibration — validate fitra-cam calibration YAML\n"
        "\n"
        "Required:\n"
        "  --calib PATH              calibration YAML\n"
        "\n"
        "Optional:\n"
        "  --require-extrinsics      fail if any camera lacks T_cw\n"
        "  --help                    show this help\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::string path;
    bool require_extrinsics = false;

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
        else if (a == "--calib")        { path = need("--calib"); }
        else if (a == "--require-extrinsics") { require_extrinsics = true; }
        else {
            std::fprintf(stderr, "unknown arg: %s\n", argv[i]);
            print_help();
            return EXIT_FAILURE;
        }
    }

    if (path.empty()) {
        print_help();
        return EXIT_FAILURE;
    }

    try {
        auto calib = fitra::lift::load_calibration(path);
        std::printf("schema=%s unit=%s cameras=%zu\n",
                    calib.schema.c_str(), calib.unit.c_str(), calib.cameras.size());
        for (const auto& cam : calib.cameras) {
            if (require_extrinsics && !cam.has_extrinsics) {
                std::fprintf(stderr, "camera %s lacks extrinsics\n", cam.id.c_str());
                return EXIT_FAILURE;
            }
            std::printf(
                "%s: %dx%d fx=%.3f fy=%.3f cx=%.3f cy=%.3f dist=%d source=%s rms=%.4f",
                cam.id.c_str(),
                cam.intrinsics.width,
                cam.intrinsics.height,
                cam.intrinsics.K.at<double>(0, 0),
                cam.intrinsics.K.at<double>(1, 1),
                cam.intrinsics.K.at<double>(0, 2),
                cam.intrinsics.K.at<double>(1, 2),
                cam.intrinsics.dist.cols,
                cam.intrinsics.source.c_str(),
                cam.intrinsics.rms_px);
            if (cam.has_extrinsics) {
                std::printf(" center_w=[%.4f %.4f %.4f] method=%s",
                            cam.extrinsics.camera_center_w[0],
                            cam.extrinsics.camera_center_w[1],
                            cam.extrinsics.camera_center_w[2],
                            cam.extrinsics.method.c_str());
            }
            std::printf("\n");
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "invalid calibration: %s\n", e.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
