// calib_io: distortion_model round-trip + model/coefficient validation.

#include "lift/calib_io.hpp"

#include <opencv2/core.hpp>

#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace {

using fitra::lift::CalibrationSet;
using fitra::lift::CameraCalibration;
using fitra::lift::load_calibration;
using fitra::lift::validate_calibration;
using fitra::lift::write_calibration;

int g_fail = 0;
#define CHECK(cond) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++g_fail; } \
} while (0)

std::string tmp_path(const char* name) {
    const char* dir = std::getenv("TMPDIR");
    std::string base = dir ? dir : "/tmp";
    return base + "/" + name;
}

cv::Mat K3() {
    return (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
}

CameraCalibration cam(const std::string& id, const std::string& model, int ncoef) {
    CameraCalibration c;
    c.id = id;
    c.intrinsics.width = 1280;
    c.intrinsics.height = 960;
    c.intrinsics.distortion_model = model;
    c.intrinsics.K = K3();
    c.intrinsics.dist = cv::Mat::zeros(1, ncoef, CV_64F);
    return c;
}

void test_roundtrip() {
    CalibrationSet set;
    set.schema = "fitra_calibration_v1";
    set.cameras.push_back(cam("cam0", "pinhole", 5));
    set.cameras.push_back(cam("cam1", "fisheye", 4));

    const std::string path = tmp_path("fitra_calib_io_test.yaml");
    write_calibration(path, set);
    CalibrationSet got = load_calibration(path);

    CHECK(got.cameras.size() == 2);
    CHECK(got.cameras[0].intrinsics.distortion_model == "pinhole");
    CHECK(!got.cameras[0].intrinsics.is_fisheye());
    CHECK(got.cameras[1].intrinsics.distortion_model == "fisheye");
    CHECK(got.cameras[1].intrinsics.is_fisheye());
    std::remove(path.c_str());
}

void test_default_pinhole_backcompat() {
    // A file with no distortion_model key loads as pinhole.
    const std::string path = tmp_path("fitra_calib_io_nomodel.yaml");
    {
        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        fs << "schema" << "fitra_calibration_v1" << "unit" << "m"
           << "coordinate_system" << "world";
        fs << "intrinsics" << "{";
        fs << "cam0" << "{";
        fs << "width" << 640 << "height" << 480;
        fs << "K" << K3();
        fs << "dist" << cv::Mat::zeros(1, 5, CV_64F);
        fs << "}" << "}";
        fs.release();
    }
    CalibrationSet got = load_calibration(path);
    CHECK(got.cameras.size() == 1);
    CHECK(got.cameras[0].intrinsics.distortion_model == "pinhole");
    std::remove(path.c_str());
}

void test_validate_rejects() {
    // fisheye with 5 coefficients → reject.
    CalibrationSet bad;
    bad.cameras.push_back(cam("cam0", "fisheye", 5));
    bool threw = false;
    try { validate_calibration(bad); } catch (const std::exception&) { threw = true; }
    CHECK(threw);

    // unknown model → reject.
    CalibrationSet bad2;
    bad2.cameras.push_back(cam("cam0", "weird", 5));
    threw = false;
    try { validate_calibration(bad2); } catch (const std::exception&) { threw = true; }
    CHECK(threw);

    // valid fisheye (4) + pinhole (5) → ok.
    CalibrationSet good;
    good.cameras.push_back(cam("cam0", "fisheye", 4));
    good.cameras.push_back(cam("cam1", "pinhole", 5));
    threw = false;
    try { validate_calibration(good); } catch (const std::exception&) { threw = true; }
    CHECK(!threw);
}

}  // namespace

int main() {
    test_roundtrip();
    test_default_pinhole_backcompat();
    test_validate_rejects();
    if (g_fail) {
        std::fprintf(stderr, "test_calib_io: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_calib_io: OK\n");
    return 0;
}
