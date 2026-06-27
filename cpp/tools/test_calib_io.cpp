// calib_io: distortion_model round-trip + model/coefficient validation.

#include "lift/calib_io.hpp"

#include <opencv2/core.hpp>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace {

namespace fs = std::filesystem;

using fitra::lift::CalibrationSet;
using fitra::lift::CameraCalibration;
using fitra::lift::clear_calib_latest;
using fitra::lift::load_calibration;
using fitra::lift::scale_intrinsics;
using fitra::lift::select_calib_cameras;
using fitra::lift::validate_calibration;
using fitra::lift::write_calibration;
using fitra::lift::write_calibration_versioned;

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

void test_scale_intrinsics() {
    using fitra::lift::Intrinsics;
    Intrinsics in;
    in.width = 1280;
    in.height = 960;
    in.distortion_model = "fisheye";
    in.K = (cv::Mat_<double>(3, 3) << 680.0, 0, 628.0, 0, 681.0, 490.0, 0, 0, 1);
    in.dist = (cv::Mat_<double>(1, 4) << 0.05, -0.01, 0.002, -0.0005);
    in.rms_px = 0.2;

    // 1280x960 -> 640x480 is a uniform 0.5x downscale.
    Intrinsics out = scale_intrinsics(in, 640, 480);
    CHECK(out.width == 640 && out.height == 480);
    CHECK(std::abs(out.K.at<double>(0, 0) - 340.0) < 1e-9);   // fx * 0.5
    CHECK(std::abs(out.K.at<double>(1, 1) - 340.5) < 1e-9);   // fy * 0.5
    CHECK(std::abs(out.K.at<double>(0, 2) - ((628.0 + 0.5) * 0.5 - 0.5)) < 1e-9);
    CHECK(std::abs(out.K.at<double>(1, 2) - ((490.0 + 0.5) * 0.5 - 0.5)) < 1e-9);
    // Distortion is scale-invariant → unchanged.
    for (int i = 0; i < 4; ++i)
        CHECK(std::abs(out.dist.at<double>(0, i) - in.dist.at<double>(0, i)) < 1e-12);
    CHECK(out.distortion_model == "fisheye");

    // Aspect-ratio change (crop, not resize) → reject.
    bool threw = false;
    try { scale_intrinsics(in, 640, 360); } catch (const std::exception&) { threw = true; }
    CHECK(threw);
}

CalibrationSet minimal_set() {
    CalibrationSet set;
    set.schema = "fitra_calibration_v1";
    set.cameras.push_back(cam("cam0", "pinhole", 5));
    return set;
}

// select_calib_cameras: subset a 3-camera calib to the views a stage uses
// (2-view subject calib / dump_keypoints_3d), in the requested id order.
void test_select_calib_cameras() {
    CalibrationSet set;
    set.schema = "fitra_calibration_v1";
    set.cameras.push_back(cam("cam0", "pinhole", 5));
    set.cameras.push_back(cam("cam1", "pinhole", 5));
    set.cameras.push_back(cam("cam2", "pinhole", 5));

    CalibrationSet two = select_calib_cameras(set, {"cam0", "cam1"});
    CHECK(two.cameras.size() == 2);
    CHECK(two.cameras[0].id == "cam0" && two.cameras[1].id == "cam1");
    CHECK(two.schema == set.schema);  // metadata preserved
    // require_camera_ids would now match a 2-view stage (cam2 dropped, not error).

    // Order follows `ids`, not the source order; missing ids are skipped.
    CalibrationSet rev = select_calib_cameras(set, {"cam1", "cam0"});
    CHECK(rev.cameras.size() == 2 && rev.cameras[0].id == "cam1");
    CalibrationSet miss = select_calib_cameras(set, {"cam0", "camX"});
    CHECK(miss.cameras.size() == 1 && miss.cameras[0].id == "cam0");
}

// latest.yaml convention → timestamped sibling + relative symlink; a second
// solve accumulates history (collision counter keeps a distinct artifact within
// the same second). An explicit (non-latest) path stays an in-place write.
void test_versioned_write_and_symlink() {
    fs::path base = tmp_path("fitra_calib_lr_ver");
    fs::remove_all(base);
    fs::path dir = base / "calibrations" / "extrinsics";
    fs::path latest = dir / "latest.yaml";

    write_calibration_versioned(latest.string(), minimal_set());
    write_calibration_versioned(latest.string(), minimal_set());

    CHECK(fs::exists(latest));
    CHECK(fs::is_symlink(latest));
    fs::path tgt = fs::read_symlink(latest);
    CHECK(tgt.parent_path().empty());           // relative (relocatable)
    CHECK(tgt.filename() != "latest.yaml");
    int artifacts = 0;
    for (const auto& e : fs::directory_iterator(dir)) {
        if (e.is_symlink()) continue;
        if (e.path().extension() == ".yaml") ++artifacts;
    }
    CHECK(artifacts >= 2);                       // history kept, not clobbered
    CHECK(load_calibration(latest.string()).cameras.size() == 1);

    // explicit path → in-place, no symlink machinery
    fs::path explicit_path = base / "extrinsics.yaml";
    write_calibration_versioned(explicit_path.string(), minimal_set());
    CHECK(fs::exists(explicit_path) && !fs::is_symlink(explicit_path));
    fs::remove_all(base);
}

// clear_calib_latest removes ONLY the latest.yaml pointer; the timestamped
// history survives, and a non-latest path is a no-op.
void test_clear_calib_latest() {
    fs::path base = tmp_path("fitra_calib_lr_clear");
    fs::remove_all(base);
    fs::path dir = base / "calibrations" / "intrinsics";
    fs::path latest = dir / "latest.yaml";
    write_calibration_versioned(latest.string(), minimal_set());

    int before = 0;
    for (const auto& e : fs::directory_iterator(dir))
        if (!e.is_symlink() && e.path().extension() == ".yaml") ++before;
    CHECK(before == 1);

    CHECK(clear_calib_latest(latest.string()));   // pointer removed
    CHECK(!fs::exists(latest) && !fs::is_symlink(latest));
    int after = 0;
    for (const auto& e : fs::directory_iterator(dir))
        if (!e.is_symlink() && e.path().extension() == ".yaml") ++after;
    CHECK(after == before);                        // history intact

    CHECK(!clear_calib_latest(latest.string()));   // already gone → no-op
    CHECK(!clear_calib_latest((base / "extrinsics.yaml").string()));  // non-latest → no-op
    fs::remove_all(base);
}

}  // namespace

int main() {
    test_roundtrip();
    test_default_pinhole_backcompat();
    test_validate_rejects();
    test_scale_intrinsics();
    test_versioned_write_and_symlink();
    test_clear_calib_latest();
    test_select_calib_cameras();
    if (g_fail) {
        std::fprintf(stderr, "test_calib_io: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_calib_io: OK\n");
    return 0;
}
