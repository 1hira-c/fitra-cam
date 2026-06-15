// Live↔replay equivalence test for the floor-AprilTag calibration replay path
// (docs/design/pose-3d-floor-apriltag-extrinsic.md).
//
// Synthesize AprilTag frames, JPEG-encode them into a temporary recorder-format
// session, then feed the SAME encoded bytes through (a) imdecode + on_frame()
// directly and (b) ExcalReplayInput — the accumulated corner state (exposed via
// state_json + ready_group_count) must match. Reuses the controller path's
// recorder format and ExcalReplayInput; the floor session ignores the paired
// controller pose, keeping only the frame timestamp.

#include "pipeline/excal_replay_input.hpp"
#include "pipeline/floor_calib_session.hpp"
#include "lift/calib_io.hpp"
#include "lift/floor_tag_map.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

using fitra::lift::FloorTag;
using fitra::lift::FloorTagMap;
using fitra::pipeline::ControllerObservation;
using fitra::pipeline::ExcalInputItem;
using fitra::pipeline::ExcalReplayInput;
using fitra::pipeline::FloorCalibConfig;
using fitra::pipeline::FloorCalibSession;

int g_fail = 0;
#define CHECK(cond) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++g_fail; } \
} while (0)

fitra::lift::CalibrationSet make_intrinsics(int n_cams) {
    fitra::lift::CalibrationSet set;
    set.schema = "fitra_calibration_v1";
    set.unit = "m";
    for (int i = 0; i < n_cams; ++i) {
        fitra::lift::CameraCalibration cam;
        cam.id = "cam" + std::to_string(i);
        cam.intrinsics.width = 640;
        cam.intrinsics.height = 480;
        cam.intrinsics.K = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
        cam.intrinsics.dist = cv::Mat::zeros(1, 5, CV_64F);
        set.cameras.push_back(cam);
    }
    return set;
}

FloorTag make_tag(int id, double size, const cv::Vec3d& t) {
    FloorTag tag; tag.id = id; tag.size_m = size;
    cv::Matx44d m = cv::Matx44d::eye();
    m(0, 3) = t[0]; m(1, 3) = t[1]; m(2, 3) = t[2];
    tag.T_world_tag = fitra::geom::T_world_marker::from_raw(m);
    return tag;
}

FloorCalibConfig make_config() {
    FloorCalibConfig cfg;
    cfg.intrinsics = make_intrinsics(2);
    cfg.map.tags.push_back(make_tag(0, 0.10, cv::Vec3d(0, 0, 0)));
    cfg.map.tags.push_back(make_tag(1, 0.10, cv::Vec3d(0.5, 0, 0)));
    cfg.burst_min = 3;
    cfg.burst_max = 1000;
    return cfg;
}

std::string jsonl_line(std::size_t cam, std::uint64_t seq, const std::string& file,
                       double ts_ms) {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "{\"cam\":%zu,\"seq\":%llu,\"file\":\"%s\","
        "\"ts_ms\":%.3f,\"ctrl\":{\"running_ok\":false,"
        "\"x\":0,\"y\":0,\"z\":0,\"qx\":0,\"qy\":0,\"qz\":0,\"qw\":1,"
        "\"stale\":true,\"age_ms\":8.0,\"tracking_result\":0,"
        "\"timestamp_s\":0.000000}}",
        cam, static_cast<unsigned long long>(seq), file.c_str(), ts_ms);
    return std::string{buf};
}

void test_equivalence() {
    cv::aruco::Dictionary dict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Mat marker;
    dict.generateImageMarker(0, 240, marker, 1);
    cv::Mat frame_tag(480, 640, CV_8UC1, cv::Scalar(255));
    marker.copyTo(frame_tag(cv::Rect((640 - 240) / 2, (480 - 240) / 2, 240, 240)));
    cv::Mat frame_blank(480, 640, CV_8UC1, cv::Scalar(255));

    std::vector<unsigned char> jpg_tag, jpg_blank;
    CHECK(cv::imencode(".jpg", frame_tag, jpg_tag));
    CHECK(cv::imencode(".jpg", frame_blank, jpg_blank));

    struct Step { std::size_t cam; bool tag; double ts_ms; };
    std::vector<Step> steps;
    double ts = 0.0;
    for (int k = 0; k < 8; ++k) {
        for (std::size_t cam = 0; cam < 2; ++cam) {
            steps.push_back(Step{cam, (k % 4 != 3), ts});  // mostly tag, occasional blank
            ts += 8.0;
        }
    }

    auto dir = std::filesystem::temp_directory_path() / "fitra_test_floor_replay";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "cam0");
    std::filesystem::create_directories(dir / "cam1");
    { std::ofstream meta{dir / "meta.json"}; meta << "{\n  \"version\": 1\n}\n"; }
    std::vector<std::uint64_t> seq{0, 0};
    std::ofstream jsonl{dir / "frames.jsonl"};
    for (const auto& s : steps) {
        const auto& jpg = s.tag ? jpg_tag : jpg_blank;
        char rel[64];
        std::snprintf(rel, sizeof(rel), "cam%zu/%06llu.jpg", s.cam,
                      static_cast<unsigned long long>(++seq[s.cam]));
        std::ofstream jf{dir / rel, std::ios::binary};
        jf.write(reinterpret_cast<const char*>(jpg.data()),
                 static_cast<std::streamsize>(jpg.size()));
        jsonl << jsonl_line(s.cam, seq[s.cam], rel, s.ts_ms) << "\n";
    }
    jsonl.close();

    // Reference: decode the stored JPEG bytes and call on_frame() directly.
    FloorCalibSession ref(make_config());
    ref.start();
    {
        std::vector<std::uint64_t> rseq{0, 0};
        for (const auto& s : steps) {
            char rel[64];
            std::snprintf(rel, sizeof(rel), "cam%zu/%06llu.jpg", s.cam,
                          static_cast<unsigned long long>(++rseq[s.cam]));
            std::ifstream f{dir / rel, std::ios::binary};
            std::vector<char> bytes{std::istreambuf_iterator<char>(f),
                                    std::istreambuf_iterator<char>()};
            CHECK(!bytes.empty());
            cv::Mat buf(1, static_cast<int>(bytes.size()), CV_8UC1, bytes.data());
            cv::Mat bgr = cv::imdecode(buf, cv::IMREAD_COLOR);
            CHECK(!bgr.empty());
            ref.on_frame(s.cam, bgr, s.ts_ms);
        }
    }

    // Replay: ExcalReplayInput must reproduce the identical accumulator state.
    FloorCalibSession rep(make_config());
    rep.start();
    ExcalReplayInput input{dir.string()};
    CHECK(input.size() == steps.size());
    ExcalInputItem item;
    std::size_t fed = 0;
    while (input.next(item)) {
        rep.on_frame(item.cam_idx, item.bgr, item.ctrl.ts_ms);
        ++fed;
    }
    CHECK(input.exhausted());
    CHECK(fed == steps.size());

    CHECK(ref.ready_group_count() > 0);  // not a vacuous pass
    CHECK(ref.ready_group_count() == rep.ready_group_count());
    CHECK(ref.state_json() == rep.state_json());

    std::filesystem::remove_all(dir);
}

}  // namespace

int main() {
    test_equivalence();
    if (g_fail) {
        std::fprintf(stderr, "test_floor_calib_replay: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_floor_calib_replay: OK\n");
    return 0;
}
