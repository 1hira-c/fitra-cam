// Live↔replay equivalence test for the calib-extrinsic offline replay path
// (docs/design/pose-3d-calib-mode-separation.md M4).
//
//  1) parse_excal_frame_line: accepts the exact tools/excal_record format,
//     rejects missing keys / broken lines.
//  2) Equivalence: synthesize AprilTag frames, JPEG-encode them into a
//     temporary recorder-format session (cam<N>/<seq>.jpg + frames.jsonl +
//     meta.json), then feed the SAME encoded bytes through (a) imdecode +
//     on_frame() directly and (b) ExcalReplayInput — the collected sample
//     streams must match element-wise. Both paths consume identical JPEG
//     bytes; comparing a raw Mat against a decoded one would test the codec,
//     not the replay plumbing.

#include "pipeline/excal_replay_input.hpp"
#include "pipeline/extrinsic_calib_session.hpp"
#include "lift/calib_io.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

using fitra::pipeline::ControllerObservation;
using fitra::pipeline::ExcalInputItem;
using fitra::pipeline::ExcalReplayInput;
using fitra::pipeline::ExcalReplayRecord;
using fitra::pipeline::ExtrinsicCalibConfig;
using fitra::pipeline::ExtrinsicCalibSession;
using fitra::pipeline::parse_excal_frame_line;

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

ExtrinsicCalibConfig make_config() {
    ExtrinsicCalibConfig cfg;
    cfg.intrinsics = make_intrinsics(2);
    cfg.board.faces.push_back(fitra::lift::MarkerFace{0, 0.10});
    cfg.lin_vel_max_mps = 1e9;     // permissive gate — equivalence, not gating
    cfg.ang_vel_max_dps = 1e9;
    cfg.burst_min = 3;
    cfg.burst_max = 1000;
    cfg.burst_gap_ms = 100.0;
    cfg.min_samples_per_group = 1;
    return cfg;
}

// One frames.jsonl line in the recorder's exact format.
std::string jsonl_line(std::size_t cam, std::uint64_t seq, const std::string& file,
                       double ts_ms, const ControllerObservation& c) {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "{\"cam\":%zu,\"seq\":%llu,\"file\":\"%s\","
        "\"ts_ms\":%.3f,\"ctrl\":{\"running_ok\":%s,"
        "\"x\":%.9g,\"y\":%.9g,\"z\":%.9g,"
        "\"qx\":%.9g,\"qy\":%.9g,\"qz\":%.9g,\"qw\":%.9g,"
        "\"stale\":false,\"age_ms\":8.0,\"tracking_result\":200,"
        "\"timestamp_s\":0.000000}}",
        cam, static_cast<unsigned long long>(seq), file.c_str(), ts_ms,
        c.running_ok ? "true" : "false", c.x, c.y, c.z, c.qx, c.qy, c.qz, c.qw);
    return std::string{buf};
}

void test_parse_excal_frame_line() {
    ControllerObservation c;
    c.running_ok = true;
    c.x = -0.5; c.y = 1.0; c.z = 2.25;
    c.qx = 0.1; c.qy = 0.2; c.qz = 0.3; c.qw = 0.9;
    const std::string ok_line = jsonl_line(1, 42, "cam1/000042.jpg", 345.625, c);

    ExcalReplayRecord rec;
    CHECK(parse_excal_frame_line(ok_line, rec));
    CHECK(rec.cam == 1);
    CHECK(rec.file == "cam1/000042.jpg");
    CHECK(rec.ctrl.running_ok);
    CHECK(rec.ctrl.x == -0.5 && rec.ctrl.y == 1.0 && rec.ctrl.z == 2.25);
    CHECK(rec.ctrl.qw == 0.9);
    CHECK(rec.ctrl.ts_ms == 345.625);

    // Missing required key (drop qw) → reject.
    auto no_qw = ok_line;
    auto p = no_qw.find(",\"qw\":0.9");
    CHECK(p != std::string::npos);
    no_qw.erase(p, std::string(",\"qw\":0.9").size());
    CHECK(!parse_excal_frame_line(no_qw, rec));

    // Truncated / garbage lines → reject.
    CHECK(!parse_excal_frame_line(ok_line.substr(0, ok_line.size() / 2), rec));
    CHECK(!parse_excal_frame_line("not json at all", rec));
    CHECK(!parse_excal_frame_line("{\"cam\":0}", rec));
    // Non-numeric value where a number is required → reject.
    auto bad_ts = ok_line;
    p = bad_ts.find("\"ts_ms\":345.625");
    bad_ts.replace(p, std::string("\"ts_ms\":345.625").size(), "\"ts_ms\":abc");
    CHECK(!parse_excal_frame_line(bad_ts, rec));
}

void test_live_replay_equivalence() {
    // Render one 36h11 marker frame and one blank frame (same technique as
    // test_extrinsic_calib_session's on_frame test).
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

    // Scenario: 3 controller poses, 6 frames per pose per camera, interleaved
    // cam0/cam1 like the recorder writes them, separated by >burst_gap_ms so
    // earlier bursts flush into samples. Poses 0 and 2 see the marker; pose 1
    // is blank (a no-detection stretch in the middle).
    struct Step { std::size_t cam; bool tag; ControllerObservation ctrl; };
    std::vector<Step> steps;
    double ts = 0.0;
    for (int pose = 0; pose < 3; ++pose) {
        ControllerObservation c;
        c.running_ok = true;
        c.x = 0.1 * pose; c.y = 1.0; c.z = 1.5 + 0.05 * pose;
        c.qw = 1.0;
        ts += 300.0;  // > burst_gap_ms → previous burst flushes on next append
        for (int k = 0; k < 6; ++k) {
            for (std::size_t cam = 0; cam < 2; ++cam) {
                Step s;
                s.cam = cam;
                s.tag = (pose != 1);
                s.ctrl = c;
                s.ctrl.ts_ms = ts;
                steps.push_back(s);
                ts += 8.0;
            }
        }
    }

    // Write the recorder-format session.
    auto dir = std::filesystem::temp_directory_path() / "fitra_test_excal_replay";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "cam0");
    std::filesystem::create_directories(dir / "cam1");
    {
        std::ofstream meta{dir / "meta.json"};
        meta << "{\n  \"version\": 1,\n  \"cameras\": []\n}\n";
    }
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
        jsonl << jsonl_line(s.cam, seq[s.cam], rel, s.ctrl.ts_ms, s.ctrl) << "\n";
    }
    jsonl.close();

    // Reference path: decode the SAME stored JPEG bytes and call on_frame()
    // directly, in jsonl order.
    ExtrinsicCalibSession ref(make_config());
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
            ref.on_frame(s.cam, bgr, s.ctrl);
        }
    }

    // Replay path: the loader must produce the identical sample stream.
    ExtrinsicCalibSession rep(make_config());
    rep.start();
    ExcalReplayInput input{dir.string()};
    CHECK(input.size() == steps.size());
    CHECK(input.camera_count() == 2);
    ExcalInputItem item;
    std::size_t fed = 0;
    while (input.next(item)) {
        rep.on_frame(item.cam_idx, item.bgr, item.ctrl);
        ++fed;
    }
    CHECK(input.exhausted());
    CHECK(fed == steps.size());

    // The scenario must actually produce samples — an empty-vs-empty
    // comparison would pass vacuously.
    auto ref_samples = ref.samples_snapshot();
    auto rep_samples = rep.samples_snapshot();
    CHECK(ref_samples.size() > 0);
    CHECK(ref_samples.size() == rep_samples.size());
    for (std::size_t i = 0; i < std::min(ref_samples.size(), rep_samples.size()); ++i) {
        CHECK(ref_samples[i].cam_index == rep_samples[i].cam_index);
        CHECK(ref_samples[i].face_id == rep_samples[i].face_id);
        const auto& a1 = ref_samples[i].T_cam_marker.raw();
        const auto& a2 = rep_samples[i].T_cam_marker.raw();
        const auto& b1 = ref_samples[i].T_world_controller.raw();
        const auto& b2 = rep_samples[i].T_world_controller.raw();
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                CHECK(a1(r, c) == a2(r, c));   // identical bytes + identical
                CHECK(b1(r, c) == b2(r, c));   // float ops → bit-exact
            }
        }
    }
    CHECK(ref.state_json() == rep.state_json());

    std::filesystem::remove_all(dir);
}

void test_replay_loader_rejects_bad_sessions() {
    auto dir = std::filesystem::temp_directory_path() / "fitra_test_excal_replay_bad";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);

    // Missing meta.json.
    bool threw = false;
    try { ExcalReplayInput input{dir.string()}; }
    catch (const std::exception&) { threw = true; }
    CHECK(threw);

    // Unsupported version.
    { std::ofstream meta{dir / "meta.json"}; meta << "{\"version\": 2}\n"; }
    { std::ofstream jsonl{dir / "frames.jsonl"}; }
    threw = false;
    try { ExcalReplayInput input{dir.string()}; }
    catch (const std::exception&) { threw = true; }
    CHECK(threw);

    // Broken line is rejected with its line number.
    { std::ofstream meta{dir / "meta.json"}; meta << "{\"version\": 1}\n"; }
    { std::ofstream jsonl{dir / "frames.jsonl"}; jsonl << "{\"cam\":0}\n"; }
    threw = false;
    try { ExcalReplayInput input{dir.string()}; }
    catch (const std::exception& e) {
        threw = true;
        CHECK(std::string(e.what()).find("line 1") != std::string::npos);
    }
    CHECK(threw);

    std::filesystem::remove_all(dir);
}

}  // namespace

int main() {
    test_parse_excal_frame_line();
    test_live_replay_equivalence();
    test_replay_loader_rejects_bad_sessions();
    if (g_fail) {
        std::fprintf(stderr, "test_excal_replay: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_excal_replay: OK\n");
    return 0;
}
