#include "camera/markerless_recording.hpp"
#include "lift/calib_io.hpp"
#include "util/sha256.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>
#include <sys/wait.h>
#include <unistd.h>
#include <opencv2/imgcodecs.hpp>

namespace {
namespace fs = std::filesystem;
using namespace fitra::camera;
void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
bool throws(const std::function<void()>& fn) {
    try { fn(); } catch (const std::exception&) { return true; }
    return false;
}
struct Temp {
    fs::path path;
    Temp() {
        char pattern[] = "/tmp/fitra-markerless-recording.XXXXXX";
        const auto* name = ::mkdtemp(pattern);
        check(name != nullptr, "mkdtemp");
        path = name;
    }
    ~Temp() { std::error_code ec; fs::remove_all(path, ec); }
};
int run(std::vector<std::string> args) {
    const auto child = ::fork();
    check(child >= 0, "fork");
    if (child == 0) {
        std::vector<char*> argv;
        for (auto& arg : args) argv.push_back(arg.data());
        argv.push_back(nullptr);
        ::execv(argv[0], argv.data());
        ::_exit(127);
    }
    int status = 0;
    check(::waitpid(child, &status, 0) == child, "waitpid");
    return WIFEXITED(status) ? WEXITSTATUS(status) : 128;
}

std::string read_all(const fs::path& path) {
    std::ifstream input(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

std::vector<MarkerlessRecordingCamera> specs() {
    std::vector<MarkerlessRecordingCamera> out;
    for (int i = 0; i < 2; ++i) {
        MarkerlessRecordingCamera camera;
        camera.id = "cam" + std::to_string(i);
        camera.config_slot = static_cast<std::size_t>(i);
        camera.capture.device_path = "/dev/synthetic-camera-" + std::to_string(i);
        camera.capture.width = 640;
        camera.capture.height = 480;
        out.push_back(camera);
    }
    return out;
}
void intrinsic(const fs::path& file) {
    fitra::lift::CalibrationSet calibration;
    calibration.schema = "fitra_calibration_v1";
    for (const auto& spec : specs()) {
        fitra::lift::CameraCalibration camera;
        camera.id = spec.id;
        camera.intrinsics.width = 640;
        camera.intrinsics.height = 480;
        camera.intrinsics.K = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
        camera.intrinsics.dist = cv::Mat::zeros(1, 5, CV_64F);
        calibration.cameras.push_back(camera);
    }
    fitra::lift::write_calibration(file.string(), calibration);
}
Frame frame(std::uint64_t seq, std::uint64_t stamp, const std::vector<std::uint8_t>& jpeg) {
    Frame out;
    out.seq = seq;
    out.v4l2_sequence = static_cast<std::uint32_t>(seq);
    out.data = jpeg;
    out.capture_width = 640;
    out.capture_height = 480;
    out.size_image = 640 * 480 * 2;
    out.pixel_format_fourcc = 0x47504a4d;
    out.frame_interval_numerator = 1;
    out.frame_interval_denominator = 2;
    out.captured_mono_ns = stamp + 100000;
    // Synthetic fixture, not a measurement of any camera timestamp behavior.
    out.v4l2_timestamp.mono_ns = stamp;
    out.v4l2_timestamp.semantics = V4l2TimestampSemantics::MonotonicSoe;
    return out;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        Temp temp;
        intrinsic(temp.path / "source.yaml");
        MarkerlessRecordingOptions options;
        options.out_dir = temp.path / "batch";
        options.intrinsics = temp.path / "source.yaml";
        options.reference = options.intrinsics;
        options.min_free_bytes = 0;
        options.height_m = 1.7;
        const auto before = fitra::util::sha256_file(options.intrinsics);
        std::vector<std::uint8_t> jpeg;
        cv::imencode(".jpg", cv::Mat(480, 640, CV_8UC3, cv::Scalar(80, 90, 100)), jpeg);
        const auto plan = markerless_capture_plan();
        int duration = 0, fit = 0, validation = 0, repeat = 0, floor = 0;
        MarkerlessRecording recording(options, specs());
        std::uint64_t start = 9007199254740993ULL;  // Beyond exact JSON-double range.
        std::uint64_t sequence = 0;
        for (std::size_t i = 0; i < plan.size(); ++i) {
            duration += plan[i].seconds;
            fit += plan[i].role == "fit";
            validation += plan[i].role == "validation";
            repeat += plan[i].role == "repeat";
            floor += plan[i].role == "floor_scale";
            recording.cue(i, start);
            for (int half = 0; half < plan[i].seconds * 2; ++half) {
                ++sequence;
                const auto time = start + 250000000ULL + static_cast<std::uint64_t>(half) * 500000000ULL;
                recording.append(0, frame(sequence, time, jpeg));
                recording.append(1, frame(sequence, time + 5000000, jpeg));
            }
            start += static_cast<std::uint64_t>(plan[i].seconds) * 1000000000ULL;
        }
        check(duration == 370 && fit == 12 && validation == 8 && repeat == 12 && floor == 4, "protocol budget/partition changed");
        recording.finish(true, "synthetic_protocol_fixture");
        check(fitra::util::sha256_file(options.intrinsics) == before, "source intrinsic was changed");
        check(fitra::util::sha256_file(options.out_dir / "intrinsics.yaml") == before, "intrinsic snapshot differs");
        check(throws([&] { MarkerlessRecording other(options, specs()); }), "existing output overwritten");
        check(throws([&] { recording.append(0, frame(1, 1, jpeg)); }), "append after close accepted");

        auto short_options = options;
        short_options.reference.clear();
        short_options.out_dir = temp.path / "partial";
        MarkerlessRecording partial(short_options, specs());
        partial.cue(0, 1000000000);
        auto one = frame(1, 1100000000, jpeg);
        partial.append(0, one);
        check(throws([&] { partial.append(0, one); }), "duplicate source accepted");
        one.seq = 2;
        one.captured_mono_ns += 1000000000;
        one.capture_width = 320;
        check(throws([&] { partial.append(0, one); }), "wrong accepted format ignored");
        partial.finish(false, "test_interruption");

        short_options.out_dir = temp.path / "capped";
        short_options.max_payload_bytes = jpeg.size() - 1;
        MarkerlessRecording capped(short_options, specs());
        check(throws([&] { capped.append(0, frame(1, 1000000000, jpeg)); }), "storage cap ignored");
        capped.finish(false, "test_capacity");

        short_options.out_dir = temp.path / "unknown-clock";
        short_options.max_payload_bytes = 1024 * 1024;
        MarkerlessRecording unknown(short_options, specs());
        auto bad_clock = frame(1, 1100000000, jpeg);
        bad_clock.v4l2_timestamp = {};
        unknown.append(0, bad_clock);
        unknown.finish(false, "test_unknown_clock");
        check(read_all(short_options.out_dir / "frames.jsonl").find("\"capture_mono_ns\":null") != std::string::npos,
              "unknown capture timestamp was replaced with host time");

        short_options.out_dir = temp.path / "bad-yuyv";
        auto yuyv_specs = specs();
        for (auto& spec : yuyv_specs) spec.capture.pixel_format = PixFmt::Yuyv;
        MarkerlessRecording short_yuyv(short_options, yuyv_specs);
        auto broken_payload = frame(1, 1100000000, {0, 1, 2, 3});
        broken_payload.pixel_format_fourcc = 0x56595559;
        broken_payload.bytes_per_line = 1280;
        short_yuyv.append(0, broken_payload);
        short_yuyv.finish(false, "test_truncated_yuyv");
        check(read_all(short_options.out_dir / "frames.jsonl").find("\"payload_shape_ok\":false") != std::string::npos,
              "truncated padded YUYV was accepted as complete pixels");
        if (argc == 4)
            check(run({argv[1], argv[2], (temp.path / "batch").string(), argv[3]}) == 0,
                  "offline QC/CLI integration tests failed");
        std::puts("markerless recording: passed (synthetic payload/timestamps, no camera access)");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "markerless recording failed: %s\n", error.what());
        return 1;
    }
}
