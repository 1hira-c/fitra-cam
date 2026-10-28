// Deterministic raw-observation replay and safe diagnostic-output regression.
#include <cmath>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <crow/json.h>

#include "lift/markerless_io.hpp"
#include "markerless_test_scene.hpp"

namespace {
using namespace fitra::lift;
int failures = 0;
#define CHECK(test) do { if (!(test)) { \
    std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #test); ++failures; \
} } while (0)

bool throws(const std::function<void()>& operation) {
    try { operation(); }
    catch (const std::exception&) { return true; }
    return false;
}

std::string observations(const std::vector<MarkerlessFrameSet>& samples) {
    std::ostringstream output;
    write_markerless_observations(output, samples);
    return output.str();
}

MarkerlessReplay read(const std::string& source, const CalibrationSet& intrinsics,
                      MarkerlessCollectionOptions options = {}) {
    std::istringstream input{source};
    return read_markerless_observations(input, intrinsics, options);
}

std::string replace_once(std::string text, const std::string& from, const std::string& to) {
    const auto pos = text.find(from);
    if (pos == std::string::npos) throw std::runtime_error("missing replacement token: " + from);
    text.replace(pos, from.size(), to);
    return text;
}

struct TempDirectory {
    std::filesystem::path path;
    TempDirectory() {
        char pattern[] = "/tmp/fitra-markerless-replay-test.XXXXXX";
        const char* name = ::mkdtemp(pattern);
        if (!name) throw std::runtime_error("cannot make temporary test directory");
        path = name;
    }
    ~TempDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }
};

std::string read_file(const std::filesystem::path& path) {
    std::ifstream input{path};
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

int run(const std::vector<std::string>& arguments) {
    // Build argv before fork; the child only performs async-signal-safe calls.
    std::vector<char*> argv;
    for (const auto& argument : arguments) argv.push_back(const_cast<char*>(argument.c_str()));
    argv.push_back(nullptr);
    const pid_t pid = ::fork();
    if (pid < 0) throw std::runtime_error("fork failed");
    if (pid == 0) {
        const int sink = ::open("/dev/null", O_WRONLY);
        if (sink >= 0) {
            ::dup2(sink, STDOUT_FILENO);
            ::dup2(sink, STDERR_FILENO);
            ::close(sink);
        }
        ::execv(argv[0], argv.data());
        ::_exit(127);
    }
    int status = 0;
    while (::waitpid(pid, &status, 0) < 0) {
        if (errno != EINTR) throw std::runtime_error("waitpid failed");
    }
    return WIFEXITED(status) ? WEXITSTATUS(status) : 128;
}

void check_roundtrip() {
    auto scene = markerless_test::make_scene();
    // IDs greater than 2^53 and timestamps near int64's upper end must not pass
    // through floating point at the text boundary.
    for (auto& sample : scene.samples) {
        sample.sample_id += std::numeric_limits<std::uint64_t>::max() - 100;
        for (auto& frame : sample.frames) {
            frame.source_seq += std::numeric_limits<std::uint64_t>::max() - 100;
            frame.stream_epoch = std::numeric_limits<std::uint64_t>::max();
            frame.capture_mono_ns += std::numeric_limits<std::int64_t>::max() - 100000000000LL;
        }
    }
    const auto replay = read(observations(scene.samples), scene.intrinsics);
    CHECK(replay.input_clean());
    CHECK(replay.records == scene.samples.size());
    CHECK(replay.collection.accepted == scene.samples.size());
    CHECK(replay.frame_sets.size() == scene.samples.size());
    for (std::size_t i = 0; i < replay.frame_sets.size(); ++i) {
        const auto& actual = replay.frame_sets[i];
        const auto& expected = scene.samples[i];
        CHECK(actual.sample_id == expected.sample_id);
        CHECK(actual.continuity_id == expected.continuity_id);
        CHECK(actual.frames.size() == expected.frames.size());
        for (std::size_t j = 0; j < actual.frames.size(); ++j) {
            const auto& a = actual.frames[j];
            const auto& e = expected.frames[j];
            CHECK(a.camera_id == e.camera_id);
            CHECK(a.source_seq == e.source_seq);
            CHECK(a.stream_epoch == e.stream_epoch);
            CHECK(a.capture_mono_ns == e.capture_mono_ns);
            CHECK(a.timestamp_kind == e.timestamp_kind);
            CHECK(a.width == e.width && a.height == e.height && a.person_count == e.person_count);
            CHECK(a.keypoints.size() == e.keypoints.size());
            for (std::size_t k = 0; k < a.keypoints.size(); ++k) {
                CHECK(a.keypoints[k].joint_index == e.keypoints[k].joint_index);
                CHECK(a.keypoints[k].pixel == e.keypoints[k].pixel);
                CHECK(a.keypoints[k].score == e.keypoints[k].score);
            }
        }
    }
}

void check_geometry_equivalence() {
    for (bool fish : {false, true}) {
        markerless_test::SceneOptions options;
        options.fisheye = fish;
        const auto scene = markerless_test::make_scene(options);
        const auto replay = read(observations(scene.samples), scene.intrinsics);
        const auto direct = solve_markerless_relative(scene.intrinsics, scene.samples);
        const auto restored = solve_markerless_relative(scene.intrinsics, replay.frame_sets);
        CHECK(direct.ok);
        CHECK(restored.ok == direct.ok);
        CHECK(restored.cameras.size() == direct.cameras.size());
        for (std::size_t i = 0; i < direct.cameras.size() && i < restored.cameras.size(); ++i)
            CHECK(cv::norm(cv::Mat(direct.cameras[i].T_camera_reference),
                           cv::Mat(restored.cameras[i].T_camera_reference)) < 1.0e-7);
        const auto report = crow::json::load(markerless_relative_report_json(restored, replay));
        CHECK(report && report["ok"].b());
        CHECK(report["schema"].s() == "fitra_markerless_relative_v1");
        CHECK(report["unit"].s() == "arbitrary_baseline");
        CHECK(report["coordinate_system"].s() == "reference_camera");
        CHECK(report["diagnostic_only"].b());
        CHECK(!report["metric_scale_known"].b());
        CHECK(!report["floor_aligned"].b());
        CHECK(!report["runtime_ready"].b());
        CHECK(report["cameras"].size() == scene.intrinsics.cameras.size());
        CHECK(report["cameras"][0]["T_camera_reference"].size() == 4);
    }
}

void check_bad_json() {
    const auto scene = markerless_test::make_scene();
    const auto source = observations(scene.samples);
    for (const auto& replacement : std::vector<std::pair<std::string, std::string>>{
        {"fitra_markerless_observations_v1", "fitra_calibration_v1"},
        {"halpe26", "coco17"},
        {"\"sample_id\":1", "\"sample_id\":1.0"},
        {"\"sample_id\":1", "\"sample_id\":1e0"},
        {"\"sample_id\":1", "\"sample_id\":01"},
        {"\"sample_id\":1", "\"sample_id\":1."},
        {"\"sample_id\":1", "\"sample_id\":\"1\""},
        {"\"sample_id\":1", "\"sample_id\":-1"},
        {"\"sample_id\":1", "\"sample_id\":18446744073709551616"},
        {"\"source_seq\":1", "\"source_seq\":true"},
        {"\"stream_epoch\":1", "\"stream_epoch\":-1"},
        {"\"capture_mono_ns\":", "\"missing_timestamp\":"},
        {"\"capture_mono_ns\":", "\"capture_mono_ns\":9223372036854775808,\"unused\":"},
        {"\"timestamp_kind\":\"synthetic_monotonic\"", "\"timestamp_kind\":\"unknown\""},
        {"\"person_count\":1", "\"person_count\":1.5"},
        {"\"width\":1280", "\"width\":2147483648"},
        {"\"sample_id\":1", "\"sample_id\":1,\"sample_id\":2"},
    }) {
        const auto candidate = replace_once(source, replacement.first, replacement.second);
        CHECK(throws([&] { read(candidate, scene.intrinsics); }));
    }
    CHECK(throws([&] { read("", scene.intrinsics); }));
    CHECK(throws([&] { read("{}\n", scene.intrinsics); }));
    CHECK(throws([&] { read(source + "\n", scene.intrinsics); }));
    CHECK(throws([&] { read(source + "not JSON\n", scene.intrinsics); }));
    const auto header_end = source.find('\n');
    const auto record_end = source.find('\n', header_end + 1);
    for (const auto position : {header_end, record_end}) {
        auto hidden_suffix = source;
        hidden_suffix.insert(position, std::string(1, '\0') + "ignored trailing bytes");
        CHECK(throws([&] { read(hidden_suffix, scene.intrinsics); }));
    }
    CHECK(throws([&] { read(source + std::string(markerless_max_jsonl_line_bytes + 1, ' '), scene.intrinsics); }));
    CHECK(throws([&] { read(std::string(20, '[') + std::string(20, ']'), scene.intrinsics); }));
    auto invalid_points = scene.samples;
    invalid_points[0].frames[0].keypoints[0].score = std::numeric_limits<double>::infinity();
    CHECK(throws([&] { observations(invalid_points); }));
}

void check_collection_status() {
    const auto scene = markerless_test::make_scene();
    MarkerlessCollectionOptions options;
    options.max_frame_sets = 8;
    const auto bounded = read(observations(scene.samples), scene.intrinsics, options);
    CHECK(bounded.input_clean());
    CHECK(bounded.frame_sets.size() == 8);
    CHECK(bounded.collection.evicted == scene.samples.size() - 8);
    CHECK(bounded.frame_sets.front().sample_id == scene.samples[scene.samples.size() - 8].sample_id);

    auto samples = scene.samples;
    samples.insert(samples.begin() + 1, samples.front());
    const auto duplicated = read(observations(samples), scene.intrinsics);
    CHECK(!duplicated.input_clean());
    CHECK(duplicated.rejected_records > 0);
    CHECK(!duplicated.diagnostics.empty());
    MarkerlessRelativeResult fake_success;
    fake_success.ok = true;
    const auto rejected_report = crow::json::load(markerless_relative_report_json(fake_success, duplicated));
    CHECK(rejected_report["geometry_ok"].b());
    CHECK(!rejected_report["ok"].b());
    CHECK(!rejected_report["input_clean"].b());
    CHECK(!rejected_report["runtime_ready"].b());

    samples = scene.samples;
    for (std::size_t i = 10; i < samples.size(); ++i) samples[i].continuity_id = "new-subject";
    const auto reset = read(observations(samples), scene.intrinsics);
    CHECK(!reset.input_clean());
    CHECK(reset.collection.resets > 0);
    for (const auto& sample : reset.frame_sets) CHECK(sample.continuity_id == "new-subject");

    samples = scene.samples;
    samples[0].frames[0].capture_mono_ns += 50000000;
    const auto skewed = read(observations(samples), scene.intrinsics);
    CHECK(!skewed.input_clean());
    CHECK(skewed.rejected_records > 0);
}

void check_new_only_report() {
    TempDirectory temp;
    const auto file = temp.path / "report.json";
    write_markerless_report_new(file.string(), "{\"diagnostic_only\":true}");
    const auto original = read_file(file);
    CHECK(throws([&] { write_markerless_report_new(file.string(), "overwrite"); }));
    CHECK(read_file(file) == original);
    const auto link = temp.path / "pointer.json";
    std::filesystem::create_symlink(file.filename(), link);
    CHECK(throws([&] { write_markerless_report_new(link.string(), "overwrite"); }));
    CHECK(read_file(file) == original);
    const auto dangling = temp.path / "dangling.json";
    std::filesystem::create_symlink("missing.json", dangling);
    CHECK(throws([&] { write_markerless_report_new(dangling.string(), "overwrite"); }));
    CHECK(!std::filesystem::exists(temp.path / "missing.json"));
    CHECK(throws([&] { write_markerless_report_new((temp.path / "latest.yaml").string(), "x"); }));
    CHECK(!std::filesystem::exists(temp.path / "latest.yaml"));
}

void check_cli(const std::string& executable) {
    const auto scene = markerless_test::make_scene();
    TempDirectory temp;
    const auto intrinsics = temp.path / "intrinsics.yaml";
    const auto input = temp.path / "observations.jsonl";
    const auto output = temp.path / "geometry.json";
    write_calibration(intrinsics.string(), scene.intrinsics);
    {
        std::ofstream stream{input};
        write_markerless_observations(stream, scene.samples);
    }
    CHECK(run({executable, "--help"}) == 0);
    CHECK(run({executable}) != 0);
    CHECK(run({executable, "--unknown", "value"}) != 0);
    CHECK(run({executable, "--intrinsics"}) != 0);
    std::vector<std::string> arguments{executable, "--intrinsics", intrinsics.string(),
        "--observations", input.string(), "--output", output.string()};
    CHECK(run(arguments) == 0);
    CHECK(std::filesystem::exists(output));
    const auto bytes = read_file(output);
    const auto report = crow::json::load(bytes);
    CHECK(report && report["ok"].b());
    CHECK(!report["runtime_ready"].b());
    CHECK(run(arguments) != 0);
    CHECK(read_file(output) == bytes);
    const auto original = read_file(input);
    arguments.back() = input.string();
    CHECK(run(arguments) != 0);
    CHECK(read_file(input) == original);
    arguments.back() = intrinsics.string();
    const auto original_intrinsics = read_file(intrinsics);
    CHECK(run(arguments) != 0);
    CHECK(read_file(intrinsics) == original_intrinsics);
    auto invalid = scene.samples;
    invalid.insert(invalid.begin() + 1, invalid.front());
    {
        std::ofstream stream{input};
        write_markerless_observations(stream, invalid);
    }
    arguments.back() = (temp.path / "rejected.json").string();
    CHECK(run(arguments) != 0);
    const auto rejected = crow::json::load(read_file(arguments.back()));
    CHECK(rejected && !rejected["ok"].b());
    CHECK(!rejected["input_clean"].b());
}

}  // namespace

int main(int argc, char** argv) {
    try {
        check_roundtrip();
        check_geometry_equivalence();
        check_bad_json();
        check_collection_status();
        check_new_only_report();
        if (argc > 1) check_cli(argv[1]);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "UNEXPECTED: %s\n", e.what());
        ++failures;
    }
    if (failures) std::fprintf(stderr, "markerless replay: %d failures\n", failures);
    return failures ? 1 : 0;
}
