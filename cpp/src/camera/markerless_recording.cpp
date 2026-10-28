#include "camera/markerless_recording.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>

#include <crow/json.h>

#include "lift/calib_io.hpp"
#include "util/sha256.hpp"

namespace fitra::camera {
namespace {
using Json = crow::json::wvalue;
namespace fs = std::filesystem;
constexpr std::uint32_t mjpg = 0x47504a4dU;
constexpr std::uint32_t yuyv = 0x56595559U;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

const char* semantics(V4l2TimestampSemantics kind) {
    if (kind == V4l2TimestampSemantics::MonotonicSoe) return "monotonic_soe";
    if (kind == V4l2TimestampSemantics::MonotonicEof) return "monotonic_eof";
    return "unavailable";
}

void write_json(const fs::path& path, const Json& json) {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    output << json.dump() << '\n';
    output.close();
    require(bool(output), "cannot write recording metadata");
}

Json plan_json(const std::vector<MarkerlessCue>& plan) {
    Json out;
    out["schema"] = "fitra_markerless_capture_plan_v1";
    out["duration_s"] = 0;
    out["duration_is_provisional"] = true;
    out["steps"] = Json::list{};
    int elapsed = 0;
    for (std::size_t i = 0; i < plan.size(); ++i) {
        const auto& cue = plan[i];
        auto& entry = out["steps"][i];
        entry["id"] = cue.id;
        entry["take"] = cue.take;
        entry["role"] = cue.role;
        entry["prompt_ja"] = cue.prompt;
        entry["start_s"] = elapsed;
        entry["duration_s"] = cue.seconds;
        entry["settle_s"] = cue.settle_seconds;
        elapsed += cue.seconds;
    }
    out["duration_s"] = elapsed;
    return out;
}
}  // namespace

std::vector<MarkerlessCue> markerless_capture_plan() {
    std::vector<MarkerlessCue> out{
        {"warmup", "setup", "setup", "画角と明るさを確認。全カメラに頭頂と足元が映る場所へ。", 10, 0},
        {"sync_start", "setup", "sync", "片腕を上げて止め、間を変えて反対の腕も上げる。素早い反復は不要。", 10, 0},
    };
    const std::vector<std::string> positions{
        "中央", "中央から半歩左", "中央から半歩右", "中央より半歩手前", "中央より半歩奥", "中央から斜めに半歩"};
    auto add_pose_take = [&](const std::string& take, const std::string& role,
                              int count, bool reverse) {
        for (int i = 0; i < count; ++i) {
            const int p = reverse ? 5 - (i / 2) : i / 2;
            const std::string pose = (i % 2 == 0)
                ? "正面で腕を体から離し、片腕を少し前へ出して静止。"
                : "体を斜め45度ほど向け、左右の腕の高さを変えて静止。";
            out.push_back({take + "_" + std::to_string(i + 1), take, role,
                positions.at(static_cast<std::size_t>(p)) + "へ無理なく移動。" + pose,
                8, 3});
        }
    };
    add_pose_take("calibration_a", "fit", 12, false);
    // Separate time/pose blocks, never adjacent frames from the fitting take.
    for (int i = 0; i < 8; ++i) {
        out.push_back({"validation_b_" + std::to_string(i + 1), "validation_b", "validation",
            positions[static_cast<std::size_t>((i * 5 + 2) % 6)]
            + "の少し違う位置へ。前の撮影と腕の配置・向きを変えて静止。", 8, 3});
    }
    add_pose_take("repeat_c", "repeat", 12, true);
    for (int i = 0; i < 4; ++i) {
        out.push_back({"floor_scale_" + std::to_string(i + 1), "floor_scale", "floor_scale",
            positions[static_cast<std::size_t>(i)]
            + "で背筋を伸ばし、両足を床に置いて静止。頭頂とつま先・踵を隠さない。", 10, 3});
    }
    const char* motions[] = {"共通画角内をゆっくり斜めに歩き、止まる。",
        "ゆっくり横移動して止まる。", "左右へ向きを変えて止まる。",
        "片腕ずつゆっくり動かし、最後に静止。"};
    for (int i = 0; i < 4; ++i)
        out.push_back({"motion_" + std::to_string(i + 1), "motion", "motion", motions[i], 10, 0});
    out.push_back({"sync_end", "end", "sync", "開始時と同じく片腕ずつ、間隔を変えて上げて止める。", 10, 0});
    out.push_back({"empty_end", "end", "empty", "撮影終了。画角から退出する。", 4, 0});
    return out;
}

std::string markerless_capture_plan_json() { return plan_json(markerless_capture_plan()).dump(); }

class MarkerlessRecording::Impl {
public:
    struct Stats {
        std::ofstream payload;
        std::uint64_t frames = 0, bytes = 0, source_gaps = 0, driver_gaps = 0;
        std::uint64_t unknown_time = 0, time_regressions = 0, incomplete_jpeg = 0, invalid_payload = 0;
        std::uint64_t last_source = 0, last_host = 0, last_kernel = 0;
        std::uint32_t last_driver = 0;
        Frame format;
        std::string file;
    };

    Impl(MarkerlessRecordingOptions opts, std::vector<MarkerlessRecordingCamera> specs)
        : options(std::move(opts)), cameras(std::move(specs)), plan(markerless_capture_plan()) {
        require(cameras.size() >= 2 && cameras.size() <= 3, "recording requires 2..3 cameras");
        require(options.max_payload_bytes > 0, "recording byte cap must be positive");
        require(std::isfinite(options.height_m) && options.height_m >= 0.0
                && options.height_m <= 2.5, "invalid measured height");
        std::set<std::string> ids, devices;
        intrinsic_origin = fs::weakly_canonical(options.intrinsics).string();
        for (const auto& camera : cameras) {
            require(!camera.id.empty() && ids.insert(camera.id).second, "duplicate camera id");
            require(!camera.capture.device_path.empty()
                    && devices.insert(fs::weakly_canonical(camera.capture.device_path).string()).second,
                    "duplicate or missing camera device");
            device_origins.push_back(fs::weakly_canonical(camera.capture.device_path).string());
        }
        fs::create_directories(options.out_dir.parent_path().empty() ? fs::path(".") : options.out_dir.parent_path());
        require(fs::create_directory(options.out_dir), "output directory must not exist");
        fs::permissions(options.out_dir, fs::perms::owner_all, fs::perm_options::replace);
        fs::copy_file(options.intrinsics, options.out_dir / "intrinsics.yaml");
        const auto intrinsic = lift::load_calibration((options.out_dir / "intrinsics.yaml").string());
        for (const auto& camera : cameras) {
            const auto found = std::find_if(intrinsic.cameras.begin(), intrinsic.cameras.end(),
                [&](const lift::CameraCalibration& c) { return c.id == camera.id; });
            require(found != intrinsic.cameras.end(), "intrinsic camera id is missing");
            require(found->intrinsics.width == camera.capture.width
                    && found->intrinsics.height == camera.capture.height,
                    "intrinsic dimensions must match configured inference output dimensions");
        }
        if (!options.reference.empty())
            fs::copy_file(options.reference, options.out_dir / "reference.yaml");
        write_json(options.out_dir / "plan.json", plan_json(plan));
        frames.open(options.out_dir / "frames.jsonl", std::ios::binary);
        cues.open(options.out_dir / "cues.jsonl", std::ios::binary);
        require(bool(frames) && bool(cues), "cannot create recording sidecars");
        for (std::size_t i = 0; i < cameras.size(); ++i) {
            stats.emplace_back();
            stats.back().file = "cam" + std::to_string(i)
                + (cameras[i].capture.pixel_format == PixFmt::Yuyv ? ".yuyv" : ".mjpeg");
            stats.back().payload.open(options.out_dir / stats.back().file, std::ios::binary);
            require(bool(stats.back().payload), "cannot create camera payload");
        }
        manifest(false, "recording_open");
    }

    void manifest(bool complete, const std::string& reason) {
        Json out;
        out["schema"] = "fitra_markerless_capture_v1";
        out["recorder_binary_sha256"] = options.recorder_binary_sha256.empty()
            ? Json(nullptr) : Json(options.recorder_binary_sha256);
        out["recording_complete"] = complete;
        out["reason"] = reason;
        out["calibration_ready"] = false;
        out["person_visibility_checked"] = false;
        out["geometry_checked"] = false;
        out["height_m"] = options.height_m > 0.0 ? Json(options.height_m) : Json(nullptr);
        out["reference_is_ground_truth"] = false;
        out["intrinsics_file"] = "intrinsics.yaml";
        out["intrinsics_source"] = options.intrinsics.string();
        out["intrinsics_resolved_source"] = intrinsic_origin;
        out["reference_file"] = options.reference.empty() ? Json(nullptr) : Json("reference.yaml");
        out["reference_source"] = options.reference.empty() ? Json(nullptr) : Json(options.reference.string());
        out["cue_count"] = cue_times.size();
        out["planned_cue_count"] = plan.size();
        out["max_payload_bytes"] = std::to_string(options.max_payload_bytes);
        out["payload_bytes"] = std::to_string(total_bytes);
        out["min_free_bytes"] = std::to_string(options.min_free_bytes);
        out["cameras"] = Json::list{};
        for (std::size_t i = 0; i < cameras.size(); ++i) {
            const auto& camera = cameras[i];
            const auto& s = stats[i];
            auto& entry = out["cameras"][i];
            entry["id"] = camera.id;
            entry["config_slot"] = camera.config_slot;
            entry["device"] = camera.capture.device_path;
            entry["resolved_device"] = device_origins[i];
            entry["payload"] = s.file;
            entry["output_width"] = camera.capture.width;
            entry["output_height"] = camera.capture.height;
            entry["runtime_transform"] = "full_frame_resize_to_output_dimensions";
            entry["fov_match_verified"] = false;
            entry["requested_width"] = camera.capture.capture_w();
            entry["requested_height"] = camera.capture.capture_h();
            entry["requested_fps"] = camera.capture.fps;
            entry["requested_exposure_mode"] = camera.capture.exposure_mode == V4l2Options::ExposureMode::Auto
                ? "auto" : "initial_manual_without_assist_loop";
            entry["applied_exposure_verified"] = false;
            entry["requested_exposure_us100"] = camera.capture.exposure_us100;
            entry["requested_gain"] = camera.capture.gain;
            entry["frames"] = std::to_string(s.frames);
            entry["source_gaps"] = std::to_string(s.source_gaps);
            entry["driver_gaps"] = std::to_string(s.driver_gaps);
            entry["unknown_timestamps"] = std::to_string(s.unknown_time);
            entry["timestamp_regressions"] = std::to_string(s.time_regressions);
            entry["incomplete_jpeg_markers"] = std::to_string(s.incomplete_jpeg);
            entry["invalid_payload_shapes"] = std::to_string(s.invalid_payload);
            entry["actual_width"] = s.format.capture_width;
            entry["actual_height"] = s.format.capture_height;
            entry["actual_fourcc"] = s.format.pixel_format_fourcc;
            entry["actual_bytes_per_line"] = s.format.bytes_per_line;
            entry["actual_size_image"] = s.format.size_image;
            entry["actual_interval_numerator"] = s.format.frame_interval_numerator;
            entry["actual_interval_denominator"] = s.format.frame_interval_denominator;
        }
        write_json(options.out_dir / "manifest.tmp", out);
        fs::rename(options.out_dir / "manifest.tmp", options.out_dir / "manifest.json");
    }

    MarkerlessRecordingOptions options;
    std::vector<MarkerlessRecordingCamera> cameras;
    std::vector<MarkerlessCue> plan;
    std::vector<Stats> stats;
    std::ofstream frames, cues;
    std::vector<std::uint64_t> cue_times;
    std::string intrinsic_origin;
    std::vector<std::string> device_origins;
    std::uint64_t total_bytes = 0;
    bool closed = false;
};

MarkerlessRecording::MarkerlessRecording(MarkerlessRecordingOptions options,
    std::vector<MarkerlessRecordingCamera> cameras)
    : impl_(std::make_unique<Impl>(std::move(options), std::move(cameras))) {}

MarkerlessRecording::~MarkerlessRecording() {
    if (impl_ && !impl_->closed) {
        try { finish(false, "recording_not_finalized"); } catch (...) {}
    }
}

void MarkerlessRecording::cue(std::size_t index, std::uint64_t monotonic_ns) {
    auto& in = *impl_;
    require(!in.closed && index == in.cue_times.size() && index < in.plan.size(), "invalid cue order");
    require(monotonic_ns > 0 && (in.cue_times.empty() || monotonic_ns > in.cue_times.back()), "cue clock regression");
    const auto& cue = in.plan[index];
    Json out;
    out["cue_id"] = cue.id;
    out["take"] = cue.take;
    out["role"] = cue.role;
    out["start_mono_ns"] = std::to_string(monotonic_ns);
    out["duration_s"] = cue.seconds;
    out["settle_s"] = cue.settle_seconds;
    in.cues << out.dump() << '\n';
    in.cues.flush();
    require(bool(in.cues), "cannot write cue sidecar");
    in.cue_times.push_back(monotonic_ns);
}

void MarkerlessRecording::append(std::size_t index, const Frame& frame) {
    auto& in = *impl_;
    require(!in.closed && index < in.stats.size(), "invalid recording camera/state");
    auto& s = in.stats[index];
    const auto& requested = in.cameras[index].capture;
    require(!frame.data.empty() && frame.captured_mono_ns > 0, "empty payload or missing host timestamp");
    require(frame.capture_width == static_cast<std::uint32_t>(requested.capture_w())
            && frame.capture_height == static_cast<std::uint32_t>(requested.capture_h()),
            "camera negotiated a different capture size");
    require(frame.pixel_format_fourcc == (requested.pixel_format == PixFmt::Yuyv ? yuyv : mjpg),
            "camera negotiated a different encoding");
    require(s.frames == 0 || (frame.seq > s.last_source && frame.captured_mono_ns > s.last_host),
            "capture source duplicated or regressed");
    if (s.frames) {
        require(frame.bytes_per_line == s.format.bytes_per_line
                && frame.size_image == s.format.size_image
                && frame.frame_interval_numerator == s.format.frame_interval_numerator
                && frame.frame_interval_denominator == s.format.frame_interval_denominator,
                "camera format changed during recording");
        s.source_gaps += frame.seq - s.last_source - 1;
        const std::uint32_t delta = frame.v4l2_sequence - s.last_driver;
        require(delta > 0 && delta < 0x80000000U, "driver sequence regressed");
        s.driver_gaps += delta - 1;
    }
    const auto bytes = frame.data.size();
    require(bytes <= in.options.max_payload_bytes - in.total_bytes, "recording payload cap reached");
    const auto space = fs::space(in.options.out_dir);
    require(space.available >= in.options.min_free_bytes
            && bytes <= space.available - in.options.min_free_bytes, "recording free-space reserve reached");
    const auto& timestamp = frame.v4l2_timestamp;
    const bool known = timestamp.mono_ns && *timestamp.mono_ns > 0
        && (timestamp.semantics == V4l2TimestampSemantics::MonotonicSoe
            || timestamp.semantics == V4l2TimestampSemantics::MonotonicEof);
    if (!known) ++s.unknown_time;
    if (known && s.last_kernel && *timestamp.mono_ns <= s.last_kernel) ++s.time_regressions;
    bool jpeg_ok = true;
    if (frame.pixel_format_fourcc == mjpg) {
        jpeg_ok = bytes >= 4 && frame.data[0] == 0xff && frame.data[1] == 0xd8
            && frame.data[bytes - 2] == 0xff && frame.data[bytes - 1] == 0xd9;
        if (!jpeg_ok) ++s.incomplete_jpeg;
    }
    const bool payload_ok = frame.pixel_format_fourcc == mjpg ? jpeg_ok
        : frame.bytes_per_line >= frame.capture_width * 2ULL
          && static_cast<std::uint64_t>(frame.bytes_per_line) * frame.capture_height <= bytes;
    if (!payload_ok) ++s.invalid_payload;
    Json row;
    row["camera_id"] = in.cameras[index].id;
    row["source_seq"] = std::to_string(frame.seq);
    row["v4l2_sequence"] = frame.v4l2_sequence;
    row["payload"] = s.file;
    row["offset"] = std::to_string(s.bytes);
    row["size"] = bytes;
    row["host_capture_mono_ns"] = std::to_string(frame.captured_mono_ns);
    row["capture_mono_ns"] = known ? Json(std::to_string(*timestamp.mono_ns)) : Json(nullptr);
    row["timestamp_semantics"] = semantics(timestamp.semantics);
    row["jpeg_markers_ok"] = jpeg_ok;
    row["payload_shape_ok"] = payload_ok;
    const auto cue = std::upper_bound(in.cue_times.begin(), in.cue_times.end(), frame.captured_mono_ns);
    row["cue_id"] = cue == in.cue_times.begin() ? Json(nullptr)
        : Json(in.plan[static_cast<std::size_t>(cue - in.cue_times.begin() - 1)].id);
    s.payload.write(reinterpret_cast<const char*>(frame.data.data()), static_cast<std::streamsize>(bytes));
    in.frames << row.dump() << '\n';
    require(bool(s.payload) && bool(in.frames), "cannot write raw capture or frame index");
    s.bytes += bytes;
    in.total_bytes += bytes;
    ++s.frames;
    s.last_source = frame.seq;
    s.last_host = frame.captured_mono_ns;
    s.last_driver = frame.v4l2_sequence;
    if (known) s.last_kernel = *timestamp.mono_ns;
    if (s.frames == 1) {
        s.format.capture_width = frame.capture_width;
        s.format.capture_height = frame.capture_height;
        s.format.pixel_format_fourcc = frame.pixel_format_fourcc;
        s.format.bytes_per_line = frame.bytes_per_line;
        s.format.size_image = frame.size_image;
        s.format.frame_interval_numerator = frame.frame_interval_numerator;
        s.format.frame_interval_denominator = frame.frame_interval_denominator;
    }
}

void MarkerlessRecording::finish(bool complete, const std::string& reason) {
    auto& in = *impl_;
    if (in.closed) return;
    in.frames.close();
    in.cues.close();
    require(bool(in.frames) && bool(in.cues), "cannot close capture sidecars");
    for (auto& stats : in.stats) {
        stats.payload.close();
        require(bool(stats.payload), "cannot close camera payload");
        complete = complete && stats.frames > 0;
    }
    complete = complete && in.cue_times.size() == in.plan.size();
    in.manifest(complete, reason);
    std::ofstream checksums(in.options.out_dir / "checksums.sha256");
    for (const auto& entry : fs::directory_iterator(in.options.out_dir)) {
        if (entry.path().filename() == "checksums.sha256") continue;
        checksums << util::sha256_file(entry.path()) << "  " << entry.path().filename().string() << '\n';
    }
    checksums.close();
    require(bool(checksums), "cannot finalize recording checksums");
    in.closed = true;
}

std::uint64_t MarkerlessRecording::payload_bytes() const { return impl_->total_bytes; }

}  // namespace fitra::camera
