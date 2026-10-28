#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "camera/v4l2_capture.hpp"

namespace fitra::camera {

struct MarkerlessCue {
    std::string id;
    std::string take;
    std::string role;
    std::string prompt;
    int seconds = 0;
    int settle_seconds = 0;
};

// Standard batch: 370 s. Timings are a capture budget, not an accuracy claim.
std::vector<MarkerlessCue> markerless_capture_plan();
std::string markerless_capture_plan_json();

struct MarkerlessRecordingCamera {
    std::string id;
    std::size_t config_slot = 0;
    V4l2Options capture;
};

struct MarkerlessRecordingOptions {
    std::filesystem::path out_dir;
    std::filesystem::path intrinsics;
    std::filesystem::path reference;  // optional comparison, not ground truth
    double height_m = 0.0;           // 0 means not measured
    std::string recorder_binary_sha256;
    std::uint64_t max_payload_bytes = 32ULL * 1024 * 1024 * 1024;
    std::uint64_t min_free_bytes = 2ULL * 1024 * 1024 * 1024;
};

// Single-owner file writer fed by try_pop_latest(), never from the DQBUF
// callback. Capture retains its size-one latest slot; missed sources are
// counted. No inference, pose relay, upload or calibration mutation.
class MarkerlessRecording {
public:
    MarkerlessRecording(MarkerlessRecordingOptions options,
                        std::vector<MarkerlessRecordingCamera> cameras);
    ~MarkerlessRecording();
    MarkerlessRecording(const MarkerlessRecording&) = delete;
    MarkerlessRecording& operator=(const MarkerlessRecording&) = delete;

    void cue(std::size_t index, std::uint64_t monotonic_ns);
    void append(std::size_t camera_index, const Frame& frame);
    void finish(bool protocol_complete, const std::string& reason);
    std::uint64_t payload_bytes() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace fitra::camera
