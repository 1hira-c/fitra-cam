#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "camera/v4l2_capture.hpp"

namespace fitra::corpus {

struct CameraSpec {
    std::size_t index = 0;  // compact runtime index (not the config slot)
    std::size_t config_slot = 0;
    std::string device_path;
    int capture_width = 0;
    int capture_height = 0;
    int output_width = 0;
    int output_height = 0;
    int requested_fps = 0;
    camera::PixFmt pixel_format = camera::PixFmt::Mjpeg;
    std::uint32_t bytes_per_line = 0;
    std::uint32_t size_image = 0;
    std::uint32_t pixel_format_fourcc = 0;
    std::uint32_t frame_interval_numerator = 0;
    std::uint32_t frame_interval_denominator = 0;
};

struct RecorderOptions {
    std::filesystem::path spool_dir;
    std::filesystem::path remote_dir;  // empty disables upload
    std::chrono::milliseconds pre_roll{5000};
    std::chrono::milliseconds post_roll{10000};
    std::chrono::milliseconds segment{5000};
    std::chrono::milliseconds upload_absent{30000};
    std::chrono::milliseconds presence_stale{2000};
    std::size_t queue_max_bytes = 128ULL * 1024ULL * 1024ULL;
    std::size_t ring_max_bytes = 512ULL * 1024ULL * 1024ULL;
    std::uintmax_t spool_max_bytes = 256ULL * 1024ULL * 1024ULL * 1024ULL;
    std::uintmax_t min_free_bytes = 10ULL * 1024ULL * 1024ULL * 1024ULL;
    bool remove_after_upload = true;
};

struct RecorderStats {
    std::uint64_t accepted_frames = 0;
    std::uint64_t queue_dropped_frames = 0;
    std::uint64_t storage_dropped_frames = 0;
    std::uint64_t written_frames = 0;
    std::uint64_t ready_sessions = 0;
    std::uint64_t uploaded_sessions = 0;
    std::uint64_t upload_failures = 0;
    std::size_t queued_bytes = 0;
    std::size_t ring_bytes = 0;
    std::uintmax_t spool_bytes = 0;
    bool recording = false;
};

// Always-on, event-triggered raw-source recorder. The public callbacks are safe
// to call from FrameSource / MultiCameraDriver threads. ingest_raw never does
// file or network I/O and never waits for queue space; a full queue drops only
// the corpus copy.
class EventCorpusRecorder {
public:
    EventCorpusRecorder(RecorderOptions options,
                        std::vector<CameraSpec> cameras);
    ~EventCorpusRecorder();

    EventCorpusRecorder(const EventCorpusRecorder&) = delete;
    EventCorpusRecorder& operator=(const EventCorpusRecorder&) = delete;

    void start();
    void stop() noexcept;

    void ingest_raw(std::size_t camera_index,
                    const camera::Frame& frame) noexcept;
    void observe_presence(std::size_t camera_index,
                          bool present,
                          std::chrono::steady_clock::time_point captured_at,
                          std::uint64_t captured_mono_ns) noexcept;

    RecorderStats stats() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace fitra::corpus
