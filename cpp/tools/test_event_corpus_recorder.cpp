// Synthetic regression tests for the always-on event corpus recorder. No
// camera, TensorRT execution, or network mount is required.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "corpus/event_corpus_recorder.hpp"
#include "corpus/event_corpus_queue.hpp"
#include "util/sha256.hpp"

namespace {

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using fitra::corpus::CameraSpec;
using fitra::corpus::EventCorpusRecorder;
using fitra::corpus::RecorderOptions;

void check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error("ASSERT FAILED: " + message);
}

struct TempRoot {
    fs::path path;
    explicit TempRoot(const std::string& label) {
        path = fs::temp_directory_path()
             / ("fitra_corpus_" + label + "_"
                + std::to_string(std::chrono::steady_clock::now()
                    .time_since_epoch().count()));
        fs::create_directories(path);
    }
    ~TempRoot() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
};

std::vector<CameraSpec> two_cameras() {
    return {
        CameraSpec{0, 0, "/dev/cam-a", 640, 480, 640, 480, 60,
                   fitra::camera::PixFmt::Mjpeg},
        CameraSpec{1, 2, "/dev/cam-b", 1280, 960, 640, 480, 60,
                   fitra::camera::PixFmt::Nvjpeg},
    };
}

fitra::camera::Frame frame(std::uint64_t seq,
                           std::initializer_list<std::uint8_t> bytes,
                           std::uint64_t captured_ns,
                           std::optional<std::uint64_t> v4l2_ns = std::nullopt) {
    fitra::camera::Frame out;
    out.seq = seq;
    out.v4l2_sequence = static_cast<std::uint32_t>(seq + 1000);
    out.capture_width = 800;
    out.capture_height = 600;
    out.bytes_per_line = 1600;
    out.size_image = 960000;
    out.pixel_format_fourcc = 0x47504a4dU;  // V4L2 little-endian "MJPG"
    out.frame_interval_numerator = 1;
    out.frame_interval_denominator = 60;
    out.data.assign(bytes);
    out.captured_at = std::chrono::steady_clock::now();
    out.captured_mono_ns = captured_ns;
    out.v4l2_timestamp.mono_ns = v4l2_ns;
    out.v4l2_timestamp.semantics = v4l2_ns
        ? fitra::camera::V4l2TimestampSemantics::MonotonicSoe
        : fitra::camera::V4l2TimestampSemantics::Unavailable;
    return out;
}

std::vector<fs::path> child_directories(const fs::path& root) {
    std::vector<fs::path> out;
    std::error_code ec;
    if (!fs::exists(root, ec)) return out;
    for (const auto& entry : fs::directory_iterator(root)) {
        if (entry.is_directory()) out.push_back(entry.path());
    }
    return out;
}

std::string read_all(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + path.string());
    std::ostringstream out;
    out << in.rdbuf();
    return out.str();
}

bool wait_until(const std::function<bool()>& predicate,
                std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) return true;
        std::this_thread::sleep_for(10ms);
    }
    return predicate();
}

void verify_segment_checksums(const fs::path& segment) {
    std::ifstream in(segment / "checksums.sha256");
    check(static_cast<bool>(in), "checksums.sha256 exists");
    std::string line;
    int checked = 0;
    while (std::getline(in, line)) {
        check(line.size() > 66 && line[64] == ' ' && line[65] == ' ',
              "checksum line syntax");
        const auto expected = line.substr(0, 64);
        const auto filename = line.substr(66);
        check(fitra::util::sha256_file(segment / filename) == expected,
              "checksum matches " + filename);
        ++checked;
    }
    check(checked >= 3, "frames, segment manifest, and payload are checksummed");
}

void test_sha256_known_vector() {
    const std::string input = "abc";
    check(fitra::util::sha256_hex(
              reinterpret_cast<const std::uint8_t*>(input.data()), input.size())
          == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
          "SHA-256 abc vector");
    const std::string two_blocks =
        "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    check(fitra::util::sha256_hex(
              reinterpret_cast<const std::uint8_t*>(two_blocks.data()),
              two_blocks.size())
          == "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1",
          "SHA-256 two-block padding vector");
}

void test_pre_post_roll_segments_and_provenance() {
    TempRoot root{"event"};
    RecorderOptions options;
    options.spool_dir = root.path / "spool";
    options.pre_roll = 1s;
    options.post_roll = 25ms;
    options.segment = 2ms;
    options.queue_max_bytes = 1 << 20;
    options.ring_max_bytes = 1 << 20;
    options.spool_max_bytes = 1 << 20;
    options.min_free_bytes = 0;

    EventCorpusRecorder recorder{options, two_cameras()};
    recorder.start();
    const auto huge_ns = 9007199254740993ULL;  // above exact JSON-number range
    recorder.ingest_raw(0, frame(1, {1, 2, 3}, 101, huge_ns));
    recorder.ingest_raw(1, frame(10, {10, 11}, 102));
    std::this_thread::sleep_for(4ms);
    recorder.observe_presence(0, true, std::chrono::steady_clock::now(), 101);
    recorder.ingest_raw(0, frame(2, {4, 5}, 103));
    recorder.ingest_raw(1, frame(11, {12, 13, 14}, 104));
    std::this_thread::sleep_for(4ms);  // force immutable segment rotation
    recorder.ingest_raw(0, frame(3, {6}, 105));
    recorder.observe_presence(0, false, std::chrono::steady_clock::now(), 105);
    recorder.ingest_raw(1, frame(12, {15}, 106));  // post-roll

    check(wait_until([&]() {
              return child_directories(options.spool_dir / "ready").size() == 1;
          }, 2s), "post-roll closes one ready session");
    recorder.stop();

    const auto sessions = child_directories(options.spool_dir / "ready");
    check(sessions.size() == 1, "one event session");
    const auto manifest = read_all(sessions[0] / "manifest.json");
    check(manifest.find("\"schema\": \"fitra_event_corpus_v1\"")
              != std::string::npos, "session schema");
    check(manifest.find("\"trigger\": \"any_inferred_person\"")
              != std::string::npos, "presence trigger recorded");
    check(manifest.find("\"raw_encoding\":\"jpeg\"")
              != std::string::npos, "raw encoding recorded");
    check(manifest.find("\"config_slot\":2") != std::string::npos,
          "sparse config slot mapping retained");
    check(manifest.find("\"capture_width\":800") != std::string::npos,
          "accepted capture dimensions replace requested values");
    check(manifest.find("\"output_width\":640") != std::string::npos,
          "runtime resize dimensions retained");
    check(manifest.find("\"v4l2_fourcc\":\"MJPG\"") != std::string::npos,
          "accepted V4L2 fourcc retained");
    check(manifest.find("\"bytes_per_line\":1600") != std::string::npos,
          "accepted V4L2 stride retained");
    check(manifest.find("\"frame_interval_denominator\":60")
              != std::string::npos, "accepted frame interval retained");

    const auto segments = child_directories(sessions[0] / "segments");
    check(segments.size() >= 2, "segment duration rotates output");
    std::string all_frames;
    for (const auto& segment : segments) {
        all_frames += read_all(segment / "frames.jsonl");
        verify_segment_checksums(segment);
        check(!segment.filename().string().ends_with(".partial"),
              "ready session has only committed segments");
    }
    for (const std::string seq : {"1", "10", "2", "11", "3", "12"}) {
        check(all_frames.find("\"seq\":\"" + seq + "\"") != std::string::npos,
              "pre/active/post sequence " + seq + " retained");
    }
    check(all_frames.find("\"v4l2_mono_ns\":\""
                          + std::to_string(huge_ns) + "\"") != std::string::npos,
          "64-bit V4L2 timestamp stored as decimal string");
    check(all_frames.find("\"v4l2_mono_ns\":null") != std::string::npos,
          "unavailable V4L2 timestamp stays unavailable");
    check(all_frames.find("\"v4l2_semantics\":\"monotonic_soe\"")
              != std::string::npos, "SOE semantics retained");
    check(all_frames.find("\"v4l2_sequence\":1001") != std::string::npos,
          "kernel V4L2 sequence retained");

    const auto stats = recorder.stats();
    check(stats.written_frames == 6, "all six event frames written");
    check(stats.queue_dropped_frames == 0, "no queue drops");
    check(stats.storage_dropped_frames == 0, "no storage drops");
}

void test_queue_and_spool_caps_drop_only_recorder_copy() {
    TempRoot root{"caps"};
    RecorderOptions options;
    options.spool_dir = root.path / "spool";
    options.pre_roll = 1s;
    options.post_roll = 0ms;
    options.segment = 1s;
    options.queue_max_bytes = 3;
    options.ring_max_bytes = 1024;
    options.spool_max_bytes = 2;
    options.min_free_bytes = 0;
    EventCorpusRecorder recorder{options, {two_cameras()[0]}};
    recorder.start();
    recorder.ingest_raw(0, frame(1, {1, 2, 3, 4}, 1));  // queue cap
    recorder.ingest_raw(0, frame(2, {5, 6, 7}, 2));     // spool cap after trigger
    recorder.observe_presence(0, true, std::chrono::steady_clock::now(), 2);
    recorder.ingest_raw(0, frame(3, {8, 9, 10}, 3));
    recorder.observe_presence(0, false, std::chrono::steady_clock::now(), 3);
    recorder.stop();
    const auto stats = recorder.stats();
    check(stats.queue_dropped_frames >= 1, "oversize queue frame dropped");
    check(stats.storage_dropped_frames >= 1, "spool-full frame dropped");
    check(stats.written_frames == 0, "caps never write oversized payload");
}

void test_presence_backlog_is_bounded_and_resynchronizes() {
    using fitra::corpus::detail::EventQueue;
    using fitra::corpus::detail::PresenceItem;
    using fitra::corpus::detail::RawItem;
    EventQueue queue;
    std::deque<EventQueue::Item> discarded;
    const auto base = std::chrono::steady_clock::now();
    for (int i = 0; i < 2; ++i) {
        RawItem raw;
        raw.seq = i + 1;
        raw.payload = {1, 2, 3};
        raw.captured_at = base;
        check(queue.push_raw(std::move(raw)), "old payload enqueued");
    }
    for (std::size_t i = 0; i < EventQueue::kMaxPresenceEvents; ++i) {
        check(queue.push_presence(PresenceItem{0, i % 2 == 0, base, i},
                                  i % 2 == 0 ? 1 : 0), "bounded edges enqueue");
    }
    for (std::uint64_t i = 1; i <= 100000; ++i) {
        check(!queue.push_presence(PresenceItem{1, i % 2 == 0, base + 1ms, i},
                                   i % 2 == 0 ? 2 : 0), "overflow uses reset slot");
        check(queue.pending_presence_events() == EventQueue::kMaxPresenceEvents,
              "control backlog never grows beyond its edge cap");
    }
    check(queue.resync_pending(), "overflow marks an explicit reset");
    // A late callback from another camera must not move the boundary backward.
    queue.push_presence(PresenceItem{0, false, base, 7}, 2);
    auto batch = queue.take(discarded);
    check(batch.reset && !batch.item, "reset precedes stale queue contents");
    check(batch.reset->presence_mask == 2, "latest complete camera mask retained");
    check(batch.reset->captured_at == base + 1ms,
          "recovery boundary keeps the newest discarded capture time");
    check(batch.reset->captured_mono_ns == 100000,
          "boundary time and lineage come from the same source");
    check(batch.discarded_raw_frames == 2 && batch.discarded_raw_bytes == 6,
          "retired payload reservations can be released exactly");
    check(batch.discarded_presence_events == EventQueue::kMaxPresenceEvents,
          "retired control edges are counted");
    check(queue.empty() && !queue.resync_pending(), "old backlog detached atomically");
    discarded.clear();  // Production releases it outside queue_mu_.

    RawItem stale;
    stale.payload = {4};
    stale.captured_at = base;
    check(!queue.push_raw(std::move(stale)), "in-flight old payload cannot reseed");
    RawItem fresh;
    fresh.seq = 42;
    fresh.payload = {5};
    fresh.captured_at = base + 2ms;
    check(queue.push_raw(std::move(fresh)), "new capture is admitted after recovery");
    check(queue.push_presence(PresenceItem{1, false, base + 3ms, 43}, 0),
          "new absence edge is admitted after recovery");
    auto next = queue.take(discarded);
    check(next.item && std::get<RawItem>(*next.item).seq == 42,
          "ordinary payload/control order is retained");
    next = queue.take(discarded);
    check(next.item && !std::get<PresenceItem>(*next.item).present,
          "absence does not remain stuck behind the discarded edges");
    queue.reset();
    RawItem restarted;
    restarted.payload = {6};
    restarted.captured_at = base;
    check(queue.push_raw(std::move(restarted)), "restart clears the old capture watermark");
}

void test_empty_payload_cannot_bypass_queue_limit() {
    TempRoot root{"empty_payload"};
    RecorderOptions options;
    options.spool_dir = root.path / "spool";
    options.queue_max_bytes = 1;
    options.min_free_bytes = 0;
    EventCorpusRecorder recorder{options, {two_cameras()[0]}};
    recorder.start();
    for (std::uint64_t i = 0; i < 10000; ++i) {
        recorder.ingest_raw(0, frame(i, {}, i));
    }
    recorder.stop();
    const auto stats = recorder.stats();
    check(stats.accepted_frames == 0 && stats.queue_dropped_frames == 10000,
          "zero-byte payloads cannot accumulate uncharged queue entries");
    check(stats.queued_bytes == 0, "empty payloads leave no reservation");
}

void test_presence_overflow_recovers_recording_and_absence() {
    TempRoot root{"presence_overflow"};
    RecorderOptions options;
    options.spool_dir = root.path / "spool";
    options.pre_roll = 0ms;
    options.post_roll = 0ms;
    options.min_free_bytes = 0;
    EventCorpusRecorder recorder{options, {two_cameras()[0]}};
    recorder.start();
    recorder.observe_presence(0, true, std::chrono::steady_clock::now(), 1);
    check(wait_until([&] { return recorder.stats().recording; }, 2s),
          "initial event starts before overload");
    for (std::uint64_t i = 0; i < 20000; ++i) {
        recorder.observe_presence(0, i % 2 != 0, std::chrono::steady_clock::now(), i + 2);
    }
    recorder.observe_presence(0, true, std::chrono::steady_clock::now(), 30000);
    check(wait_until([&] {
              const auto stats = recorder.stats();
              return stats.presence_resyncs > 0 && stats.recording
                  && stats.pending_presence_events == 0 && !stats.presence_resync_pending;
          }, 3s), "overflow converges to the latest present state");
    recorder.ingest_raw(0, frame(4242, {1, 2, 3}, 40000));
    check(wait_until([&] { return recorder.stats().written_frames == 1; }, 2s),
          "fresh capture is written after recovery");
    recorder.observe_presence(0, false, std::chrono::steady_clock::now(), 40001);
    check(wait_until([&] {
              const auto stats = recorder.stats();
              return !stats.recording && stats.pending_presence_events == 0
                  && !stats.presence_resync_pending;
          }, 3s), "later absence closes the recovered event");
    recorder.stop();
    const auto stats = recorder.stats();
    check(stats.presence_dropped_events > 0 && stats.queued_bytes == 0,
          "overflow is visible and retired reservations are released");
    bool found_boundary = false, found_capture = false;
    for (const auto& session : child_directories(options.spool_dir / "ready")) {
        const auto manifest = read_all(session / "manifest.json");
        found_boundary |= manifest.find("presence_queue_overflow") != std::string::npos;
        for (const auto& segment : child_directories(session / "segments")) {
            found_capture |= read_all(segment / "frames.jsonl").find("\"seq\":\"4242\"")
                             != std::string::npos;
        }
    }
    check(found_boundary && found_capture,
          "committed sessions retain the explicit boundary and new capture");
}

void test_stale_presence_closes_disconnected_camera_event() {
    TempRoot root{"stale_presence"};
    RecorderOptions options;
    options.spool_dir = root.path / "spool";
    options.pre_roll = 100ms;
    options.post_roll = 0ms;
    options.segment = 1s;
    options.presence_stale = 20ms;
    options.queue_max_bytes = 1 << 20;
    options.ring_max_bytes = 1 << 20;
    options.spool_max_bytes = 1 << 20;
    options.min_free_bytes = 0;
    EventCorpusRecorder recorder{options, {two_cameras()[0]}};
    recorder.start();
    recorder.ingest_raw(0, frame(1, {1, 2, 3}, 1));
    recorder.observe_presence(0, true, std::chrono::steady_clock::now(), 1);
    recorder.ingest_raw(0, frame(2, {4, 5, 6}, 2));
    // No false observation: model a camera/source thread that disappeared.
    check(wait_until([&]() {
              return child_directories(options.spool_dir / "ready").size() == 1;
          }, 2s), "stale presence expires and closes the event");
    recorder.stop();
    check(recorder.stats().written_frames == 2,
          "frames before disconnect remain committed");
}

void test_post_roll_uses_capture_timeline() {
    TempRoot root{"post_timeline"};
    RecorderOptions options;
    options.spool_dir = root.path / "spool";
    options.pre_roll = 1s;
    options.post_roll = 20ms;
    options.segment = 1s;
    options.queue_max_bytes = 1 << 20;
    options.ring_max_bytes = 1 << 20;
    options.spool_max_bytes = 1 << 20;
    options.min_free_bytes = 0;
    EventCorpusRecorder recorder{options, {two_cameras()[0]}};
    recorder.start();

    const auto base = std::chrono::steady_clock::now();
    recorder.observe_presence(0, true, base, 100);
    auto active = frame(1, {1}, 100);
    active.captured_at = base;
    recorder.ingest_raw(0, active);
    recorder.observe_presence(0, false, base, 100);
    auto within_post = frame(2, {2}, 110);
    within_post.captured_at = base + 10ms;
    recorder.ingest_raw(0, within_post);
    auto beyond_post = frame(3, {3}, 130);
    beyond_post.captured_at = base + 30ms;
    recorder.ingest_raw(0, beyond_post);

    check(wait_until([&]() {
              return child_directories(options.spool_dir / "ready").size() == 1;
          }, 2s), "source capture time closes post-roll despite writer timing");
    recorder.stop();
    const auto sessions = child_directories(options.spool_dir / "ready");
    const auto segments = child_directories(sessions[0] / "segments");
    std::string all_frames;
    for (const auto& segment : segments) {
        all_frames += read_all(segment / "frames.jsonl");
    }
    check(all_frames.find("\"seq\":\"1\"") != std::string::npos,
          "active frame retained");
    check(all_frames.find("\"seq\":\"2\"") != std::string::npos,
          "frame inside post-roll retained");
    check(all_frames.find("\"seq\":\"3\"") == std::string::npos,
          "frame beyond post-roll is not appended to the closed event");
}

void record_short_event(EventCorpusRecorder& recorder) {
    recorder.ingest_raw(0, frame(1, {0xff, 0xd8, 0xff, 0xd9}, 11));
    recorder.observe_presence(0, true, std::chrono::steady_clock::now(), 11);
    recorder.ingest_raw(0, frame(2, {0xff, 0xd8, 1, 0xff, 0xd9}, 12));
    recorder.observe_presence(0, false, std::chrono::steady_clock::now(), 12);
}

void test_remote_requires_marker_and_retains_local() {
    TempRoot root{"marker"};
    RecorderOptions options;
    options.spool_dir = root.path / "spool";
    options.remote_dir = root.path / "remote";
    options.pre_roll = 1s;
    options.post_roll = 0ms;
    options.segment = 1s;
    options.upload_absent = 0ms;
    options.queue_max_bytes = 1 << 20;
    options.ring_max_bytes = 1 << 20;
    options.spool_max_bytes = 1 << 20;
    options.min_free_bytes = 0;
    fs::create_directories(options.remote_dir);  // deliberately no marker

    EventCorpusRecorder recorder{options, {two_cameras()[0]}};
    recorder.start();
    record_short_event(recorder);
    check(wait_until([&]() {
              return child_directories(options.spool_dir / "ready").size() == 1;
          }, 2s), "local event becomes ready");
    std::this_thread::sleep_for(650ms);
    recorder.stop();
    check(child_directories(options.spool_dir / "ready").size() == 1,
          "missing remote marker retains local session");
    check(child_directories(options.remote_dir / "ready").empty(),
          "missing marker prevents remote writes");
}

void test_remote_waits_for_initial_presence_observation() {
    TempRoot root{"startup_gate"};
    RecorderOptions options;
    options.spool_dir = root.path / "spool";
    options.remote_dir = root.path / "remote";
    options.upload_absent = 0ms;
    options.presence_stale = 5s;
    options.queue_max_bytes = 1 << 20;
    options.ring_max_bytes = 1 << 20;
    options.spool_max_bytes = 1 << 20;
    options.min_free_bytes = 0;
    const auto old_session = options.spool_dir / "ready" / "old-session";
    fs::create_directories(old_session);
    std::ofstream(old_session / "manifest.json") << "{}\n";
    fs::create_directories(options.remote_dir);
    std::ofstream(options.remote_dir / ".fitra-corpus-target") << "mounted\n";

    EventCorpusRecorder recorder{options, {two_cameras()[0]}};
    recorder.start();
    std::this_thread::sleep_for(650ms);  // uploader has completed its first poll
    check(fs::is_directory(old_session),
          "startup does not assume an unobserved camera is empty");
    check(child_directories(options.remote_dir / "ready").empty(),
          "unobserved startup state blocks remote transfer");

    recorder.observe_presence(0, false, std::chrono::steady_clock::now(), 1);
    check(wait_until([&]() {
              return child_directories(options.remote_dir / "ready").size() == 1
                  && !fs::exists(old_session);
          }, 2s), "an observed empty state releases verified transfer");
    recorder.stop();
}

void test_verified_remote_commit_then_local_delete() {
    TempRoot root{"upload"};
    RecorderOptions options;
    options.spool_dir = root.path / "spool";
    options.remote_dir = root.path / "remote";
    options.pre_roll = 1s;
    options.post_roll = 0ms;
    options.segment = 1s;
    options.upload_absent = 0ms;
    options.queue_max_bytes = 1 << 20;
    options.ring_max_bytes = 1 << 20;
    options.spool_max_bytes = 1 << 20;
    options.min_free_bytes = 0;
    fs::create_directories(options.remote_dir);
    std::ofstream(options.remote_dir / ".fitra-corpus-target") << "mounted\n";

    EventCorpusRecorder recorder{options, {two_cameras()[0]}};
    recorder.start();
    record_short_event(recorder);
    check(wait_until([&]() {
              return child_directories(options.remote_dir / "ready").size() == 1
                  && child_directories(options.spool_dir / "ready").empty();
          }, 5s), "verified remote commit precedes local deletion");
    recorder.stop();
    const auto remote = child_directories(options.remote_dir / "ready");
    check(remote.size() == 1, "one remotely committed session");
    check(fs::is_regular_file(remote[0] / "manifest.json"),
          "remote manifest exists");
    check(child_directories(options.remote_dir / ".partial").empty(),
          "no partial session remains after commit");
    check(recorder.stats().uploaded_sessions == 1, "upload stat increments");
}

struct TestCase {
    const char* name;
    void (*fn)();
};

const TestCase kTests[] = {
    {"sha256_known_vector", test_sha256_known_vector},
    {"pre_post_roll_segments_and_provenance",
     test_pre_post_roll_segments_and_provenance},
    {"queue_and_spool_caps_drop_only_recorder_copy",
     test_queue_and_spool_caps_drop_only_recorder_copy},
    {"presence_backlog_is_bounded_and_resynchronizes",
     test_presence_backlog_is_bounded_and_resynchronizes},
    {"empty_payload_cannot_bypass_queue_limit",
     test_empty_payload_cannot_bypass_queue_limit},
    {"presence_overflow_recovers_recording_and_absence",
     test_presence_overflow_recovers_recording_and_absence},
    {"stale_presence_closes_disconnected_camera_event",
     test_stale_presence_closes_disconnected_camera_event},
    {"post_roll_uses_capture_timeline",
     test_post_roll_uses_capture_timeline},
    {"remote_requires_marker_and_retains_local",
     test_remote_requires_marker_and_retains_local},
    {"remote_waits_for_initial_presence_observation",
     test_remote_waits_for_initial_presence_observation},
    {"verified_remote_commit_then_local_delete",
     test_verified_remote_commit_then_local_delete},
};

}  // namespace

int main() {
    int failed = 0;
    for (const auto& test : kTests) {
        try {
            test.fn();
            std::printf("[PASS] %s\n", test.name);
        } catch (const std::exception& e) {
            std::printf("[FAIL] %s: %s\n", test.name, e.what());
            ++failed;
        }
    }
    if (failed != 0) {
        std::printf("\n%d test(s) failed\n", failed);
        return EXIT_FAILURE;
    }
    std::printf("\nall %zu tests passed\n", sizeof(kTests) / sizeof(kTests[0]));
    return EXIT_SUCCESS;
}
