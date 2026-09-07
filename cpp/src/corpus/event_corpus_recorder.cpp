#include "corpus/event_corpus_recorder.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <variant>

#include "util/logging.hpp"
#include "util/sha256.hpp"

namespace fitra::corpus {

namespace fs = std::filesystem;

namespace {

constexpr const char* kSchema = "fitra_event_corpus_v1";
constexpr const char* kRemoteMarker = ".fitra-corpus-target";

std::uint64_t steady_now_ns() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::uint64_t unix_now_ns() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
}

std::string utc_compact_now() {
    const auto now = std::chrono::system_clock::now();
    const auto sec = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;
    std::tm tm{};
    gmtime_r(&sec, &tm);
    std::ostringstream out;
    out << std::put_time(&tm, "%Y%m%dT%H%M%S")
        << std::setw(3) << std::setfill('0') << ms << 'Z';
    return out.str();
}

std::string json_escape(const std::string& s) {
    std::ostringstream out;
    for (const unsigned char c : s) {
        switch (c) {
            case '"': out << "\\\""; break;
            case '\\': out << "\\\\"; break;
            case '\b': out << "\\b"; break;
            case '\f': out << "\\f"; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default:
                if (c < 0x20) {
                    out << "\\u" << std::hex << std::setw(4)
                        << std::setfill('0') << static_cast<unsigned>(c)
                        << std::dec;
                } else {
                    out << static_cast<char>(c);
                }
        }
    }
    return out.str();
}

const char* pixel_format_name(camera::PixFmt format) {
    switch (format) {
        case camera::PixFmt::Mjpeg: return "mjpeg";
        case camera::PixFmt::Yuyv: return "yuyv";
        case camera::PixFmt::Nvjpeg: return "nvjpeg";
    }
    return "unknown";
}

const char* raw_encoding_name(camera::PixFmt format) {
    return format == camera::PixFmt::Yuyv ? "yuyv422" : "jpeg";
}

std::string fourcc_name(std::uint32_t value) {
    std::string out(4, '.');
    for (std::size_t i = 0; i < out.size(); ++i) {
        const auto c = static_cast<unsigned char>((value >> (8U * i)) & 0xffU);
        if (c >= 0x20U && c <= 0x7eU) out[i] = static_cast<char>(c);
    }
    return out;
}

std::uintmax_t directory_bytes(const fs::path& root) {
    std::error_code ec;
    if (!fs::exists(root, ec) || ec) return 0;
    std::uintmax_t total = 0;
    fs::recursive_directory_iterator it(
        root, fs::directory_options::skip_permission_denied, ec);
    const fs::recursive_directory_iterator end;
    while (!ec && it != end) {
        if (it->is_regular_file(ec) && !ec) {
            const auto n = it->file_size(ec);
            if (!ec && n <= std::numeric_limits<std::uintmax_t>::max() - total) {
                total += n;
            }
        }
        it.increment(ec);
    }
    return total;
}

std::vector<fs::path> regular_files_sorted(const fs::path& root) {
    std::vector<fs::path> out;
    std::error_code ec;
    fs::recursive_directory_iterator it(
        root, fs::directory_options::skip_permission_denied, ec);
    const fs::recursive_directory_iterator end;
    while (!ec && it != end) {
        if (it->is_regular_file(ec) && !ec) out.push_back(it->path());
        it.increment(ec);
    }
    if (ec) throw std::runtime_error("cannot enumerate " + root.string()
                                     + ": " + ec.message());
    std::sort(out.begin(), out.end());
    return out;
}

fs::path unique_destination(fs::path candidate) {
    std::error_code ec;
    if (!fs::exists(candidate, ec)) return candidate;
    for (unsigned i = 1; i < 10000; ++i) {
        fs::path next = candidate.string() + "-" + std::to_string(i);
        ec.clear();
        if (!fs::exists(next, ec)) return next;
    }
    throw std::runtime_error("cannot choose unique path near " + candidate.string());
}

void write_text(const fs::path& path, const std::string& body) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) throw std::runtime_error("cannot open for write: " + path.string());
    out << body;
    if (!out) throw std::runtime_error("cannot write: " + path.string());
    out.close();
    if (!out) throw std::runtime_error("cannot close: " + path.string());
}

// nullopt means the caller requested a pause; false means a real mismatch.
std::optional<bool> tree_equal(
    const fs::path& source, const fs::path& destination,
    const std::function<bool()>& keep_going = [] { return true; }) {
    try {
        for (const auto& src : regular_files_sorted(source)) {
            if (!keep_going()) return std::nullopt;
            const auto rel = src.lexically_relative(source);
            const auto dst = destination / rel;
            std::error_code ec;
            if (!fs::is_regular_file(dst, ec) || ec) return false;
            const auto source_size = fs::file_size(src, ec);
            if (ec) return false;
            const auto destination_size = fs::file_size(dst, ec);
            if (ec || source_size != destination_size) return false;
            std::string source_hash;
            std::string destination_hash;
            if (!util::sha256_file_interruptible(src, keep_going, source_hash)
                || !util::sha256_file_interruptible(
                    dst, keep_going, destination_hash)) {
                return std::nullopt;
            }
            if (source_hash != destination_hash) return false;
        }
        return true;
    } catch (...) {
        return false;
    }
}

enum class CopyResult { Complete, Paused };

CopyResult copy_file_verified(const fs::path& source,
                              const fs::path& destination,
                              const std::function<bool()>& keep_going) {
    if (!keep_going()) return CopyResult::Paused;
    std::error_code ec;
    fs::create_directories(destination.parent_path(), ec);
    if (ec) throw std::runtime_error("cannot create remote directory: " + ec.message());

    const fs::path temp = destination.string() + ".copying";
    std::ifstream in(source, std::ios::binary);
    std::ofstream out(temp, std::ios::binary | std::ios::trunc);
    if (!in || !out) {
        throw std::runtime_error("cannot open transfer file: " + source.string());
    }
    std::array<char, 1024 * 1024> buffer{};
    while (in) {
        if (!keep_going()) {
            out.close();
            return CopyResult::Paused;
        }
        in.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const auto n = in.gcount();
        if (n > 0) out.write(buffer.data(), n);
        if (!out) throw std::runtime_error("remote write failed: " + temp.string());
    }
    if (!in.eof()) throw std::runtime_error("source read failed: " + source.string());
    out.close();
    if (!out) throw std::runtime_error("remote close failed: " + temp.string());
    if (!keep_going()) return CopyResult::Paused;

    const auto source_size = fs::file_size(source);
    const auto copied_size = fs::file_size(temp);
    std::string source_hash;
    std::string copied_hash;
    if (!util::sha256_file_interruptible(source, keep_going, source_hash)
        || !util::sha256_file_interruptible(temp, keep_going, copied_hash)) {
        return CopyResult::Paused;
    }
    if (source_size != copied_size || source_hash != copied_hash) {
        throw std::runtime_error("remote size/SHA-256 mismatch: " + source.string());
    }
    if (!keep_going()) return CopyResult::Paused;
    fs::remove(destination, ec);  // exact file only; makes retry replaceable
    ec.clear();
    fs::rename(temp, destination, ec);
    if (ec) throw std::runtime_error("cannot commit remote file: " + ec.message());
    return CopyResult::Complete;
}

}  // namespace

class EventCorpusRecorder::Impl {
public:
    Impl(RecorderOptions options, std::vector<CameraSpec> cameras)
        : options_{std::move(options)}, cameras_{std::move(cameras)},
          total_frame_counts_(cameras_.size(), 0) {
        if (options_.spool_dir.empty()) {
            throw std::invalid_argument("corpus spool_dir must not be empty");
        }
        if (cameras_.empty() || cameras_.size() > 63) {
            throw std::invalid_argument("corpus needs 1..63 cameras");
        }
        for (std::size_t i = 0; i < cameras_.size(); ++i) {
            if (cameras_[i].index != i) {
                throw std::invalid_argument("corpus camera indices must be compact");
            }
        }
        if (options_.segment.count() <= 0 || options_.pre_roll.count() < 0
            || options_.post_roll.count() < 0
            || options_.upload_absent.count() < 0
            || options_.presence_stale.count() <= 0
            || options_.queue_max_bytes == 0 || options_.ring_max_bytes == 0
            || options_.spool_max_bytes == 0) {
            throw std::invalid_argument("invalid corpus limits");
        }
    }

    ~Impl() { stop(); }

    void start() {
        bool expected = false;
        if (!started_.compare_exchange_strong(expected, true)) return;
        try {
            std::error_code ec;
            fs::create_directories(open_root(), ec);
            if (ec) throw std::runtime_error("cannot create corpus open/: " + ec.message());
            fs::create_directories(ready_root(), ec);
            if (ec) throw std::runtime_error("cannot create corpus ready/: " + ec.message());
            fs::create_directories(uploaded_root(), ec);
            if (ec) throw std::runtime_error("cannot create corpus uploaded/: " + ec.message());
            recover_open_sessions();
            spool_bytes_.store(directory_bytes(options_.spool_dir));
            presence_mask_.store(0);
            presence_observed_mask_.store(0);
            writer_presence_mask_ = 0;
            last_presence_observed_ns_.fill(0);
            post_deadline_.reset();
            space_known_ = false;
            accepting_.store(true);
            upload_stop_.store(false);
            absent_since_ns_.store(steady_now_ns());
            upload_not_before_ns_.store(
                steady_now_ns() + static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        options_.presence_stale).count()));
            writer_ = std::thread{&Impl::writer_loop, this};
            if (!options_.remote_dir.empty()) {
                uploader_ = std::thread{&Impl::uploader_loop, this};
            }
            FITRA_LOG_INFO(
                "corpus: enabled spool={} remote={} pre={}ms post={}ms segment={}ms cap={}GiB",
                options_.spool_dir.string(),
                options_.remote_dir.empty() ? "disabled" : options_.remote_dir.string(),
                options_.pre_roll.count(), options_.post_roll.count(),
                options_.segment.count(),
                options_.spool_max_bytes / (1024ULL * 1024ULL * 1024ULL));
        } catch (...) {
            accepting_.store(false);
            upload_stop_.store(true);
            queue_cv_.notify_all();
            upload_cv_.notify_all();
            if (writer_.joinable()) writer_.join();
            if (uploader_.joinable()) uploader_.join();
            started_.store(false);
            throw;
        }
    }

    void stop() noexcept {
        if (!started_.exchange(false)) return;
        accepting_.store(false);
        queue_cv_.notify_all();
        if (writer_.joinable()) writer_.join();
        upload_stop_.store(true);
        upload_cv_.notify_all();
        if (uploader_.joinable()) uploader_.join();
        FITRA_LOG_INFO(
            "corpus: stopped accepted={} written={} queue_drop={} storage_drop={} uploaded={}",
            accepted_frames_.load(), written_frames_.load(),
            queue_dropped_frames_.load(), storage_dropped_frames_.load(),
            uploaded_sessions_.load());
    }

    void ingest_raw(std::size_t camera_index, const camera::Frame& frame) noexcept {
        if (!accepting_.load(std::memory_order_relaxed)
            || camera_index >= cameras_.size()) return;
        inflight_copies_.fetch_add(1, std::memory_order_acq_rel);
        auto finish = [this]() {
            inflight_copies_.fetch_sub(1, std::memory_order_acq_rel);
            queue_cv_.notify_one();
        };
        if (!accepting_.load(std::memory_order_relaxed)) {
            finish();
            return;
        }
        const std::size_t bytes = frame.data.size();
        std::size_t current = queued_bytes_.load(std::memory_order_relaxed);
        while (true) {
            if (bytes > options_.queue_max_bytes
                || current > options_.queue_max_bytes - bytes) {
                queue_dropped_frames_.fetch_add(1);
                finish();
                return;
            }
            if (queued_bytes_.compare_exchange_weak(
                    current, current + bytes, std::memory_order_acq_rel)) break;
        }
        try {
            RawItem item;
            item.camera_index = camera_index;
            item.payload = frame.data;
            item.seq = frame.seq;
            item.v4l2_sequence = frame.v4l2_sequence;
            item.capture_width = frame.capture_width;
            item.capture_height = frame.capture_height;
            item.bytes_per_line = frame.bytes_per_line;
            item.size_image = frame.size_image;
            item.pixel_format_fourcc = frame.pixel_format_fourcc;
            item.frame_interval_numerator = frame.frame_interval_numerator;
            item.frame_interval_denominator = frame.frame_interval_denominator;
            item.captured_at = frame.captured_at;
            item.captured_mono_ns = frame.captured_mono_ns;
            item.v4l2_timestamp = frame.v4l2_timestamp;
            item.accepted_wall_unix_ns = unix_now_ns();
            {
                std::lock_guard<std::mutex> lock(queue_mu_);
                queue_.emplace_back(std::move(item));
            }
            accepted_frames_.fetch_add(1);
            queue_cv_.notify_one();
        } catch (...) {
            queued_bytes_.fetch_sub(bytes);
            queue_dropped_frames_.fetch_add(1);
        }
        finish();
    }

    void observe_presence(std::size_t camera_index, bool present,
                          std::chrono::steady_clock::time_point captured_at,
                          std::uint64_t captured_mono_ns) noexcept {
        if (!accepting_.load(std::memory_order_relaxed)
            || camera_index >= cameras_.size()) return;
        const auto now = std::chrono::steady_clock::now();
        const auto now_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count());
        const std::uint64_t bit = 1ULL << camera_index;
        std::uint64_t old = 0;
        std::uint64_t next = 0;
        {
            std::lock_guard<std::mutex> state_lock(presence_state_mu_);
            last_presence_observed_ns_[camera_index] = now_ns;
            presence_observed_mask_.fetch_or(bit, std::memory_order_release);
            old = presence_mask_.load(std::memory_order_relaxed);
            next = present ? (old | bit) : (old & ~bit);
            if (next == old) return;
            presence_mask_.store(next, std::memory_order_release);
            absent_since_ns_.store(next == 0 ? now_ns : 0,
                                   std::memory_order_release);
        }
        try {
            std::lock_guard<std::mutex> lock(queue_mu_);
            if (!accepting_.load(std::memory_order_relaxed)) return;
            PresenceItem item;
            item.camera_index = camera_index;
            item.present = present;
            item.captured_at = captured_at;
            item.captured_mono_ns = captured_mono_ns;
            queue_.emplace_back(std::move(item));
            queue_cv_.notify_one();
        } catch (...) {
            // The atomic gate still prevents upload while present. A failed
            // allocation only loses this recorder state transition; it must
            // never escape into the inference thread.
        }
    }

    RecorderStats stats() const noexcept {
        RecorderStats out;
        out.accepted_frames = accepted_frames_.load();
        out.queue_dropped_frames = queue_dropped_frames_.load();
        out.storage_dropped_frames = storage_dropped_frames_.load();
        out.written_frames = written_frames_.load();
        out.ready_sessions = ready_sessions_.load();
        out.uploaded_sessions = uploaded_sessions_.load();
        out.upload_failures = upload_failures_.load();
        out.queued_bytes = queued_bytes_.load();
        out.ring_bytes = ring_bytes_stat_.load();
        out.spool_bytes = spool_bytes_.load();
        out.recording = recording_.load();
        return out;
    }

private:
    struct RawItem {
        std::size_t camera_index = 0;
        std::vector<std::uint8_t> payload;
        std::uint64_t seq = 0;
        std::uint32_t v4l2_sequence = 0;
        std::uint32_t capture_width = 0;
        std::uint32_t capture_height = 0;
        std::uint32_t bytes_per_line = 0;
        std::uint32_t size_image = 0;
        std::uint32_t pixel_format_fourcc = 0;
        std::uint32_t frame_interval_numerator = 0;
        std::uint32_t frame_interval_denominator = 0;
        std::chrono::steady_clock::time_point captured_at{};
        std::uint64_t captured_mono_ns = 0;
        camera::V4l2CaptureTimestamp v4l2_timestamp{};
        std::uint64_t accepted_wall_unix_ns = 0;
    };

    struct PresenceItem {
        std::size_t camera_index = 0;
        bool present = false;
        std::chrono::steady_clock::time_point captured_at{};
        std::uint64_t captured_mono_ns = 0;
    };

    using QueueItem = std::variant<RawItem, PresenceItem>;

    fs::path open_root() const { return options_.spool_dir / "open"; }
    fs::path ready_root() const { return options_.spool_dir / "ready"; }
    fs::path uploaded_root() const { return options_.spool_dir / "uploaded"; }

    void recover_open_sessions() {
        std::error_code ec;
        for (fs::directory_iterator it(open_root(), ec), end; !ec && it != end;
             it.increment(ec)) {
            if (!it->is_directory()) continue;
            const auto source = it->path();
            const auto destination = unique_destination(
                ready_root() / (source.filename().string() + "-recovered-incomplete"));
            try {
                write_text(source / "RECOVERED_INCOMPLETE.txt",
                           "Previous process ended before the session commit.\n");
                fs::rename(source, destination);
                FITRA_LOG_WARN("corpus: recovered incomplete session as {}",
                               destination.string());
            } catch (const std::exception& e) {
                FITRA_LOG_WARN("corpus: cannot recover {}: {}",
                               source.string(), e.what());
            }
        }
        if (ec) throw std::runtime_error("cannot scan corpus open/: " + ec.message());
    }

    void writer_loop() noexcept {
        while (true) {
            std::optional<QueueItem> item;
            {
                std::unique_lock<std::mutex> lock(queue_mu_);
                queue_cv_.wait_for(lock, std::chrono::milliseconds(50), [&]() {
                    return !queue_.empty()
                        || (!accepting_.load(std::memory_order_relaxed)
                            && inflight_copies_.load(std::memory_order_acquire) == 0);
                });
                if (!queue_.empty()) {
                    item.emplace(std::move(queue_.front()));
                    queue_.pop_front();
                } else if (!accepting_.load(std::memory_order_relaxed)
                           && inflight_copies_.load(std::memory_order_acquire) == 0) {
                    break;
                }
            }

            try {
                const auto now = std::chrono::steady_clock::now();
                expire_stale_presence(now);
                if (!item) {
                    maybe_close_post_roll(now);
                    continue;
                }
                if (auto* raw = std::get_if<RawItem>(&*item)) {
                    queued_bytes_.fetch_sub(raw->payload.size());
                    process_raw(std::move(*raw));
                } else {
                    process_presence(std::get<PresenceItem>(*item));
                }
            } catch (const std::exception& e) {
                FITRA_LOG_ERROR("corpus: writer error: {}", e.what());
                abandon_active_session("io_error");
            }
        }
        try {
            if (recording_.load()) close_event("shutdown");
        } catch (const std::exception& e) {
            FITRA_LOG_ERROR("corpus: close on shutdown failed: {}", e.what());
            abandon_active_session("shutdown_io_error");
        }
        ring_.clear();
        ring_bytes_ = 0;
        ring_bytes_stat_.store(0);
    }

    void process_presence(const PresenceItem& item) {
        // Inference can lag behind capture. Split on the source-frame clock so
        // a backed-up writer neither discards queued post-roll frames nor joins
        // a genuinely later re-entry to the previous event.
        if (item.present && recording_.load() && post_deadline_
            && item.captured_at > *post_deadline_) {
            close_event("post_roll_complete");
            post_deadline_.reset();
        }
        const std::uint64_t bit = 1ULL << item.camera_index;
        const bool was_any = writer_presence_mask_ != 0;
        if (item.present) writer_presence_mask_ |= bit;
        else writer_presence_mask_ &= ~bit;
        const bool is_any = writer_presence_mask_ != 0;

        if (!was_any && is_any) {
            post_deadline_.reset();
            if (!recording_.load()) begin_event(item);
        } else if (was_any && !is_any && recording_.load()) {
            post_deadline_ = item.captured_at + options_.post_roll;
        } else if (is_any) {
            post_deadline_.reset();
        }
    }

    void process_raw(RawItem item) {
        auto& camera = cameras_.at(item.camera_index);
        if (item.capture_width != 0) camera.capture_width = item.capture_width;
        if (item.capture_height != 0) camera.capture_height = item.capture_height;
        camera.bytes_per_line = item.bytes_per_line;
        camera.size_image = item.size_image;
        if (item.pixel_format_fourcc != 0) {
            camera.pixel_format_fourcc = item.pixel_format_fourcc;
        }
        if (item.frame_interval_numerator != 0
            && item.frame_interval_denominator != 0) {
            camera.frame_interval_numerator = item.frame_interval_numerator;
            camera.frame_interval_denominator = item.frame_interval_denominator;
        }
        if (recording_.load() && post_deadline_
            && item.captured_at > *post_deadline_) {
            close_event("post_roll_complete");
            post_deadline_.reset();
        }
        if (recording_.load()) {
            write_raw(item);
            return;
        }
        if (options_.pre_roll.count() == 0) return;
        ring_bytes_ += item.payload.size();
        ring_.push_back(std::move(item));
        const auto newest = ring_.back().captured_at;
        while (!ring_.empty()
               && (ring_bytes_ > options_.ring_max_bytes
                   || newest - ring_.front().captured_at > options_.pre_roll)) {
            ring_bytes_ -= ring_.front().payload.size();
            ring_.pop_front();
        }
        ring_bytes_stat_.store(ring_bytes_);
    }

    void maybe_close_post_roll(std::chrono::steady_clock::time_point now) {
        if (recording_.load() && post_deadline_ && now >= *post_deadline_) {
            close_event("post_roll_complete");
            post_deadline_.reset();
        }
    }

    void expire_stale_presence(std::chrono::steady_clock::time_point now) {
        if (writer_presence_mask_ == 0) return;
        const auto now_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count());
        const auto stale_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                options_.presence_stale).count());
        std::uint64_t expired = 0;
        {
            // Serialize expiry with the per-frame heartbeat. Without this tiny
            // lock, a heartbeat could refresh the timestamp while an expiry
            // concurrently clears its bit, causing a false session split.
            std::lock_guard<std::mutex> state_lock(presence_state_mu_);
            for (std::size_t i = 0; i < cameras_.size(); ++i) {
                const std::uint64_t bit = 1ULL << i;
                if ((writer_presence_mask_ & bit) == 0) continue;
                const auto observed = last_presence_observed_ns_[i];
                if (observed != 0 && now_ns - observed >= stale_ns) expired |= bit;
            }
            if (expired != 0) {
                presence_mask_.fetch_and(~expired, std::memory_order_acq_rel);
            }
        }
        if (expired == 0) return;
        const bool was_any = writer_presence_mask_ != 0;
        writer_presence_mask_ &= ~expired;
        if (presence_mask_.load(std::memory_order_acquire) == 0) {
            absent_since_ns_.store(now_ns, std::memory_order_release);
        }
        FITRA_LOG_WARN("corpus: expired stale presence mask 0x{:x}", expired);
        if (was_any && writer_presence_mask_ == 0 && recording_.load()) {
            post_deadline_ = now + options_.post_roll;
        }
    }

    std::string next_session_id() {
        return utc_compact_now() + "-" + std::to_string(session_counter_++);
    }

    void begin_event(const PresenceItem& trigger) {
        session_id_ = next_session_id();
        session_path_ = unique_destination(open_root() / session_id_);
        fs::create_directories(session_path_ / "segments");
        session_started_unix_ns_ = unix_now_ns();
        trigger_capture_mono_ns_ = trigger.captured_mono_ns;
        total_frame_counts_.assign(cameras_.size(), 0);
        session_accounted_bytes_ = 0;
        segment_index_ = 0;
        segment_open_ = false;
        recording_.store(true, std::memory_order_release);
        FITRA_LOG_INFO("corpus: event started {} (pre-roll frames={})",
                       session_id_, ring_.size());
        while (!ring_.empty()) {
            RawItem item = std::move(ring_.front());
            ring_bytes_ -= item.payload.size();
            ring_.pop_front();
            write_raw(item);
        }
        ring_bytes_stat_.store(0);
    }

    bool reserve_spool(std::size_t bytes) {
        const auto now = std::chrono::steady_clock::now();
        if (!space_known_ || now - last_space_check_ >= std::chrono::seconds(1)) {
            std::error_code ec;
            const auto info = fs::space(options_.spool_dir, ec);
            if (ec) return false;
            estimated_available_bytes_ = info.available;
            last_space_check_ = now;
            space_known_ = true;
        }
        if (bytes > estimated_available_bytes_
            || estimated_available_bytes_ - bytes < options_.min_free_bytes) {
            return false;
        }
        auto current = spool_bytes_.load(std::memory_order_relaxed);
        while (true) {
            if (bytes > options_.spool_max_bytes
                || current > options_.spool_max_bytes - bytes) return false;
            if (spool_bytes_.compare_exchange_weak(
                    current, current + bytes, std::memory_order_acq_rel)) {
                estimated_available_bytes_ -= bytes;
                return true;
            }
        }
    }

    void open_segment(const RawItem& first) {
        std::ostringstream name;
        name << std::setw(6) << std::setfill('0') << segment_index_;
        segment_partial_path_ = session_path_ / "segments" / (name.str() + ".partial");
        segment_final_path_ = session_path_ / "segments" / name.str();
        fs::create_directories(segment_partial_path_);
        frames_stream_.open(segment_partial_path_ / "frames.jsonl",
                            std::ios::binary | std::ios::trunc);
        if (!frames_stream_) throw std::runtime_error("cannot open segment frames.jsonl");
        payload_streams_.clear();
        payload_streams_.resize(cameras_.size());
        payload_offsets_.assign(cameras_.size(), 0);
        segment_frame_counts_.assign(cameras_.size(), 0);
        segment_first_at_ = first.captured_at;
        segment_open_ = true;
    }

    void write_raw(const RawItem& item) {
        if (!segment_open_) open_segment(item);
        if (item.captured_at - segment_first_at_ >= options_.segment
            && std::any_of(segment_frame_counts_.begin(), segment_frame_counts_.end(),
                           [](std::uint64_t n) { return n != 0; })) {
            close_segment();
            ++segment_index_;
            open_segment(item);
        }

        const std::size_t bytes = item.payload.size();
        if (!reserve_spool(bytes)) {
            const auto dropped = storage_dropped_frames_.fetch_add(1) + 1;
            if (dropped == 1 || dropped % 300 == 0) {
                FITRA_LOG_WARN("corpus: spool cap/free-space guard reached; recorder "
                               "dropped {} frames (inference continues)", dropped);
            }
            return;
        }
        auto& payload = payload_streams_.at(item.camera_index);
        const std::string payload_name = "cam" + std::to_string(item.camera_index)
            + (cameras_[item.camera_index].pixel_format == camera::PixFmt::Yuyv
                   ? ".yuyv" : ".mjpeg");
        if (!payload.is_open()) {
            payload.open(segment_partial_path_ / payload_name,
                         std::ios::binary | std::ios::trunc);
        }
        if (!payload) {
            spool_bytes_.fetch_sub(bytes);
            estimated_available_bytes_ += bytes;
            throw std::runtime_error("cannot open " + payload_name);
        }
        payload.write(reinterpret_cast<const char*>(item.payload.data()),
                      static_cast<std::streamsize>(item.payload.size()));
        if (!payload) {
            spool_bytes_.fetch_sub(bytes);
            estimated_available_bytes_ += bytes;
            throw std::runtime_error("cannot write " + payload_name);
        }
        session_accounted_bytes_ += bytes;

        const auto& ts = item.v4l2_timestamp;
        frames_stream_
            << "{\"camera\":" << item.camera_index
            << ",\"seq\":\"" << item.seq << "\""
            << ",\"v4l2_sequence\":" << item.v4l2_sequence
            << ",\"payload\":\"" << payload_name << "\""
            << ",\"offset\":\"" << payload_offsets_[item.camera_index] << "\""
            << ",\"size\":" << bytes
            << ",\"captured_mono_ns\":\"" << item.captured_mono_ns << "\""
            << ",\"v4l2_mono_ns\":";
        if (ts.mono_ns) frames_stream_ << "\"" << *ts.mono_ns << "\"";
        else frames_stream_ << "null";
        frames_stream_
            << ",\"v4l2_semantics\":\""
            << camera::v4l2_timestamp_semantics_name(ts.semantics) << "\""
            << ",\"accepted_wall_unix_ns\":\""
            << item.accepted_wall_unix_ns << "\"}\n";
        if (!frames_stream_) throw std::runtime_error("cannot write frames.jsonl");

        payload_offsets_[item.camera_index] += bytes;
        ++segment_frame_counts_[item.camera_index];
        ++total_frame_counts_[item.camera_index];
        written_frames_.fetch_add(1);
    }

    void close_segment() {
        if (!segment_open_) return;
        frames_stream_.close();
        for (std::size_t i = 0; i < payload_streams_.size(); ++i) {
            auto& stream = payload_streams_[i];
            if (stream.is_open()) stream.close();
            if (!stream) {
                throw std::runtime_error(
                    "cannot close payload for camera " + std::to_string(i));
            }
        }
        if (!frames_stream_) throw std::runtime_error("cannot close frames.jsonl");

        std::ostringstream manifest;
        manifest << "{\n  \"schema\": \"" << kSchema << "\",\n"
                 << "  \"segment_index\": " << segment_index_ << ",\n"
                 << "  \"frame_counts\": [";
        for (std::size_t i = 0; i < segment_frame_counts_.size(); ++i) {
            if (i) manifest << ',';
            manifest << segment_frame_counts_[i];
        }
        manifest << "]\n}\n";
        write_text(segment_partial_path_ / "segment.json", manifest.str());

        std::vector<fs::path> files;
        for (const auto& path : regular_files_sorted(segment_partial_path_)) {
            if (path.filename() != "checksums.sha256") files.push_back(path);
        }
        std::ostringstream checksums;
        for (const auto& path : files) {
            checksums << util::sha256_file(path) << "  "
                      << path.filename().string() << '\n';
        }
        write_text(segment_partial_path_ / "checksums.sha256", checksums.str());
        std::error_code ec;
        fs::rename(segment_partial_path_, segment_final_path_, ec);
        if (ec) throw std::runtime_error("cannot commit segment: " + ec.message());
        segment_open_ = false;
    }

    std::string session_manifest(const std::string& close_reason) const {
        std::ostringstream out;
        out << "{\n"
            << "  \"schema\": \"" << kSchema << "\",\n"
            << "  \"session_id\": \"" << json_escape(session_id_) << "\",\n"
            << "  \"trigger\": \"any_inferred_person\",\n"
            << "  \"started_wall_unix_ns\": \"" << session_started_unix_ns_ << "\",\n"
            << "  \"ended_wall_unix_ns\": \"" << unix_now_ns() << "\",\n"
            << "  \"trigger_capture_mono_ns\": \"" << trigger_capture_mono_ns_ << "\",\n"
            << "  \"close_reason\": \"" << json_escape(close_reason) << "\",\n"
            << "  \"pre_roll_ms\": " << options_.pre_roll.count() << ",\n"
            << "  \"post_roll_ms\": " << options_.post_roll.count() << ",\n"
            << "  \"segment_ms\": " << options_.segment.count() << ",\n"
            << "  \"cameras\": [\n";
        for (std::size_t i = 0; i < cameras_.size(); ++i) {
            const auto& c = cameras_[i];
            out << "    {\"index\":" << c.index
                << ",\"config_slot\":" << c.config_slot
                << ",\"device\":\"" << json_escape(c.device_path) << "\""
                << ",\"capture_width\":" << c.capture_width
                << ",\"capture_height\":" << c.capture_height
                << ",\"output_width\":" << c.output_width
                << ",\"output_height\":" << c.output_height
                << ",\"requested_fps\":" << c.requested_fps
                << ",\"pixel_format\":\"" << pixel_format_name(c.pixel_format) << "\""
                << ",\"raw_encoding\":\"" << raw_encoding_name(c.pixel_format) << "\""
                << ",\"bytes_per_line\":" << c.bytes_per_line
                << ",\"size_image\":" << c.size_image
                << ",\"v4l2_fourcc\":\""
                << json_escape(fourcc_name(c.pixel_format_fourcc)) << "\""
                << ",\"v4l2_fourcc_u32\":" << c.pixel_format_fourcc
                << ",\"frame_interval_numerator\":"
                << c.frame_interval_numerator
                << ",\"frame_interval_denominator\":"
                << c.frame_interval_denominator
                << ",\"frame_count\":" << total_frame_counts_[i] << '}';
            if (i + 1 != cameras_.size()) out << ',';
            out << '\n';
        }
        out << "  ],\n"
            << "  \"recorder_drops\": {\"queue\":"
            << queue_dropped_frames_.load() << ",\"storage\":"
            << storage_dropped_frames_.load() << "}\n"
            << "}\n";
        return out.str();
    }

    void account_session_metadata() {
        const auto actual = directory_bytes(session_path_);
        if (actual > session_accounted_bytes_) {
            spool_bytes_.fetch_add(actual - session_accounted_bytes_);
        } else if (session_accounted_bytes_ > actual) {
            const auto difference = session_accounted_bytes_ - actual;
            auto current = spool_bytes_.load();
            while (!spool_bytes_.compare_exchange_weak(
                current, current > difference ? current - difference : 0)) {}
        }
        session_accounted_bytes_ = actual;
    }

    void reset_session_state() {
        segment_open_ = false;
        post_deadline_.reset();
        payload_streams_.clear();
        frames_stream_ = std::ofstream{};
        session_path_.clear();
        session_id_.clear();
        recording_.store(false, std::memory_order_release);
        upload_cv_.notify_one();
    }

    void close_event(const std::string& close_reason) {
        if (!recording_.load()) return;
        close_segment();
        write_text(session_path_ / "manifest.json", session_manifest(close_reason));
        account_session_metadata();
        const fs::path destination = unique_destination(ready_root() / session_id_);
        std::error_code ec;
        fs::rename(session_path_, destination, ec);
        if (ec) throw std::runtime_error("cannot commit session: " + ec.message());
        ready_sessions_.fetch_add(1);
        std::uint64_t session_frames = 0;
        for (const auto count : total_frame_counts_) session_frames += count;
        FITRA_LOG_INFO("corpus: event ready {} frames={}",
                       destination.string(), session_frames);
        reset_session_state();
    }

    void abandon_active_session(const std::string& reason) noexcept {
        if (!recording_.load()) return;
        try {
            if (frames_stream_.is_open()) frames_stream_.close();
            for (auto& stream : payload_streams_) {
                if (stream.is_open()) stream.close();
            }
            write_text(session_path_ / "RECOVERED_INCOMPLETE.txt",
                       "Recorder abandoned this session after: " + reason + "\n");
            account_session_metadata();
            const fs::path destination = unique_destination(
                ready_root() / (session_id_ + "-recovered-incomplete"));
            fs::rename(session_path_, destination);
            ready_sessions_.fetch_add(1);
        } catch (const std::exception& e) {
            FITRA_LOG_ERROR("corpus: cannot preserve incomplete session: {}", e.what());
        }
        reset_session_state();
    }

    bool upload_eligible() const {
        if (presence_mask_.load(std::memory_order_acquire) != 0
            || recording_.load(std::memory_order_acquire)) return false;
        const auto since = absent_since_ns_.load(std::memory_order_acquire);
        if (since == 0) return false;
        const std::uint64_t all_cameras = (1ULL << cameras_.size()) - 1ULL;
        if (presence_observed_mask_.load(std::memory_order_acquire) != all_cameras
            && steady_now_ns() < upload_not_before_ns_.load(std::memory_order_acquire)) {
            return false;
        }
        const auto dwell_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                options_.upload_absent).count());
        return steady_now_ns() - since >= dwell_ns;
    }

    bool keep_uploading() const {
        return !upload_stop_.load(std::memory_order_relaxed)
            && presence_mask_.load(std::memory_order_acquire) == 0
            && !recording_.load(std::memory_order_acquire);
    }

    enum class UploadResult { None, Complete, Paused };

    UploadResult upload_one_ready() {
        if (!upload_eligible()) return UploadResult::None;
        std::error_code ec;
        const fs::path marker = options_.remote_dir / kRemoteMarker;
        if (!fs::is_regular_file(marker, ec) || ec) {
            return UploadResult::None;
        }
        fs::create_directories(options_.remote_dir / ".partial", ec);
        if (ec) throw std::runtime_error("cannot create remote .partial/: " + ec.message());
        fs::create_directories(options_.remote_dir / "ready", ec);
        if (ec) throw std::runtime_error("cannot create remote ready/: " + ec.message());

        std::vector<fs::path> sessions;
        for (fs::directory_iterator it(ready_root(), ec), end; !ec && it != end;
             it.increment(ec)) {
            if (it->is_directory()) sessions.push_back(it->path());
        }
        if (ec) throw std::runtime_error("cannot scan local ready/: " + ec.message());
        if (sessions.empty()) return UploadResult::None;
        std::sort(sessions.begin(), sessions.end());
        const fs::path source = sessions.front();
        const auto name = source.filename();
        const fs::path partial = options_.remote_dir / ".partial" / name;
        const fs::path committed = options_.remote_dir / "ready" / name;

        if (fs::exists(committed, ec) && !ec) {
            const auto equal = tree_equal(source, committed,
                                          [this] { return keep_uploading(); });
            if (!equal) return UploadResult::Paused;
            if (!*equal) {
                throw std::runtime_error("remote session collision: " + committed.string());
            }
        } else {
            fs::create_directories(partial, ec);
            if (ec) throw std::runtime_error("cannot create remote session: " + ec.message());
            const auto keep = [this]() { return keep_uploading(); };
            for (const auto& src : regular_files_sorted(source)) {
                const auto rel = src.lexically_relative(source);
                if (copy_file_verified(src, partial / rel, keep) == CopyResult::Paused) {
                    return UploadResult::Paused;
                }
            }
            if (!keep_uploading()) return UploadResult::Paused;
            const auto partial_equal = tree_equal(source, partial, keep);
            if (!partial_equal) return UploadResult::Paused;
            if (!*partial_equal) {
                throw std::runtime_error("remote tree verification failed: " + partial.string());
            }
            fs::rename(partial, committed, ec);
            if (ec) throw std::runtime_error("cannot commit remote session: " + ec.message());
        }
        const auto committed_equal = tree_equal(
            source, committed, [this] { return keep_uploading(); });
        if (!committed_equal) return UploadResult::Paused;
        if (!*committed_equal) {
            throw std::runtime_error("committed remote tree verification failed: "
                                     + committed.string());
        }

        const auto local_bytes = directory_bytes(source);
        if (options_.remove_after_upload) {
            const auto removed = fs::remove_all(source, ec);
            if (ec || removed == 0) {
                throw std::runtime_error("remote committed but local cleanup failed: "
                                         + ec.message());
            }
            auto current = spool_bytes_.load();
            while (!spool_bytes_.compare_exchange_weak(
                current, current > local_bytes ? current - local_bytes : 0)) {}
        } else {
            fs::path local_done = unique_destination(uploaded_root() / name);
            fs::rename(source, local_done, ec);
            if (ec) throw std::runtime_error("cannot mark local session uploaded: "
                                             + ec.message());
        }
        uploaded_sessions_.fetch_add(1);
        FITRA_LOG_INFO("corpus: uploaded {} -> {}{}", source.string(),
                       committed.string(),
                       options_.remove_after_upload ? " (local removed)" : "");
        return UploadResult::Complete;
    }

    void uploader_loop() noexcept {
        bool marker_warned = false;
        while (!upload_stop_.load(std::memory_order_relaxed)) {
            {
                std::unique_lock<std::mutex> lock(upload_mu_);
                upload_cv_.wait_for(lock, std::chrono::milliseconds(500), [&]() {
                    return upload_stop_.load(std::memory_order_relaxed);
                });
            }
            if (upload_stop_.load(std::memory_order_relaxed)) break;
            std::error_code marker_ec;
            if (!fs::is_regular_file(options_.remote_dir / kRemoteMarker, marker_ec)) {
                if (!marker_warned) {
                    FITRA_LOG_WARN(
                        "corpus: remote paused; marker {} is absent (prevents writes to an unmounted NAS)",
                        (options_.remote_dir / kRemoteMarker).string());
                    marker_warned = true;
                }
                continue;
            }
            marker_warned = false;
            try {
                while (upload_eligible() && !upload_stop_.load()) {
                    const auto result = upload_one_ready();
                    if (result == UploadResult::None || result == UploadResult::Paused) break;
                }
            } catch (const std::exception& e) {
                upload_failures_.fetch_add(1);
                FITRA_LOG_WARN("corpus: upload failed; local copy retained: {}", e.what());
                std::unique_lock<std::mutex> lock(upload_mu_);
                upload_cv_.wait_for(lock, std::chrono::seconds(5));
            }
        }
    }

    RecorderOptions options_;
    std::vector<CameraSpec> cameras_;

    std::atomic<bool> started_{false};
    std::atomic<bool> accepting_{false};
    std::atomic<bool> upload_stop_{false};
    std::thread writer_;
    std::thread uploader_;

    std::mutex queue_mu_;
    std::condition_variable queue_cv_;
    std::deque<QueueItem> queue_;
    std::atomic<std::size_t> queued_bytes_{0};
    std::atomic<std::uint64_t> inflight_copies_{0};

    std::mutex upload_mu_;
    std::condition_variable upload_cv_;
    std::mutex presence_state_mu_;
    std::atomic<std::uint64_t> presence_mask_{0};
    std::atomic<std::uint64_t> presence_observed_mask_{0};
    std::atomic<std::uint64_t> absent_since_ns_{0};
    std::atomic<std::uint64_t> upload_not_before_ns_{0};
    std::array<std::uint64_t, 64> last_presence_observed_ns_{};
    std::uint64_t writer_presence_mask_ = 0;

    std::deque<RawItem> ring_;
    std::size_t ring_bytes_ = 0;
    std::atomic<std::size_t> ring_bytes_stat_{0};
    std::optional<std::chrono::steady_clock::time_point> post_deadline_;
    std::chrono::steady_clock::time_point last_space_check_{};
    std::uintmax_t estimated_available_bytes_ = 0;
    bool space_known_ = false;

    std::string session_id_;
    fs::path session_path_;
    std::uint64_t session_counter_ = 0;
    std::uint64_t session_started_unix_ns_ = 0;
    std::uint64_t trigger_capture_mono_ns_ = 0;
    std::uintmax_t session_accounted_bytes_ = 0;
    std::size_t segment_index_ = 0;
    bool segment_open_ = false;
    fs::path segment_partial_path_;
    fs::path segment_final_path_;
    std::chrono::steady_clock::time_point segment_first_at_{};
    std::ofstream frames_stream_;
    std::vector<std::ofstream> payload_streams_;
    std::vector<std::uint64_t> payload_offsets_;
    std::vector<std::uint64_t> segment_frame_counts_;
    std::vector<std::uint64_t> total_frame_counts_;

    std::atomic<std::uint64_t> accepted_frames_{0};
    std::atomic<std::uint64_t> queue_dropped_frames_{0};
    std::atomic<std::uint64_t> storage_dropped_frames_{0};
    std::atomic<std::uint64_t> written_frames_{0};
    std::atomic<std::uint64_t> ready_sessions_{0};
    std::atomic<std::uint64_t> uploaded_sessions_{0};
    std::atomic<std::uint64_t> upload_failures_{0};
    std::atomic<std::uintmax_t> spool_bytes_{0};
    std::atomic<bool> recording_{false};
};

EventCorpusRecorder::EventCorpusRecorder(RecorderOptions options,
                                         std::vector<CameraSpec> cameras)
    : impl_{std::make_unique<Impl>(std::move(options), std::move(cameras))} {}

EventCorpusRecorder::~EventCorpusRecorder() = default;

void EventCorpusRecorder::start() { impl_->start(); }
void EventCorpusRecorder::stop() noexcept { impl_->stop(); }

void EventCorpusRecorder::ingest_raw(std::size_t camera_index,
                                     const camera::Frame& frame) noexcept {
    impl_->ingest_raw(camera_index, frame);
}

void EventCorpusRecorder::observe_presence(std::size_t camera_index,
                                           bool present,
                                           std::chrono::steady_clock::time_point
                                               captured_at,
                                           std::uint64_t captured_mono_ns) noexcept {
    impl_->observe_presence(camera_index, present, captured_at,
                            captured_mono_ns);
}

RecorderStats EventCorpusRecorder::stats() const noexcept { return impl_->stats(); }

}  // namespace fitra::corpus
