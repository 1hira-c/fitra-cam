#pragma once

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "camera/v4l2_capture.hpp"

namespace fitra::corpus::detail {

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

struct PresenceReset {
    std::uint64_t presence_mask = 0;
    std::chrono::steady_clock::time_point captured_at{};
    std::uint64_t captured_mono_ns = 0;
};

// Serialized by the recorder's queue mutex. Payload reservations are owned by
// the recorder; this queue bounds control edges independently of payload bytes.
// Overflow collapses into one latest-state reset rather than dropping an edge
// and leaving the writer's person state permanently inconsistent.
class EventQueue {
public:
    static constexpr std::size_t kMaxPresenceEvents = 256;
    using Item = std::variant<RawItem, PresenceItem>;

    struct Taken {
        std::optional<Item> item;
        std::optional<PresenceReset> reset;
        std::size_t discarded_raw_bytes = 0;
        std::size_t discarded_raw_frames = 0;
        std::size_t discarded_presence_events = 0;
    };

    bool empty() const { return items_.empty() && !reset_; }
    std::size_t pending_presence_events() const { return presence_events_; }
    bool resync_pending() const { return reset_.has_value(); }

    void reset() {
        items_.clear();
        raw_bytes_ = raw_frames_ = presence_events_ = 0;
        reset_.reset();
        capture_watermark_.reset();
    }

    bool push_raw(RawItem&& item) {
        if (item.payload.empty() || reset_
            || (capture_watermark_ && item.captured_at <= *capture_watermark_)) {
            return false;
        }
        const auto bytes = item.payload.size();
        items_.emplace_back(std::move(item));
        raw_bytes_ += bytes;
        ++raw_frames_;
        return true;
    }

    bool push_presence(const PresenceItem& item, std::uint64_t latest_mask) noexcept {
        if (!reset_ && presence_events_ < kMaxPresenceEvents) {
            try {
                items_.emplace_back(item);
                ++presence_events_;
                return true;
            } catch (...) {
                // The allocation-free reset also covers a failed edge enqueue.
            }
        }
        const auto boundary = reset_ && reset_->captured_at > item.captured_at
            ? *reset_ : PresenceReset{latest_mask, item.captured_at, item.captured_mono_ns};
        reset_ = boundary;
        reset_->presence_mask = latest_mask;
        return false;
    }

    Taken take(std::deque<Item>& discarded) {
        assert(discarded.empty());
        Taken out;
        if (reset_) {
            out.reset = reset_;
            reset_.reset();
            if (!capture_watermark_ || *capture_watermark_ < out.reset->captured_at) {
                capture_watermark_ = out.reset->captured_at;
            }
            out.discarded_raw_bytes = raw_bytes_;
            out.discarded_raw_frames = raw_frames_;
            out.discarded_presence_events = presence_events_;
            discarded.swap(items_);
            raw_bytes_ = raw_frames_ = presence_events_ = 0;
        } else if (!items_.empty()) {
            out.item.emplace(std::move(items_.front()));
            items_.pop_front();
            if (const auto* raw = std::get_if<RawItem>(&*out.item)) {
                raw_bytes_ -= raw->payload.size();
                --raw_frames_;
            } else {
                --presence_events_;
            }
        }
        return out;
    }

private:
    std::deque<Item> items_;
    std::size_t presence_events_ = 0;
    std::size_t raw_bytes_ = 0;
    std::size_t raw_frames_ = 0;
    std::optional<PresenceReset> reset_;
    std::optional<std::chrono::steady_clock::time_point> capture_watermark_;
};

}  // namespace fitra::corpus::detail
