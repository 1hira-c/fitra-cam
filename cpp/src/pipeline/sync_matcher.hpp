#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace fitra::pipeline {

// A synchronization boundary is deliberately distinct from an individual
// unmatched poll. The latter is just a short wait for a better cross-camera
// combination; only the former must invalidate a downstream track.
enum class SynchronizedFrameEventKind {
    None,
    Matched,
    UnavailableBoundary,
};

template <typename T>
struct SynchronizedFrameEvent {
    SynchronizedFrameEventKind kind = SynchronizedFrameEventKind::None;
    std::vector<T> frames;
    std::optional<double> sync_dt_ms;
};

// Bounded, latest-oriented cross-camera matcher. The camera capture queues
// remain latest-frame-wins; this small queue exists only after inference so a
// central loop can choose adjacent frames from different cameras instead of
// comparing the same frame index. It also owns the one-shot loss boundary
// state, which prevents a temporary skew from becoming a rapid
// Unavailable/Reacquired sequence.
template <typename T>
class SynchronizedFrameQueue {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    explicit SynchronizedFrameQueue(std::size_t camera_count,
                                    std::size_t queue_depth = 6)
        : queues_(camera_count),
          queue_depth_(std::max<std::size_t>(queue_depth, 1)) {}

    std::size_t camera_count() const { return queues_.size(); }

    void push(std::size_t camera_index, TimePoint captured_at, T value) {
        if (camera_index >= queues_.size()) return;
        auto& queue = queues_[camera_index];
        // A source-clock regression is not a valid continuation of the old
        // match stream. Drop that camera's stale candidates; the next complete
        // combination will be selected from the new clock segment.
        if (!queue.empty() && captured_at < queue.back().captured_at) {
            queue.clear();
        }
        queue.push_back(Entry{captured_at, std::move(value)});
        while (queue.size() > queue_depth_) queue.pop_front();
    }

    bool has_pending() const {
        return std::any_of(queues_.begin(), queues_.end(),
                           [](const auto& queue) { return !queue.empty(); });
    }

    // Lifecycle transitions must not allow frames from the previous phase to
    // participate in a new synchronized combination.
    void clear() {
        for (auto& queue : queues_) queue.clear();
        wait_started_.reset();
        unavailable_active_ = false;
    }

    // Poll after a new input and periodically while waiting. A successful
    // match consumes the selected frame and every older frame in each camera
    // queue. A loss boundary consumes all queued candidates so recovery cannot
    // combine a post-loss frame with a pre-loss frame.
    SynchronizedFrameEvent<T> poll(TimePoint now,
                                    double sync_window_ms,
                                    double loss_timeout_ms,
                                    std::optional<TimePoint> last_input_at =
                                        std::nullopt) {
        SynchronizedFrameEvent<T> event;
        const auto candidate = closest_candidate();
        const double window_ms = std::max(0.0, sync_window_ms);
        if (candidate && candidate->sync_dt_ms <= window_ms + 1.0e-6) {
            event.kind = SynchronizedFrameEventKind::Matched;
            event.sync_dt_ms = candidate->sync_dt_ms;
            event.frames.reserve(queues_.size());
            for (std::size_t camera = 0; camera < queues_.size(); ++camera) {
                auto& queue = queues_[camera];
                const std::size_t selected = candidate->queue_indices[camera];
                event.frames.push_back(std::move(queue[selected].value));
                queue.erase(queue.begin(), queue.begin()
                            + static_cast<std::ptrdiff_t>(selected + 1));
            }
            wait_started_.reset();
            unavailable_active_ = false;
            return event;
        }

        if (!has_pending()) {
            if (unavailable_active_) return event;
            // If every camera has stopped, there is no queued candidate to
            // start the normal mismatch timer. Use the last producer input as
            // the timer origin so a previously Fresh stream still fails
            // closed once, even when the queues are empty.
            if (!last_input_at) {
                wait_started_.reset();
                return event;
            }
            if (!wait_started_) wait_started_ = *last_input_at;
        } else if (!wait_started_) {
            wait_started_ = now;
        }

        const double timeout_ms = std::max(0.0, loss_timeout_ms);
        const double waited_ms = std::chrono::duration<double, std::milli>(
            now - *wait_started_).count();
        if (!unavailable_active_ && waited_ms >= timeout_ms) {
            event.kind = SynchronizedFrameEventKind::UnavailableBoundary;
            if (candidate) event.sync_dt_ms = candidate->sync_dt_ms;
            for (auto& queue : queues_) queue.clear();
            wait_started_.reset();
            unavailable_active_ = true;
        }
        return event;
    }

private:
    struct Entry {
        TimePoint captured_at;
        T value;
    };

    struct Candidate {
        std::vector<std::size_t> queue_indices;
        TimePoint oldest{};
        TimePoint newest{};
        double sync_dt_ms = 0.0;
    };

    std::optional<Candidate> closest_candidate() const {
        for (const auto& queue : queues_) {
            if (queue.empty()) return std::nullopt;
        }

        std::optional<Candidate> best;
        std::vector<std::size_t> chosen(queues_.size(), 0);
        std::function<void(std::size_t, TimePoint, TimePoint)> visit;
        visit = [&](std::size_t camera, TimePoint oldest, TimePoint newest) {
            if (camera == queues_.size()) {
                Candidate candidate;
                candidate.queue_indices = chosen;
                candidate.oldest = oldest;
                candidate.newest = newest;
                candidate.sync_dt_ms = std::chrono::duration<double, std::milli>(
                    newest - oldest).count();

                constexpr double kTieEpsilonMs = 1.0e-6;
                const bool better = !best
                    || candidate.sync_dt_ms < best->sync_dt_ms - kTieEpsilonMs
                    || (std::abs(candidate.sync_dt_ms - best->sync_dt_ms)
                            <= kTieEpsilonMs
                        && candidate.newest > best->newest);
                if (better) best = std::move(candidate);
                return;
            }

            for (std::size_t index = 0;
                 index < queues_[camera].size(); ++index) {
                chosen[camera] = index;
                const auto captured_at = queues_[camera][index].captured_at;
                if (camera == 0) {
                    visit(1, captured_at, captured_at);
                } else {
                    visit(camera + 1, std::min(oldest, captured_at),
                          std::max(newest, captured_at));
                }
            }
        };
        visit(0, TimePoint{}, TimePoint{});
        return best;
    }

    std::vector<std::deque<Entry>> queues_;
    std::size_t queue_depth_ = 6;
    std::optional<TimePoint> wait_started_;
    bool unavailable_active_ = false;
};

}  // namespace fitra::pipeline
