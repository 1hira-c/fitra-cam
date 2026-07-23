#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

namespace fitra::camera {

// One shared edge counter lets a consumer wait for "any of N sources" without
// polling every source. The consumer carries the returned generation into its
// next wait. Notifications that arrive while it is busy make that next wait
// return immediately, so no edge is lost and active operation needs one slot
// scan per batch.
class FrameReadySignal {
public:
    using Ticket = std::uint64_t;

    void notify() {
        {
            std::lock_guard<std::mutex> lk{mu_};
            ++generation_;
        }
        cv_.notify_one();
    }

    Ticket wait(Ticket observed,
                const std::atomic<bool>& consumer_stop,
                std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lk{mu_};
        cv_.wait_for(lk, timeout, [&] {
            return consumer_stop.load(std::memory_order_relaxed)
                || generation_ != observed;
        });
        return generation_;
    }

    // The stop flag is external to this object, so take the same mutex as wait
    // before notifying to close the predicate-check -> park lost-wakeup window.
    void wake() {
        std::lock_guard<std::mutex> lk{mu_};
        cv_.notify_all();
    }

private:
    std::mutex              mu_;
    std::condition_variable cv_;
    Ticket                  generation_ = 0;
};

// Single-producer/single-consumer latest-value slot.  publish() overwrites an
// unconsumed value (latest-frame-wins); try_pop() transfers ownership instead
// of copying large frame/preprocess buffers into the consumer.
template <typename T>
class LatestSlot {
public:
    explicit LatestSlot(FrameReadySignal* aggregate_signal = nullptr)
        : aggregate_signal_{aggregate_signal} {}

    LatestSlot(const LatestSlot&) = delete;
    LatestSlot& operator=(const LatestSlot&) = delete;

    // Configure before producer/consumer threads start.
    void set_aggregate_signal(FrameReadySignal* signal) {
        aggregate_signal_ = signal;
    }

    void publish(T value) {
        {
            std::lock_guard<std::mutex> lk{mu_};
            if (latest_) {
                using std::swap;
                swap(*latest_, value);
            } else {
                latest_.emplace(std::move(value));
            }
            available_ = true;
        }
        cv_.notify_one();
        if (aggregate_signal_) aggregate_signal_->notify();
    }

    // Publish by exchange so the producer receives the consumer's previous
    // value back in `value`. Keeping that object across iterations recycles its
    // vector/cv::Mat capacities instead of allocating a new frame payload.
    void publish_exchange(T& value) {
        {
            std::lock_guard<std::mutex> lk{mu_};
            if (latest_) {
                using std::swap;
                swap(*latest_, value);
            } else {
                latest_.emplace(std::move(value));
            }
            available_ = true;
        }
        cv_.notify_one();
        if (aggregate_signal_) aggregate_signal_->notify();
    }

    bool try_pop(T& out) {
        std::lock_guard<std::mutex> lk{mu_};
        if (!available_) return false;
        using std::swap;
        swap(out, *latest_);
        available_ = false;
        return true;
    }

    bool wait_available(const std::atomic<bool>& consumer_stop,
                        std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lk{mu_};
        cv_.wait_for(lk, timeout, [&] {
            return consumer_stop.load(std::memory_order_relaxed)
                || available_;
        });
        return available_;
    }

    void wake() {
        std::lock_guard<std::mutex> lk{mu_};
        cv_.notify_all();
    }

private:
    std::mutex              mu_;
    std::condition_variable cv_;
    std::optional<T>        latest_;
    bool                    available_ = false;
    FrameReadySignal*       aggregate_signal_ = nullptr;
};

}  // namespace fitra::camera
