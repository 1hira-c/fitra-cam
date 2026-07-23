// Synthetic 3-camera readiness benchmark. A producer publishes round-robin at
// 180 frames/s (3 x 60), while the consumer either polls every 2ms (old path)
// or waits on FrameReadySignal (current path). No camera/GPU is required.

#include "camera/latest_slot.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <memory>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

struct Event {
    std::uint64_t seq = 0;
    Clock::time_point published_at{};
};

struct Result {
    std::size_t processed = 0;
    std::size_t scans = 0;
    double cpu_ms = 0.0;
    double avg_latency_ms = 0.0;
    double p95_latency_ms = 0.0;
};

double thread_cpu_ms() {
    timespec ts{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return static_cast<double>(ts.tv_sec) * 1000.0
         + static_cast<double>(ts.tv_nsec) / 1.0e6;
}

Result run(bool event_driven, int frames) {
    fitra::camera::FrameReadySignal signal;
    std::array<std::unique_ptr<fitra::camera::LatestSlot<Event>>, 3> slots;
    for (auto& slot : slots) {
        slot = std::make_unique<fitra::camera::LatestSlot<Event>>(
            event_driven ? &signal : nullptr);
    }

    std::atomic<bool> done{false};
    const auto period = std::chrono::duration<double>(1.0 / 180.0);
    std::thread producer([&] {
        const auto start = Clock::now() + std::chrono::milliseconds(10);
        for (int i = 0; i < frames; ++i) {
            std::this_thread::sleep_until(start +
                std::chrono::duration_cast<Clock::duration>(period * i));
            Event ev;
            ev.seq = static_cast<std::uint64_t>(i + 1);
            ev.published_at = Clock::now();
            slots[static_cast<std::size_t>(i) % slots.size()]->publish(std::move(ev));
        }
        done.store(true, std::memory_order_relaxed);
        signal.wake();
    });

    std::vector<double> latencies;
    latencies.reserve(static_cast<std::size_t>(frames));
    std::size_t scans = 0;
    fitra::camera::FrameReadySignal::Ticket ticket = 0;
    const double cpu_begin = thread_cpu_ms();
    for (;;) {
        if (event_driven && !done.load(std::memory_order_relaxed)) {
            ticket = signal.wait(ticket, done, std::chrono::milliseconds(100));
        }
        bool found = false;
        Event ev;
        for (auto& slot : slots) {
            ++scans;
            if (!slot->try_pop(ev)) continue;
            found = true;
            latencies.push_back(
                std::chrono::duration<double, std::milli>(Clock::now() - ev.published_at).count());
        }
        if (done.load(std::memory_order_relaxed)) {
            bool remaining = false;
            for (auto& slot : slots) {
                ++scans;
                if (slot->try_pop(ev)) {
                    remaining = true;
                    latencies.push_back(
                        std::chrono::duration<double, std::milli>(Clock::now() - ev.published_at).count());
                }
            }
            if (!remaining) break;
            continue;
        }
        if (found) continue;
        if (!event_driven) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    const double cpu_end = thread_cpu_ms();
    producer.join();

    Result out;
    out.processed = latencies.size();
    out.scans = scans;
    out.cpu_ms = cpu_end - cpu_begin;
    if (!latencies.empty()) {
        for (double v : latencies) out.avg_latency_ms += v;
        out.avg_latency_ms /= static_cast<double>(latencies.size());
        std::sort(latencies.begin(), latencies.end());
        const std::size_t p95 = static_cast<std::size_t>(
            std::ceil(0.95 * static_cast<double>(latencies.size()))) - 1;
        out.p95_latency_ms = latencies[std::min(p95, latencies.size() - 1)];
    }
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    const int frames = argc > 1 ? std::max(180, std::atoi(argv[1])) : 720;
    const auto polling = run(false, frames);
    const auto signaled = run(true, frames);

    std::printf("synthetic load: 3 cameras x 60fps, %d aggregate frames\n", frames);
    std::printf("2ms polling: processed=%zu scans=%zu cpu=%.3fms avg_latency=%.3fms p95=%.3fms\n",
                polling.processed, polling.scans, polling.cpu_ms,
                polling.avg_latency_ms, polling.p95_latency_ms);
    std::printf("shared CV:   processed=%zu scans=%zu cpu=%.3fms avg_latency=%.3fms p95=%.3fms\n",
                signaled.processed, signaled.scans, signaled.cpu_ms,
                signaled.avg_latency_ms, signaled.p95_latency_ms);
    std::printf("delta: cpu=%.1f%% latency(avg)=%.1f%% latency(p95)=%.1f%%\n",
                polling.cpu_ms > 0.0 ? 100.0 * (signaled.cpu_ms / polling.cpu_ms - 1.0) : 0.0,
                polling.avg_latency_ms > 0.0
                    ? 100.0 * (signaled.avg_latency_ms / polling.avg_latency_ms - 1.0) : 0.0,
                polling.p95_latency_ms > 0.0
                    ? 100.0 * (signaled.p95_latency_ms / polling.p95_latency_ms - 1.0) : 0.0);
    return signaled.processed == static_cast<std::size_t>(frames) ? 0 : 1;
}
