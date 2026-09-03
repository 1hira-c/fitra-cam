#include <algorithm>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "pipeline/sync_matcher.hpp"

namespace {

using Clock = fitra::pipeline::SynchronizedFrameQueue<int>::Clock;
using EventKind = fitra::pipeline::SynchronizedFrameEventKind;

void check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

Clock::time_point at_ms(int ms) {
    return Clock::time_point{std::chrono::milliseconds{ms}};
}

void test_phase_offset_uses_adjacent_frames() {
    for (const int phase_ms : {10, 25}) {
        fitra::pipeline::SynchronizedFrameQueue<int> queue{2, 6};
        int matched = 0;
        double maximum_dt = 0.0;
        auto consume = [&](const auto& event) {
            if (event.kind != EventKind::Matched) return;
            ++matched;
            maximum_dt = std::max(maximum_dt, event.sync_dt_ms.value_or(0.0));
        };
        for (int frame = 0; frame < 30; ++frame) {
            const int base_ms = frame * 33;
            queue.push(0, at_ms(base_ms), frame);
            auto event = queue.poll(at_ms(base_ms), 15.0, 100.0);
            consume(event);

            queue.push(1, at_ms(base_ms + phase_ms), frame);
            event = queue.poll(at_ms(base_ms + phase_ms), 15.0, 100.0);
            consume(event);
        }
        check(matched >= 29,
              "nearly all synthetic phase-offset frames must match");
        check(maximum_dt <= 15.0 + 1.0e-6,
              "accepted synthetic matches must stay inside sync_window_ms");
    }
}

void test_loss_emits_one_boundary_then_recovers() {
    fitra::pipeline::SynchronizedFrameQueue<int> queue{2, 6};
    queue.push(0, at_ms(0), 0);
    queue.push(1, at_ms(0), 0);
    auto event = queue.poll(at_ms(0), 15.0, 100.0);
    check(event.kind == EventKind::Matched, "initial pair must match");

    // Camera 1 stops. Polling the live camera alone must not emit a boundary
    // on each frame, nor may it emit a second boundary after the first one.
    for (int frame = 1; frame <= 3; ++frame) {
        const int now_ms = frame * 33;
        queue.push(0, at_ms(now_ms), frame);
        event = queue.poll(at_ms(now_ms), 15.0, 100.0);
        check(event.kind == EventKind::None,
              "short unmatched intervals must remain in the waiting state");
    }
    event = queue.poll(at_ms(140), 15.0, 100.0);
    check(event.kind == EventKind::UnavailableBoundary,
          "a sustained camera loss must emit one Unavailable boundary");
    event = queue.poll(at_ms(240), 15.0, 100.0);
    check(event.kind == EventKind::None,
          "the loss state must not repeat Unavailable every timer tick");

    queue.push(0, at_ms(300), 10);
    queue.push(1, at_ms(300), 10);
    event = queue.poll(at_ms(300), 15.0, 100.0);
    check(event.kind == EventKind::Matched,
          "a new complete pair must recover after the boundary");
}

void test_all_cameras_stopped_emits_one_boundary() {
    fitra::pipeline::SynchronizedFrameQueue<int> queue{2, 6};
    queue.push(0, at_ms(0), 0);
    queue.push(1, at_ms(0), 0);
    auto event = queue.poll(at_ms(0), 15.0, 100.0);
    check(event.kind == EventKind::Matched, "initial pair must match");

    event = queue.poll(at_ms(99), 15.0, 100.0, at_ms(0));
    check(event.kind == EventKind::None,
          "camera silence before timeout must not invalidate the stream");
    event = queue.poll(at_ms(100), 15.0, 100.0, at_ms(0));
    check(event.kind == EventKind::UnavailableBoundary,
          "all-camera silence must emit one Unavailable boundary");
    event = queue.poll(at_ms(200), 15.0, 100.0, at_ms(0));
    check(event.kind == EventKind::None,
          "all-camera silence must not repeat the boundary");
}

}  // namespace

int main() {
    try {
        test_phase_offset_uses_adjacent_frames();
        test_loss_emits_one_boundary_then_recovers();
        test_all_cameras_stopped_emits_one_boundary();
        std::puts("test_sync_matcher ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_sync_matcher failed: %s\n", e.what());
        return 1;
    }
}
