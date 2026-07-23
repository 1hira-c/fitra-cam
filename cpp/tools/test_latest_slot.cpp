#include "camera/latest_slot.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

namespace {

using fitra::camera::FrameReadySignal;
using fitra::camera::LatestSlot;

int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++g_fail; } \
} while (0)

struct Payload {
    std::uint64_t seq = 0;
    std::vector<std::uint8_t> bytes;
    std::unique_ptr<int> move_only;
};

void test_pop_moves_large_payload() {
    LatestSlot<Payload> slot;
    Payload in;
    in.seq = 7;
    in.bytes.resize(3 * 256 * 192 * sizeof(float), 0x5a);
    in.move_only = std::make_unique<int>(42);
    const auto* storage = in.bytes.data();

    slot.publish(std::move(in));
    Payload out;
    CHECK(slot.try_pop(out));
    CHECK(out.seq == 7);
    CHECK(out.bytes.data() == storage);
    CHECK(out.move_only && *out.move_only == 42);
    CHECK(!slot.try_pop(out));
}

void test_latest_value_wins() {
    LatestSlot<Payload> slot;
    Payload first;
    first.seq = 1;
    slot.publish(std::move(first));
    Payload second;
    second.seq = 2;
    slot.publish(std::move(second));

    Payload out;
    CHECK(slot.try_pop(out));
    CHECK(out.seq == 2);
}

void test_exchange_recycles_consumer_storage() {
    LatestSlot<Payload> slot;
    Payload producer;
    producer.seq = 1;
    producer.bytes.resize(1024, 0x11);
    slot.publish_exchange(producer);

    Payload consumer;
    CHECK(slot.try_pop(consumer));
    const auto* first_storage = consumer.bytes.data();

    producer.seq = 2;
    producer.bytes.resize(2048, 0x22);
    slot.publish_exchange(producer);
    CHECK(slot.try_pop(consumer));
    // The second pop returns the new payload and leaves the first consumer
    // buffer in the slot. The next exchange hands that buffer to the producer.
    producer.seq = 3;
    slot.publish_exchange(producer);
    CHECK(producer.bytes.data() == first_storage);
}

void test_aggregate_signal_cannot_lose_scan_race() {
    FrameReadySignal signal;
    LatestSlot<Payload> slot{&signal};
    std::atomic<bool> stop{false};

    // This models: ticket -> scan finds empty -> producer publishes -> wait.
    // wait() must observe the changed generation without needing another edge.
    Payload in;
    in.seq = 9;
    slot.publish(std::move(in));
    auto ticket = signal.wait(0, stop, std::chrono::milliseconds(0));
    CHECK(ticket != 0);

    Payload out;
    CHECK(slot.try_pop(out));
    CHECK(out.seq == 9);

    stop.store(true);
    CHECK(signal.wait(ticket, stop, std::chrono::milliseconds(0)) == ticket);
}

void test_stop_wake_releases_waiter() {
    FrameReadySignal signal;
    std::atomic<bool> stop{false};
    std::atomic<bool> entered{false};
    std::atomic<bool> returned{false};
    std::thread waiter([&] {
        entered.store(true, std::memory_order_release);
        signal.wait(0, stop, std::chrono::seconds(5));
        returned.store(true, std::memory_order_release);
    });
    while (!entered.load(std::memory_order_acquire)) std::this_thread::yield();
    stop.store(true, std::memory_order_relaxed);
    signal.wake();
    waiter.join();
    CHECK(returned.load(std::memory_order_acquire));
}

}  // namespace

int main() {
    test_pop_moves_large_payload();
    test_latest_value_wins();
    test_exchange_recycles_consumer_storage();
    test_aggregate_signal_cannot_lose_scan_race();
    test_stop_wake_releases_waiter();
    if (g_fail) {
        std::fprintf(stderr, "test_latest_slot: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_latest_slot: OK\n");
    return 0;
}
