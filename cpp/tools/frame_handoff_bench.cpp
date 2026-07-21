// Microbenchmark for the decoded-frame SPSC handoff. This intentionally
// excludes decode/preprocess work and isolates the old deep-copy cost versus
// LatestSlot's ownership exchange. Default payload is one RTMPose-M CHW input.

#include "camera/latest_slot.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

struct Payload {
    std::uint64_t seq = 0;
    std::vector<std::uint8_t> bytes;
};

double elapsed_ms(std::chrono::steady_clock::time_point begin,
                  std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

}  // namespace

int main(int argc, char** argv) {
    const int iterations = argc > 1 ? std::max(1, std::atoi(argv[1])) : 20000;
    const int persons    = argc > 2 ? std::max(1, std::atoi(argv[2])) : 1;
    const std::size_t payload_bytes =
        static_cast<std::size_t>(persons) * 3 * 256 * 192 * sizeof(float);
    volatile std::uint64_t checksum = 0;

    // Previous FrameSource handoff: out = *latest_ (vector deep copy). Keep
    // destination capacity warm so this measures memcpy, not allocator noise.
    Payload copy_src;
    Payload copy_dst;
    copy_src.bytes.resize(payload_bytes, 0x5a);
    copy_dst.bytes.reserve(payload_bytes);
    auto copy_begin = std::chrono::steady_clock::now();
    for (int i = 0; i < iterations; ++i) {
        copy_src.seq = static_cast<std::uint64_t>(i);
        copy_dst = copy_src;
        checksum = checksum + copy_dst.bytes[static_cast<std::size_t>(i) % payload_bytes];
    }
    auto copy_end = std::chrono::steady_clock::now();

    // Current handoff: producer publishes by exchange, consumer pops by
    // exchange, and old consumer storage returns to the producer. Warm the
    // three circulating objects before measuring steady-state ownership swaps.
    fitra::camera::LatestSlot<Payload> slot;
    Payload producer;
    Payload consumer;
    for (int i = 0; i < 4; ++i) {
        producer.bytes.resize(payload_bytes, 0x5a);
        producer.seq = static_cast<std::uint64_t>(i);
        slot.publish_exchange(producer);
        if (!slot.try_pop(consumer)) return 2;
    }
    auto move_begin = std::chrono::steady_clock::now();
    for (int i = 0; i < iterations; ++i) {
        producer.bytes.resize(payload_bytes);
        producer.seq = static_cast<std::uint64_t>(i);
        slot.publish_exchange(producer);
        if (!slot.try_pop(consumer)) return 2;
        checksum = checksum + consumer.bytes[static_cast<std::size_t>(i) % payload_bytes];
    }
    auto move_end = std::chrono::steady_clock::now();

    const double copy_ms = elapsed_ms(copy_begin, copy_end);
    const double move_ms = elapsed_ms(move_begin, move_end);
    const double mib = static_cast<double>(payload_bytes) / (1024.0 * 1024.0);
    std::printf("payload=%.3f MiB iterations=%d persons=%d\n", mib, iterations, persons);
    std::printf("deep-copy handoff: %.3f ms total, %.3f us/frame\n",
                copy_ms, copy_ms * 1000.0 / iterations);
    std::printf("exchange handoff:  %.3f ms total, %.3f us/frame\n",
                move_ms, move_ms * 1000.0 / iterations);
    std::printf("handoff speedup:   %.2fx (checksum=%llu)\n",
                move_ms > 0.0 ? copy_ms / move_ms : 0.0,
                static_cast<unsigned long long>(checksum));
    return 0;
}
