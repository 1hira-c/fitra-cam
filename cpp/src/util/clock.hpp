#pragma once

#include <cstdint>
#include <ctime>

namespace fitra::util {

// Linux CLOCK_MONOTONIC nanoseconds. This is the content-time clock used by
// fusion-facing observations; it must not be replaced with wall-clock time.
inline std::uint64_t monotonic_ns() noexcept {
    ::timespec ts{};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ULL
         + static_cast<std::uint64_t>(ts.tv_nsec);
}

}  // namespace fitra::util
