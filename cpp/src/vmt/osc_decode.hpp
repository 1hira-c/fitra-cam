#pragma once
//
// Minimal OSC 1.0 big-endian decode helpers shared by the HMD and controller
// pose receivers. Encoding lives in osc_writer.hpp; this is the read side.
//
// Pure, header-only, no I/O — usable from receive loops and unit tests alike.

#include <cstdint>
#include <cstring>
#include <string>

namespace fitra::vmt::osc {

// Read a big-endian uint32 from `p` (no bounds check; caller has confirmed
// at least 4 bytes remain).
inline std::uint32_t read_be32(const std::uint8_t* p) {
    return (static_cast<std::uint32_t>(p[0]) << 24)
         | (static_cast<std::uint32_t>(p[1]) << 16)
         | (static_cast<std::uint32_t>(p[2]) <<  8)
         |  static_cast<std::uint32_t>(p[3]);
}

inline float read_be_float(const std::uint8_t* p) {
    std::uint32_t u = read_be32(p);
    float f;
    static_assert(sizeof(f) == sizeof(u));
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

// OSC 1.0 strings are null-terminated + zero-padded to a 4-byte boundary.
// Return the total byte count consumed (including padding) and copy the raw
// string content into `out`. Returns 0 on malformed input (no null before
// `len_remaining`, or padding overruns the buffer).
inline std::size_t consume_osc_string(const std::uint8_t* p,
                                      std::size_t len_remaining,
                                      std::string& out) {
    std::size_t nul = 0;
    while (nul < len_remaining && p[nul] != '\0') ++nul;
    if (nul == len_remaining) return 0;       // no null terminator
    out.assign(reinterpret_cast<const char*>(p), nul);
    std::size_t total = nul + 1;              // include the null
    std::size_t pad   = (4 - (total % 4)) % 4;
    if (total + pad > len_remaining) return 0;
    return total + pad;
}

}  // namespace fitra::vmt::osc
