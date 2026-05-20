#pragma once
//
// Hand-rolled OSC 1.0 wire-format serializer.
//
// Built for the Phase 11 SlimeVR/VMC publisher: produces a single UDP datagram
// per cycle containing a `#bundle` of N messages (one `/VMC/Ext/Tra/Pos` per
// tracker plus optional `/VMC/Ext/Root/Pos` and `/VMC/Ext/T`). The project
// never parses OSC on the hot path, so we deliberately avoid `oscpp`/`liblo`.
//
// Spec reference: Open Sound Control 1.0 (Stanford, 2002).
//   - Strings: NUL-terminated + zero-pad to a 4-byte boundary.
//   - int32, float32: 4-byte big-endian.
//   - Bundle: `#bundle\0` (8 bytes) + 64-bit OSC timetag (big-endian, NTP) +
//             per-element `<int32 length, bytes>` records.
//
// Usage:
//   OscWriter w;
//   w.clear();
//   w.begin_bundle(osc_timetag_ntp);
//     w.begin_message("/VMC/Ext/Tra/Pos");
//     w.add_string("waist");
//     w.add_float(0.0f); w.add_float(0.94f); w.add_float(-0.10f);
//     w.add_float(0); w.add_float(0); w.add_float(0); w.add_float(1);
//     w.end_message();
//   w.end_bundle();
//   ::sendto(fd, w.data().data(), w.data().size(), ...);

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace fitra::slimevr {

class OscWriter {
public:
    OscWriter();

    // Reset the buffer for the next datagram. Does NOT release the underlying
    // capacity (we reuse the same writer per publisher iteration).
    void clear();

    // Bundle markers: produce `#bundle\0` + 64-bit timetag, then per-element
    // length-prefixed records, then nothing at end_bundle (the spec lets the
    // bundle end implicitly at packet boundary). Bundles cannot be nested in
    // OSC 1.0; calling begin_bundle twice without an intervening clear() is
    // a logic error.
    void begin_bundle(std::uint64_t osc_timetag_ntp);
    void end_bundle();

    // Message construction. begin_message captures the address; add_int /
    // add_float / add_string append a typetag character and the argument
    // payload into a scratch area. end_message assembles `address_padded ||
    // ","+typetag_padded || args` and either appends it directly to the
    // packet (standalone message) or prepends a 4-byte big-endian length and
    // appends inside the current bundle.
    void begin_message(std::string_view address);
    void add_int(std::int32_t v);
    void add_float(float v);
    void add_string(std::string_view s);
    void end_message();

    // Raw bytes of the assembled packet. Valid until the next clear() /
    // begin_*. The span is over the internal vector storage so it does not
    // outlive a subsequent append that resizes the buffer.
    std::span<const std::uint8_t> data() const {
        return std::span<const std::uint8_t>{buf_.data(), buf_.size()};
    }

    // OSC 1.0 timetag (seconds since 1900-01-01 UTC in the top 32 bits, NTP
    // fractional seconds in the bottom 32). Helper for callers that want a
    // "send now" timetag derived from the system clock.
    static std::uint64_t ntp_timetag_now();

private:
    // Output helpers: write directly into buf_.
    void append_be32(std::uint32_t v);
    void append_be64(std::uint64_t v);
    void append_bytes(const std::uint8_t* p, std::size_t n);
    // OSC string: bytes of `s`, then a NUL, then zero-pad to a 4-byte boundary.
    static void emit_osc_string(std::vector<std::uint8_t>& out, std::string_view s);

    std::vector<std::uint8_t> buf_;
    bool                      bundle_open_   = false;
    bool                      msg_open_      = false;
    std::string               msg_address_;
    std::string               msg_typetag_;  // accumulated chars without the leading ','
    std::vector<std::uint8_t> msg_args_;     // already padded per-argument
};

}  // namespace fitra::slimevr
