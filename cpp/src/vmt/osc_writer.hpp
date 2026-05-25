#pragma once
//
// Hand-rolled OSC 1.0 wire-format serializer (Phase 14 / VMT path).
//
// Restored from Phase 11 commit a64becf (originally under fitra::slimevr for
// the VMC publisher) and re-namespaced into fitra::vmt for the SteamVR-direct
// path. The VMC original was removed in commit 14ec5d4 when Phase 11
// switched to SlimeVR's native Firmware UDP protocol; Phase 14 needs OSC
// again for `/VMT/Room/Driver`, so we bring back the same serializer rather
// than re-deriving it.
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
//     w.begin_message("/VMT/Room/Driver");
//     w.add_int(0);  w.add_int(1);  w.add_float(0.0f);
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

namespace fitra::vmt {

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
    void append_be32(std::uint32_t v);
    void append_be64(std::uint64_t v);
    void append_bytes(const std::uint8_t* p, std::size_t n);
    static void emit_osc_string(std::vector<std::uint8_t>& out, std::string_view s);

    std::vector<std::uint8_t> buf_;
    bool                      bundle_open_   = false;
    bool                      msg_open_      = false;
    std::string               msg_address_;
    std::string               msg_typetag_;
    std::vector<std::uint8_t> msg_args_;
};

}  // namespace fitra::vmt
