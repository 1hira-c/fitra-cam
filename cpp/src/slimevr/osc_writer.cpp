#include "slimevr/osc_writer.hpp"

#include <chrono>
#include <cstring>
#include <stdexcept>

namespace fitra::slimevr {

namespace {

constexpr std::uint8_t kBundleHeader[8] = {'#', 'b', 'u', 'n', 'd', 'l', 'e', '\0'};

// Number of zero bytes to append so the running buffer length lands on the
// next 4-byte boundary. Argument `n` is the position after the just-written
// content (typically buf.size()). OSC 1.0 pads to 4-byte multiples.
inline std::size_t pad4(std::size_t n) {
    return (4 - (n % 4)) % 4;
}

}  // namespace

OscWriter::OscWriter() = default;

void OscWriter::clear() {
    buf_.clear();
    bundle_open_ = false;
    msg_open_    = false;
    msg_address_.clear();
    msg_typetag_.clear();
    msg_args_.clear();
}

void OscWriter::append_be32(std::uint32_t v) {
    buf_.push_back(static_cast<std::uint8_t>((v >> 24) & 0xff));
    buf_.push_back(static_cast<std::uint8_t>((v >> 16) & 0xff));
    buf_.push_back(static_cast<std::uint8_t>((v >>  8) & 0xff));
    buf_.push_back(static_cast<std::uint8_t>( v        & 0xff));
}

void OscWriter::append_be64(std::uint64_t v) {
    append_be32(static_cast<std::uint32_t>(v >> 32));
    append_be32(static_cast<std::uint32_t>(v & 0xffffffffULL));
}

void OscWriter::append_bytes(const std::uint8_t* p, std::size_t n) {
    buf_.insert(buf_.end(), p, p + n);
}

void OscWriter::emit_osc_string(std::vector<std::uint8_t>& out, std::string_view s) {
    out.insert(out.end(), s.data(), s.data() + s.size());
    out.push_back('\0');
    std::size_t pad = pad4(s.size() + 1);
    for (std::size_t i = 0; i < pad; ++i) out.push_back('\0');
}

void OscWriter::begin_bundle(std::uint64_t osc_timetag_ntp) {
    if (bundle_open_) throw std::logic_error("OscWriter: nested bundle");
    if (msg_open_)    throw std::logic_error("OscWriter: bundle inside open message");
    append_bytes(kBundleHeader, sizeof(kBundleHeader));
    append_be64(osc_timetag_ntp);
    bundle_open_ = true;
}

void OscWriter::end_bundle() {
    if (!bundle_open_) throw std::logic_error("OscWriter: end_bundle without begin");
    if (msg_open_)     throw std::logic_error("OscWriter: end_bundle with open message");
    bundle_open_ = false;
}

void OscWriter::begin_message(std::string_view address) {
    if (msg_open_) throw std::logic_error("OscWriter: nested message");
    msg_open_ = true;
    msg_address_.assign(address.data(), address.size());
    msg_typetag_.clear();
    msg_args_.clear();
}

void OscWriter::add_int(std::int32_t v) {
    if (!msg_open_) throw std::logic_error("OscWriter: add_int outside message");
    msg_typetag_.push_back('i');
    std::uint32_t u = static_cast<std::uint32_t>(v);
    msg_args_.push_back(static_cast<std::uint8_t>((u >> 24) & 0xff));
    msg_args_.push_back(static_cast<std::uint8_t>((u >> 16) & 0xff));
    msg_args_.push_back(static_cast<std::uint8_t>((u >>  8) & 0xff));
    msg_args_.push_back(static_cast<std::uint8_t>( u        & 0xff));
}

void OscWriter::add_float(float v) {
    if (!msg_open_) throw std::logic_error("OscWriter: add_float outside message");
    msg_typetag_.push_back('f');
    std::uint32_t u;
    static_assert(sizeof(u) == sizeof(v));
    std::memcpy(&u, &v, sizeof(u));
    msg_args_.push_back(static_cast<std::uint8_t>((u >> 24) & 0xff));
    msg_args_.push_back(static_cast<std::uint8_t>((u >> 16) & 0xff));
    msg_args_.push_back(static_cast<std::uint8_t>((u >>  8) & 0xff));
    msg_args_.push_back(static_cast<std::uint8_t>( u        & 0xff));
}

void OscWriter::add_string(std::string_view s) {
    if (!msg_open_) throw std::logic_error("OscWriter: add_string outside message");
    msg_typetag_.push_back('s');
    emit_osc_string(msg_args_, s);
}

void OscWriter::end_message() {
    if (!msg_open_) throw std::logic_error("OscWriter: end_message without begin");

    // Assemble: address(padded) || ","+typetag(padded) || args(already padded).
    std::vector<std::uint8_t> body;
    body.reserve(msg_address_.size() + 4 + msg_typetag_.size() + 4 + msg_args_.size());
    emit_osc_string(body, msg_address_);

    // Build the ","+typetag string in a stack buffer to feed emit_osc_string.
    std::string tt;
    tt.reserve(1 + msg_typetag_.size());
    tt.push_back(',');
    tt.append(msg_typetag_);
    emit_osc_string(body, tt);

    body.insert(body.end(), msg_args_.begin(), msg_args_.end());

    if (bundle_open_) {
        append_be32(static_cast<std::uint32_t>(body.size()));
    }
    append_bytes(body.data(), body.size());

    msg_open_ = false;
    msg_address_.clear();
    msg_typetag_.clear();
    msg_args_.clear();
}

std::uint64_t OscWriter::ntp_timetag_now() {
    // OSC 1.0 timetag = (seconds since 1900-01-01 UTC) << 32 | NTP fraction.
    // Unix epoch (1970-01-01) is 2208988800 seconds after the NTP epoch.
    using clock = std::chrono::system_clock;
    auto since_epoch = clock::now().time_since_epoch();
    auto secs = std::chrono::duration_cast<std::chrono::seconds>(since_epoch).count();
    auto frac_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       since_epoch - std::chrono::seconds(secs)).count();
    std::uint64_t ntp_sec  = static_cast<std::uint64_t>(secs) + 2208988800ULL;
    // NTP fractional: nanoseconds * 2^32 / 1e9. Use 64-bit arithmetic.
    std::uint64_t ntp_frac = static_cast<std::uint64_t>(
        (static_cast<__uint128_t>(frac_ns) * (static_cast<__uint128_t>(1) << 32)) / 1000000000ULL);
    return (ntp_sec << 32) | (ntp_frac & 0xffffffffULL);
}

}  // namespace fitra::slimevr
