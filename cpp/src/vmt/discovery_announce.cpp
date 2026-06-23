#include "vmt/discovery_announce.hpp"

#include <cstdio>

#include <unistd.h>   // gethostname

#include "vmt/osc_decode.hpp"

namespace fitra::vmt {

using osc::consume_osc_string;
using osc::read_be32;

const char* discovery_role_name(DiscoveryRole r) {
    switch (r) {
        case DiscoveryRole::Jetson: return "jetson";
        case DiscoveryRole::Vmt:    return "vmt";
    }
    return "jetson";
}

bool parse_discovery_role(std::string_view s, DiscoveryRole& out) {
    if (s == "jetson") { out = DiscoveryRole::Jetson; return true; }
    if (s == "vmt")    { out = DiscoveryRole::Vmt;    return true; }
    return false;
}

void encode_announce(OscWriter& w, const Announce& a) {
    w.begin_message(kAnnounceAddress);
    w.add_string(discovery_role_name(a.role));   // s
    w.add_string(a.instance_id);                 // s
    w.add_string(a.instance_name);               // s
    w.add_int(a.proto_version);                  // i
    w.add_int(a.osc_recv_port);                  // i
    w.add_string(a.capabilities);                // s
    w.add_string(a.pairing_token);               // s
    w.end_message();
}

std::vector<std::uint8_t> encode_announce(const Announce& a) {
    OscWriter w;
    w.clear();
    encode_announce(w, a);
    auto sp = w.data();
    return std::vector<std::uint8_t>(sp.begin(), sp.end());
}

bool parse_announce(const std::uint8_t* data, std::size_t len, Announce& out) {
    if (!data || len < 8) return false;

    std::string addr;
    std::size_t off = consume_osc_string(data, len, addr);
    if (off == 0 || addr != kAnnounceAddress) return false;

    std::string typetag;
    std::size_t adv = consume_osc_string(data + off, len - off, typetag);
    if (adv == 0 || typetag != kAnnounceTypetag) return false;
    off += adv;

    // off <= len is maintained by consume_osc_string (it never returns a count
    // that overruns len_remaining), so `len - off` below never underflows.
    auto take_str = [&](std::string& dst) -> bool {
        std::size_t a = consume_osc_string(data + off, len - off, dst);
        if (a == 0) return false;
        off += a;
        return true;
    };

    std::string role_s;
    if (!take_str(role_s)) return false;
    if (!parse_discovery_role(role_s, out.role)) return false;
    if (!take_str(out.instance_id)) return false;
    if (!take_str(out.instance_name)) return false;

    if (len - off < 8) return false;
    out.proto_version = static_cast<std::int32_t>(read_be32(data + off)); off += 4;
    out.osc_recv_port = static_cast<std::int32_t>(read_be32(data + off)); off += 4;

    if (!take_str(out.capabilities)) return false;
    if (!take_str(out.pairing_token)) return false;

    return true;
}

std::string stable_instance_id_from(std::string_view hostname) {
    if (hostname.empty()) return "fitra-unknown";
    constexpr std::uint64_t kFnvOffset = 1469598103934665603ULL;
    constexpr std::uint64_t kFnvPrime  = 1099511628211ULL;
    std::uint64_t h = kFnvOffset;
    for (unsigned char c : hostname) {
        h ^= static_cast<std::uint64_t>(c);
        h *= kFnvPrime;
    }
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx",
                  static_cast<unsigned long long>(h));
    return std::string(buf);
}

std::string stable_instance_id() {
    char host[256];
    if (::gethostname(host, sizeof(host)) != 0) {
        return stable_instance_id_from(std::string_view{});
    }
    host[sizeof(host) - 1] = '\0';
    return stable_instance_id_from(host);
}

}  // namespace fitra::vmt
