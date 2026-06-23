#pragma once
//
// VMT ⇔ Jetson zeroconf discovery: /fitra/announce OSC 1.0 wire codec.
//
// A pure control-plane message both peers broadcast at 1 Hz so each learns the
// other's pose-plane endpoint (src_ip : osc_recv_port). The pose wire itself
// (/VMT/Room/Driver 39570, /fitra/tracked_pose 39571, /fitra/punch) is
// completely unchanged — discovery only resolves the IP:port the existing
// senders aim at. See docs/design/vr-output-zeroconf-discovery.md.
//
//   address = "/fitra/announce"
//   typetag = ",sssiiss"
//   args    = role(s)           "jetson" | "vmt"
//             instance_id(s)    stable per-host id (hostname-derived hex)
//             instance_name(s)  human-readable label (UI display)
//             proto_version(i)  1
//             osc_recv_port(i)  this host's pose-plane receive port
//                               (jetson=39571, vmt=39570)
//             capabilities(s)   csv, e.g. "pose,hmd,controller"
//             pairing_token(s)  "" = any; non-empty = only same-token peers
//
// encode/parse reuse OscWriter / osc_decode. Pure (no sockets) — unit-testable.

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "vmt/osc_writer.hpp"

namespace fitra::vmt {

enum class DiscoveryRole : std::int32_t {
    Jetson = 0,
    Vmt    = 1,
};

inline constexpr int          kDiscoveryProtoVersion = 1;
inline constexpr const char*  kAnnounceAddress       = "/fitra/announce";
inline constexpr const char*  kAnnounceTypetag       = ",sssiiss";

// Default pose-plane receive ports advertised in osc_recv_port. Jetson listens
// for /fitra/tracked_pose on 39571; VMT listens for /VMT/Room/Driver on 39570.
inline constexpr std::uint16_t kJetsonOscRecvPort = 39571;
inline constexpr std::uint16_t kVmtOscRecvPort    = 39570;

const char* discovery_role_name(DiscoveryRole r);            // "jetson"|"vmt"
bool        parse_discovery_role(std::string_view s, DiscoveryRole& out);

struct Announce {
    DiscoveryRole role          = DiscoveryRole::Jetson;
    std::string   instance_id;
    std::string   instance_name;
    std::int32_t  proto_version = kDiscoveryProtoVersion;
    std::int32_t  osc_recv_port = 0;
    std::string   capabilities;
    std::string   pairing_token;
};

// Encode a standalone /fitra/announce OSC message (no bundle).
void encode_announce(OscWriter& w, const Announce& a);
std::vector<std::uint8_t> encode_announce(const Announce& a);

// Parse bytes -> Announce. Returns false on wrong address/typetag, short
// buffer, malformed strings, or unknown role. proto_version is parsed but NOT
// validated here (the caller decides admission via announce_admissible).
// Payload is strings + ints only, so there are no NaN/Inf concerns.
bool parse_announce(const std::uint8_t* data, std::size_t len, Announce& out);

// Stable per-host instance id: lowercase hex of an FNV-1a 64 hash of the
// hostname. Deterministic and collision-resistant enough for LAN peer pinning;
// no crypto strength needed — the id is opaque and compared only as a string,
// and each host generates its own (so the hash algorithm is not a wire
// contract). Empty/failed hostname falls back to a fixed token.
std::string stable_instance_id_from(std::string_view hostname);
std::string stable_instance_id();   // uses gethostname()

}  // namespace fitra::vmt
