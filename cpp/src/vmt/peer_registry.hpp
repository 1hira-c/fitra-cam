#pragma once
//
// Discovery peer tracking + selection (pure logic) and the runtime endpoint
// bus that carries the resolved peer to VmtPublisher / TrackedPoseReceiver.
//
// PeerRegistry is deterministic: every method that needs "now" takes it as a
// monotonic-ms argument, so unit tests pin time without reading a clock. The
// IMPURE socket/announce-thread shell lives in DiscoveryBeacon
// (discovery_beacon.{hpp,cpp}). See docs/design/vr-output-zeroconf-discovery.md.

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include "vmt/discovery_announce.hpp"

namespace fitra::vmt {

struct Peer {
    std::string   instance_id;
    std::string   instance_name;
    std::string   ip;                  // dotted-quad src IP of the announce
    std::uint16_t osc_recv_port = 0;   // peer's advertised pose-plane port
    std::string   pairing_token;
    DiscoveryRole role = DiscoveryRole::Vmt;
    double        last_seen_ms = 0.0;  // monotonic ms at last observe()
};

// Resolved adopted peer published to the endpoint bus. have=false means "no
// peer yet" — senders skip / stay no-op until it flips true.
struct ResolvedPeer {
    bool          have = false;
    std::string   ip;
    std::uint16_t port = 0;
    std::string   instance_id;
    std::string   instance_name;
    double        age_ms = 0.0;        // now - last_seen_ms (filled at select)
};

struct PeerRegistryConfig {
    std::string   pair_id;             // pin a peer by instance_id; "" = auto
    std::string   pairing_token;       // require equality when non-empty
    double        peer_timeout_ms = 5000.0;
    DiscoveryRole want_role = DiscoveryRole::Vmt;  // role we accept (jetson→vmt)
    std::string   self_instance_id;    // ignore our own (loopback) announce
};

// --- Pure free helpers (no clock, no lock) — pinned directly by ctest ---

// Is this announce admissible given our config?
//   - proto_version == kDiscoveryProtoVersion
//   - role == cfg.want_role
//   - not our own announce (instance_id != cfg.self_instance_id when set)
//   - if cfg.pairing_token non-empty, the announce token must match it
bool announce_admissible(const Announce& a, const PeerRegistryConfig& cfg);

// Select among peers, skipping stale (now - last_seen > timeout):
//   1. cfg.pair_id non-empty -> the live peer whose id matches, else nullptr
//   2. else the live peer with the lexicographically-smallest instance_id
// Returns nullptr if nothing qualifies. Pure — no clock read.
const Peer* select_peer(const std::vector<Peer>& peers,
                        const PeerRegistryConfig& cfg, double now_ms);

class PeerRegistry {
public:
    explicit PeerRegistry(PeerRegistryConfig cfg);

    // Admit + upsert an announce observed from src_ip at now_ms. Returns false
    // (ignored) when not admissible. Upsert key = instance_id, so the
    // multicast + broadcast duplicate of one announce collapses to one peer.
    bool observe(const Announce& a, std::string_view src_ip, double now_ms);

    // Resolve the currently-adopted peer (pure selection over live peers).
    ResolvedPeer select(double now_ms) const;

    // Live (non-stale) peers, for /stats3d display.
    std::vector<Peer> live_peers(double now_ms) const;

    std::size_t size() const;   // total tracked entries (incl. stale)
    void        clear();

    const PeerRegistryConfig& config() const { return cfg_; }

private:
    mutable std::mutex mu_;
    PeerRegistryConfig cfg_;
    std::vector<Peer>  peers_;
};

// Latest-wins single-slot resolved endpoint, mirroring HmdPoseBus. Producer =
// DiscoveryBeacon thread; consumers = VmtPublisher / TrackedPoseReceiver.
class DiscoveryEndpointBus {
public:
    DiscoveryEndpointBus() = default;
    DiscoveryEndpointBus(const DiscoveryEndpointBus&) = delete;
    DiscoveryEndpointBus& operator=(const DiscoveryEndpointBus&) = delete;

    void publish(const ResolvedPeer& p);
    ResolvedPeer snapshot() const;

private:
    mutable std::mutex mu_;
    ResolvedPeer       latest_{};
};

}  // namespace fitra::vmt
