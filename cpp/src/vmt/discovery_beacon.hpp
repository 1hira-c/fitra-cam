#pragma once
//
// DiscoveryBeacon — the impure socket/thread shell around PeerRegistry.
//
// Owns one UDP socket bound to INADDR_ANY:39580, joined to the discovery
// multicast group, with SO_BROADCAST enabled. A single thread announces this
// host (role + osc_recv_port) at 1 Hz to both the group and 255.255.255.255
// (TTL=1, link-local only), receives peer announces, upserts them into the
// PeerRegistry, and publishes the currently-adopted endpoint onto a
// DiscoveryEndpointBus that VmtPublisher / TrackedPoseReceiver poll.
//
// The pose wire is untouched — this only resolves "which IP:port". See
// docs/design/vr-output-zeroconf-discovery.md.

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <netinet/in.h>

#include "vmt/discovery_announce.hpp"
#include "vmt/osc_writer.hpp"
#include "vmt/peer_registry.hpp"

namespace fitra::vmt {

struct DiscoveryBeaconOptions {
    DiscoveryRole self_role          = DiscoveryRole::Jetson;
    std::string   group              = "239.255.42.99";
    std::uint16_t port               = 39580;
    std::uint16_t self_osc_recv_port = kJetsonOscRecvPort;  // our pose recv port
    std::string   instance_id;          // empty -> stable_instance_id()
    std::string   instance_name;        // human label (UI)
    std::string   capabilities         = "pose,hmd,controller";
    std::string   pair_id;              // pin a peer by instance_id
    std::string   pairing_token;        // cross-rig isolation
    double        announce_interval_ms = 1000.0;
    double        peer_timeout_ms      = 5000.0;
};

struct DiscoveryBeaconStats {
    bool          socket_up        = false;
    std::uint64_t announces_sent   = 0;
    std::uint64_t announces_recv   = 0;   // admitted into the registry
    std::uint64_t packets_rejected = 0;   // parse fail / not admissible
    std::string   self_id;
    std::string   group;
    std::uint16_t port = 0;
    ResolvedPeer  resolved;               // current adopted peer
    std::size_t   peer_count = 0;         // live peers at last loop iteration
};

class DiscoveryBeacon {
public:
    explicit DiscoveryBeacon(DiscoveryBeaconOptions opts);
    ~DiscoveryBeacon();

    DiscoveryBeacon(const DiscoveryBeacon&) = delete;
    DiscoveryBeacon& operator=(const DiscoveryBeacon&) = delete;

    // Open the socket + start the announce/recv thread. Returns false (logged)
    // on socket failure; callers continue without discovery.
    bool start();
    void stop();   // idempotent; joins thread, closes socket

    // Consumers (VmtPublisher / TrackedPoseReceiver) subscribe to this bus.
    const DiscoveryEndpointBus& endpoint_bus() const { return bus_; }

    DiscoveryBeaconStats stats() const;
    std::vector<Peer>    peers() const;     // live peers for /stats3d
    const std::string&   self_id() const { return self_id_; }
    bool is_manual() const { return false; }

private:
    void loop();
    bool setup_socket_();
    void send_announce_();

    DiscoveryBeaconOptions opts_;
    std::string            self_id_;
    Announce               self_announce_;
    PeerRegistry           registry_;
    DiscoveryEndpointBus   bus_;
    OscWriter              announce_writer_;

    int                    sock_fd_ = -1;
    sockaddr_in            group_addr_{};
    sockaddr_in            bcast_addr_{};

    std::thread            thread_;
    std::atomic<bool>      stop_{false};

    mutable std::mutex     stats_mu_;
    DiscoveryBeaconStats   stats_;
};

}  // namespace fitra::vmt
