#include "vmt/peer_registry.hpp"

#include <utility>

namespace fitra::vmt {

bool announce_admissible(const Announce& a, const PeerRegistryConfig& cfg) {
    if (a.proto_version != kDiscoveryProtoVersion) return false;
    if (a.role != cfg.want_role) return false;
    // Network input — never trust the port as a send target. A 0/negative/
    // >65535 value would truncate to a bogus uint16_t (0/65535/4464/...) and
    // aim the publisher and punch at the wrong UDP port.
    if (a.osc_recv_port < 1 || a.osc_recv_port > 65535) return false;
    if (!cfg.self_instance_id.empty() && a.instance_id == cfg.self_instance_id) {
        return false;  // our own loopback announce
    }
    if (!cfg.pairing_token.empty() && a.pairing_token != cfg.pairing_token) {
        return false;  // cross-rig isolation
    }
    return true;
}

const Peer* select_peer(const std::vector<Peer>& peers,
                        const PeerRegistryConfig& cfg, double now_ms) {
    const Peer* best = nullptr;
    for (const auto& p : peers) {
        if (now_ms - p.last_seen_ms > cfg.peer_timeout_ms) continue;  // stale
        if (!cfg.pair_id.empty()) {
            if (p.instance_id == cfg.pair_id) return &p;  // pin wins outright
            continue;
        }
        if (!best || p.instance_id < best->instance_id) best = &p;
    }
    // A pin that matched no live peer resolves to "no peer".
    return cfg.pair_id.empty() ? best : nullptr;
}

PeerRegistry::PeerRegistry(PeerRegistryConfig cfg) : cfg_(std::move(cfg)) {}

bool PeerRegistry::observe(const Announce& a, std::string_view src_ip,
                           double now_ms) {
    if (!announce_admissible(a, cfg_)) return false;
    std::lock_guard<std::mutex> lk(mu_);
    for (auto& p : peers_) {
        if (p.instance_id == a.instance_id) {
            p.instance_name = a.instance_name;
            p.ip            = std::string(src_ip);
            p.osc_recv_port = static_cast<std::uint16_t>(a.osc_recv_port);
            p.pairing_token = a.pairing_token;
            p.role          = a.role;
            p.last_seen_ms  = now_ms;
            return true;
        }
    }
    Peer p;
    p.instance_id   = a.instance_id;
    p.instance_name = a.instance_name;
    p.ip            = std::string(src_ip);
    p.osc_recv_port = static_cast<std::uint16_t>(a.osc_recv_port);
    p.pairing_token = a.pairing_token;
    p.role          = a.role;
    p.last_seen_ms  = now_ms;
    peers_.push_back(std::move(p));
    return true;
}

ResolvedPeer PeerRegistry::select(double now_ms) const {
    std::lock_guard<std::mutex> lk(mu_);
    const Peer* best = select_peer(peers_, cfg_, now_ms);
    ResolvedPeer r;
    if (best) {
        r.have          = true;
        r.ip            = best->ip;
        r.port          = best->osc_recv_port;
        r.instance_id   = best->instance_id;
        r.instance_name = best->instance_name;
        r.age_ms        = now_ms - best->last_seen_ms;
    }
    return r;
}

std::vector<Peer> PeerRegistry::live_peers(double now_ms) const {
    std::lock_guard<std::mutex> lk(mu_);
    std::vector<Peer> out;
    for (const auto& p : peers_) {
        if (now_ms - p.last_seen_ms > cfg_.peer_timeout_ms) continue;
        out.push_back(p);
    }
    return out;
}

std::size_t PeerRegistry::size() const {
    std::lock_guard<std::mutex> lk(mu_);
    return peers_.size();
}

void PeerRegistry::clear() {
    std::lock_guard<std::mutex> lk(mu_);
    peers_.clear();
}

void DiscoveryEndpointBus::publish(const ResolvedPeer& p) {
    std::lock_guard<std::mutex> lk(mu_);
    // Bump the generation only on an endpoint-identity change. age_ms drifts
    // every tick, so a full-struct compare would never dedup; consumers care
    // only about have/ip/port (the send/punch target), so key on those.
    const bool changed = p.have != latest_.have || p.ip != latest_.ip ||
                         p.port != latest_.port;
    latest_ = p;
    if (changed) generation_.fetch_add(1, std::memory_order_release);
}

ResolvedPeer DiscoveryEndpointBus::snapshot() const {
    std::lock_guard<std::mutex> lk(mu_);
    return latest_;
}

}  // namespace fitra::vmt
