#pragma once
//
// Shared "resolve the current endpoint from the discovery bus" latch.
//
// Both the VMT publisher (send destination) and the tracked-pose receiver
// (punch destination) need the exact same step: snapshot the bus, notice when
// the adopted peer changed, parse its IP, and latch the new endpoint so the
// same peer is not re-resolved (and re-parsed) every loop tick. This is that
// one place — keeping it shared stops the two copies from drifting in their
// change-detection / error-handling rules (which they previously had).

#include <cstdint>
#include <string>
#include <utility>

#include <arpa/inet.h>     // inet_pton, htons
#include <netinet/in.h>    // sockaddr_in

#include "vmt/peer_registry.hpp"   // DiscoveryEndpointBus, ResolvedPeer

namespace fitra::vmt {

// Tracks the last bus generation a consumer acted on. poll() is cheap in the
// steady state: it reads only the bus's atomic generation counter and returns
// immediately when the adopted endpoint has not changed — no lock, no string
// copy. Only when a NEW peer is published does it take the full snapshot and
// parse. Single-consumer: each owner polls from its own loop thread.
class DiscoveryEndpointLatch {
public:
    struct Resolved {
        sockaddr_in   addr{};            // ready-to-send destination
        std::string   ip;
        std::uint16_t port = 0;
        std::string   instance_name;     // for the caller's log line
    };

    // Returns true and fills `out` when the bus advertises a NEW, parseable
    // peer since the last accepted generation. Returns false (out untouched)
    // when the endpoint is unchanged, there is no peer yet, or the IP fails to
    // parse — in which case the caller keeps its previously-latched destination
    // (best-effort: keep streaming through a transient loss / bad value).
    bool poll(const DiscoveryEndpointBus& bus, Resolved& out) {
        const std::uint64_t gen = bus.generation();
        if (gen == last_gen_) return false;   // endpoint identity unchanged
        last_gen_ = gen;

        ResolvedPeer rp = bus.snapshot();
        if (!rp.have) return false;           // peer lost -> keep last destination

        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port   = htons(rp.port);
        if (::inet_pton(AF_INET, rp.ip.c_str(), &a.sin_addr) != 1) return false;

        out.addr          = a;
        out.port          = rp.port;
        out.ip            = std::move(rp.ip);
        out.instance_name = std::move(rp.instance_name);
        return true;
    }

private:
    std::uint64_t last_gen_ = 0;
};

}  // namespace fitra::vmt
