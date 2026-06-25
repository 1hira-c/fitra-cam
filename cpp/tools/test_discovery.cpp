// test_discovery — pure-logic coverage for the VMT zeroconf discovery layer:
//   - /fitra/announce OSC 1.0 wire codec (golden bytes + round-trip + reject)
//   - stable instance-id determinism
//   - PeerRegistry / select_peer (single, multi, pin, token, stale, proto)
//
// No sockets, no threads — the impure DiscoveryBeacon shell is exercised at
// runtime, not here. Hand-rolled asserts (throw on failure) mirror
// test_vmt_osc_writer.cpp. Maps to the "検証" bullets of
// docs/design/vr-output-zeroconf-discovery.md.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include "vmt/discovery_announce.hpp"
#include "vmt/discovery_endpoint.hpp"
#include "vmt/peer_registry.hpp"

namespace {

using namespace fitra::vmt;

std::string hex_dump(const std::uint8_t* p, std::size_t n) {
    std::string out;
    char tmp[8];
    for (std::size_t i = 0; i < n; ++i) {
        std::snprintf(tmp, sizeof(tmp), "%02x ", p[i]);
        out += tmp;
        if ((i + 1) % 16 == 0) out += "\n";
    }
    return out;
}

void check(bool cond, const std::string& label) {
    if (!cond) throw std::runtime_error("FAILED: " + label);
}

void check_bytes(const std::vector<std::uint8_t>& got,
                 const std::vector<std::uint8_t>& want,
                 const std::string& label) {
    if (got.size() != want.size() ||
        std::memcmp(got.data(), want.data(), got.size()) != 0) {
        throw std::runtime_error(
            label + " mismatch\n  want (" + std::to_string(want.size()) +
            " bytes):\n" + hex_dump(want.data(), want.size()) + "\n  got  (" +
            std::to_string(got.size()) + " bytes):\n" +
            hex_dump(got.data(), got.size()));
    }
}

void want_str(std::vector<std::uint8_t>& v, const char* s) {
    while (*s) v.push_back(static_cast<std::uint8_t>(*s++));
    v.push_back('\0');
    while (v.size() % 4 != 0) v.push_back('\0');
}
void want_be32(std::vector<std::uint8_t>& v, std::uint32_t x) {
    v.push_back(static_cast<std::uint8_t>((x >> 24) & 0xff));
    v.push_back(static_cast<std::uint8_t>((x >> 16) & 0xff));
    v.push_back(static_cast<std::uint8_t>((x >>  8) & 0xff));
    v.push_back(static_cast<std::uint8_t>( x        & 0xff));
}

Peer mk_peer(const std::string& id, const std::string& ip, std::uint16_t port,
             double last_seen_ms, DiscoveryRole role = DiscoveryRole::Vmt,
             const std::string& token = "") {
    Peer p;
    p.instance_id   = id;
    p.instance_name = id;
    p.ip            = ip;
    p.osc_recv_port = port;
    p.pairing_token = token;
    p.role          = role;
    p.last_seen_ms  = last_seen_ms;
    return p;
}

// 1. Golden announce byte layout (addr/typetag/order fixed).
void test_announce_golden_bytes() {
    Announce a;
    a.role          = DiscoveryRole::Jetson;
    a.instance_id   = "rig";
    a.instance_name = "Living Rig";
    a.proto_version = 1;
    a.osc_recv_port = 39571;
    a.capabilities  = "pose,hmd,controller";
    a.pairing_token = "";

    std::vector<std::uint8_t> want;
    want_str(want, "/fitra/announce");
    want_str(want, ",sssiiss");
    want_str(want, "jetson");
    want_str(want, "rig");
    want_str(want, "Living Rig");
    want_be32(want, 1);
    want_be32(want, 39571);
    want_str(want, "pose,hmd,controller");
    want_str(want, "");   // empty token -> 4 zero bytes

    check_bytes(encode_announce(a), want, "announce_golden_bytes");
}

// 2. encode -> parse round-trip (empty token + multibyte-ish name + vmt role).
void test_announce_round_trip() {
    auto rt = [](const Announce& a, const std::string& label) {
        auto bytes = encode_announce(a);
        Announce out;
        check(parse_announce(bytes.data(), bytes.size(), out), label + " parse");
        check(out.role == a.role, label + " role");
        check(out.instance_id == a.instance_id, label + " id");
        check(out.instance_name == a.instance_name, label + " name");
        check(out.proto_version == a.proto_version, label + " proto");
        check(out.osc_recv_port == a.osc_recv_port, label + " port");
        check(out.capabilities == a.capabilities, label + " caps");
        check(out.pairing_token == a.pairing_token, label + " token");
    };

    Announce j;
    j.role = DiscoveryRole::Jetson; j.instance_id = "abc123";
    j.instance_name = "Living Rig 居間"; j.proto_version = 1;
    j.osc_recv_port = 39571; j.capabilities = "pose,hmd,controller";
    j.pairing_token = "";
    rt(j, "jetson");

    Announce v;
    v.role = DiscoveryRole::Vmt; v.instance_id = "deadbeef";
    v.instance_name = "VMT-PC"; v.proto_version = 1;
    v.osc_recv_port = 39570; v.capabilities = "pose"; v.pairing_token = "lab7";
    rt(v, "vmt");
}

// 3. Reject malformed packets — false, no crash.
void test_announce_reject_malformed() {
    Announce out;
    // short buffer
    const std::uint8_t tiny[4] = {'/', 'f', 'i', 't'};
    check(!parse_announce(tiny, sizeof(tiny), out), "reject short");
    check(!parse_announce(nullptr, 0, out), "reject null");

    // wrong address
    {
        std::vector<std::uint8_t> b;
        want_str(b, "/fitra/bogus");
        want_str(b, ",sssiiss");
        want_str(b, "jetson"); want_str(b, "x"); want_str(b, "y");
        want_be32(b, 1); want_be32(b, 39571);
        want_str(b, "pose"); want_str(b, "");
        check(!parse_announce(b.data(), b.size(), out), "reject wrong addr");
    }
    // wrong typetag
    {
        std::vector<std::uint8_t> b;
        want_str(b, "/fitra/announce");
        want_str(b, ",ssiiss");   // missing one 's'
        want_str(b, "jetson"); want_str(b, "x");
        want_be32(b, 1); want_be32(b, 39571);
        want_str(b, "pose"); want_str(b, "");
        check(!parse_announce(b.data(), b.size(), out), "reject wrong typetag");
    }
    // truncated: valid addr+typetag but no args
    {
        std::vector<std::uint8_t> b;
        want_str(b, "/fitra/announce");
        want_str(b, ",sssiiss");
        check(!parse_announce(b.data(), b.size(), out), "reject truncated args");
    }
    // unknown role string
    {
        std::vector<std::uint8_t> b;
        want_str(b, "/fitra/announce");
        want_str(b, ",sssiiss");
        want_str(b, "alien"); want_str(b, "x"); want_str(b, "y");
        want_be32(b, 1); want_be32(b, 39571);
        want_str(b, "pose"); want_str(b, "");
        check(!parse_announce(b.data(), b.size(), out), "reject unknown role");
    }
}

// 4. Stable instance id determinism.
void test_stable_instance_id() {
    check(stable_instance_id_from("hostA") == stable_instance_id_from("hostA"),
          "id stable for same host");
    check(stable_instance_id_from("hostA") != stable_instance_id_from("hostB"),
          "id differs across hosts");
    check(!stable_instance_id_from("hostA").empty(), "id non-empty");
    check(stable_instance_id_from("") == "fitra-unknown", "empty host fallback");
    // 16 lowercase hex chars for a non-empty host.
    check(stable_instance_id_from("hostA").size() == 16, "id hex width");
}

// 5. Single live peer -> adopted.
void test_select_single() {
    PeerRegistryConfig cfg; cfg.peer_timeout_ms = 5000.0;
    Announce a; a.role = DiscoveryRole::Vmt; a.instance_id = "vmt1";
    a.instance_name = "VMT-PC"; a.proto_version = 1; a.osc_recv_port = 39570;
    PeerRegistry reg(cfg);
    check(reg.observe(a, "192.168.1.20", 1000.0), "observe single");
    auto r = reg.select(1100.0);
    check(r.have, "single resolved");
    check(r.ip == "192.168.1.20" && r.port == 39570, "single endpoint");
    check(r.instance_id == "vmt1", "single id");
}

// 6. Multiple peers -> lexicographically-smallest instance_id (deterministic).
void test_select_min_id() {
    std::vector<Peer> peers = {
        mk_peer("zzz", "10.0.0.3", 39570, 1000.0),
        mk_peer("aaa", "10.0.0.1", 39570, 1000.0),
        mk_peer("mmm", "10.0.0.2", 39570, 1000.0),
    };
    PeerRegistryConfig cfg; cfg.peer_timeout_ms = 5000.0;
    const Peer* best = select_peer(peers, cfg, 1100.0);
    check(best && best->instance_id == "aaa", "min id selected");
}

// 7. pair_id pin: chosen even if not smallest; absent pin -> no peer.
void test_select_pin() {
    std::vector<Peer> peers = {
        mk_peer("aaa", "10.0.0.1", 39570, 1000.0),
        mk_peer("zzz", "10.0.0.3", 39570, 1000.0),
    };
    {
        PeerRegistryConfig cfg; cfg.pair_id = "zzz"; cfg.peer_timeout_ms = 5000.0;
        const Peer* best = select_peer(peers, cfg, 1100.0);
        check(best && best->instance_id == "zzz", "pin selects non-min");
    }
    {
        PeerRegistryConfig cfg; cfg.pair_id = "qqq"; cfg.peer_timeout_ms = 5000.0;
        const Peer* best = select_peer(peers, cfg, 1100.0);
        check(best == nullptr, "pin absent -> no peer");
    }
}

// 8. token filter via admission.
void test_token_filter() {
    Announce a; a.role = DiscoveryRole::Vmt; a.instance_id = "vmt1";
    a.proto_version = 1; a.osc_recv_port = 39570; a.pairing_token = "lab7";

    PeerRegistryConfig match;    match.pairing_token = "lab7";
    PeerRegistryConfig mismatch; mismatch.pairing_token = "labX";
    PeerRegistryConfig anytok;   anytok.pairing_token = "";

    check(announce_admissible(a, match), "token match admitted");
    check(!announce_admissible(a, mismatch), "token mismatch rejected");
    check(announce_admissible(a, anytok), "empty cfg token admits any");

    // Self announce ignored.
    PeerRegistryConfig self; self.self_instance_id = "vmt1";
    check(!announce_admissible(a, self), "self announce ignored");

    // Wrong role rejected.
    PeerRegistryConfig wantJetson; wantJetson.want_role = DiscoveryRole::Jetson;
    check(!announce_admissible(a, wantJetson), "wrong role rejected");
}

// 9. stale timeout: dropped from select/live, revived by re-observe (recovery).
void test_stale_timeout() {
    PeerRegistryConfig cfg; cfg.peer_timeout_ms = 5000.0;
    Announce a; a.role = DiscoveryRole::Vmt; a.instance_id = "vmt1";
    a.proto_version = 1; a.osc_recv_port = 39570;
    PeerRegistry reg(cfg);
    reg.observe(a, "10.0.0.5", 1000.0);

    check(reg.select(2000.0).have, "fresh selected");
    check(reg.live_peers(2000.0).size() == 1, "fresh live");

    // 6 s later: stale (> 5 s timeout).
    check(!reg.select(7001.0).have, "stale dropped from select");
    check(reg.live_peers(7001.0).empty(), "stale dropped from live");

    // Re-announce revives it (auto recovery).
    reg.observe(a, "10.0.0.5", 7100.0);
    check(reg.select(7200.0).have, "revived after re-observe");
}

// 10. proto mismatch rejected.
void test_proto_mismatch() {
    Announce a; a.role = DiscoveryRole::Vmt; a.instance_id = "vmt1";
    a.proto_version = 2; a.osc_recv_port = 39570;
    PeerRegistryConfig cfg;
    check(!announce_admissible(a, cfg), "proto mismatch rejected");
    a.proto_version = kDiscoveryProtoVersion;
    check(announce_admissible(a, cfg), "matching proto admitted");
}

// 11. osc_recv_port out of [1,65535] rejected at admission (network input is
//     never trusted as a send target — a 0/negative/overflowing port would
//     truncate to a bogus uint16 and misaim the publisher/punch).
void test_port_validation() {
    PeerRegistryConfig cfg;
    auto mk = [](std::int32_t port) {
        Announce a; a.role = DiscoveryRole::Vmt; a.instance_id = "vmt1";
        a.proto_version = 1; a.osc_recv_port = port;
        return a;
    };
    check(!announce_admissible(mk(0), cfg), "port 0 rejected");
    check(!announce_admissible(mk(-1), cfg), "negative port rejected");
    check(!announce_admissible(mk(65536), cfg), "port > 65535 rejected");
    check(announce_admissible(mk(1), cfg), "port 1 admitted");
    check(announce_admissible(mk(65535), cfg), "port 65535 admitted");
    check(announce_admissible(mk(39570), cfg), "typical port admitted");

    // observe() drops the bad-port announce entirely (never stored).
    PeerRegistry reg(cfg);
    check(!reg.observe(mk(0), "10.0.0.9", 1000.0), "observe drops port 0");
    check(reg.size() == 0, "no peer stored for bad port");
}

// 12. endpoint bus generation: bumps only on have/ip/port change (not on the
//     per-tick age refresh), and DiscoveryEndpointLatch resolves once per change.
void test_endpoint_bus_generation_and_latch() {
    DiscoveryEndpointBus bus;
    DiscoveryEndpointLatch latch;
    DiscoveryEndpointLatch::Resolved r;

    check(bus.generation() == 0, "fresh bus generation 0");
    check(!latch.poll(bus, r), "no peer -> latch resolves nothing");

    ResolvedPeer p;
    p.have = true; p.ip = "10.0.0.5"; p.port = 39570;
    p.instance_name = "VMT-PC"; p.age_ms = 1.0;
    bus.publish(p);
    const std::uint64_t g1 = bus.generation();
    check(g1 != 0, "first endpoint bumps generation");
    check(latch.poll(bus, r), "latch resolves the new endpoint");
    check(r.ip == "10.0.0.5" && r.port == 39570, "resolved ip/port");
    check(r.addr.sin_family == AF_INET && r.addr.sin_port == htons(39570),
          "resolved sockaddr filled");
    check(!latch.poll(bus, r), "no re-resolve when endpoint unchanged");

    // Same identity, only age_ms drifts (the beacon republishes every tick):
    // generation must NOT move, so the hot path skips the snapshot+copy.
    p.age_ms = 250.0;
    bus.publish(p);
    check(bus.generation() == g1, "age-only republish does not bump generation");
    check(!latch.poll(bus, r), "age-only change -> latch stays put");

    // Port change is a real endpoint change.
    p.port = 39571;
    bus.publish(p);
    check(bus.generation() == g1 + 1, "port change bumps generation");
    check(latch.poll(bus, r), "latch picks up the port change");
    check(r.port == 39571, "resolved new port");

    // Peer lost: generation bumps, but the latch keeps the last destination
    // (poll returns false so the caller does not clear its target).
    ResolvedPeer gone; gone.have = false;
    bus.publish(gone);
    check(bus.generation() == g1 + 2, "peer-lost bumps generation");
    check(!latch.poll(bus, r), "peer lost -> latch holds last endpoint");
}

}  // namespace

int main() {
    try {
        test_announce_golden_bytes();
        test_announce_round_trip();
        test_announce_reject_malformed();
        test_stable_instance_id();
        test_select_single();
        test_select_min_id();
        test_select_pin();
        test_token_filter();
        test_stale_timeout();
        test_proto_mismatch();
        test_port_validation();
        test_endpoint_bus_generation_and_latch();
        std::puts("test_discovery ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_discovery failed: %s\n", e.what());
        return 1;
    }
}
