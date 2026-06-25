#include "vmt/discovery_beacon.hpp"

#include <chrono>
#include <cstring>
#include <utility>

#include <arpa/inet.h>
#include <errno.h>
#include <sys/socket.h>
#include <unistd.h>

#include "util/logging.hpp"

namespace fitra::vmt {

namespace {

double steady_ms() {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

PeerRegistryConfig make_registry_cfg(const DiscoveryBeaconOptions& o,
                                     const std::string& self_id) {
    PeerRegistryConfig c;
    c.pair_id          = o.pair_id;
    c.pairing_token    = o.pairing_token;
    c.peer_timeout_ms  = o.peer_timeout_ms;
    // A jetson adopts vmt peers and vice versa.
    c.want_role        = (o.self_role == DiscoveryRole::Jetson)
                             ? DiscoveryRole::Vmt
                             : DiscoveryRole::Jetson;
    c.self_instance_id = self_id;
    return c;
}

}  // namespace

DiscoveryBeacon::DiscoveryBeacon(DiscoveryBeaconOptions opts)
    : opts_(std::move(opts)),
      self_id_(opts_.instance_id.empty() ? stable_instance_id()
                                         : opts_.instance_id),
      registry_(make_registry_cfg(opts_, self_id_)) {
    self_announce_.role          = opts_.self_role;
    self_announce_.instance_id   = self_id_;
    self_announce_.instance_name = opts_.instance_name;
    self_announce_.proto_version = kDiscoveryProtoVersion;
    self_announce_.osc_recv_port = opts_.self_osc_recv_port;
    self_announce_.capabilities  = opts_.capabilities;
    self_announce_.pairing_token = opts_.pairing_token;
}

DiscoveryBeacon::~DiscoveryBeacon() {
    try { stop(); } catch (...) {}
}

bool DiscoveryBeacon::setup_socket_() {
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        FITRA_LOG_WARN("[discovery] socket() failed: {}", std::strerror(errno));
        return false;
    }

    int one = 1;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));

    timeval tv{};
    tv.tv_sec  = 0;
    tv.tv_usec = 100 * 1000;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Bind INADDR_ANY (not the group addr): receives both the multicast (after
    // join) and the 255.255.255.255 broadcast leg, and is portable.
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(opts_.port);
    if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        FITRA_LOG_WARN("[discovery] bind(0.0.0.0:{}) failed: {}", opts_.port,
                       std::strerror(errno));
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    // Join the multicast group. Failure is non-fatal — the broadcast leg still
    // reaches peers on the same subnet.
    ip_mreq mreq{};
    if (::inet_pton(AF_INET, opts_.group.c_str(), &mreq.imr_multiaddr) == 1) {
        mreq.imr_interface.s_addr = htonl(INADDR_ANY);
        if (::setsockopt(sock_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq,
                         sizeof(mreq)) < 0) {
            FITRA_LOG_WARN("[discovery] IP_ADD_MEMBERSHIP({}) failed: {} "
                           "(broadcast leg still active)",
                           opts_.group, std::strerror(errno));
        }
    } else {
        FITRA_LOG_WARN("[discovery] invalid group '{}'", opts_.group);
    }

    int ttl = 1;  // link-local: never leak past the subnet.
    ::setsockopt(sock_fd_, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));

    // Pre-resolve the two announce destinations.
    group_addr_ = sockaddr_in{};
    group_addr_.sin_family = AF_INET;
    group_addr_.sin_port   = htons(opts_.port);
    ::inet_pton(AF_INET, opts_.group.c_str(), &group_addr_.sin_addr);

    bcast_addr_ = sockaddr_in{};
    bcast_addr_.sin_family      = AF_INET;
    bcast_addr_.sin_port        = htons(opts_.port);
    bcast_addr_.sin_addr.s_addr = htonl(INADDR_BROADCAST);  // 255.255.255.255
    return true;
}

bool DiscoveryBeacon::start() {
    if (sock_fd_ >= 0) return true;
    if (!setup_socket_()) return false;
    {
        std::lock_guard<std::mutex> lk(stats_mu_);
        stats_.socket_up = true;
        stats_.self_id   = self_id_;
        stats_.group     = opts_.group;
        stats_.port      = opts_.port;
    }
    stop_.store(false);
    thread_ = std::thread([this]() { loop(); });
    FITRA_LOG_INFO("[discovery] beacon up: role={} id={} group={}:{} "
                   "osc_recv_port={} (announce {} Hz)",
                   discovery_role_name(opts_.self_role), self_id_, opts_.group,
                   opts_.port, opts_.self_osc_recv_port,
                   1000.0 / opts_.announce_interval_ms);
    return true;
}

void DiscoveryBeacon::stop() {
    if (stop_.exchange(true)) return;
    if (thread_.joinable()) thread_.join();
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

void DiscoveryBeacon::send_announce_() {
    announce_writer_.clear();
    encode_announce(announce_writer_, self_announce_);
    auto sp = announce_writer_.data();

    if (group_addr_.sin_addr.s_addr != 0) {
        ::sendto(sock_fd_, sp.data(), sp.size(), 0,
                 reinterpret_cast<const sockaddr*>(&group_addr_),
                 sizeof(group_addr_));
    }
    ::sendto(sock_fd_, sp.data(), sp.size(), 0,
             reinterpret_cast<const sockaddr*>(&bcast_addr_),
             sizeof(bcast_addr_));

    std::lock_guard<std::mutex> lk(stats_mu_);
    ++stats_.announces_sent;
}

void DiscoveryBeacon::loop() {
    // Fire the first announce immediately.
    double last_announce = steady_ms() - opts_.announce_interval_ms;
    std::uint8_t buf[2048];

    while (!stop_.load(std::memory_order_relaxed)) {
        const double now = steady_ms();
        if (now - last_announce >= opts_.announce_interval_ms) {
            send_announce_();
            last_announce = now;
        }

        sockaddr_in src{};
        socklen_t   src_len = sizeof(src);
        ssize_t n = ::recvfrom(sock_fd_, buf, sizeof(buf), 0,
                               reinterpret_cast<sockaddr*>(&src), &src_len);
        if (n > 0) {
            Announce a;
            bool ok = parse_announce(buf, static_cast<std::size_t>(n), a);
            if (ok) {
                char ip[INET_ADDRSTRLEN] = {0};
                ::inet_ntop(AF_INET, &src.sin_addr, ip, sizeof(ip));
                if (registry_.observe(a, ip, steady_ms())) {
                    std::lock_guard<std::mutex> lk(stats_mu_);
                    ++stats_.announces_recv;
                } else {
                    std::lock_guard<std::mutex> lk(stats_mu_);
                    ++stats_.packets_rejected;
                }
            } else {
                std::lock_guard<std::mutex> lk(stats_mu_);
                ++stats_.packets_rejected;
            }
        }

        const double t = steady_ms();
        ResolvedPeer resolved = registry_.select(t);
        bus_.publish(resolved);
        {
            std::lock_guard<std::mutex> lk(stats_mu_);
            stats_.resolved   = resolved;
            stats_.peer_count = registry_.live_peers(t).size();
        }
    }
}

DiscoveryBeaconStats DiscoveryBeacon::stats() const {
    std::lock_guard<std::mutex> lk(stats_mu_);
    return stats_;
}

std::vector<Peer> DiscoveryBeacon::peers() const {
    return registry_.live_peers(steady_ms());
}

}  // namespace fitra::vmt
