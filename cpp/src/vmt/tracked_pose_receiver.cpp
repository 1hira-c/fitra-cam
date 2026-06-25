#include "vmt/tracked_pose_receiver.hpp"

#include "vmt/osc_decode.hpp"
#include "vmt/osc_writer.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <utility>

namespace fitra::vmt {

namespace {

constexpr const char* kTrackedAddress = "/fitra/tracked_pose";
constexpr const char* kTrackedTypetag = ",iiiiffffffff";

using osc::consume_osc_string;
using osc::read_be32;
using osc::read_be_float;

bool parse_role(std::int32_t raw, TrackedPoseRole& out) {
    switch (raw) {
        case 0: out = TrackedPoseRole::Hmd; return true;
        case 1: out = TrackedPoseRole::LeftController; return true;
        case 2: out = TrackedPoseRole::RightController; return true;
        default: return false;
    }
}

HmdPose to_hmd_pose(const TrackedPose& p) {
    HmdPose h;
    // Legacy HmdPose::valid meant calibration-grade tracking. Preserve that
    // contract by folding ETrackingResult here.
    h.valid       = p.running_ok();
    h.timestamp_s = p.timestamp_s;
    h.x = p.x; h.y = p.y; h.z = p.z;
    h.qx = p.qx; h.qy = p.qy; h.qz = p.qz; h.qw = p.qw;
    return h;
}

ControllerPose to_controller_pose(const TrackedPose& p) {
    ControllerPose c;
    c.valid           = p.valid;
    c.tracking_result = p.tracking_result;
    c.timestamp_s     = p.timestamp_s;
    c.x = p.x; c.y = p.y; c.z = p.z;
    c.qx = p.qx; c.qy = p.qy; c.qz = p.qz; c.qw = p.qw;
    return c;
}

bool is_bundle(const std::uint8_t* data, std::size_t len) {
    return len >= 16 && std::memcmp(data, "#bundle", 7) == 0 && data[7] == '\0';
}

}  // namespace

bool parse_tracked_pose_packet(const std::uint8_t* data, std::size_t len,
                               TrackedPose& out) {
    if (!data || len < 16) return false;

    std::string addr;
    std::size_t off = consume_osc_string(data, len, addr);
    if (off == 0 || addr != kTrackedAddress) return false;

    std::string typetag;
    std::size_t adv = consume_osc_string(data + off, len - off, typetag);
    if (adv == 0 || typetag != kTrackedTypetag) return false;
    off += adv;

    constexpr std::size_t kArgBytes = 12 * 4;  // 4*i32 + 8*f32
    if (len - off < kArgBytes) return false;

    const std::uint8_t* a = data + off;
    TrackedPoseRole role;
    if (!parse_role(static_cast<std::int32_t>(read_be32(a + 0)), role)) {
        return false;
    }
    out.role            = role;
    out.device_index    = static_cast<std::int32_t>(read_be32(a + 4));
    out.valid           = (read_be32(a + 8) != 0);
    out.tracking_result = static_cast<std::int32_t>(read_be32(a + 12));
    out.timestamp_s     = read_be_float(a + 16);
    out.x               = read_be_float(a + 20);
    out.y               = read_be_float(a + 24);
    out.z               = read_be_float(a + 28);
    out.qx              = read_be_float(a + 32);
    out.qy              = read_be_float(a + 36);
    out.qz              = read_be_float(a + 40);
    out.qw              = read_be_float(a + 44);
    return true;
}

bool parse_tracked_pose_role(const std::string& s, TrackedPoseRole& out) {
    if (s == "hmd") {
        out = TrackedPoseRole::Hmd;
        return true;
    }
    if (s == "left" || s == "left_controller" || s == "left-controller") {
        out = TrackedPoseRole::LeftController;
        return true;
    }
    if (s == "right" || s == "right_controller" || s == "right-controller") {
        out = TrackedPoseRole::RightController;
        return true;
    }
    return false;
}

const char* tracked_pose_role_name(TrackedPoseRole role) {
    switch (role) {
        case TrackedPoseRole::Hmd:             return "hmd";
        case TrackedPoseRole::LeftController:  return "left";
        case TrackedPoseRole::RightController: return "right";
    }
    return "unknown";
}

TrackedPoseReceiver::TrackedPoseReceiver(HmdPoseBus& hmd_bus,
                                         ControllerPoseBus& controller_bus,
                                         TrackedPoseReceiverOptions opts)
    : hmd_bus_(hmd_bus),
      controller_bus_(controller_bus),
      opts_(std::move(opts)) {}

TrackedPoseReceiver::~TrackedPoseReceiver() { stop(); }

bool TrackedPoseReceiver::start() {
    if (sock_fd_ >= 0) return true;

    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        std::fprintf(stderr, "[tracked_pose_receiver] socket() failed: %s\n",
                     std::strerror(errno));
        return false;
    }

    timeval tv{};
    tv.tv_sec  = 0;
    tv.tv_usec = 100 * 1000;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    int one = 1;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(opts_.port);
    if (opts_.bind.empty() || opts_.bind == "0.0.0.0") {
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(AF_INET, opts_.bind.c_str(), &addr.sin_addr) != 1) {
        std::fprintf(stderr, "[tracked_pose_receiver] invalid bind address '%s'\n",
                     opts_.bind.c_str());
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "[tracked_pose_receiver] bind(%s:%u) failed: %s\n",
                     opts_.bind.c_str(), opts_.port, std::strerror(errno));
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    // Resolve the punch destination (VMT host). The punch goes out from this
    // same bound socket so the VMT driver sees our real source IP. A non-empty
    // punch_host is a fixed manual destination; an empty punch_host with a
    // discovery bus attached resolves the destination at runtime in recv_loop.
    punch_enabled_       = false;
    punch_use_discovery_ = false;
    if (!opts_.punch_host.empty() && opts_.punch_port != 0) {
        punch_addr_ = sockaddr_in{};
        punch_addr_.sin_family = AF_INET;
        punch_addr_.sin_port = htons(opts_.punch_port);
        if (::inet_pton(AF_INET, opts_.punch_host.c_str(),
                        &punch_addr_.sin_addr) == 1) {
            OscWriter w;
            w.begin_message("/fitra/punch");
            w.end_message();
            auto sp = w.data();
            punch_packet_.assign(sp.begin(), sp.end());
            punch_enabled_ = true;
            std::printf("[tracked_pose_receiver] punch -> %s:%u every %.0f ms "
                        "(makes VMT learn our IP)\n",
                        opts_.punch_host.c_str(), opts_.punch_port,
                        opts_.punch_interval_ms);
        } else {
            std::fprintf(stderr, "[tracked_pose_receiver] invalid punch_host "
                         "'%s' — punch disabled\n", opts_.punch_host.c_str());
        }
    } else if (disc_bus_ != nullptr) {
        // Discovery: punch packet is constant; only the destination is resolved
        // at runtime once a peer is adopted.
        OscWriter w;
        w.begin_message("/fitra/punch");
        w.end_message();
        auto sp = w.data();
        punch_packet_.assign(sp.begin(), sp.end());
        punch_use_discovery_ = true;
        std::printf("[tracked_pose_receiver] punch via discovery "
                    "(destination resolved at runtime, every %.0f ms)\n",
                    opts_.punch_interval_ms);
    }

    stop_.store(false);
    recv_thread_ = std::thread([this]() { recv_loop(); });
    std::printf("[tracked_pose_receiver] listening on %s:%u "
                "(stale_ms=%.1f, controller_role=%s)\n",
                opts_.bind.c_str(), opts_.port, opts_.stale_ms,
                tracked_pose_role_name(opts_.controller_role));
    return true;
}

void TrackedPoseReceiver::send_punch_() {
    if (!punch_enabled_ || sock_fd_ < 0) return;
    // Best-effort: a down host / transient error is fine, the next tick retries.
    ::sendto(sock_fd_, punch_packet_.data(), punch_packet_.size(), 0,
             reinterpret_cast<const sockaddr*>(&punch_addr_),
             sizeof(punch_addr_));
}

void TrackedPoseReceiver::stop() {
    stop_.store(true);
    if (recv_thread_.joinable()) recv_thread_.join();
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

TrackedPoseReceiverStats TrackedPoseReceiver::stats() const {
    std::lock_guard<std::mutex> g(stats_mu_);
    return stats_;
}

bool TrackedPoseReceiver::ingest_packet(const std::uint8_t* data, std::size_t len) {
    return dispatch_packet_(data, len);
}

bool TrackedPoseReceiver::dispatch_message_(const std::uint8_t* data,
                                            std::size_t len,
                                            TrackedPoseReceiverStats& delta) {
    TrackedPose tracked;
    if (parse_tracked_pose_packet(data, len, tracked)) {
        if (tracked.role == TrackedPoseRole::Hmd) {
            hmd_bus_.publish(to_hmd_pose(tracked));
        } else if (tracked.role == opts_.controller_role) {
            controller_bus_.publish(to_controller_pose(tracked));
        }
        ++delta.tracked_pose_messages;
        return true;
    }

    HmdPose hmd;
    if (parse_hmd_pose_packet(data, len, hmd)) {
        hmd_bus_.publish(hmd);
        ++delta.legacy_hmd_messages;
        return true;
    }

    ControllerPose controller;
    if (parse_controller_pose_packet(data, len, controller)) {
        controller_bus_.publish(controller);
        ++delta.legacy_controller_messages;
        return true;
    }

    return false;
}

bool TrackedPoseReceiver::dispatch_packet_(const std::uint8_t* data,
                                           std::size_t len) {
    if (!data || len == 0) return false;  // is_bundle() dereferences data
    TrackedPoseReceiverStats delta;
    bool any = false;

    if (is_bundle(data, len)) {
        std::size_t off = 16;  // "#bundle\0" + 8-byte timetag
        while (off + 4 <= len) {
            std::uint32_t elem_size = read_be32(data + off);
            off += 4;
            // off <= len here, so len - off is safe and avoids the off+elem_size
            // overflow that could otherwise bypass this bound (elem_size is
            // attacker-controlled 32-bit from the wire).
            if (elem_size == 0 || elem_size > len - off) {
                return false;
            }
            if (dispatch_message_(data + off, elem_size, delta)) {
                any = true;
            }
            off += elem_size;
        }
        if (off != len) return false;
    } else {
        any = dispatch_message_(data, len, delta);
    }

    std::lock_guard<std::mutex> g(stats_mu_);
    if (any) {
        ++stats_.packets_total;
        stats_.tracked_pose_messages += delta.tracked_pose_messages;
        stats_.legacy_hmd_messages += delta.legacy_hmd_messages;
        stats_.legacy_controller_messages += delta.legacy_controller_messages;
    } else {
        ++stats_.packets_rejected;
    }
    return any;
}

void TrackedPoseReceiver::recv_loop() {
    using clock = std::chrono::steady_clock;
    const auto punch_interval =
        std::chrono::duration<double, std::milli>(opts_.punch_interval_ms);
    // Initialize in the past so the first punch fires immediately at start-up.
    auto last_punch = clock::now() - std::chrono::hours(1);

    std::uint8_t buf[2048];
    while (!stop_.load()) {
        // Discovery: refresh the punch destination from the bus (recv-thread
        // only). last-known latch — a transient peer loss keeps the last dest.
        if (punch_use_discovery_ && disc_bus_) {
            DiscoveryEndpointLatch::Resolved r;
            if (punch_latch_.poll(*disc_bus_, r)) {
                punch_addr_    = r.addr;
                punch_enabled_ = true;
                std::printf("[tracked_pose_receiver] discovery punch -> %s:%u\n",
                            r.ip.c_str(), r.port);
            }
        }
        if (punch_enabled_) {
            const auto now = clock::now();
            if (now - last_punch >= punch_interval) {
                send_punch_();
                last_punch = now;
            }
        }
        sockaddr_in src{};
        socklen_t src_len = sizeof(src);
        ssize_t n = ::recvfrom(sock_fd_, buf, sizeof(buf), 0,
                               reinterpret_cast<sockaddr*>(&src), &src_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }
            {
                std::lock_guard<std::mutex> g(stats_mu_);
                ++stats_.recv_errors;
            }
            std::fprintf(stderr, "[tracked_pose_receiver] recvfrom failed: %s\n",
                         std::strerror(errno));
            continue;
        }
        dispatch_packet_(buf, static_cast<std::size_t>(n));
    }
}

}  // namespace fitra::vmt
