#include "vmt/controller_pose_receiver.hpp"

#include "vmt/osc_decode.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <limits>

namespace fitra::vmt {

namespace {

constexpr const char* kAddress = "/fitra/controller_pose";
constexpr const char* kTypetag = ",iiffffffff";

inline double now_steady_ms() {
    using namespace std::chrono;
    return duration_cast<duration<double, std::milli>>(
        steady_clock::now().time_since_epoch()).count();
}

using osc::consume_osc_string;
using osc::read_be32;
using osc::read_be_float;

}  // namespace

// ---------------------------------------------------------------------------
// Parser (pure function)
// ---------------------------------------------------------------------------

bool parse_controller_pose_packet(const std::uint8_t* data, std::size_t len,
                                  ControllerPose& out) {
    if (!data || len < 24) return false;  // address (24 bytes) alone is the floor

    std::string addr;
    std::size_t off = consume_osc_string(data, len, addr);
    if (off == 0 || addr != kAddress) return false;

    std::string typetag;
    std::size_t adv = consume_osc_string(data + off, len - off, typetag);
    if (adv == 0 || typetag != kTypetag) return false;
    off += adv;

    // Remaining must be exactly 10 × 4 byte args (2 × i32 + 8 × f32).
    constexpr std::size_t kArgBytes = 10 * 4;
    if (len - off < kArgBytes) return false;

    const std::uint8_t* a = data + off;
    out.valid           = (read_be32(a + 0) != 0);
    out.tracking_result = static_cast<std::int32_t>(read_be32(a + 4));
    out.timestamp_s     = read_be_float(a + 8);
    out.x               = read_be_float(a + 12);
    out.y               = read_be_float(a + 16);
    out.z               = read_be_float(a + 20);
    out.qx              = read_be_float(a + 24);
    out.qy              = read_be_float(a + 28);
    out.qz              = read_be_float(a + 32);
    out.qw              = read_be_float(a + 36);
    return true;
}

// ---------------------------------------------------------------------------
// ControllerPoseBus
// ---------------------------------------------------------------------------

void ControllerPoseBus::publish(const ControllerPose& pose) {
    std::lock_guard<std::mutex> g(mu_);
    latest_              = pose;
    have_any_            = true;
    last_recv_steady_ms_ = now_steady_ms();
}

ControllerPoseSnapshot ControllerPoseBus::snapshot(double stale_threshold_ms) const {
    std::lock_guard<std::mutex> g(mu_);
    ControllerPoseSnapshot s;
    s.have_any = have_any_;
    s.pose     = latest_;
    if (!have_any_) {
        s.age_ms = std::numeric_limits<double>::infinity();
        s.stale  = true;
    } else {
        s.age_ms = now_steady_ms() - last_recv_steady_ms_;
        s.stale  = s.age_ms > stale_threshold_ms;
    }
    return s;
}

void ControllerPoseBus::reset() {
    std::lock_guard<std::mutex> g(mu_);
    latest_              = ControllerPose{};
    have_any_            = false;
    last_recv_steady_ms_ = 0.0;
}

// ---------------------------------------------------------------------------
// ControllerPoseReceiver
// ---------------------------------------------------------------------------

ControllerPoseReceiver::ControllerPoseReceiver(ControllerPoseBus& bus,
                                               ControllerPoseReceiverOptions opts)
    : bus_(bus), opts_(std::move(opts)) {}

ControllerPoseReceiver::~ControllerPoseReceiver() { stop(); }

bool ControllerPoseReceiver::start() {
    if (sock_fd_ >= 0) return true;  // already started (idempotent-ish)

    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        std::fprintf(stderr, "[controller_pose_receiver] socket() failed: %s\n",
                     std::strerror(errno));
        return false;
    }

    // Short recvfrom timeout via SO_RCVTIMEO so stop() exits the loop quickly.
    timeval tv{};
    tv.tv_sec  = 0;
    tv.tv_usec = 100 * 1000;  // 100 ms
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    int one = 1;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(opts_.port);
    if (opts_.bind.empty() || opts_.bind == "0.0.0.0") {
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(AF_INET, opts_.bind.c_str(), &addr.sin_addr) != 1) {
        std::fprintf(stderr,
            "[controller_pose_receiver] invalid bind address '%s'\n",
            opts_.bind.c_str());
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "[controller_pose_receiver] bind(%s:%u) failed: %s\n",
                     opts_.bind.c_str(), opts_.port, std::strerror(errno));
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    stop_.store(false);
    recv_thread_ = std::thread([this]() { recv_loop(); });
    std::printf("[controller_pose_receiver] listening on %s:%u (stale_ms=%.1f)\n",
                opts_.bind.c_str(), opts_.port, opts_.stale_ms);
    return true;
}

void ControllerPoseReceiver::stop() {
    stop_.store(true);
    if (recv_thread_.joinable()) recv_thread_.join();
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

ControllerPoseReceiverStats ControllerPoseReceiver::stats() const {
    std::lock_guard<std::mutex> g(stats_mu_);
    return stats_;
}

void ControllerPoseReceiver::recv_loop() {
    std::uint8_t buf[512];  // packet is 72 bytes; oversize for safety
    while (!stop_.load()) {
        sockaddr_in src{};
        socklen_t   src_len = sizeof(src);
        ssize_t n = ::recvfrom(sock_fd_, buf, sizeof(buf), 0,
                               reinterpret_cast<sockaddr*>(&src), &src_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;  // timeout, loop again
            }
            {
                std::lock_guard<std::mutex> g(stats_mu_);
                ++stats_.recv_errors;
            }
            std::fprintf(stderr,
                "[controller_pose_receiver] recvfrom failed: %s\n",
                std::strerror(errno));
            continue;
        }

        ControllerPose pose;
        if (!parse_controller_pose_packet(buf, static_cast<std::size_t>(n), pose)) {
            std::lock_guard<std::mutex> g(stats_mu_);
            ++stats_.packets_rejected;
            continue;
        }

        bus_.publish(pose);
        {
            std::lock_guard<std::mutex> g(stats_mu_);
            ++stats_.packets_total;
        }
    }
}

}  // namespace fitra::vmt
