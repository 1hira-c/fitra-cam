#include "vmt/hmd_pose_receiver.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

namespace fitra::vmt {

namespace {

constexpr const char* kAddress = "/fitra/hmd_pose";
constexpr const char* kTypetag = ",iffffffff";

inline double now_steady_ms() {
    using namespace std::chrono;
    return duration_cast<duration<double, std::milli>>(
        steady_clock::now().time_since_epoch()).count();
}

// Read a big-endian uint32 from `p` (no bounds check; caller has confirmed
// at least 4 bytes remain).
inline std::uint32_t read_be32(const std::uint8_t* p) {
    return (static_cast<std::uint32_t>(p[0]) << 24)
         | (static_cast<std::uint32_t>(p[1]) << 16)
         | (static_cast<std::uint32_t>(p[2]) <<  8)
         |  static_cast<std::uint32_t>(p[3]);
}

inline float read_be_float(const std::uint8_t* p) {
    std::uint32_t u = read_be32(p);
    float f;
    static_assert(sizeof(f) == sizeof(u));
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

// OSC 1.0 strings are null-terminated + zero-padded to a 4-byte boundary.
// Return the total byte count consumed (including padding) and copy the
// raw string content into `out`. Returns 0 on malformed input (no null
// before `len_remaining`).
std::size_t consume_osc_string(const std::uint8_t* p,
                               std::size_t len_remaining,
                               std::string& out) {
    std::size_t nul = 0;
    while (nul < len_remaining && p[nul] != '\0') ++nul;
    if (nul == len_remaining) return 0;       // no null terminator
    out.assign(reinterpret_cast<const char*>(p), nul);
    std::size_t total = nul + 1;              // include the null
    std::size_t pad   = (4 - (total % 4)) % 4;
    if (total + pad > len_remaining) return 0;
    return total + pad;
}

}  // namespace

// ---------------------------------------------------------------------------
// Parser (pure function)
// ---------------------------------------------------------------------------

bool parse_hmd_pose_packet(const std::uint8_t* data, std::size_t len, HmdPose& out) {
    if (!data || len < 16) return false;  // address alone is at least 16 bytes

    std::string addr;
    std::size_t off = consume_osc_string(data, len, addr);
    if (off == 0 || addr != kAddress) return false;

    std::string typetag;
    std::size_t adv = consume_osc_string(data + off, len - off, typetag);
    if (adv == 0 || typetag != kTypetag) return false;
    off += adv;

    // Remaining must be exactly 9 × 4 byte args (i32 + 8 × f32).
    constexpr std::size_t kArgBytes = 9 * 4;
    if (len - off < kArgBytes) return false;

    const std::uint8_t* a = data + off;
    out.valid       = (read_be32(a + 0)  != 0);
    out.timestamp_s = read_be_float(a + 4);
    out.x           = read_be_float(a + 8);
    out.y           = read_be_float(a + 12);
    out.z           = read_be_float(a + 16);
    out.qx          = read_be_float(a + 20);
    out.qy          = read_be_float(a + 24);
    out.qz          = read_be_float(a + 28);
    out.qw          = read_be_float(a + 32);
    return true;
}

// ---------------------------------------------------------------------------
// HmdPoseBus
// ---------------------------------------------------------------------------

void HmdPoseBus::publish(const HmdPose& pose) {
    std::lock_guard<std::mutex> g(mu_);
    latest_              = pose;
    have_any_            = true;
    last_recv_steady_ms_ = now_steady_ms();
}

HmdPoseSnapshot HmdPoseBus::snapshot(double stale_threshold_ms) const {
    std::lock_guard<std::mutex> g(mu_);
    HmdPoseSnapshot s;
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

void HmdPoseBus::reset() {
    std::lock_guard<std::mutex> g(mu_);
    latest_              = HmdPose{};
    have_any_            = false;
    last_recv_steady_ms_ = 0.0;
}

// ---------------------------------------------------------------------------
// HmdPoseReceiver
// ---------------------------------------------------------------------------

HmdPoseReceiver::HmdPoseReceiver(HmdPoseBus& bus, HmdPoseReceiverOptions opts)
    : bus_(bus), opts_(std::move(opts)) {}

HmdPoseReceiver::~HmdPoseReceiver() { stop(); }

bool HmdPoseReceiver::start() {
    if (sock_fd_ >= 0) return true;  // already started (idempotent-ish)

    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        std::fprintf(stderr, "[hmd_pose_receiver] socket() failed: %s\n",
                     std::strerror(errno));
        return false;
    }

    // Non-blocking with short recvfrom timeout via SO_RCVTIMEO so stop()
    // exits the loop quickly without needing eventfd.
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
            "[hmd_pose_receiver] invalid bind address '%s'\n",
            opts_.bind.c_str());
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "[hmd_pose_receiver] bind(%s:%u) failed: %s\n",
                     opts_.bind.c_str(), opts_.port, std::strerror(errno));
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    stop_.store(false);
    recv_thread_ = std::thread([this]() { recv_loop(); });
    std::printf("[hmd_pose_receiver] listening on %s:%u (stale_ms=%.1f)\n",
                opts_.bind.c_str(), opts_.port, opts_.stale_ms);
    return true;
}

void HmdPoseReceiver::stop() {
    stop_.store(true);
    if (recv_thread_.joinable()) recv_thread_.join();
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

HmdPoseReceiverStats HmdPoseReceiver::stats() const {
    std::lock_guard<std::mutex> g(stats_mu_);
    return stats_;
}

void HmdPoseReceiver::recv_loop() {
    std::uint8_t buf[512];  // /fitra/hmd_pose is 64 bytes; oversize for safety
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
                "[hmd_pose_receiver] recvfrom failed: %s\n",
                std::strerror(errno));
            continue;
        }

        HmdPose pose;
        if (!parse_hmd_pose_packet(buf, static_cast<std::size_t>(n), pose)) {
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
