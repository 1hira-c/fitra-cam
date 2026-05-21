#include "slimevr/native_publisher.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "util/logging.hpp"

namespace fitra::slimevr {

namespace {

// Pick the largest-area person in the snapshot for tracker extraction. Empty
// snapshot → nullptr.
const infer::Skeleton3D* pick_skeleton(const pipeline::Skeleton3DSnapshot& s) {
    if (s.persons.empty()) return nullptr;
    return &s.persons.front();   // upstream IK already picks the dominant subject
}

bool sendto_buf(int fd, const std::vector<std::uint8_t>& buf) {
    ssize_t n = ::send(fd, buf.data(), buf.size(), MSG_NOSIGNAL);
    if (n < 0) {
        FITRA_LOG_WARN("[slimevr] send failed: {} ({})", std::strerror(errno), errno);
        return false;
    }
    return true;
}

}  // namespace

NativePublisher::NativePublisher(pipeline::Skeleton3DBus& bus,
                                 NativePublisherOptions opts)
    : bus_{bus}, opts_{std::move(opts)} {
    // Initialize prev_quat to identity so the first frame doesn't slerp
    // against a degenerate zero vector.
    for (auto& q : prev_quat_) q = cv::Vec4f{1.0f, 0.0f, 0.0f, 0.0f};
}

NativePublisher::~NativePublisher() {
    try { stop(); } catch (...) {}
}

bool NativePublisher::start() {
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        FITRA_LOG_WARN("[slimevr] socket() failed: {}", std::strerror(errno));
        return false;
    }
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(opts_.port);
    if (::inet_pton(AF_INET, opts_.host.c_str(), &dst.sin_addr) != 1) {
        FITRA_LOG_WARN("[slimevr] inet_pton({}) failed", opts_.host);
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }
    // Connecting a UDP socket gives us plain send()/recv() and a stable peer
    // for the recv loop (received ping packets will all come from this peer).
    if (::connect(sock_fd_, reinterpret_cast<sockaddr*>(&dst), sizeof(dst)) != 0) {
        FITRA_LOG_WARN("[slimevr] connect({}:{}) failed: {}",
                       opts_.host, opts_.port, std::strerror(errno));
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }
    // Non-blocking recv with timeout so the recv thread can observe stop_.
    timeval tv{};
    tv.tv_sec  = 0;
    tv.tv_usec = 250 * 1000;   // 250 ms
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (!send_introduction()) {
        FITRA_LOG_WARN("[slimevr] introduction sequence failed; aborting publisher start");
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    stop_.store(false);
    send_thread_ = std::thread([this]() { send_loop(); });
    recv_thread_ = std::thread([this]() { recv_loop(); });
    FITRA_LOG_INFO("[slimevr] native publisher up: {}:{} @ {} Hz, 10 trackers",
                   opts_.host, opts_.port, opts_.send_rate_hz);
    return true;
}

void NativePublisher::stop() {
    if (stop_.exchange(true)) return;
    // Closing the socket unblocks any in-flight recv(). Joining picks up the
    // thread exits.
    if (sock_fd_ >= 0) {
        // shutdown lets a blocked recv return immediately on some kernels;
        // close handles the rest.
        ::shutdown(sock_fd_, SHUT_RDWR);
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
    if (send_thread_.joinable()) send_thread_.join();
    if (recv_thread_.joinable()) recv_thread_.join();
}

NativePublisherStats NativePublisher::stats() const {
    std::lock_guard<std::mutex> lk{stats_mu_};
    return stats_;
}

bool NativePublisher::send_introduction() {
    // Handshake (seq override = 0 by convention; firmware also uses 0 here).
    MacBytes mac = mac_from_hostname();
    auto hs = encode_handshake(0, mac, opts_.firmware_version);
    if (!sendto_buf(sock_fd_, hs)) return false;
    {
        std::lock_guard<std::mutex> lk{stats_mu_};
        ++stats_.sent_handshakes;
    }
    // SensorInfo × 10: declare body-part assignment so the SlimeVR GUI shows
    // each tracker by name rather than as a sequential placeholder.
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        TrackerRole role = static_cast<TrackerRole>(i);
        auto pkt = encode_sensor_info(sequence_++, sensor_id_for(role), position_for(role));
        if (!sendto_buf(sock_fd_, pkt)) return false;
        {
            std::lock_guard<std::mutex> lk{stats_mu_};
            ++stats_.sent_sensor_info;
        }
    }
    return true;
}

bool NativePublisher::send_rotation_burst(const pipeline::Skeleton3DSnapshot& snap) {
    const infer::Skeleton3D* sk = pick_skeleton(snap);
    if (!sk) return false;

    auto trackers = extract_trackers(*sk);
    apply_quat_smoothing(trackers, prev_quat_, opts_.quat_smooth);

    std::uint64_t sent_this_burst = 0;
    for (const auto& t : trackers) {
        if (!t.valid) continue;
        // World wxyz → SlimeVR xyzw (Y-up, Unity LH).
        QuatXyzw qs = world_quat_to_slime(t.quat_wxyz[0], t.quat_wxyz[1],
                                           t.quat_wxyz[2], t.quat_wxyz[3]);
        auto pkt = encode_rotation_data(sequence_++, sensor_id_for(t.role), qs);
        if (!sendto_buf(sock_fd_, pkt)) return false;
        ++sent_this_burst;
    }
    if (sent_this_burst == 0) return false;
    {
        std::lock_guard<std::mutex> lk{stats_mu_};
        stats_.sent_rotations += sent_this_burst;
        stats_.last_send_ms = std::chrono::duration<double, std::milli>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
    return true;
}

void NativePublisher::send_loop() {
    using clk = std::chrono::steady_clock;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, opts_.send_rate_hz));
    const auto period_d = std::chrono::duration_cast<clk::duration>(period);

    const auto heartbeat_period =
        std::chrono::duration_cast<clk::duration>(
            std::chrono::duration<double>(1.0 / std::max(0.1, opts_.heartbeat_hz)));

    auto next = clk::now() + period_d;
    auto next_heartbeat = clk::now() + heartbeat_period;

    while (!stop_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_until(next);
        next += period_d;

        auto snap = bus_.snapshot();
        if (!snap.stats.enabled || !snap.stats.ik_locked) {
            std::lock_guard<std::mutex> lk{stats_mu_};
            ++stats_.skipped_invalid;
        } else {
            if (!send_rotation_burst(snap)) {
                std::lock_guard<std::mutex> lk{stats_mu_};
                ++stats_.skipped_invalid;
            }
        }
        // Heartbeat at coarser cadence (1 Hz default) so SlimeVR keeps us
        // listed as connected even during long stretches of !ik_locked.
        if (clk::now() >= next_heartbeat) {
            auto hb = encode_heartbeat(sequence_++);
            if (sendto_buf(sock_fd_, hb)) {
                std::lock_guard<std::mutex> lk{stats_mu_};
                ++stats_.sent_heartbeats;
            }
            next_heartbeat += heartbeat_period;
        }
    }
}

void NativePublisher::recv_loop() {
    std::array<std::uint8_t, 256> buf{};
    while (!stop_.load(std::memory_order_relaxed)) {
        ssize_t n = ::recv(sock_fd_, buf.data(), buf.size(), 0);
        if (n < 0) {
            // EAGAIN/EWOULDBLOCK from the rcvtimeo: loop and re-check stop_.
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            if (errno == EBADF || errno == EINTR) break;
            FITRA_LOG_WARN("[slimevr] recv failed: {}", std::strerror(errno));
            continue;
        }
        if (n == 0) continue;

        std::uint32_t ping_id = 0;
        if (decode_ping(buf.data(), static_cast<std::size_t>(n), ping_id)) {
            auto reply = encode_ping_reply(ping_id);
            (void)sendto_buf(sock_fd_, reply);
            std::lock_guard<std::mutex> lk{stats_mu_};
            ++stats_.ping_count;
        }
        // Any other server-bound packet (config flag, etc.) is ignored for
        // now — the server tolerates trackers that don't speak those.
    }
}

}  // namespace fitra::slimevr
