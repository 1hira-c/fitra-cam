#include "slimevr/native_publisher.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
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

struct QuatWxyz {
    float w;
    float x;
    float y;
    float z;
};

QuatWxyz mul(QuatWxyz a, QuatWxyz b) {
    return QuatWxyz{
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.x*b.w + a.w*b.x - a.z*b.y + a.y*b.z,
        a.y*b.w + a.z*b.x + a.w*b.y - a.x*b.z,
        a.z*b.w - a.y*b.x + a.x*b.y + a.w*b.z,
    };
}

int normalize_quarters(int q) {
    q %= 4;
    if (q < 0) q += 4;
    if (q == 3) return -1;
    return q; // 0, 1, or 2
}

QuatWxyz quarter_axis(int quarters, char axis) {
    constexpr float kHalfPi = 1.5707963267948966f;
    float half_angle = static_cast<float>(normalize_quarters(quarters)) * kHalfPi * 0.5f;
    float c = std::cos(half_angle);
    float s = std::sin(half_angle);
    switch (axis) {
        case 'x': return QuatWxyz{c, s, 0.0f, 0.0f};
        case 'y': return QuatWxyz{c, 0.0f, s, 0.0f};
        case 'z': return QuatWxyz{c, 0.0f, 0.0f, s};
    }
    return QuatWxyz{1.0f, 0.0f, 0.0f, 0.0f};
}

QuatWxyz debug_correction_quat(const NativePublisherDebugCorrection& c) {
    // Local Y/X/Z order: yaw, then pitch, then roll in SlimeVR/Unity axes.
    return mul(mul(quarter_axis(c.yaw_quarters, 'y'),
                   quarter_axis(c.pitch_quarters, 'x')),
               quarter_axis(c.roll_quarters, 'z'));
}

bool sendto_buf(int fd, const std::vector<std::uint8_t>& buf) {
    ssize_t n = ::send(fd, buf.data(), buf.size(), MSG_NOSIGNAL);
    if (n < 0) {
        FITRA_LOG_WARN("[slimevr] send failed: {} ({})", std::strerror(errno), errno);
        return false;
    }
    return true;
}

QuatXyzw world_quat_to_preview_wire(TrackerRole role,
                                    const cv::Vec4f& quat_wxyz,
                                    bool preview_no_reset,
                                    const NativePublisherDebugCorrection& debug_correction) {
    if (!preview_no_reset) {
        return world_quat_to_slime(quat_wxyz[0], quat_wxyz[1],
                                   quat_wxyz[2], quat_wxyz[3]);
    }

    // SlimeVR's no-reset preview needs the default mountingOrientation
    // pre-cancelled, then empirical bone-space corrections confirmed through
    // the WebUI quarter-turn debugger:
    //   * chest/hip/feet: Rx(+90°)
    //   * upper arms:     Rx(-180°)
    //   * thighs/shins:   Rz(180°)
    static constexpr float kInvSqrt2 = 0.70710678f;
    QuatWxyz base{kInvSqrt2, kInvSqrt2, 0.0f, 0.0f};
    switch (role) {
        case TrackerRole::Chest:
        case TrackerRole::Waist:
        case TrackerRole::LeftFoot:
        case TrackerRole::RightFoot:
            break; // Rx(+90°)
        case TrackerRole::LeftUpperArm:
        case TrackerRole::RightUpperArm:
            base = QuatWxyz{0.0f, -1.0f, 0.0f, 0.0f}; // Rx(-180°)
            break;
        case TrackerRole::LeftUpperLeg:
        case TrackerRole::RightUpperLeg:
        case TrackerRole::LeftLowerLeg:
        case TrackerRole::RightLowerLeg:
            base = QuatWxyz{0.0f, 0.0f, 0.0f, 1.0f}; // Rz(180°)
            break;
        case TrackerRole::Count:
            break;
    }
    QuatWxyz corr = mul(base, debug_correction_quat(debug_correction));
    return world_quat_to_slime_no_reset_preview_adjusted(
        quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3],
        corr.w, corr.x, corr.y, corr.z);
}

}  // namespace

NativePublisher::NativePublisher(pipeline::Skeleton3DBus& skel_bus,
                                 SlimeTrackerBus&         tracker_bus,
                                 NativePublisherOptions   opts)
    : skel_bus_{skel_bus},
      tracker_bus_{tracker_bus},
      opts_{std::move(opts)} {}

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
    FITRA_LOG_INFO("[slimevr] native publisher up: {}:{} @ {} Hz, 10 trackers, "
                   "preview_no_reset={}",
                   opts_.host, opts_.port, opts_.send_rate_hz,
                   opts_.preview_no_reset ? "on" : "off");
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

std::array<NativePublisherDebugCorrection, kTrackerCount>
NativePublisher::debug_corrections() const {
    std::lock_guard<std::mutex> lk{debug_mu_};
    return debug_corrections_;
}

void NativePublisher::set_debug_correction(TrackerRole role,
                                           NativePublisherDebugCorrection correction) {
    if (role == TrackerRole::Count) return;
    correction.yaw_quarters = normalize_quarters(correction.yaw_quarters);
    correction.pitch_quarters = normalize_quarters(correction.pitch_quarters);
    correction.roll_quarters = normalize_quarters(correction.roll_quarters);
    std::lock_guard<std::mutex> lk{debug_mu_};
    debug_corrections_[static_cast<std::size_t>(role)] = correction;
}

void NativePublisher::reset_debug_corrections() {
    std::lock_guard<std::mutex> lk{debug_mu_};
    debug_corrections_ = {};
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

bool NativePublisher::send_rotation_burst(const SlimeTrackerSnapshot& tracker_snap) {
    if (!tracker_snap.has_data) return false;

    std::uint64_t sent_this_burst = 0;
    const auto debug_corrections = this->debug_corrections();
    for (const auto& t : tracker_snap.trackers) {
        if (!t.valid) continue;
        // World wxyz → SlimeVR xyzw (Y-up, Unity LH). preview_no_reset also
        // pre-cancels SlimeVR Server's default mountingOrientation and applies
        // role-specific pitch corrections for the GUI skeleton preview.
        const auto role_idx = static_cast<std::size_t>(t.role);
        QuatXyzw qs = world_quat_to_preview_wire(
            t.role,
            t.quat_wxyz,
            opts_.preview_no_reset,
            role_idx < debug_corrections.size()
                ? debug_corrections[role_idx]
                : NativePublisherDebugCorrection{});
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

        auto skel_snap = skel_bus_.snapshot();
        if (!skel_snap.stats.enabled || !skel_snap.stats.ik_locked) {
            std::lock_guard<std::mutex> lk{stats_mu_};
            ++stats_.skipped_invalid;
        } else {
            auto tracker_snap = tracker_bus_.snapshot();
            if (!send_rotation_burst(tracker_snap)) {
                std::lock_guard<std::mutex> lk{stats_mu_};
                ++stats_.skipped_invalid;
            } else if (skel_snap.t_capture_oldest.time_since_epoch().count() != 0) {
                // Age of the freshest 3D skeleton at send time = end-to-end
                // capture->send latency. EMA so the stat is readable at a glance.
                double e2e = std::chrono::duration<double, std::milli>(
                                 clk::now() - skel_snap.t_capture_oldest).count();
                std::lock_guard<std::mutex> lk{stats_mu_};
                stats_.e2e_capture_to_send_ms =
                    stats_.e2e_capture_to_send_ms == 0.0
                        ? e2e
                        : 0.9 * stats_.e2e_capture_to_send_ms + 0.1 * e2e;
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
