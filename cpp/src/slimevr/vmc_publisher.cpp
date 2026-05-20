#include "slimevr/vmc_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "util/logging.hpp"

namespace fitra::slimevr {

namespace {

constexpr double kStatsLogIntervalSec = 5.0;

}  // namespace

VmcPublisher::VmcPublisher(pipeline::Skeleton3DBus& bus, VmcPublisherOptions opts)
    : bus_(bus), opts_(std::move(opts)) {
    for (auto& q : prev_quat_) q = cv::Vec4f{1.0f, 0.0f, 0.0f, 0.0f};
}

VmcPublisher::~VmcPublisher() {
    try { stop(); } catch (...) {}
}

bool VmcPublisher::start() {
    if (thread_.joinable()) return true;  // already running

    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        FITRA_LOG_WARN("[slimevr] socket() failed: {} -- VMC publisher disabled",
                       std::strerror(errno));
        return false;
    }

    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(opts_.port);
    if (::inet_pton(AF_INET, opts_.host.c_str(), &sa.sin_addr) != 1) {
        FITRA_LOG_WARN("[slimevr] inet_pton failed for host={} -- VMC publisher disabled",
                       opts_.host);
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }
    // connect() pins the default destination so we can use send() and skip
    // re-resolving the sockaddr per cycle. The error path falls back to
    // sendto inside the loop.
    if (::connect(sock_fd_, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) < 0) {
        FITRA_LOG_WARN("[slimevr] connect() failed: {} -- VMC publisher disabled",
                       std::strerror(errno));
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    stop_.store(false);
    start_time_ = std::chrono::steady_clock::now();
    thread_ = std::thread{&VmcPublisher::loop, this};
    FITRA_LOG_INFO("[slimevr] VMC publisher started -> udp://{}:{} @ {:.1f} Hz",
                   opts_.host, opts_.port, opts_.send_rate_hz);
    return true;
}

void VmcPublisher::stop() {
    if (!thread_.joinable() && sock_fd_ < 0) return;
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

VmcPublisherStats VmcPublisher::stats() const {
    std::lock_guard<std::mutex> lk{stats_mu_};
    return stats_;
}

void VmcPublisher::loop() {
    using clock = std::chrono::steady_clock;
    auto period = std::chrono::duration<double>(1.0 / std::max(opts_.send_rate_hz, 1.0));
    auto next = clock::now();
    auto last_stats_log = next;
    std::uint64_t last_log_sent = 0;
    std::uint64_t last_log_skipped = 0;

    while (!stop_.load()) {
        next += std::chrono::duration_cast<clock::duration>(period);
        std::this_thread::sleep_until(next);
        if (stop_.load()) break;

        auto loop_start = clock::now();
        auto snap = bus_.snapshot();
        if (!snap.stats.enabled || snap.persons.empty() || !snap.stats.ik_locked) {
            // Without an IK lock the bone lengths are still drifting; sending
            // would scatter the avatar. Just count the skip and continue.
            std::lock_guard<std::mutex> lk{stats_mu_};
            ++stats_.skipped_invalid;
            continue;
        }

        auto trackers = extract_vmc_trackers(snap.persons.front());
        apply_quat_smoothing(trackers, prev_quat_, opts_.quat_smooth);

        writer_.clear();
        writer_.begin_bundle(OscWriter::ntp_timetag_now());

        if (opts_.send_time) {
            double t_loop = std::chrono::duration<double>(loop_start - start_time_).count();
            writer_.begin_message("/VMC/Ext/T");
            writer_.add_float(static_cast<float>(t_loop));
            writer_.end_message();
        }
        if (opts_.send_root_pos) {
            // /VMC/Ext/Root/Pos {name} {pos*3} {quat*4} -- world root anchor.
            // We park the root at the world origin with identity orientation
            // because all tracker positions are already in absolute Unity
            // coordinates.
            writer_.begin_message("/VMC/Ext/Root/Pos");
            writer_.add_string("root");
            writer_.add_float(0.0f); writer_.add_float(0.0f); writer_.add_float(0.0f);
            writer_.add_float(0.0f); writer_.add_float(0.0f); writer_.add_float(0.0f);
            writer_.add_float(1.0f);
            writer_.end_message();
        }
        for (const auto& t : trackers) {
            if (!t.valid) continue;
            writer_.begin_message("/VMC/Ext/Tra/Pos");
            writer_.add_string(kRoleNames[static_cast<std::size_t>(t.role)]);
            writer_.add_float(t.pos[0]);
            writer_.add_float(t.pos[1]);
            writer_.add_float(t.pos[2]);
            // Wire order is xyzw; VmcTracker.quat_wxyz stores wxyz so we
            // read [1..3, 0] to get x, y, z, w.
            writer_.add_float(t.quat_wxyz[1]);
            writer_.add_float(t.quat_wxyz[2]);
            writer_.add_float(t.quat_wxyz[3]);
            writer_.add_float(t.quat_wxyz[0]);
            writer_.end_message();
        }
        writer_.end_bundle();

        auto packet = writer_.data();
        // Use send() since we connect()'d in start(); fall back to sendto on
        // ENOTCONN (e.g., if connect failed but we got here anyway).
        ::ssize_t n = ::send(sock_fd_, packet.data(), packet.size(), 0);
        if (n < 0) {
            FITRA_LOG_WARN("[slimevr] send failed: {} ({} bytes pending)",
                           std::strerror(errno), packet.size());
        }
        auto loop_end = clock::now();
        {
            std::lock_guard<std::mutex> lk{stats_mu_};
            ++stats_.sent_bundles;
            stats_.last_send_ms = std::chrono::duration<double, std::milli>(
                                      loop_end - loop_start).count();
        }

        if (std::chrono::duration<double>(loop_end - last_stats_log).count()
            >= kStatsLogIntervalSec) {
            VmcPublisherStats snapshot_stats;
            {
                std::lock_guard<std::mutex> lk{stats_mu_};
                snapshot_stats = stats_;
            }
            auto sent_delta    = snapshot_stats.sent_bundles    - last_log_sent;
            auto skipped_delta = snapshot_stats.skipped_invalid - last_log_skipped;
            double elapsed = std::chrono::duration<double>(loop_end - last_stats_log).count();
            FITRA_LOG_INFO(
                "[slimevr] sent={} skipped={} rate={:.1f} Hz last_send={:.2f} ms",
                sent_delta, skipped_delta, sent_delta / elapsed,
                snapshot_stats.last_send_ms);
            last_stats_log = loop_end;
            last_log_sent = snapshot_stats.sent_bundles;
            last_log_skipped = snapshot_stats.skipped_invalid;
        }
    }
}

}  // namespace fitra::slimevr
