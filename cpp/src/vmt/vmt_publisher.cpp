#include "vmt/vmt_publisher.hpp"

#include <chrono>
#include <cstring>

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "util/logging.hpp"
#include "vmt/vmt_protocol.hpp"

namespace fitra::vmt {

const char* degen_mode_name(DegenMode m) {
    switch (m) {
        case DegenMode::Hold:    return "hold";
        case DegenMode::Disable: return "disable";
        case DegenMode::Skip:    return "skip";
    }
    return "hold";
}

bool parse_degen_mode(const std::string& s, DegenMode& out) {
    if (s == "hold")    { out = DegenMode::Hold;    return true; }
    if (s == "disable") { out = DegenMode::Disable; return true; }
    if (s == "skip")    { out = DegenMode::Skip;    return true; }
    return false;
}

namespace {

bool sendto_buf(int fd, const std::uint8_t* data, std::size_t n) {
    ssize_t r = ::send(fd, data, n, MSG_NOSIGNAL);
    if (r < 0) {
        FITRA_LOG_WARN("[vmt] send failed: {} ({})", std::strerror(errno), errno);
        return false;
    }
    if (static_cast<std::size_t>(r) != n) {
        FITRA_LOG_WARN("[vmt] send truncated: {} of {} bytes", r, n);
        return false;
    }
    return true;
}

}  // namespace

VmtPublisher::VmtPublisher(pipeline::Skeleton3DBus&  skel_bus,
                           slimevr::SlimeTrackerBus& tracker_bus,
                           VmtPublisherOptions       opts)
    : skel_bus_{skel_bus},
      tracker_bus_{tracker_bus},
      opts_{std::move(opts)} {}

VmtPublisher::~VmtPublisher() {
    try { stop(); } catch (...) {}
}

bool VmtPublisher::start() {
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        FITRA_LOG_WARN("[vmt] socket() failed: {}", std::strerror(errno));
        return false;
    }
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(opts_.port);
    if (::inet_pton(AF_INET, opts_.host.c_str(), &dst.sin_addr) != 1) {
        FITRA_LOG_WARN("[vmt] inet_pton({}) failed", opts_.host);
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }
    if (::connect(sock_fd_, reinterpret_cast<sockaddr*>(&dst), sizeof(dst)) != 0) {
        FITRA_LOG_WARN("[vmt] connect({}:{}) failed: {}",
                       opts_.host, opts_.port, std::strerror(errno));
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    stop_.store(false);
    send_thread_ = std::thread([this]() { send_loop(); });
    FITRA_LOG_INFO("[vmt] publisher up: {}:{} @ {} Hz, 10 trackers, degen={}",
                   opts_.host, opts_.port, opts_.send_rate_hz,
                   degen_mode_name(opts_.degeneracy_mode));
    return true;
}

void VmtPublisher::stop() {
    if (stop_.exchange(true)) return;
    if (send_thread_.joinable()) send_thread_.join();
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

VmtPublisherStats VmtPublisher::stats() const {
    std::lock_guard<std::mutex> lk{stats_mu_};
    return stats_;
}

void VmtPublisher::set_alignment(const VmtAlignment& alignment) {
    std::lock_guard<std::mutex> lk{alignment_mu_};
    alignment_ = alignment;
}

VmtAlignment VmtPublisher::alignment() const {
    std::lock_guard<std::mutex> lk{alignment_mu_};
    return alignment_;
}

void VmtPublisher::send_loop() {
    using clk = std::chrono::steady_clock;
    const double rate = opts_.send_rate_hz > 0.0 ? opts_.send_rate_hz : 60.0;
    const auto period_d =
        std::chrono::duration_cast<clk::duration>(
            std::chrono::duration<double>(1.0 / rate));

    OscWriter writer;
    auto next = clk::now() + period_d;

    while (!stop_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_until(next);
        const auto now = clk::now();
        next += period_d;
        // If we fell behind (system lag / suspend), reset the schedule to
        // avoid a 100% CPU spin until `next` catches up.
        if (next < now) next = now + period_d;

        auto skel_snap = skel_bus_.snapshot();
        if (!skel_snap.stats.enabled || !skel_snap.stats.ik_locked) {
            std::lock_guard<std::mutex> lk{stats_mu_};
            ++stats_.skipped_invalid_bundles;
            continue;
        }
        auto tracker_snap = tracker_bus_.snapshot();
        if (!tracker_snap.has_data) {
            std::lock_guard<std::mutex> lk{stats_mu_};
            ++stats_.skipped_invalid_bundles;
            continue;
        }

        writer.clear();
        writer.begin_bundle(OscWriter::ntp_timetag_now());
        const VmtAlignment alignment = this->alignment();

        std::uint64_t msgs_this_bundle = 0;
        std::uint64_t disabled_this_bundle = 0;

        for (std::size_t i = 0; i < slimevr::kTrackerCount; ++i) {
            const auto& t = tracker_snap.trackers[i];
            int  enable = 1;
            bool emit   = true;

            if (!t.valid) {
                switch (opts_.degeneracy_mode) {
                    case DegenMode::Hold:    enable = 1; break;
                    case DegenMode::Disable: enable = 0; break;
                    case DegenMode::Skip:    emit   = false; break;
                }
            }
            if (opts_.disable_below_floor && t.pos[2] < 0.0f) {
                enable = 0;
            }
            if (!emit) continue;

            VmtPos  pos  = world_pos_to_vmt(t.pos[0], t.pos[1], t.pos[2]);
            VmtQuat quat = world_quat_to_vmt(t.quat_wxyz[0], t.quat_wxyz[1],
                                              t.quat_wxyz[2], t.quat_wxyz[3]);
            apply_vmt_alignment(pos, quat, alignment);
            encode_vmt_room_driver(writer,
                                   vmt_index_for(t.role),
                                   enable,
                                   /*timeoffset=*/0.0f,
                                   pos, quat);
            ++msgs_this_bundle;
            if (enable == 0) ++disabled_this_bundle;
        }
        writer.end_bundle();

        if (msgs_this_bundle == 0) {
            std::lock_guard<std::mutex> lk{stats_mu_};
            ++stats_.skipped_invalid_bundles;
            continue;
        }

        auto span = writer.data();
        if (!sendto_buf(sock_fd_, span.data(), span.size())) {
            std::lock_guard<std::mutex> lk{stats_mu_};
            ++stats_.skipped_invalid_bundles;
            continue;
        }

        std::lock_guard<std::mutex> lk{stats_mu_};
        ++stats_.sent_bundles;
        stats_.sent_trackers  += msgs_this_bundle;
        stats_.disabled_count += disabled_this_bundle;
        stats_.last_send_ms = std::chrono::duration<double, std::milli>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
}

}  // namespace fitra::vmt
