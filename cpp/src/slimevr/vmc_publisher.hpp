#pragma once
//
// Phase 11: UDP publisher thread that snapshots the live 3D skeleton at a
// fixed rate (default 60 Hz), converts to 8 VMC trackers, and sends one
// OSC bundle per cycle to a SlimeVR Server (or any VMC receiver).
//
// Architecture mirrors `web::CrowServer::publisher_loop`: a single std::thread
// paced via `steady_clock + sleep_until`, copy-out of the latest snapshot
// under the bus mutex, then off-lock OSC build + sendto. The publisher
// performs no I/O on the inference threads, so toggling --slimevr-out should
// not move the aggregate pose throughput.

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>

#include "pipeline/snapshot.hpp"
#include "slimevr/osc_writer.hpp"
#include "slimevr/tracker_extract.hpp"

namespace fitra::slimevr {

struct VmcPublisherOptions {
    std::string host          = "127.0.0.1";
    std::uint16_t port        = 39539;       // VMC Marionette/receiver default
    double      send_rate_hz  = 60.0;
    float       quat_smooth   = 0.5f;        // 0 = freeze, 1 = no smoothing
    bool        send_root_pos = true;        // /VMC/Ext/Root/Pos identity
    bool        send_time     = true;        // /VMC/Ext/T loop time
};

struct VmcPublisherStats {
    std::uint64_t sent_bundles    = 0;
    std::uint64_t skipped_invalid = 0;  // !ik_locked or empty snapshot
    double        last_send_ms    = 0.0;
};

class VmcPublisher {
public:
    VmcPublisher(pipeline::Skeleton3DBus& bus, VmcPublisherOptions opts);
    ~VmcPublisher();

    VmcPublisher(const VmcPublisher&) = delete;
    VmcPublisher& operator=(const VmcPublisher&) = delete;

    // Open the UDP socket and start the loop. Returns false (with a logged
    // warning) if the socket setup fails -- pose pipeline keeps running.
    bool start();

    // Idempotent. Joinable from any thread; destructor calls it.
    void stop();

    VmcPublisherStats stats() const;
    const VmcPublisherOptions& options() const { return opts_; }

private:
    void loop();

    pipeline::Skeleton3DBus&        bus_;
    VmcPublisherOptions             opts_;
    int                             sock_fd_ = -1;
    std::thread                     thread_;
    std::atomic<bool>               stop_{false};
    std::array<cv::Vec4f, kTrackerCount> prev_quat_{};
    OscWriter                       writer_;
    std::chrono::steady_clock::time_point start_time_{};
    mutable std::mutex              stats_mu_;
    VmcPublisherStats               stats_;
};

}  // namespace fitra::slimevr
