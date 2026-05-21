#pragma once
//
// Phase 11: SlimeVR Firmware UDP publisher thread.
//
// Owns a UDP socket connected to the SlimeVR Server (default 127.0.0.1:6969;
// typically the Windows host's IP in our deployment), runs its own paced send
// loop at `send_rate_hz` (default 60 Hz), and a separate recv loop that
// replies to PingPong packets so the server doesn't mark us disconnected.
//
// Architecture mirrors the (deleted) VMC publisher: snapshot the latest 3D
// skeleton under the bus mutex, build the trackers off-lock, transform to
// SlimeVR Y-up coordinates, serialize via firmware_protocol::encode_*,
// sendto(). The send loop performs no I/O on the inference threads, so
// toggling --slimevr-out should not move aggregate pose throughput.

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>

#include "pipeline/snapshot.hpp"
#include "slimevr/firmware_protocol.hpp"
#include "slimevr/tracker_extract.hpp"

namespace fitra::slimevr {

struct NativePublisherOptions {
    std::string   host         = "127.0.0.1";
    std::uint16_t port         = kDefaultPort;   // 6969
    double        send_rate_hz = 60.0;
    float         quat_smooth  = 0.5f;           // 0 = freeze, 1 = no smoothing
    double        heartbeat_hz = 1.0;            // background keep-alive
    std::string   firmware_version = "fitra-cam 0.1";
};

struct NativePublisherStats {
    std::uint64_t sent_handshakes  = 0;
    std::uint64_t sent_sensor_info = 0;
    std::uint64_t sent_rotations   = 0;   // total RotationData packets across all sensors
    std::uint64_t sent_heartbeats  = 0;
    std::uint64_t skipped_invalid  = 0;   // !ik_locked or empty snapshot
    std::uint64_t ping_count       = 0;   // server pings we replied to
    double        last_send_ms     = 0.0; // wall-clock of last RotationData burst
};

class NativePublisher {
public:
    NativePublisher(pipeline::Skeleton3DBus& bus, NativePublisherOptions opts);
    ~NativePublisher();

    NativePublisher(const NativePublisher&) = delete;
    NativePublisher& operator=(const NativePublisher&) = delete;

    // Open the UDP socket, send the initial handshake + SensorInfo declarations,
    // start the send + recv threads. Returns false (with a logged warning) if
    // socket setup fails; the pose pipeline keeps running in that case.
    bool start();

    // Idempotent. Joinable from any thread; destructor calls it.
    void stop();

    NativePublisherStats stats() const;
    const NativePublisherOptions& options() const { return opts_; }

private:
    void send_loop();
    void recv_loop();

    // Build and dispatch one RotationData burst for `snap`. Updates
    // `prev_quat_` (in-place smoothing) and the rotation stats. Returns true
    // if at least one tracker was sent.
    bool send_rotation_burst(const pipeline::Skeleton3DSnapshot& snap);

    // One-shot handshake + 10× SensorInfo on startup.
    bool send_introduction();

    pipeline::Skeleton3DBus&        bus_;
    NativePublisherOptions          opts_;
    int                             sock_fd_ = -1;

    std::thread                     send_thread_;
    std::thread                     recv_thread_;
    std::atomic<bool>               stop_{false};

    std::uint64_t                   sequence_ = 1;   // Handshake uses 0, then we increment
    std::array<cv::Vec4f, kTrackerCount> prev_quat_{};

    mutable std::mutex              stats_mu_;
    NativePublisherStats            stats_;
};

}  // namespace fitra::slimevr
