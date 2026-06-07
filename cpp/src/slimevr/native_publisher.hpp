#pragma once
//
// SlimeVR Firmware UDP publisher thread.
//
// Owns a UDP socket connected to the SlimeVR Server (default 127.0.0.1:6969;
// typically the Windows host's IP in our deployment), runs its own paced send
// loop at `send_rate_hz` (default 60 Hz), and a separate recv loop that
// replies to PingPong packets so the server doesn't mark us disconnected.
//
// Hot path: snapshot the smoothed trackers from SlimeTrackerBus, transform to
// SlimeVR Y-up coordinates, serialize via firmware_protocol::encode_*,
// sendto(). The send loop performs no I/O on the inference threads, so
// toggling --slimevr-out should not move aggregate pose throughput.

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "pipeline/snapshot.hpp"
#include "slimevr/firmware_protocol.hpp"
#include "slimevr/slime_tracker_bus.hpp"
#include "slimevr/tracker_extract.hpp"

namespace fitra::slimevr {

struct NativePublisherOptions {
    std::string   host         = "127.0.0.1";
    std::uint16_t port         = kDefaultPort;   // 6969
    double        send_rate_hz = 60.0;
    float         quat_smooth  = 0.5f;           // 0 = freeze, 1 = no smoothing
    double        heartbeat_hz = 1.0;            // background keep-alive
    std::string   firmware_version = "fitra-cam 0.1";
    // Pre-cancel SlimeVR Server's default mountingOrientation so the GUI
    // skeleton preview matches fitra-cam's bone-space rotations before any
    // SlimeVR full/yaw/mounting reset.
    bool          preview_no_reset = false;
};

struct NativePublisherStats {
    std::uint64_t sent_handshakes  = 0;
    std::uint64_t sent_sensor_info = 0;
    std::uint64_t sent_rotations   = 0;   // total RotationData packets across all sensors
    std::uint64_t sent_heartbeats  = 0;
    std::uint64_t skipped_invalid  = 0;   // !ik_locked or empty snapshot
    std::uint64_t ping_count       = 0;   // server pings we replied to
    double        last_send_ms     = 0.0; // wall-clock of last RotationData burst
    // End-to-end latency: age (ms) of the freshest 3D skeleton at the moment
    // of the last successful RotationData burst (capture -> send). EMA-smoothed.
    double        e2e_capture_to_send_ms = 0.0;
};

struct NativePublisherDebugCorrection {
    // Quarter turns in SlimeVR/Unity local axes, applied after the built-in
    // no-reset preview correction. 1 = +90 degrees, 2 = 180 degrees.
    int yaw_quarters   = 0;  // local Y
    int pitch_quarters = 0;  // local X
    int roll_quarters  = 0;  // local Z
};

class NativePublisher {
public:
    // Consumes pre-smoothed tracker snapshots from `tracker_bus` (produced by
    // TrackerExtractor). Smoothing state lives there so the WebUI viz and the
    // UDP send path share one history.
    //
    // `skel_bus` is read only for the ik_locked / enabled flag in
    // Skeleton3DStats: the publisher skips RotationData bursts while 3D
    // inference is paused.
    NativePublisher(pipeline::Skeleton3DBus& skel_bus,
                    SlimeTrackerBus&         tracker_bus,
                    NativePublisherOptions   opts);
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
    std::array<NativePublisherDebugCorrection, kTrackerCount>
    debug_corrections() const;
    void set_debug_correction(TrackerRole role,
                              NativePublisherDebugCorrection correction);
    void reset_debug_corrections();

private:
    void send_loop();
    void recv_loop();

    // Serialize one RotationData burst from `trackers`. Returns true if at
    // least one tracker was sent.
    bool send_rotation_burst(const SlimeTrackerSnapshot& tracker_snap);

    // One-shot handshake + 10× SensorInfo on startup.
    bool send_introduction();

    pipeline::Skeleton3DBus&        skel_bus_;     // ik_locked gating only
    SlimeTrackerBus&                tracker_bus_;  // smoothed tracker source
    NativePublisherOptions          opts_;
    int                             sock_fd_ = -1;

    std::thread                     send_thread_;
    std::thread                     recv_thread_;
    std::atomic<bool>               stop_{false};

    std::uint64_t                   sequence_ = 1;   // Handshake uses 0, then we increment

    mutable std::mutex              stats_mu_;
    NativePublisherStats            stats_;

    mutable std::mutex              debug_mu_;
    std::array<NativePublisherDebugCorrection, kTrackerCount> debug_corrections_{};
};

}  // namespace fitra::slimevr
