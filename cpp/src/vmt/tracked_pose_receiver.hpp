#pragma once
//
// Unified VMT pose relay receiver.
//
// Canonical wire format (single OSC message, or message elements inside an
// OSC bundle):
//
//   address = "/fitra/tracked_pose"
//   typetag = ",iiiiffffffff"
//   args    = role(i32), device_index(i32), valid(i32), tracking_result(i32),
//             timestamp_s(f32),
//             x(f32), y(f32), z(f32), qx(f32), qy(f32), qz(f32), qw(f32)
//
// Roles: 0=HMD, 1=left controller, 2=right controller. Poses are absolute
// SteamVR Standing universe poses in metres, quaternion xyzw.

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <netinet/in.h>   // sockaddr_in for the punch destination

#include "vmt/controller_pose_receiver.hpp"
#include "vmt/hmd_pose_receiver.hpp"

namespace fitra::vmt {

enum class TrackedPoseRole : std::int32_t {
    Hmd             = 0,
    LeftController  = 1,
    RightController = 2,
};

struct TrackedPose {
    TrackedPoseRole role = TrackedPoseRole::Hmd;
    std::int32_t    device_index = -1;
    bool            valid = false;
    std::int32_t    tracking_result = 0;
    float           timestamp_s = 0.0f;
    float           x = 0.0f, y = 0.0f, z = 0.0f;
    float           qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f;

    bool running_ok() const {
        return valid && tracking_result == kTrackingResultRunningOk;
    }
};

struct TrackedPoseReceiverOptions {
    std::string     bind = "0.0.0.0";
    std::uint16_t   port = 39571;
    double          stale_ms = 200.0;
    TrackedPoseRole controller_role = TrackedPoseRole::RightController;

    // Outbound "punch": periodically send a tiny OSC datagram from the bound
    // receive socket to the VMT host, so the VMT driver learns our (Jetson)
    // source IP and starts relaying poses back. VMT's driver records the
    // source IP of any non-loopback OSC packet it receives, before address
    // dispatch (see refs/VirtualMotionTracker CommunicationManager.cpp,
    // "Phase 15.5"), so the punch content is irrelevant — only the round trip
    // matters. Without this, a mode that runs no VMT publisher (calib-extrinsic)
    // never sends anything to the host, so no controller/HMD pose ever comes
    // back. Empty host (or port 0) disables the punch.
    //
    // TODO(pose-3d/flow-daemon): replace the unicast punch with broadcast/
    // multicast auto-discovery so the host IP need not be configured.
    std::string     punch_host;             // VMT host (PC); empty = disabled
    std::uint16_t   punch_port = 0;          // VMT OSC receive port (e.g. 39570)
    double          punch_interval_ms = 1000.0;
};

struct TrackedPoseReceiverStats {
    std::uint64_t packets_total = 0;
    std::uint64_t packets_rejected = 0;
    std::uint64_t recv_errors = 0;
    std::uint64_t tracked_pose_messages = 0;
    std::uint64_t legacy_hmd_messages = 0;
    std::uint64_t legacy_controller_messages = 0;
};

class TrackedPoseReceiver {
public:
    TrackedPoseReceiver(HmdPoseBus& hmd_bus,
                        ControllerPoseBus& controller_bus,
                        TrackedPoseReceiverOptions opts);
    ~TrackedPoseReceiver();

    TrackedPoseReceiver(const TrackedPoseReceiver&) = delete;
    TrackedPoseReceiver& operator=(const TrackedPoseReceiver&) = delete;

    bool start();
    void stop();

    // Test/diagnostic entry point: parse and dispatch one UDP payload without
    // using the socket receive loop. Accepts either a single OSC message or an
    // OSC bundle containing pose messages.
    bool ingest_packet(const std::uint8_t* data, std::size_t len);

    TrackedPoseReceiverStats stats() const;
    const TrackedPoseReceiverOptions& options() const { return opts_; }

private:
    void recv_loop();
    bool dispatch_packet_(const std::uint8_t* data, std::size_t len);
    bool dispatch_message_(const std::uint8_t* data, std::size_t len,
                           TrackedPoseReceiverStats& delta);
    void send_punch_();   // no-op when punch is disabled

    HmdPoseBus&                hmd_bus_;
    ControllerPoseBus&         controller_bus_;
    TrackedPoseReceiverOptions opts_;
    int                        sock_fd_ = -1;
    std::thread                recv_thread_;
    std::atomic<bool>          stop_{false};

    // Punch state (resolved in start(); used only by recv_loop's thread).
    bool                       punch_enabled_ = false;
    sockaddr_in                punch_addr_{};
    std::vector<std::uint8_t>  punch_packet_;

    mutable std::mutex         stats_mu_;
    TrackedPoseReceiverStats   stats_;
};

bool parse_tracked_pose_packet(const std::uint8_t* data, std::size_t len,
                               TrackedPose& out);

bool parse_tracked_pose_role(const std::string& s, TrackedPoseRole& out);
const char* tracked_pose_role_name(TrackedPoseRole role);

}  // namespace fitra::vmt
