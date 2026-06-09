#pragma once
//
// Receive VR controller pose datagrams from the Windows-side sender, used as
// the moving 6DoF reference for the controller-marker extrinsic calibration
// (see docs/design/pose-3d-controller-marker-extrinsic.md). This is a parallel
// channel to the HMD pose path (hmd_pose_receiver.hpp): same transport, same
// Standing + VMT coordinate frame, latest-wins bus.
//
// Wire format (single OSC 1.0 message per UDP packet, no bundle):
//
//   address  = "/fitra/controller_pose"
//   typetag  = ",iiffffffff"
//   args     = valid(i32) tracking_result(i32) timestamp_s(f32)
//              x(f32) y(f32) z(f32) qx(f32) qy(f32) qz(f32) qw(f32)
//
// The extra `tracking_result` (vs the HMD `,iffffffff`) carries OpenVR's
// ETrackingResult so the collector can drop `valid` but degraded poses
// (extrapolated / out-of-range / rotation-only) — `bPoseIsValid` alone is not
// enough for sub-mm calibration. The hand-eye gate requires both
// `valid == true` and `tracking_result == kTrackingResultRunningOk`.
//
// Coordinate frame matches `world_pos_to_vmt` / `world_quat_to_vmt` output
// (SteamVR Standing universe, Y-up RH, X-right, Z-back, metres) so the
// controller pose lines up with the HMD pose and the VMT trackers in one frame.

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace fitra::vmt {

// OpenVR ETrackingResult value for a fully tracked device. Mirrors
// vr::TrackingResult_Running_OK so the gate can be expressed without pulling
// in the OpenVR headers on the Jetson side.
constexpr std::int32_t kTrackingResultRunningOk = 200;

struct ControllerPose {
    bool         valid           = false;
    std::int32_t tracking_result = 0;     // OpenVR ETrackingResult (200 = OK)
    float        timestamp_s     = 0.0f;  // seconds since the sender booted
    float        x = 0.0f, y = 0.0f, z = 0.0f;
    float        qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f;

    // Calibration-grade gate: tracked AND not degraded. The collector samples
    // only when this holds (plus its own motion gate).
    bool running_ok() const {
        return valid && tracking_result == kTrackingResultRunningOk;
    }
};

struct ControllerPoseSnapshot {
    ControllerPose pose;
    bool           have_any = false;   // ever received a packet
    double         age_ms   = 0.0;     // monotonic ms since last packet (∞ if none)
    bool           stale    = true;    // age_ms > stale_threshold_ms
};

// Latest-wins single-slot ControllerPose, mirroring HmdPoseBus. Producer =
// ControllerPoseReceiver thread; consumer = any number of collector / Crow
// threads.
class ControllerPoseBus {
public:
    ControllerPoseBus() = default;
    ControllerPoseBus(const ControllerPoseBus&) = delete;
    ControllerPoseBus& operator=(const ControllerPoseBus&) = delete;

    void publish(const ControllerPose& pose);
    ControllerPoseSnapshot snapshot(double stale_threshold_ms) const;
    void reset();

private:
    mutable std::mutex mu_;
    ControllerPose     latest_{};
    bool               have_any_            = false;
    double             last_recv_steady_ms_ = 0.0;
};

struct ControllerPoseReceiverOptions {
    std::string   bind     = "0.0.0.0";
    std::uint16_t port     = 39572;     // HMD path uses 39571
    double        stale_ms = 200.0;
};

struct ControllerPoseReceiverStats {
    std::uint64_t packets_total    = 0;
    std::uint64_t packets_rejected = 0;
    std::uint64_t recv_errors      = 0;
};

class ControllerPoseReceiver {
public:
    ControllerPoseReceiver(ControllerPoseBus& bus, ControllerPoseReceiverOptions opts);
    ~ControllerPoseReceiver();

    ControllerPoseReceiver(const ControllerPoseReceiver&) = delete;
    ControllerPoseReceiver& operator=(const ControllerPoseReceiver&) = delete;

    bool start();
    void stop();  // idempotent

    ControllerPoseReceiverStats stats() const;
    const ControllerPoseReceiverOptions& options() const { return opts_; }

private:
    void recv_loop();

    ControllerPoseBus&            bus_;
    ControllerPoseReceiverOptions opts_;
    int                           sock_fd_ = -1;
    std::thread                   recv_thread_;
    std::atomic<bool>             stop_{false};

    mutable std::mutex            stats_mu_;
    ControllerPoseReceiverStats   stats_;
};

// Parse a single OSC 1.0 `/fitra/controller_pose ,iiffffffff` packet. Returns
// true and populates `out` on success; false on malformed / unexpected address
// or typetag. Pure function — used by both the receive loop and unit tests.
bool parse_controller_pose_packet(const std::uint8_t* data, std::size_t len,
                                  ControllerPose& out);

}  // namespace fitra::vmt
