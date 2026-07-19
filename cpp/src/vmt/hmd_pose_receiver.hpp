#pragma once
//
// Receive HMD pose datagrams from the Windows-side
// `vmt_hmd_pose_sender.exe`.
//
// Wire format (single OSC 1.0 message per UDP packet, no bundle):
//
//   address  = "/fitra/hmd_pose"
//   typetag  = ",iffffffff"
//   args     = valid(i32) timestamp_s(f32)
//              x(f32) y(f32) z(f32) qx(f32) qy(f32) qz(f32) qw(f32)
//
// Coordinate frame matches `world_pos_to_vmt` / `world_quat_to_vmt` output
// — SteamVR Standing universe, Y-up RH, X-right, Z-back, metres. The auto
// alignment solver compares this directly against the chest tracker pose
// that VmtPublisher would emit.
//

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace fitra::vmt {

struct HmdPose {
    bool   valid       = false;
    float  timestamp_s = 0.0f;  // seconds since the sender booted
    float  x = 0.0f, y = 0.0f, z = 0.0f;
    float  qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f;
};

struct HmdPoseSnapshot {
    HmdPose pose;
    bool    have_any = false;   // ever received a packet
    double  age_ms   = 0.0;     // monotonic ms since last packet (∞ if none)
    bool    stale    = true;    // age_ms > stale_threshold_ms
};

// Latest-wins single-slot HmdPose, mirroring TrackerBus's snapshot
// pattern. Producer = HmdPoseReceiver thread; consumer = any number of
// solver/Crow threads.
class HmdPoseBus {
public:
    HmdPoseBus() = default;
    HmdPoseBus(const HmdPoseBus&) = delete;
    HmdPoseBus& operator=(const HmdPoseBus&) = delete;

    void publish(const HmdPose& pose);
    HmdPoseSnapshot snapshot(double stale_threshold_ms) const;
    void reset();

private:
    mutable std::mutex mu_;
    HmdPose            latest_{};
    bool               have_any_         = false;
    // monotonic milliseconds since steady_clock epoch when latest_ was set
    double             last_recv_steady_ms_ = 0.0;
};

struct HmdPoseReceiverOptions {
    std::string   bind        = "0.0.0.0";
    std::uint16_t port        = 39571;
    double        stale_ms    = 200.0;     // for snapshot.stale flag
};

struct HmdPoseReceiverStats {
    std::uint64_t packets_total       = 0;  // datagrams that passed parse
    std::uint64_t packets_rejected    = 0;  // wrong address / typetag / size
    std::uint64_t recv_errors         = 0;  // recvfrom() syscall errors
};

class HmdPoseReceiver {
public:
    HmdPoseReceiver(HmdPoseBus& bus, HmdPoseReceiverOptions opts);
    ~HmdPoseReceiver();

    HmdPoseReceiver(const HmdPoseReceiver&) = delete;
    HmdPoseReceiver& operator=(const HmdPoseReceiver&) = delete;

    // Open UDP socket and start the listen thread. Returns false (logged)
    // on socket failure; the pose pipeline keeps running.
    bool start();
    void stop();  // idempotent

    HmdPoseReceiverStats stats() const;
    const HmdPoseReceiverOptions& options() const { return opts_; }

private:
    void recv_loop();

    HmdPoseBus&             bus_;
    HmdPoseReceiverOptions  opts_;
    int                     sock_fd_ = -1;
    std::thread             recv_thread_;
    std::atomic<bool>       stop_{false};

    mutable std::mutex      stats_mu_;
    HmdPoseReceiverStats    stats_;
};

// Parse a single OSC 1.0 `/fitra/hmd_pose ,iffffffff` packet. Returns true
// on success and populates `out`. Returns false on malformed / unexpected
// address or typetag. Pure function — no I/O, used both by the receive loop
// and unit tests.
bool parse_hmd_pose_packet(const std::uint8_t* data, std::size_t len, HmdPose& out);

}  // namespace fitra::vmt
