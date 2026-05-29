#pragma once
//
// Virtual Motion Tracker (VMT) publisher thread.
//
// Owns a UDP socket pointed at a VMT Manager (default 127.0.0.1:39570; usually
// the Windows host running SteamVR + VMT Driver). Runs a single paced send
// loop at `send_rate_hz` (default 60 Hz) that snapshots the SlimeTrackerBus,
// transforms each tracker into the SteamVR Driver Y-up RH frame, and emits
// one `/VMT/Room/Driver` OSC message per tracker inside one `#bundle`
// datagram.
//
// VMT does not ack on the wire (publisher → driver is one-way OSC over UDP),
// so unlike NativePublisher there is no recv loop.
//
// Architecture mirrors NativePublisher: read-only consumer of the shared
// TrackerExtractor state (single producer), no I/O on the inference threads,
// gated by Skeleton3DStats::ik_locked.

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "pipeline/snapshot.hpp"
#include "slimevr/slime_tracker_bus.hpp"
#include "slimevr/tracker_extract.hpp"
#include "vmt/osc_writer.hpp"
#include "vmt/vmt_protocol.hpp"

namespace fitra::vmt {

// What to do for a tracker whose extractor flagged `valid=false`
// (degenerate roll / occluded keypoints / invalid IK).
enum class DegenMode {
    Hold,     // enable=1, send last-known pos+quat. SteamVR IK stays settled.
    Disable,  // enable=0, send last-known pos+quat. SteamVR marks tracker non-tracking.
    Skip,     // omit the message entirely. VMT driver times the tracker out (~250ms).
};

const char* degen_mode_name(DegenMode m);
bool        parse_degen_mode(const std::string& s, DegenMode& out);

struct VmtPublisherOptions {
    std::string   host         = "127.0.0.1";
    std::uint16_t port         = 39570;       // VMT receive port
    double        send_rate_hz = 60.0;
    int           index_base   = 10;          // publish as VMT_10..VMT_19 by default
    DegenMode     degeneracy_mode = DegenMode::Hold;
    // If true, any tracker with pos.z < 0 (= below the world floor, which
    // happens when Room Matrix calibration isn't done yet) is sent with
    // enable=0 regardless of degeneracy_mode. Debug flag, default off.
    bool          disable_below_floor = false;
};

struct VmtPublisherStats {
    std::uint64_t sent_bundles            = 0; // one bundle per send-loop tick
    std::uint64_t sent_trackers           = 0; // total /VMT/Room/Driver messages
    std::uint64_t disabled_count          = 0; // messages sent with enable=0
    std::uint64_t skipped_invalid_bundles = 0; // bundle skipped: !ik_locked or no data
    double        last_send_ms            = 0.0;
    // End-to-end latency: age (ms) of the freshest 3D skeleton at the moment
    // of the last sent bundle (capture -> send). EMA-smoothed.
    double        e2e_capture_to_send_ms  = 0.0;
};

class VmtPublisher {
public:
    VmtPublisher(pipeline::Skeleton3DBus&  skel_bus,
                 slimevr::SlimeTrackerBus& tracker_bus,
                 VmtPublisherOptions       opts);
    ~VmtPublisher();

    VmtPublisher(const VmtPublisher&) = delete;
    VmtPublisher& operator=(const VmtPublisher&) = delete;

    // Open UDP socket + start send thread. Returns false (logged) on socket
    // failure; the pose pipeline keeps running in that case.
    bool start();

    // Idempotent.
    void stop();

    VmtPublisherStats stats() const;
    const VmtPublisherOptions& options() const { return opts_; }

    // Temporary runtime-only manual offset for aligning VMT trackers with the
    // SteamVR HMD playspace. Thread-safe; changes take effect on the next
    // send-loop bundle.
    void set_alignment(const VmtAlignment& alignment);
    VmtAlignment alignment() const;

private:
    void send_loop();

    pipeline::Skeleton3DBus&    skel_bus_;
    slimevr::SlimeTrackerBus&   tracker_bus_;
    VmtPublisherOptions         opts_;
    int                         sock_fd_ = -1;

    std::thread                 send_thread_;
    std::atomic<bool>           stop_{false};

    mutable std::mutex          stats_mu_;
    VmtPublisherStats           stats_;

    mutable std::mutex          alignment_mu_;
    VmtAlignment                alignment_;
};

}  // namespace fitra::vmt
