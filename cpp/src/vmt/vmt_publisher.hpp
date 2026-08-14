#pragma once
//
// Virtual Motion Tracker (VMT) publisher thread.
//
// Owns a UDP socket pointed at a VMT Manager (default 127.0.0.1:39570; usually
// the Windows host running SteamVR + VMT Driver). Runs a single paced send
// loop at `send_rate_hz` (default 60 Hz) that snapshots the TrackerBus,
// transforms each tracker into the SteamVR Driver Y-up RH frame, and emits
// one `/VMT/Room/Driver` OSC message per tracker inside one `#bundle`
// datagram.
//
// VMT does not ack on the wire (publisher → driver is one-way OSC over UDP),
// so the publisher has no receive loop.
//
// Read-only consumer of the shared TrackerExtractor state (single producer),
// with no I/O on the inference threads. The normal source remains gated by
// Skeleton3DStats::ik_locked; an explicit raw source is intentionally ready
// without an IK lock.

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <netinet/in.h>   // sockaddr_in (runtime-swappable destination)

#include "pipeline/snapshot.hpp"
#include "tracking/tracker_bus.hpp"
#include "tracking/tracker_extract.hpp"
#include "vmt/discovery_endpoint.hpp"   // DiscoveryEndpointLatch
#include "vmt/osc_writer.hpp"
#include "vmt/peer_registry.hpp"   // DiscoveryEndpointBus
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

// The legacy postprocessed path requires a locked IK solver before VMT sends.
// Raw source mode deliberately bypasses that solver, so it instead requires at
// least one valid triangulated joint. The data check rejects startup, sync-miss,
// and idle snapshots even though they retain the raw-source provenance marker.
bool vmt_skeleton_ready(const pipeline::Skeleton3DStats& stats);

struct VmtPublisherOptions {
    std::string   host         = "127.0.0.1";
    std::uint16_t port         = 39570;       // VMT receive port
    double        send_rate_hz = 60.0;
    int           index_base   = 10;          // publish as VMT_10..VMT_19 by default
    // Which TrackerRoles to publish. Default P8 = VRChat standard 8-point
    // (hip/chest/feet/knees/elbows); the shin roles are dropped. See
    // VmtTrackerPreset in vmt_protocol.hpp. Runtime-settable via set_preset().
    VmtTrackerPreset preset     = VmtTrackerPreset::P8;
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
    std::uint64_t skipped_invalid_bundles = 0; // source not ready or no tracker data
    std::uint64_t skipped_no_endpoint     = 0; // discovery: no peer resolved yet
    double        last_send_ms            = 0.0;
    // End-to-end latency: age (ms) of the freshest 3D skeleton at the moment
    // of the last sent bundle (capture -> send). EMA-smoothed.
    double        e2e_capture_to_send_ms  = 0.0;
};

class VmtPublisher {
public:
    VmtPublisher(pipeline::Skeleton3DBus&  skel_bus,
                 tracking::TrackerBus& tracker_bus,
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

    // Runtime-settable tracker preset (which TrackerRoles are published).
    // Thread-safe; takes effect on the next send-loop bundle. Same ownership
    // model as set_alignment. NB: changing the published tracker set requires
    // re-running VRChat FBT calibration.
    void set_preset(VmtTrackerPreset preset);
    VmtTrackerPreset preset() const;

    // Discovery wiring. Call set_discovery_bus() BEFORE start(). When the
    // configured `host` is empty and a bus is attached, the send destination is
    // resolved at runtime from the bus (zeroconf); a non-empty `host` always
    // wins (manual override) and the bus is ignored.
    void set_discovery_bus(const DiscoveryEndpointBus* bus) { disc_bus_ = bus; }

    // Thread-safe runtime destination setter (mirrors set_alignment). Returns
    // false (and leaves the destination unchanged) when `ip` fails to parse,
    // so callers never latch a destination that was never applied.
    bool set_destination(const std::string& ip, std::uint16_t port);
    bool have_destination() const;

private:
    void send_loop();
    void refresh_discovery_destination_();  // send-thread only
    void apply_destination_(const sockaddr_in& dst);  // locks dst_mu_

    pipeline::Skeleton3DBus&    skel_bus_;
    tracking::TrackerBus&   tracker_bus_;
    VmtPublisherOptions         opts_;
    int                         sock_fd_ = -1;

    std::thread                 send_thread_;
    std::atomic<bool>           stop_{false};

    mutable std::mutex          stats_mu_;
    VmtPublisherStats           stats_;

    mutable std::mutex          alignment_mu_;
    VmtAlignment                alignment_;

    // Tracker preset + its derived per-role publish mask (indexed by
    // static_cast<int>(TrackerRole)). Guarded together; set_preset updates both.
    mutable std::mutex          preset_mu_;
    VmtTrackerPreset            preset_       = VmtTrackerPreset::P8;
    std::array<bool, tracking::kTrackerCount> role_enabled_ =
        role_mask_for(VmtTrackerPreset::P8);

    // Runtime-swappable destination. In manual mode it is resolved once in
    // start(); in discovery mode the send loop updates it from disc_bus_.
    const DiscoveryEndpointBus* disc_bus_ = nullptr;
    bool                        use_discovery_ = false;
    mutable std::mutex          dst_mu_;
    sockaddr_in                 dst_{};
    bool                        have_dst_ = false;
    // Bus->destination change detector (send-thread only).
    DiscoveryEndpointLatch      disc_latch_;
};

}  // namespace fitra::vmt
