#include "app/pose_relay_builder.hpp"

#include <cstdint>

namespace fitra::app {

PoseRelay make_pose_relay(const config::MainOptions& opts, bool listen) {
    PoseRelay relay;
    relay.hmd_bus = std::make_unique<vmt::HmdPoseBus>();
    relay.controller_bus = std::make_unique<vmt::ControllerPoseBus>();
    if (!vmt::parse_tracked_pose_role(opts.excal_controller_role,
                                      relay.controller_role)) {
        // validate_options should have caught this, but keep the runtime
        // path deterministic if a caller constructed MainOptions manually.
        relay.controller_role = vmt::TrackedPoseRole::RightController;
    }

    // Zeroconf discovery beacon. Built when discovery is on, no manual host is
    // pinned, and some VMT path is active (this relay's punch and/or the VMT
    // publisher). Shared with output_builder via PoseRelay::beacon's bus.
    if (opts.vmt_discovery && opts.vmt_host.empty() && (listen || opts.vmt_out)) {
        vmt::DiscoveryBeaconOptions bopts;
        bopts.self_role          = vmt::DiscoveryRole::Jetson;
        bopts.group              = opts.vmt_discovery_group;
        bopts.port               = static_cast<std::uint16_t>(opts.vmt_discovery_port);
        // Jetson receives /fitra/tracked_pose here; advertise that to peers.
        bopts.self_osc_recv_port = static_cast<std::uint16_t>(opts.hmd_listen_port);
        bopts.instance_name      = opts.vmt_instance_name;
        bopts.pair_id            = opts.vmt_pair_id;
        bopts.pairing_token      = opts.vmt_pairing_token;
        bopts.peer_timeout_ms    = opts.vmt_peer_timeout_s * 1000.0;
        relay.beacon = std::make_unique<vmt::DiscoveryBeacon>(bopts);
        if (!relay.beacon->start()) {
            relay.beacon.reset();  // socket failure → continue without discovery
        }
    }

    if (!listen) return relay;

    vmt::TrackedPoseReceiverOptions popts;
    popts.bind = opts.hmd_listen_bind;
    popts.port = static_cast<std::uint16_t>(opts.hmd_listen_port);
    popts.stale_ms = opts.hmd_stale_ms;
    popts.controller_role = relay.controller_role;
    // Punch the VMT host so it learns our IP and relays poses back, even in
    // modes that run no VMT publisher (calib-extrinsic sends nothing to the
    // host otherwise → no controller/HMD pose ever arrives). When vmt.host is
    // empty the destination is resolved at runtime from the discovery bus;
    // otherwise it's the manual VMT OSC endpoint (vmt.host:vmt.port).
    popts.punch_host = opts.vmt_host;
    popts.punch_port = static_cast<std::uint16_t>(opts.vmt_port);
    relay.receiver = std::make_unique<vmt::TrackedPoseReceiver>(
        *relay.hmd_bus, *relay.controller_bus, popts);
    if (relay.beacon) {
        relay.receiver->set_discovery_bus(&relay.beacon->endpoint_bus());
    }
    if (!relay.receiver->start()) {
        relay.receiver.reset();
    }
    return relay;
}

}  // namespace fitra::app
