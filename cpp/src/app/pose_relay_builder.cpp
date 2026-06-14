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
    if (!listen) return relay;

    vmt::TrackedPoseReceiverOptions popts;
    popts.bind = opts.hmd_listen_bind;
    popts.port = static_cast<std::uint16_t>(opts.hmd_listen_port);
    popts.stale_ms = opts.hmd_stale_ms;
    popts.controller_role = relay.controller_role;
    // Punch the VMT host so it learns our IP and relays poses back, even in
    // modes that run no VMT publisher (calib-extrinsic sends nothing to the
    // host otherwise → no controller/HMD pose ever arrives). Destination is
    // the VMT OSC receive endpoint (vmt.host:vmt.port). A loopback host is a
    // no-op on VMT's side but harmless. See docs/design/pose-3d-flow-daemon.md.
    popts.punch_host = opts.vmt_host;
    popts.punch_port = static_cast<std::uint16_t>(opts.vmt_port);
    relay.receiver = std::make_unique<vmt::TrackedPoseReceiver>(
        *relay.hmd_bus, *relay.controller_bus, popts);
    if (!relay.receiver->start()) {
        relay.receiver.reset();
    }
    return relay;
}

}  // namespace fitra::app
