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
    relay.receiver = std::make_unique<vmt::TrackedPoseReceiver>(
        *relay.hmd_bus, *relay.controller_bus, popts);
    if (!relay.receiver->start()) {
        relay.receiver.reset();
    }
    return relay;
}

}  // namespace fitra::app
