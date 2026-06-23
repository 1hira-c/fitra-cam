#pragma once
//
// Unified VMT pose relay (HMD + selected-controller buses + UDP receiver).
// Input-only — distinct from the VMT *publisher* (output), which setup modes
// never construct. run uses it for HMD-driven alignment (when
// --hmd-listen-enabled); calib-extrinsic always uses it (the controller pose
// is the calibration input).

#include <memory>

#include "config/main_config.hpp"
#include "vmt/controller_pose_receiver.hpp"
#include "vmt/discovery_beacon.hpp"
#include "vmt/hmd_pose_receiver.hpp"
#include "vmt/tracked_pose_receiver.hpp"

namespace fitra::app {

struct PoseRelay {
    std::unique_ptr<vmt::HmdPoseBus>          hmd_bus;
    std::unique_ptr<vmt::ControllerPoseBus>   controller_bus;
    // Receiver thread; nullptr when `listen` was false or the socket failed.
    std::unique_ptr<vmt::TrackedPoseReceiver> receiver;
    // Zeroconf discovery beacon; nullptr when discovery is off or vmt.host is
    // pinned. Shared with the VMT publisher (output_builder reads its bus), so
    // it lives here — the relay outlives the publisher in the shutdown order.
    std::unique_ptr<vmt::DiscoveryBeacon>     beacon;
    vmt::TrackedPoseRole controller_role = vmt::TrackedPoseRole::RightController;

    void stop() {
        // Consumers (receiver, and the publisher in RunOutputs) must stop
        // reading the endpoint bus before the beacon that owns it goes away.
        if (receiver) receiver->stop();
        if (beacon)   beacon->stop();
    }
};

// Buses are always constructed (cheap; consumers hold pointers). The receiver
// binds + starts only when `listen` is true.
PoseRelay make_pose_relay(const config::MainOptions& opts, bool listen);

}  // namespace fitra::app
