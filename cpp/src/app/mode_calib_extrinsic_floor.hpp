#pragma once
//
// calib-extrinsic-floor mode runner: the VR-free floor-AprilTag extrinsic path.
// See docs/design/pose-3d-floor-apriltag-extrinsic.md.
//
// Live and replay both collect tag-corner observations against a known
// FloorTagMap and solve per-camera multi-tag PnP into the extrinsics YAML. The
// replay path is fully unattended (no cameras, no web); the live path runs the
// capture loop and (in M7) a Crow server for start/solve from the web UI.

#include "app/flow.hpp"
#include "config/main_config.hpp"

namespace fitra::app {

int run_mode_calib_extrinsic_floor(const config::MainOptions& opts,
                                   FlowControl& flow);

}  // namespace fitra::app
