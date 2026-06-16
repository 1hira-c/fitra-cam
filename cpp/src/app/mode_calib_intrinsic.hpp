#pragma once
//
// calib-intrinsic mode runner: ChArUco per-camera intrinsic calibration.
// See docs/design/pose-3d-intrinsic-calibration.md. Replay is unattended
// (frames in → intrinsics YAML out); live runs the capture loop and (in I5) a
// Crow server for start/solve from the web UI.

#include "app/flow.hpp"
#include "config/main_config.hpp"

namespace fitra::app {

int run_mode_calib_intrinsic(const config::MainOptions& opts, FlowControl& flow);

}  // namespace fitra::app
