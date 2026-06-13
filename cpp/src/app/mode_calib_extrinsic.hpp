#pragma once
//
// calib-extrinsic mode: decode-only cameras + AprilTag collection + VMT pose
// relay receiver + /api/excal web UI. No TensorRT, no MultiCameraDriver, no
// publishers. Solve success auto-exits with a restart guidance; the only
// artifact handed to the next mode is the extrinsics YAML.
// See docs/design/pose-3d-calib-mode-separation.md.

#include "app/flow.hpp"
#include "config/main_config.hpp"

namespace fitra::app {

int run_mode_calib_extrinsic(const config::MainOptions& opts, FlowControl& flow);

}  // namespace fitra::app
