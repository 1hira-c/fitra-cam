#pragma once
//
// setup mode runner: the GPU-less first-run module. Serves a Crow web surface
// (no TensorRT/CUDA, no 3D graph, no publishers) for camera enumeration and
// config composition, then hands off to the next stage via a flow exit code.
// See docs/design/core-pipeline-setup-mode.md.

#include "app/flow.hpp"
#include "config/main_config.hpp"

namespace fitra::app {

int run_mode_setup(const config::MainOptions& opts, FlowControl& flow);

}  // namespace fitra::app
