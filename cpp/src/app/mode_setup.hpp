#pragma once
//
// setup mode runner: the GPU-less first-run module. Serves a Crow web surface
// (no TensorRT/CUDA, no 3D graph, no publishers) for camera enumeration and
// config composition, then hands off to the next stage via a flow exit code.
// See docs/design/core-pipeline-setup-mode.md.

#include <string>

#include "app/flow.hpp"
#include "config/main_config.hpp"

namespace fitra::app {

// `config_path` is the union config the daemon spawned us with (--config); the
// setup module writes the composed config back there so the next stage reloads
// it. Empty when launched standalone without --config (proceed then fails with
// a clear error).
int run_mode_setup(const config::MainOptions& opts, const std::string& config_path,
                   FlowControl& flow);

}  // namespace fitra::app
