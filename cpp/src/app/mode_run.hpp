#pragma once
//
// run mode: capture → TRT pose → optional 3D lifting → VR publishers + web.
// Knows nothing about calibration sessions — the calibration YAMLs it reads
// at boot are the entire calib→runtime contract
// (docs/design/pose-3d-calib-mode-separation.md).

#include <atomic>

#include "config/main_config.hpp"

namespace fitra::app {

int run_mode_run(const config::MainOptions& opts, std::atomic<bool>& stop);

}  // namespace fitra::app
