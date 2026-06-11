#pragma once
//
// calib-subject mode: the subject-profile wizard. Same capture + TRT + 3D
// stack as run (extrinsics YAML required), plus the CalibrationSession wired
// to the driver taps — and no VR publishers. The approved profile YAML is the
// mode's whole output (docs/design/pose-3d-calib-mode-separation.md).

#include <atomic>

#include "config/main_config.hpp"

namespace fitra::app {

int run_mode_calib_subject(const config::MainOptions& opts,
                           std::atomic<bool>& stop);

}  // namespace fitra::app
