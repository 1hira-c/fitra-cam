#pragma once
//
// Capture loop for calib-intrinsic mode: pulls frames from an ExcalInputSource
// and feeds IntrinsicCalibSession::on_frame() until `stop` or source exhaustion.
// Reuses the controller-path input abstraction (the paired ControllerObservation
// is ignored; only the frame + timestamp are used). Live and replay share it.

#include <atomic>

#include "pipeline/excal_input_source.hpp"

namespace fitra::pipeline { class IntrinsicCalibSession; }

namespace fitra::app {

void run_intrinsic_calib_loop(pipeline::ExcalInputSource& input,
                              pipeline::IntrinsicCalibSession& session,
                              std::atomic<bool>& stop);

}  // namespace fitra::app
