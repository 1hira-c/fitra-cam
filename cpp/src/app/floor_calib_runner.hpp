#pragma once
//
// Capture loop for calib-extrinsic-floor mode: pulls frames from an
// ExcalInputSource and feeds FloorCalibSession::on_frame() until `stop` is set
// or the source is exhausted (replay end). Reuses the controller-path input
// abstraction (ExcalInputSource / ExcalInputItem) — the floor path simply
// ignores the paired ControllerObservation, keeping the live frame timestamp
// for the UI "age" display. Live and replay inputs share this loop, so the
// replay path exercises the same code the live path runs.

#include <atomic>

#include "pipeline/excal_input_source.hpp"

namespace fitra::pipeline { class FloorCalibSession; }

namespace fitra::app {

void run_floor_calib_loop(pipeline::ExcalInputSource& input,
                          pipeline::FloorCalibSession& session,
                          std::atomic<bool>& stop);

}  // namespace fitra::app
