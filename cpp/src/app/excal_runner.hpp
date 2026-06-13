#pragma once
//
// Capture loop for calib-extrinsic mode: pulls paired frames from an
// ExcalInputSource and feeds ExtrinsicCalibSession::on_frame() until `stop`
// is set or the source is exhausted (replay end). Replaces the
// MultiCameraDriver frame-tap mux — calib-extrinsic does not construct the
// driver (or TensorRT) at all. Live and replay inputs share this loop, so the
// replay path exercises the same code the live path runs.

#include <atomic>

#include "pipeline/excal_input_source.hpp"

namespace fitra::pipeline { class ExtrinsicCalibSession; }

namespace fitra::app {

void run_excal_loop(pipeline::ExcalInputSource& input,
                    pipeline::ExtrinsicCalibSession& session,
                    std::atomic<bool>& stop);

}  // namespace fitra::app
