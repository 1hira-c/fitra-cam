#pragma once
//
// Main-thread stats loop shared by the run and calib-subject runners: log
// per-camera pipeline stats every log_every_s until `stop` is set.

#include <atomic>

#include "pipeline/multi_pipeline.hpp"

namespace fitra::app {

void run_stats_loop(pipeline::MultiCameraDriver& driver,
                    double log_every_s,
                    std::atomic<bool>& stop);

}  // namespace fitra::app
