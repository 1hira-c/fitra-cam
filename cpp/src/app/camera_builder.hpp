#pragma once
//
// Per-camera FrameSource construction shared by all modes. Three variants,
// selected by arguments:
//   - run:            trt != nullptr, recording_flag == nullptr  (prebake)
//   - calib-subject:  trt != nullptr, recording_flag != nullptr  (prebake +
//                     pause-while-recording flag)
//   - calib-extrinsic: trt == nullptr                            (decode-only)

#include <atomic>
#include <memory>
#include <vector>

#include "app/trt_stack.hpp"
#include "camera/frame_source.hpp"
#include "config/main_config.hpp"

namespace fitra::app {

struct CameraSet {
    // Per-camera YOLOX execution contexts wrapping the shared ICudaEngine.
    // Declared before sources so they outlive the FrameSources that hold
    // Yolox instances referencing them.
    std::vector<std::unique_ptr<infer::TrtEngine>>    yolox_engines;
    std::vector<std::unique_ptr<camera::FrameSource>> sources;
};

// `idle_flag` (issue #37): when non-null, each FrameSource skips YOLOX +
// RTMPose pre-bake while *idle_flag is true (standby). Run mode only; calib
// modes pass null. Must outlive the returned FrameSources.
CameraSet make_frame_sources(const config::MainOptions& opts,
                             TrtStack* trt,
                             std::shared_ptr<std::atomic<bool>> recording_flag,
                             const std::atomic<bool>* idle_flag = nullptr);

}  // namespace fitra::app
