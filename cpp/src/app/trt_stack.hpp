#pragma once
//
// TensorRT runtime + engine bundle for the pipeline modes (run /
// calib-subject). calib-extrinsic never constructs this — it is decode-only.
//
// Member order encodes the lifetime contract: the logger must outlive the
// runtime, the runtime its engines, the engines the RtmPose instance.
// (Members destroy in reverse declaration order.)

#include <memory>

#include <NvInfer.h>

#include "config/main_config.hpp"
#include "infer/rtmpose.hpp"
#include "infer/trt_engine.hpp"

namespace fitra::app {

class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override;
};

struct TrtStack {
    TrtLogger logger;
    std::unique_ptr<nvinfer1::IRuntime>    runtime;
    std::shared_ptr<nvinfer1::ICudaEngine> yolox_shared;
    std::unique_ptr<infer::TrtEngine>      rtmpose_eng;
    // RTMPose stays a single shared instance — batching across cameras
    // requires one execution context fed serially from the central thread.
    std::unique_ptr<infer::RtmPose>        rtmpose;
};

// Loads both engines per opts. Throws (std::runtime_error from the TRT
// wrappers) on failure.
std::unique_ptr<TrtStack> make_trt_stack(const config::MainOptions& opts);

}  // namespace fitra::app
