#include "app/trt_stack.hpp"

#include "infer/trt_engine.hpp"
#include "util/cuda_check.hpp"
#include "util/logging.hpp"

namespace fitra::app {

void TrtLogger::log(Severity severity, const char* msg) noexcept {
    if (severity > Severity::kWARNING) return;
    using S = Severity;
    switch (severity) {
        case S::kINTERNAL_ERROR: FITRA_LOG_ERROR("[trt] INTERNAL: {}", msg); return;
        case S::kERROR:          FITRA_LOG_ERROR("[trt] {}",          msg); return;
        case S::kWARNING:        FITRA_LOG_WARN ("[trt] {}",          msg); return;
        case S::kINFO:           FITRA_LOG_INFO ("[trt] {}",          msg); return;
        case S::kVERBOSE:        FITRA_LOG_TRACE("[trt] {}",          msg); return;
    }
}

std::unique_ptr<TrtStack> make_trt_stack(const config::MainOptions& opts) {
    auto stack = std::make_unique<TrtStack>();
    stack->runtime.reset(nvinfer1::createInferRuntime(stack->logger));
    TRT_CHECK(stack->runtime != nullptr);

    FITRA_LOG_INFO("loading YOLOX engine (shared): {}", opts.det_engine);
    stack->yolox_shared =
        infer::TrtEngine::load_shared(*stack->runtime, opts.det_engine);
    FITRA_LOG_INFO("loading RTMPose engine: {}", opts.pose_engine);
    stack->rtmpose_eng =
        infer::TrtEngine::from_file(*stack->runtime, opts.pose_engine, stack->logger);
    stack->rtmpose = std::make_unique<infer::RtmPose>(*stack->rtmpose_eng);
    return stack;
}

}  // namespace fitra::app
