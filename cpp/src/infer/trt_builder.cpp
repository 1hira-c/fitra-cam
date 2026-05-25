#include "infer/trt_builder.hpp"

#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>

#include <NvOnnxParser.h>

#include "infer/int8_calibrator.hpp"
#include "util/cuda_check.hpp"
#include "util/logging.hpp"

namespace fitra::infer {

static std::string dims_str(const nvinfer1::Dims& d) {
    std::ostringstream oss;
    oss << "(";
    for (int i = 0; i < d.nbDims; ++i) {
        if (i) oss << ",";
        oss << d.d[i];
    }
    oss << ")";
    return oss.str();
}

std::size_t build_engine(const BuildOptions& opts, nvinfer1::ILogger& logger) {
    FITRA_LOG_INFO("build_engine: onnx={} -> engine={} (fp16={}, ws={}MB)",
                   opts.onnx_path, opts.engine_path,
                   opts.fp16 ? "yes" : "no", opts.workspace_mb);

    std::unique_ptr<nvinfer1::IBuilder> builder{nvinfer1::createInferBuilder(logger)};
    TRT_CHECK(builder != nullptr);

    // TRT 10 networks are explicit-batch by default.
    std::unique_ptr<nvinfer1::INetworkDefinition> network{builder->createNetworkV2(0)};
    TRT_CHECK(network != nullptr);

    std::unique_ptr<nvonnxparser::IParser> parser{
        nvonnxparser::createParser(*network, logger)};
    TRT_CHECK(parser != nullptr);

    if (!parser->parseFromFile(opts.onnx_path.c_str(),
                               static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
        for (int i = 0; i < parser->getNbErrors(); ++i) {
            const auto* e = parser->getError(i);
            FITRA_LOG_ERROR("onnx parse [{}]: {}", i, e->desc());
        }
        throw std::runtime_error("ONNX parse failed: " + opts.onnx_path);
    }

    // Log inputs/outputs we parsed
    for (int i = 0; i < network->getNbInputs(); ++i) {
        auto* t = network->getInput(i);
        FITRA_LOG_INFO("  input  [{}] {} {}", i, t->getName(),
                       dims_str(t->getDimensions()));
    }
    for (int i = 0; i < network->getNbOutputs(); ++i) {
        auto* t = network->getOutput(i);
        FITRA_LOG_INFO("  output [{}] {} {}", i, t->getName(),
                       dims_str(t->getDimensions()));
    }

    std::unique_ptr<nvinfer1::IBuilderConfig> config{builder->createBuilderConfig()};
    TRT_CHECK(config != nullptr);

    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE,
                               static_cast<std::size_t>(opts.workspace_mb) * (1ull << 20));

    if (opts.fp16) {
        if (!builder->platformHasFastFp16()) {
            FITRA_LOG_WARN("platform does not advertise fast FP16; building anyway");
        }
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
    }
    // Calibrator must outlive buildSerializedNetwork(), so allocate in this
    // scope.
    std::unique_ptr<Int8EntropyCalibrator2> calibrator;
    if (opts.int8) {
        if (!builder->platformHasFastInt8()) {
            FITRA_LOG_WARN("platform does not advertise fast INT8; building anyway");
        }
        config->setFlag(nvinfer1::BuilderFlag::kINT8);

        if (network->getNbInputs() < 1) {
            throw std::runtime_error("INT8 build requires at least one network input");
        }
        auto* in0 = network->getInput(0);
        std::string in_name = opts.int8_input_name.empty()
            ? std::string{in0->getName()}
            : opts.int8_input_name;
        auto dims = in0->getDimensions();
        std::size_t per_image_elems = 1;
        for (int i = 0; i < dims.nbDims; ++i) {
            per_image_elems *= (dims.d[i] > 0)
                ? static_cast<std::size_t>(dims.d[i])
                : 1;
        }
        std::size_t bytes_per_image = per_image_elems * sizeof(float);

        std::string cache_path = opts.int8_cache_path.empty()
            ? opts.engine_path + ".calib_cache"
            : opts.int8_cache_path;

        if (opts.int8_blob_path.empty()) {
            FITRA_LOG_WARN(
                "INT8 build without --int8-blobs: relying on cache at {} "
                "(may produce poor accuracy if cache is absent)",
                cache_path);
        } else {
            calibrator = std::make_unique<Int8EntropyCalibrator2>(
                opts.int8_blob_path,
                bytes_per_image,
                opts.int8_batch_size,
                in_name,
                cache_path);
            // setInt8Calibrator is the PTQ path in TRT 10.3 — TRT marks it
            // deprecated because the long-term direction is explicit QDQ in
            // the ONNX graph, but the deprecation is informational and the
            // function is still the only PTQ entry point.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            config->setInt8Calibrator(calibrator.get());
#pragma GCC diagnostic pop
            FITRA_LOG_INFO("INT8 calibrator wired: input='{}' per_image_bytes={} batch={} N={}",
                           in_name, bytes_per_image, opts.int8_batch_size,
                           calibrator->total_images());
        }
    }

    if (!opts.profiles.empty()) {
        auto* profile = builder->createOptimizationProfile();
        for (const auto& p : opts.profiles) {
            using nvinfer1::OptProfileSelector;
            profile->setDimensions(p.input_name.c_str(), OptProfileSelector::kMIN, p.min_dims);
            profile->setDimensions(p.input_name.c_str(), OptProfileSelector::kOPT, p.opt_dims);
            profile->setDimensions(p.input_name.c_str(), OptProfileSelector::kMAX, p.max_dims);
            FITRA_LOG_INFO("  profile: {} min={} opt={} max={}",
                           p.input_name,
                           dims_str(p.min_dims),
                           dims_str(p.opt_dims),
                           dims_str(p.max_dims));
        }
        config->addOptimizationProfile(profile);
    }

    FITRA_LOG_INFO("building serialized network (this may take minutes)...");
    std::unique_ptr<nvinfer1::IHostMemory> plan{
        builder->buildSerializedNetwork(*network, *config)};
    TRT_CHECK(plan != nullptr);
    FITRA_LOG_INFO("serialized network size = {} bytes", plan->size());

    {
        std::ofstream out{opts.engine_path, std::ios::binary | std::ios::trunc};
        if (!out.is_open()) {
            throw std::runtime_error("failed to open engine output: " + opts.engine_path);
        }
        out.write(reinterpret_cast<const char*>(plan->data()),
                  static_cast<std::streamsize>(plan->size()));
    }
    FITRA_LOG_INFO("wrote {}", opts.engine_path);
    return plan->size();
}

}  // namespace fitra::infer
