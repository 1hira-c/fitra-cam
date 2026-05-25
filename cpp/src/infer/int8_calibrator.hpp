#pragma once
//
// Post-training INT8 calibrator for fitra-cam TRT engines.
//
// Implements IInt8EntropyCalibrator2 backed by a raw float32 blob file
// produced by a Python helper. The blob layout is:
//   N consecutive tensors of shape (C, H, W), float32 little-endian, no
//   header. The calibrator reads N from the file size and bytes_per_image
//   (caller-supplied from the parsed network input dims).
//
// Used by build_engines:
//   build_engines --preset yolox --int8
//                 --int8-blobs models/calib_yolox_416.bin
//                 --onnx ... --output ...
//
// Calibration cache:
//   On construction, attempts to read `cache_path` if it exists. After TRT
//   finishes calibration, writeCalibrationCache writes back to the same
//   path. A cache hit lets the second build skip getBatch entirely.
//
// Lifetime: the calibrator object must outlive the IBuilderConfig that
// references it (i.e. it must remain alive through buildSerializedNetwork).
// build_engine() in trt_builder.cpp keeps it as a local unique_ptr.

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include <NvInfer.h>

namespace fitra::infer {

class Int8EntropyCalibrator2 final : public nvinfer1::IInt8EntropyCalibrator2 {
public:
    // bytes_per_image = product(per-image dims) * sizeof(float). For YOLOX
    // static (1,3,S,S) the caller passes 3*S*S*4. For RTMPose dynamic
    // (-1,3,256,192) the caller substitutes 1 for the batch dim.
    //
    // batch_size must match the network's static batch dim (YOLOX: 1) or
    // the calibration profile's opt batch (RTMPose dynamic: opt dim batch).
    Int8EntropyCalibrator2(std::string blob_path,
                           std::size_t bytes_per_image,
                           int batch_size,
                           std::string input_name,
                           std::string cache_path);
    ~Int8EntropyCalibrator2() override;

    Int8EntropyCalibrator2(const Int8EntropyCalibrator2&) = delete;
    Int8EntropyCalibrator2& operator=(const Int8EntropyCalibrator2&) = delete;

    int getBatchSize() const noexcept override { return batch_size_; }
    bool getBatch(void* bindings[], const char* names[], int nbBindings) noexcept override;
    const void* readCalibrationCache(std::size_t& length) noexcept override;
    void writeCalibrationCache(const void* cache, std::size_t length) noexcept override;

    std::size_t total_images() const noexcept { return total_images_; }

private:
    std::string  blob_path_;
    std::size_t  bytes_per_image_;
    int          batch_size_;
    std::string  input_name_;
    std::string  cache_path_;

    std::ifstream blob_stream_;
    std::size_t   total_images_ = 0;
    std::size_t   consumed_     = 0;

    std::vector<float> host_batch_;
    void*       device_batch_       = nullptr;
    std::size_t device_batch_bytes_ = 0;

    std::vector<std::uint8_t> cache_buffer_;
};

}  // namespace fitra::infer
