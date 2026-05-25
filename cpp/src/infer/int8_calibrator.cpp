#include "infer/int8_calibrator.hpp"

#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include <cuda_runtime_api.h>

#include "util/cuda_check.hpp"
#include "util/logging.hpp"

namespace fitra::infer {

Int8EntropyCalibrator2::Int8EntropyCalibrator2(std::string blob_path,
                                                std::size_t bytes_per_image,
                                                int batch_size,
                                                std::string input_name,
                                                std::string cache_path)
    : blob_path_(std::move(blob_path)),
      bytes_per_image_(bytes_per_image),
      batch_size_(batch_size),
      input_name_(std::move(input_name)),
      cache_path_(std::move(cache_path)) {
    if (batch_size_ < 1) {
        throw std::runtime_error("Int8EntropyCalibrator2: batch_size must be >= 1");
    }
    if (bytes_per_image_ == 0) {
        throw std::runtime_error("Int8EntropyCalibrator2: bytes_per_image is 0");
    }
    if (input_name_.empty()) {
        throw std::runtime_error("Int8EntropyCalibrator2: input_name is empty");
    }
    if (!std::filesystem::exists(blob_path_)) {
        throw std::runtime_error("calibration blob not found: " + blob_path_);
    }
    const auto file_size = std::filesystem::file_size(blob_path_);
    if (file_size % bytes_per_image_ != 0) {
        throw std::runtime_error(
            "calibration blob size " + std::to_string(file_size) +
            " not divisible by bytes_per_image " + std::to_string(bytes_per_image_) +
            " — blob path / model input_size mismatch?");
    }
    total_images_ = file_size / bytes_per_image_;
    if (total_images_ < static_cast<std::size_t>(batch_size_)) {
        throw std::runtime_error(
            "calibration blob has " + std::to_string(total_images_) +
            " images, fewer than batch_size " + std::to_string(batch_size_));
    }
    blob_stream_.open(blob_path_, std::ios::binary);
    if (!blob_stream_.is_open()) {
        throw std::runtime_error("failed to open calibration blob: " + blob_path_);
    }

    device_batch_bytes_ = bytes_per_image_ * static_cast<std::size_t>(batch_size_);
    host_batch_.resize(device_batch_bytes_ / sizeof(float));
    CUDA_CHECK(cudaMalloc(&device_batch_, device_batch_bytes_));

    FITRA_LOG_INFO("INT8 calibrator: blob={} batch={} per_image_bytes={} total_images={}",
                   blob_path_, batch_size_, bytes_per_image_, total_images_);
    if (!cache_path_.empty()) {
        FITRA_LOG_INFO("INT8 calibrator: cache_path={}", cache_path_);
    }
}

Int8EntropyCalibrator2::~Int8EntropyCalibrator2() {
    if (device_batch_) {
        cudaFree(device_batch_);
        device_batch_ = nullptr;
    }
}

bool Int8EntropyCalibrator2::getBatch(void* bindings[], const char* names[],
                                       int nbBindings) noexcept {
    if (consumed_ + static_cast<std::size_t>(batch_size_) > total_images_) {
        return false;
    }
    blob_stream_.read(reinterpret_cast<char*>(host_batch_.data()),
                      static_cast<std::streamsize>(device_batch_bytes_));
    if (!blob_stream_) {
        FITRA_LOG_ERROR("INT8 calibrator: short read at image {}", consumed_);
        return false;
    }
    cudaError_t err = cudaMemcpy(device_batch_, host_batch_.data(),
                                 device_batch_bytes_, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        FITRA_LOG_ERROR("INT8 calibrator: cudaMemcpy H2D failed: {}",
                        cudaGetErrorString(err));
        return false;
    }
    int found = -1;
    for (int i = 0; i < nbBindings; ++i) {
        if (names[i] && input_name_ == names[i]) {
            found = i;
            break;
        }
    }
    if (found < 0) {
        FITRA_LOG_ERROR("INT8 calibrator: input binding '{}' not present among {} bindings",
                        input_name_, nbBindings);
        return false;
    }
    bindings[found] = device_batch_;
    consumed_ += static_cast<std::size_t>(batch_size_);
    if ((consumed_ % 50) == 0 || consumed_ == total_images_) {
        FITRA_LOG_INFO("INT8 calibrator: served {} / {} images",
                       consumed_, total_images_);
    }
    return true;
}

const void* Int8EntropyCalibrator2::readCalibrationCache(std::size_t& length) noexcept {
    cache_buffer_.clear();
    length = 0;
    if (cache_path_.empty() || !std::filesystem::exists(cache_path_)) {
        return nullptr;
    }
    std::ifstream f{cache_path_, std::ios::binary | std::ios::ate};
    if (!f.is_open()) return nullptr;
    const std::streamsize n = f.tellg();
    if (n <= 0) return nullptr;
    f.seekg(0);
    cache_buffer_.resize(static_cast<std::size_t>(n));
    f.read(reinterpret_cast<char*>(cache_buffer_.data()), n);
    length = cache_buffer_.size();
    FITRA_LOG_INFO("INT8 calibrator: loaded cache {} ({} bytes)",
                   cache_path_, length);
    return cache_buffer_.data();
}

void Int8EntropyCalibrator2::writeCalibrationCache(const void* cache,
                                                    std::size_t length) noexcept {
    if (cache_path_.empty()) return;
    std::ofstream f{cache_path_, std::ios::binary | std::ios::trunc};
    if (!f.is_open()) {
        FITRA_LOG_WARN("INT8 calibrator: could not open cache for write: {}",
                       cache_path_);
        return;
    }
    f.write(reinterpret_cast<const char*>(cache),
            static_cast<std::streamsize>(length));
    FITRA_LOG_INFO("INT8 calibrator: wrote cache {} ({} bytes)",
                   cache_path_, length);
}

}  // namespace fitra::infer
