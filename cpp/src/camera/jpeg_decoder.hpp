#pragma once
//
// MJPEG -> BGR cv::Mat decode via cv::imdecode (libjpeg-turbo backed) on the
// CPU. Adequate for current throughput targets; a GPU nvjpeg path is a
// future optimization.

#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace fitra::camera {

class JpegDecoder {
public:
    JpegDecoder() = default;

    // Decode `jpeg` into `out_bgr` (CV_8UC3 BGR). Returns true on success.
    // `out_bgr` is reused across calls to avoid re-allocating.
    bool decode(const std::uint8_t* jpeg, std::size_t bytes, cv::Mat& out_bgr);

    bool decode(const std::vector<std::uint8_t>& jpeg, cv::Mat& out_bgr) {
        return decode(jpeg.data(), jpeg.size(), out_bgr);
    }
};

}  // namespace fitra::camera
