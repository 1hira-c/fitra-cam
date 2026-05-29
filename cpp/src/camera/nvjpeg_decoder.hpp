#pragma once
//
// Loader for the isolated Jetson hardware-NVJPEG decoder
// (libfitra_nvjpeg.so). Decodes MJPEG -> BGR on the HW NVJPEG block + VIC,
// freeing the CPU from JPEG entropy decode.
//
// The .so is dlopen'd with RTLD_DEEPBIND|RTLD_LOCAL so its bundled libjpeg-8b
// symbols (pulled in by libnvjpeg) do NOT interpose the main binary's
// libjpeg-turbo, which cv::imdecode (the default --pixel-format mjpeg path)
// depends on. See cpp/src/camera/nvjpeg/fitra_nvjpeg_iso.cpp and
// docs/design/core-pipeline-nvjpeg-decode.md.
//
// One instance per decode thread (the underlying NvJPEGDecoder is not
// thread-safe), mirroring the per-camera Yolox context pattern.

#include <cstddef>
#include <cstdint>

#include <opencv2/core.hpp>

namespace fitra::camera {

class NvJpegHwDecoder {
public:
    // Loads libfitra_nvjpeg.so and creates an HW decoder handle.
    // Throws std::runtime_error if the library or HW decoder is unavailable
    // (e.g. non-Jetson host, or the .so was not built/installed).
    NvJpegHwDecoder();
    ~NvJpegHwDecoder();

    NvJpegHwDecoder(const NvJpegHwDecoder&) = delete;
    NvJpegHwDecoder& operator=(const NvJpegHwDecoder&) = delete;

    // Decode `jpeg` (MJPEG bytes) into `out_bgr` (CV_8UC3 BGR). Returns true on
    // success. `out_bgr` is reused across calls to avoid reallocation.
    bool decode(const std::uint8_t* jpeg, std::size_t bytes, cv::Mat& out_bgr);

private:
    void* lib_    = nullptr;  // dlopen handle for libfitra_nvjpeg.so
    void* handle_ = nullptr;  // per-thread decoder handle inside the .so
    // Resolved C entry points.
    void* (*create_)()                                                       = nullptr;
    // Returns mapped RGBA8 (zero-copy) + width/height/row-pitch; null on error.
    const unsigned char* (*decode_)(void*, const unsigned char*, unsigned long,
                                    int*, int*, int*)                        = nullptr;
    void (*destroy_)(void*)                                                  = nullptr;
};

}  // namespace fitra::camera
