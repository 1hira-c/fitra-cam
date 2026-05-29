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
    //
    // When the EGL->CUDA bridge is enabled (FITRA_NVJPEG_EGL=1, all-GPU
    // front-end M1) this routes through fitra_nvjpeg_decode_cuda, which also
    // registers the decode output as a CUDA device pointer and periodically
    // regression-checks it against the CPU map (logged). The BGR output is
    // unchanged either way. egl_enabled() reports which path is active.
    bool decode(const std::uint8_t* jpeg, std::size_t bytes, cv::Mat& out_bgr);

    bool egl_enabled() const { return egl_mode_; }

private:
    void* lib_    = nullptr;  // dlopen handle for libfitra_nvjpeg.so
    void* handle_ = nullptr;  // per-thread decoder handle inside the .so
    // Resolved C entry points.
    void* (*create_)()                                                       = nullptr;
    // Returns mapped RGBA8 (zero-copy) + width/height/row-pitch; null on error.
    const unsigned char* (*decode_)(void*, const unsigned char*, unsigned long,
                                    int*, int*, int*)                        = nullptr;
    // EGL->CUDA path: returns 0 on success, fills device ptr + host map + check
    // means. May be null if the .so predates the bridge.
    int (*decode_cuda_)(void*, const unsigned char*, unsigned long,
                        int*, int*, int*, void**,
                        const unsigned char**, int*,
                        int, double*, double*)                               = nullptr;
    void (*destroy_)(void*)                                                  = nullptr;

    bool          egl_mode_   = false;  // FITRA_NVJPEG_EGL=1 and decode_cuda_ present
    std::uint64_t egl_frames_ = 0;      // frame counter for periodic regression log
};

}  // namespace fitra::camera
