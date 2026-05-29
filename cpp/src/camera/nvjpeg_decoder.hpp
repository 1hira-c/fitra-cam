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

    // All-GPU front-end (M2). device_capable() is true when the loaded .so
    // exports the EGL/CUDA device entry points, so the per-camera worker can
    // feed the RTMPose input straight from device memory.
    bool device_capable() const { return decode_cuda_ && preprocess_from_last_; }
    // M3: the .so also exposes the YOLOX letterbox preprocess + pure device
    // decode (no host map), so the full front-end can run on the GPU.
    bool yolox_device_capable() const {
        return decode_to_device_ && preprocess_yolox_from_last_;
    }

    // Pure device decode: HW NVJPEG -> VIC RGBA -> EGL CUDA ptr, no CPU map / no
    // RGBA->BGR. Use when the BGR host image is not needed (YOLOX + RTMPose both
    // on the GPU path). Sets w/h; the RGBA dev ptr is retained for the
    // following preprocess_into / preprocess_yolox_into. Returns true on success.
    bool decode_to_device(const std::uint8_t* jpeg, std::size_t bytes, int& w, int& h);

    // Run the YOLOX letterbox preprocess from the LAST decode into `dst_chw_dev`
    // (device, target*target*3 floats) on `stream` (the YOLOX engine's stream).
    // Does NOT synchronize (caller enqueues on the same stream). Writes the
    // letterbox scale to *out_r. Returns true on success.
    bool preprocess_yolox_into(int target, float pad, float* dst_chw_dev,
                               void* stream, float* out_r);

    // Decode into `out_bgr` (for YOLOX / calibration) AND retain the RGBA CUDA
    // device buffer of the same frame so preprocess_into() can run the GPU
    // preprocess without re-decoding. Returns true on success.
    bool decode_keep_device(const std::uint8_t* jpeg, std::size_t bytes, cv::Mat& out_bgr);

    // Run the RTMPose preprocess kernel from the LAST decode_keep_device frame
    // into `dst_chw_dev` (device, 3*out_h*out_w floats), using the inverse
    // affine M_inv6 and BGR ImageNet mean/inv_std (3 each). Returns true on
    // success. Synchronizes (the buffer is ready on return).
    bool preprocess_into(const double* M_inv6, int out_w, int out_h,
                         const float* mean_bgr, const float* inv_std_bgr,
                         float* dst_chw_dev);

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
    // Run preprocess kernel from the last decode into a device CHW buffer.
    int (*preprocess_from_last_)(void*, const double*, int, int,
                                 const float*, const float*, float*)         = nullptr;
    // M3: pure device decode + YOLOX letterbox preprocess.
    int (*decode_to_device_)(void*, const unsigned char*, unsigned long,
                             int*, int*, int*, void**)                       = nullptr;
    int (*preprocess_yolox_from_last_)(void*, int, float, float*, void*, float*) = nullptr;
    void (*destroy_)(void*)                                                  = nullptr;

    bool          egl_mode_   = false;  // FITRA_NVJPEG_EGL=1 and decode_cuda_ present
    std::uint64_t egl_frames_ = 0;      // frame counter for periodic regression log
};

}  // namespace fitra::camera
