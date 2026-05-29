#ifndef _GNU_SOURCE
#define _GNU_SOURCE  // RTLD_DEEPBIND
#endif
#include "camera/nvjpeg_decoder.hpp"

#include <dlfcn.h>
#include <unistd.h>

#include <opencv2/imgproc.hpp>

#include <array>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "util/logging.hpp"

namespace fitra::camera {

namespace {

// Candidate paths for libfitra_nvjpeg.so, in priority order:
//   1. $FITRA_NVJPEG_SO (explicit override)
//   2. <dir of this executable>/libfitra_nvjpeg.so (the build lands it next to `main`)
//   3. bare soname (rely on rpath / LD_LIBRARY_PATH)
std::vector<std::string> candidate_paths() {
    std::vector<std::string> out;
    if (const char* env = std::getenv("FITRA_NVJPEG_SO"); env && *env) out.emplace_back(env);

    std::array<char, 4096> buf{};
    ssize_t n = ::readlink("/proc/self/exe", buf.data(), buf.size() - 1);
    if (n > 0) {
        std::string exe(buf.data(), static_cast<std::size_t>(n));
        auto slash = exe.find_last_of('/');
        if (slash != std::string::npos)
            out.emplace_back(exe.substr(0, slash + 1) + "libfitra_nvjpeg.so");
    }
    out.emplace_back("libfitra_nvjpeg.so");
    return out;
}

}  // namespace

NvJpegHwDecoder::NvJpegHwDecoder() {
    std::string last_err;
    for (const auto& path : candidate_paths()) {
        // RTLD_DEEPBIND keeps the .so's libjpeg-8b symbols from interposing the
        // main binary's libjpeg-turbo (used by cv::imdecode).
        lib_ = ::dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);
        if (lib_) break;
        const char* e = ::dlerror();
        last_err = e ? e : "unknown dlopen error";
    }
    if (!lib_) {
        throw std::runtime_error(
            "NvJpegHwDecoder: cannot load libfitra_nvjpeg.so (" + last_err +
            "). HW NVJPEG decode requires a Jetson build with the MMAPI .so present.");
    }

    create_  = reinterpret_cast<void* (*)()>(::dlsym(lib_, "fitra_nvjpeg_create"));
    decode_  = reinterpret_cast<const unsigned char* (*)(
                   void*, const unsigned char*, unsigned long, int*, int*, int*)>(
                   ::dlsym(lib_, "fitra_nvjpeg_decode_rgba"));
    destroy_ = reinterpret_cast<void (*)(void*)>(::dlsym(lib_, "fitra_nvjpeg_destroy"));
    if (!create_ || !decode_ || !destroy_) {
        ::dlclose(lib_);
        lib_ = nullptr;
        throw std::runtime_error("NvJpegHwDecoder: libfitra_nvjpeg.so missing expected symbols");
    }

    // Optional EGL->CUDA bridge (all-GPU front-end M1). Opt-in via env so the
    // shipped CPU/RGBA path is unaffected; requires a .so new enough to export
    // fitra_nvjpeg_decode_cuda.
    decode_cuda_ = reinterpret_cast<int (*)(
        void*, const unsigned char*, unsigned long, int*, int*, int*, void**,
        const unsigned char**, int*, int, double*, double*)>(
        ::dlsym(lib_, "fitra_nvjpeg_decode_cuda"));
    if (const char* e = std::getenv("FITRA_NVJPEG_EGL"); e && *e && std::string(e) != "0") {
        if (decode_cuda_) {
            egl_mode_ = true;
        } else {
            FITRA_LOG_WARN("nvjpeg: FITRA_NVJPEG_EGL set but libfitra_nvjpeg.so "
                           "has no decode_cuda; using CPU-map path");
        }
    }

    handle_ = create_();
    if (!handle_) {
        ::dlclose(lib_);
        lib_ = nullptr;
        throw std::runtime_error("NvJpegHwDecoder: fitra_nvjpeg_create failed (HW decoder init)");
    }
    FITRA_LOG_INFO("nvjpeg: HW NVJPEG decoder ready{}",
                   egl_mode_ ? " (EGL->CUDA bridge enabled)" : "");
}

NvJpegHwDecoder::~NvJpegHwDecoder() {
    if (handle_ && destroy_) destroy_(handle_);
    if (lib_) ::dlclose(lib_);
}

bool NvJpegHwDecoder::decode(const std::uint8_t* jpeg, std::size_t bytes, cv::Mat& out_bgr) {
    if (!handle_ || !jpeg || bytes == 0) return false;
    int w = 0, h = 0, pitch = 0;
    const unsigned char* rgba = nullptr;

    if (egl_mode_) {
        // All-GPU front-end M1: decode straight onto the HW blocks and register
        // the RGBA output as a CUDA device pointer. Regression-check the bridge
        // against the CPU map every 300 frames (~5s @60fps) — cheap and logged,
        // not on every frame. M1 still builds BGR from the host map below; M2
        // will consume the device pointer directly and drop this.
        constexpr std::uint64_t kCheckEvery = 300;
        const bool check = (egl_frames_ % kCheckEvery) == 0;
        void* dev = nullptr;
        int dev_pitch = 0, host_pitch = 0;
        double cpu_mean = 0.0, dev_mean = 0.0;
        int rc = decode_cuda_(handle_, jpeg, static_cast<unsigned long>(bytes),
                              &w, &h, &dev_pitch, &dev, &rgba, &host_pitch,
                              check ? 1 : 0, &cpu_mean, &dev_mean);
        if (rc != 0 || !rgba || !dev || w <= 0 || h <= 0 || host_pitch < w * 4) return false;
        if (check) {
            const double diff = (dev_mean < 0.0) ? -1.0 : (cpu_mean - dev_mean);
            FITRA_LOG_INFO("nvjpeg/EGL: {}x{} dev={} dev_pitch={} "
                           "R-mean cpu={} dev={} diff={}",
                           w, h, dev, dev_pitch, cpu_mean, dev_mean, diff);
        }
        ++egl_frames_;
        pitch = host_pitch;
    } else {
        rgba = decode_(handle_, jpeg, static_cast<unsigned long>(bytes), &w, &h, &pitch);
        if (!rgba || w <= 0 || h <= 0 || pitch < w * 4) return false;
    }

    // VIC output is RGBA (24-bit BGR is unsupported). One NEON cv::cvtColor is
    // the only full-frame CPU pass on the HW route. cvtColor writes a fresh
    // out_bgr, so we don't alias the .so's mapped buffer past this call.
    cv::Mat rgba_view(h, w, CV_8UC4, const_cast<unsigned char*>(rgba),
                      static_cast<std::size_t>(pitch));
    cv::cvtColor(rgba_view, out_bgr, cv::COLOR_RGBA2BGR);
    return true;
}

}  // namespace fitra::camera
