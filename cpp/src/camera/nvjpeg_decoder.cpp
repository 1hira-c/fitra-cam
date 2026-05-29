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

    handle_ = create_();
    if (!handle_) {
        ::dlclose(lib_);
        lib_ = nullptr;
        throw std::runtime_error("NvJpegHwDecoder: fitra_nvjpeg_create failed (HW decoder init)");
    }
    FITRA_LOG_INFO("nvjpeg: HW NVJPEG decoder ready");
}

NvJpegHwDecoder::~NvJpegHwDecoder() {
    if (handle_ && destroy_) destroy_(handle_);
    if (lib_) ::dlclose(lib_);
}

bool NvJpegHwDecoder::decode(const std::uint8_t* jpeg, std::size_t bytes, cv::Mat& out_bgr) {
    if (!handle_ || !jpeg || bytes == 0) return false;
    int w = 0, h = 0, pitch = 0;
    const unsigned char* rgba =
        decode_(handle_, jpeg, static_cast<unsigned long>(bytes), &w, &h, &pitch);
    if (!rgba || w <= 0 || h <= 0 || pitch < w * 4) return false;
    // VIC output is RGBA (24-bit BGR is unsupported). One NEON cv::cvtColor is
    // the only full-frame CPU pass on the HW route. cvtColor writes a fresh
    // out_bgr, so we don't alias the .so's mapped buffer past this call.
    cv::Mat rgba_view(h, w, CV_8UC4, const_cast<unsigned char*>(rgba),
                      static_cast<std::size_t>(pitch));
    cv::cvtColor(rgba_view, out_bgr, cv::COLOR_RGBA2BGR);
    return true;
}

}  // namespace fitra::camera
