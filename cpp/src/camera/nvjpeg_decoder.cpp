#define _GNU_SOURCE  // RTLD_DEEPBIND
#include "camera/nvjpeg_decoder.hpp"

#include <dlfcn.h>
#include <unistd.h>

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
                   void*, const unsigned char*, unsigned long, int*, int*)>(
                   ::dlsym(lib_, "fitra_nvjpeg_decode_bgr"));
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
    int w = 0, h = 0;
    const unsigned char* bgr = decode_(handle_, jpeg, static_cast<unsigned long>(bytes), &w, &h);
    if (!bgr || w <= 0 || h <= 0) return false;
    // Copy out of the .so's internal buffer (valid only until the next decode).
    cv::Mat(h, w, CV_8UC3, const_cast<unsigned char*>(bgr)).copyTo(out_bgr);
    return true;
}

}  // namespace fitra::camera
