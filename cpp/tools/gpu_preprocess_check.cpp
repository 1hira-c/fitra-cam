// Correctness check for the all-GPU front-end RTMPose preprocess kernel (M2).
//
// For each sampled frame of a recorded video and a set of bboxes, runs:
//   (CPU ref) RtmPose::preprocess_to_blob  -> cpu_chw  (+ inverse affine M_inv)
//   (GPU)     libfitra_nvjpeg preprocess kernel, fed the SAME M_inv and a
//             BGR->RGBA copy of the frame                 -> gpu_chw
// and reports max / mean abs diff and L2 over the CHW tensors. The two must
// agree to within float-bilinear-vs-OpenCV-fixed-point tolerance.
//
// The .so is dlopen'd with RTLD_DEEPBIND|RTLD_LOCAL (its bundled libjpeg-8b
// must not interpose this tool's libjpeg-turbo), mirroring nvjpeg_decoder.cpp.

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <dlfcn.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "infer/rtmpose.hpp"
#include "infer/types.hpp"

using fitra::infer::Bbox;
using fitra::infer::RtmPose;

namespace {

using PreprocHostFn = int (*)(const unsigned char*, int, int, int,
                              const double*, int, int,
                              const float*, const float*, float*);

PreprocHostFn load_kernel(void** lib_out) {
    const char* env = std::getenv("FITRA_NVJPEG_SO");
    std::vector<std::string> paths;
    if (env && *env) paths.emplace_back(env);
    paths.emplace_back("cpp/build/libfitra_nvjpeg.so");
    paths.emplace_back("libfitra_nvjpeg.so");
    for (const auto& p : paths) {
        void* lib = ::dlopen(p.c_str(), RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);
        if (!lib) continue;
        auto fn = reinterpret_cast<PreprocHostFn>(
            ::dlsym(lib, "fitra_nvjpeg_preprocess_rgba_host"));
        if (fn) { *lib_out = lib; return fn; }
        ::dlclose(lib);
    }
    return nullptr;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string video = (argc > 1)
        ? argv[1]
        : "outputs/recorded_rtmpose/20260515_064342/raw_cam0.mp4";

    void* lib = nullptr;
    PreprocHostFn gpu = load_kernel(&lib);
    if (!gpu) {
        std::fprintf(stderr, "could not load fitra_nvjpeg_preprocess_rgba_host "
                             "(set FITRA_NVJPEG_SO or run from repo root)\n");
        return 2;
    }

    cv::VideoCapture cap(video);
    if (!cap.isOpened()) {
        std::fprintf(stderr, "cannot open video: %s\n", video.c_str());
        return 2;
    }

    RtmPose::Options opts;  // input_w=192 input_h=256, padding 1.25
    const int OW = opts.input_w, OH = opts.input_h;
    const std::size_t per = RtmPose::blob_floats_per_item(opts);
    const float mean_bgr[3]    = {103.53f, 116.28f, 123.675f};
    const float inv_std_bgr[3] = {1.f / 57.375f, 1.f / 57.12f, 1.f / 58.395f};

    std::vector<float> cpu_chw(per), gpu_chw(per);

    // A spread of bboxes: full frame, centered crop, off-center, tall, wide.
    auto bb = [](float x1, float y1, float x2, float y2) { return Bbox{x1, y1, x2, y2, 0.f}; };

    double worst_max = 0.0, sum_mean = 0.0, sum_l2 = 0.0;
    int cases = 0, fail = 0;
    const int total = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    const int step = (total > 0) ? std::max(1, total / 8) : 100;

    cv::Mat frame, rgba;
    int fidx = 0;
    while (cap.read(frame)) {
        if (frame.empty()) break;
        if (fidx % step == 0) {
            const float W = static_cast<float>(frame.cols);
            const float H = static_cast<float>(frame.rows);
            std::vector<Bbox> boxes = {
                bb(0, 0, W, H),
                bb(W * 0.25f, H * 0.10f, W * 0.75f, H * 0.95f),
                bb(W * 0.05f, H * 0.30f, W * 0.45f, H * 0.90f),
                bb(W * 0.30f, H * 0.20f, W * 0.55f, H * 0.98f),
            };
            cv::cvtColor(frame, rgba, cv::COLOR_BGR2RGBA);
            const int pitch = rgba.cols * 4;
            for (const auto& box : boxes) {
                cv::Mat M_inv;
                RtmPose::preprocess_to_blob(opts, frame, box, cpu_chw.data(), M_inv);
                const double* m6 = M_inv.ptr<double>();
                if (gpu(rgba.data, rgba.cols, rgba.rows, pitch, m6, OW, OH,
                        mean_bgr, inv_std_bgr, gpu_chw.data()) != 0) {
                    std::fprintf(stderr, "GPU kernel failed (frame %d)\n", fidx);
                    return 2;
                }
                double mx = 0.0, sa = 0.0, l2 = 0.0;
                for (std::size_t i = 0; i < per; ++i) {
                    double d = std::fabs(static_cast<double>(cpu_chw[i]) - gpu_chw[i]);
                    mx = std::max(mx, d);
                    sa += d;
                    l2 += d * d;
                }
                sa /= per;
                l2 = std::sqrt(l2 / per);
                worst_max = std::max(worst_max, mx);
                sum_mean += sa;
                sum_l2 += l2;
                ++cases;
                // Threshold = OpenCV's fixed-point interpolation floor. The CPU
                // reference is cv::warpAffine, which quantizes bilinear weights
                // to 1/32 (INTER_BITS=5); a single pixel can therefore differ by
                // up to 255*(1/64)/std ~= 0.07 in normalized units from this
                // float kernel (the kernel is the more accurate of the two). A
                // diff materially above this signals a real bug (geometry,
                // channel order), which would show as diffs of order 1-4.
                if (mx > 0.1) ++fail;
            }
        }
        ++fidx;
    }

    if (cases == 0) { std::fprintf(stderr, "no frames sampled\n"); return 2; }
    std::printf("gpu_preprocess_check: %d cases (%d frames @ step %d)\n",
                cases, fidx, step);
    std::printf("  worst max abs diff = %.6f\n", worst_max);
    std::printf("  mean abs diff      = %.6f (avg over cases)\n", sum_mean / cases);
    std::printf("  mean L2            = %.6f (avg over cases)\n", sum_l2 / cases);
    std::printf("  result: %s (threshold max<0.1, OpenCV fixed-point floor)\n",
                fail ? "FAIL" : "PASS");
    if (lib) ::dlclose(lib);
    return fail ? 1 : 0;
}
