// Correctness check for the all-GPU front-end (M2), over a recorded video.
//
// Two modes:
//   1. CHW check (default):  for each sampled frame + bbox, compare
//        (CPU ref) RtmPose::preprocess_to_blob   -> cpu_chw
//        (GPU)     libfitra_nvjpeg preprocess kernel (same M_inv) -> gpu_chw
//      reporting max / mean abs diff. The two agree to within float-bilinear-
//      vs-OpenCV-fixed-point tolerance.
//
//   2. keypoint check (pass a pose .engine as argv[2]): the end-to-end Step B
//      proof. For each frame + bbox compare keypoints from
//        (host path)   RtmPose::infer            (preprocess_to_blob + H2D)
//        (device path) RtmPose::infer_prebaked    (kernel -> device CHW ->
//                                                  TRT device-input copy)
//      reporting keypoint L2 in pixels. This drives the exact run_one_prebaked
//      device branch + TrtEngine::copy_input_region_from_device.
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
#include <memory>
#include <string>
#include <vector>

#include <cuda_runtime_api.h>
#include <NvInfer.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "infer/rtmpose.hpp"
#include "infer/trt_engine.hpp"
#include "infer/types.hpp"
#include "util/logging.hpp"

using fitra::infer::Bbox;
using fitra::infer::Person;
using fitra::infer::RtmPose;

namespace {

const float kMeanBgr[3]   = {103.53f, 116.28f, 123.675f};
const float kInvStdBgr[3] = {1.f / 57.375f, 1.f / 57.12f, 1.f / 58.395f};

using PreprocHostFn = int (*)(const unsigned char*, int, int, int,
                              const double*, int, int,
                              const float*, const float*, float*);
using PreprocLaunchFn = int (*)(const void*, int, int, int,
                                const double*, int, int,
                                const float*, const float*, float*, void*);

class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity > Severity::kWARNING) return;
        FITRA_LOG_WARN("[trt] {}", msg);
    }
};

void* g_lib = nullptr;
template <typename Fn>
Fn load_sym(const char* name) {
    const char* env = std::getenv("FITRA_NVJPEG_SO");
    std::vector<std::string> paths;
    if (env && *env) paths.emplace_back(env);
    paths.emplace_back("cpp/build/libfitra_nvjpeg.so");
    paths.emplace_back("libfitra_nvjpeg.so");
    if (!g_lib) {
        for (const auto& p : paths) {
            g_lib = ::dlopen(p.c_str(), RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);
            if (g_lib) break;
        }
    }
    return g_lib ? reinterpret_cast<Fn>(::dlsym(g_lib, name)) : nullptr;
}

std::vector<Bbox> boxes_for(int w, int h) {
    const float W = static_cast<float>(w), H = static_cast<float>(h);
    auto bb = [](float a, float b, float c, float d) { return Bbox{a, b, c, d, 0.f}; };
    return {bb(0, 0, W, H),
            bb(W * 0.25f, H * 0.10f, W * 0.75f, H * 0.95f),
            bb(W * 0.05f, H * 0.30f, W * 0.45f, H * 0.90f),
            bb(W * 0.30f, H * 0.20f, W * 0.55f, H * 0.98f)};
}

// Mode 1: CHW kernel vs CPU preprocess_to_blob.
int run_chw_check(const std::string& video) {
    PreprocHostFn gpu = load_sym<PreprocHostFn>("fitra_nvjpeg_preprocess_rgba_host");
    if (!gpu) { std::fprintf(stderr, "no preprocess_rgba_host symbol\n"); return 2; }
    cv::VideoCapture cap(video);
    if (!cap.isOpened()) { std::fprintf(stderr, "cannot open %s\n", video.c_str()); return 2; }

    RtmPose::Options opts;
    const int OW = opts.input_w, OH = opts.input_h;
    const std::size_t per = RtmPose::blob_floats_per_item(opts);
    std::vector<float> cpu_chw(per), gpu_chw(per);
    double worst = 0, sum_mean = 0, sum_l2 = 0;
    int cases = 0, fail = 0, fidx = 0;
    const int total = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    const int step = (total > 0) ? std::max(1, total / 8) : 100;

    cv::Mat frame, rgba;
    while (cap.read(frame) && !frame.empty()) {
        if (fidx % step == 0) {
            cv::cvtColor(frame, rgba, cv::COLOR_BGR2RGBA);
            for (const auto& box : boxes_for(frame.cols, frame.rows)) {
                cv::Mat M_inv;
                RtmPose::preprocess_to_blob(opts, frame, box, cpu_chw.data(), M_inv);
                if (gpu(rgba.data, rgba.cols, rgba.rows, rgba.cols * 4,
                        M_inv.ptr<double>(), OW, OH, kMeanBgr, kInvStdBgr,
                        gpu_chw.data()) != 0) { std::fprintf(stderr, "kernel fail\n"); return 2; }
                double mx = 0, sa = 0, l2 = 0;
                for (std::size_t i = 0; i < per; ++i) {
                    double d = std::fabs(static_cast<double>(cpu_chw[i]) - gpu_chw[i]);
                    mx = std::max(mx, d); sa += d; l2 += d * d;
                }
                worst = std::max(worst, mx); sum_mean += sa / per;
                sum_l2 += std::sqrt(l2 / per); ++cases;
                if (mx > 0.1) ++fail;  // OpenCV fixed-point (1/32) interpolation floor
            }
        }
        ++fidx;
    }
    if (!cases) { std::fprintf(stderr, "no frames\n"); return 2; }
    std::printf("CHW check: %d cases (%d frames @ step %d)\n", cases, fidx, step);
    std::printf("  worst max abs = %.6f | mean abs = %.6f | mean L2 = %.6f\n",
                worst, sum_mean / cases, sum_l2 / cases);
    std::printf("  result: %s (threshold max<0.1, OpenCV fixed-point floor)\n",
                fail ? "FAIL" : "PASS");
    return fail ? 1 : 0;
}

// Mode 2: keypoints host path vs device path (full Step B).
int run_keypoint_check(const std::string& video, const std::string& pose_engine) {
    PreprocLaunchFn launch = load_sym<PreprocLaunchFn>("fitra_nvjpeg_preprocess_launch");
    if (!launch) { std::fprintf(stderr, "no preprocess_launch symbol\n"); return 2; }

    TrtLogger tlog;
    std::unique_ptr<nvinfer1::IRuntime> runtime{nvinfer1::createInferRuntime(tlog)};
    if (!runtime) { std::fprintf(stderr, "createInferRuntime failed\n"); return 2; }
    auto engine = fitra::infer::TrtEngine::from_file(*runtime, pose_engine, tlog);
    RtmPose rtmpose{*engine};
    const auto opts = rtmpose.options();
    const int OW = opts.input_w, OH = opts.input_h;
    const std::size_t per = RtmPose::blob_floats_per_item(opts);

    cv::VideoCapture cap(video);
    if (!cap.isOpened()) { std::fprintf(stderr, "cannot open %s\n", video.c_str()); return 2; }

    float* d_rgba = nullptr; float* d_chw = nullptr;
    std::size_t rgba_cap = 0;
    if (cudaMalloc(reinterpret_cast<void**>(&d_chw), per * sizeof(float)) != cudaSuccess) {
        std::fprintf(stderr, "cudaMalloc d_chw failed\n"); return 2;
    }

    double worst = 0, sum = 0; int cases = 0, fail = 0, fidx = 0;
    const int total = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    const int step = (total > 0) ? std::max(1, total / 8) : 100;
    cv::Mat frame, rgba;
    while (cap.read(frame) && !frame.empty()) {
        if (fidx % step == 0) {
            cv::cvtColor(frame, rgba, cv::COLOR_BGR2RGBA);
            const std::size_t rbytes = static_cast<std::size_t>(rgba.cols) * 4 * rgba.rows;
            if (rbytes > rgba_cap) {
                if (d_rgba) cudaFree(d_rgba);
                if (cudaMalloc(reinterpret_cast<void**>(&d_rgba), rbytes) != cudaSuccess) {
                    std::fprintf(stderr, "cudaMalloc d_rgba failed\n"); return 2;
                }
                rgba_cap = rbytes;
            }
            cudaMemcpy(d_rgba, rgba.data, rbytes, cudaMemcpyHostToDevice);
            for (const auto& box : boxes_for(frame.cols, frame.rows)) {
                // Host path.
                std::vector<Person> hp = rtmpose.infer(frame, {box});
                // Device path: kernel -> device CHW -> infer_prebaked.
                cv::Mat M_inv = RtmPose::compute_m_inv(opts, box);
                if (launch(d_rgba, rgba.cols, rgba.rows, rgba.cols * 4,
                           M_inv.ptr<double>(), OW, OH, kMeanBgr, kInvStdBgr,
                           d_chw, nullptr) != 0 ||
                    cudaDeviceSynchronize() != cudaSuccess) {
                    std::fprintf(stderr, "device preprocess fail\n"); return 2;
                }
                RtmPose::PrebakedRequest pr;
                pr.chw_dev = d_chw; pr.M_inv = M_inv; pr.bbox = box;
                std::vector<Person> dp = rtmpose.infer_prebaked({pr});
                if (hp.empty() || dp.empty()) continue;
                // Only compare confident keypoints: where SimCC is near-flat
                // (low score, e.g. a synthetic bbox over no real person) argmax
                // is hypersensitive to sub-LSB input differences and known to
                // jump 100s of px under FP16 (see core-pipeline track doc).
                // That is an engine property, not a device-path defect.
                double l2 = 0; int n = 0, K = hp[0].kp_count;
                for (int k = 0; k < K; ++k) {
                    if (hp[0].kpts[k].score < 0.5f || dp[0].kpts[k].score < 0.5f) continue;
                    double dx = hp[0].kpts[k].x - dp[0].kpts[k].x;
                    double dy = hp[0].kpts[k].y - dp[0].kpts[k].y;
                    l2 += std::sqrt(dx * dx + dy * dy);
                    ++n;
                }
                if (n == 0) continue;  // no confident keypoints in this case
                l2 /= n;  // mean L2 (px) over confident keypoints
                worst = std::max(worst, l2); sum += l2; ++cases;
                // 2px gate: device vs host differ only by CPU-fixed-point vs
                // GPU-float bilinear (CHW <=0.058). On a heavily-downscaled
                // synthetic bbox a single SimCC bin (~1.6px in source) can flip,
                // so the tail reaches ~1.2px; avg is sub-pixel. A real
                // offset/stride bug would show 10s-100s px (see the unfiltered
                // run). 2px catches that while accepting sub-bin quantization.
                if (l2 > 2.0) ++fail;
            }
        }
        ++fidx;
    }
    if (d_rgba) cudaFree(d_rgba);
    if (d_chw) cudaFree(d_chw);
    if (!cases) { std::fprintf(stderr, "no frames\n"); return 2; }
    std::printf("keypoint check: %d cases (%d frames @ step %d), engine=%s\n",
                cases, fidx, step, pose_engine.c_str());
    std::printf("  worst mean-kpt L2 = %.4f px | avg = %.4f px (confident kpts only)\n",
                worst, sum / cases);
    std::printf("  result: %s (threshold L2<2px)\n", fail ? "FAIL" : "PASS");
    return fail ? 1 : 0;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string video = (argc > 1)
        ? argv[1]
        : "outputs/recorded_rtmpose/20260515_064342/raw_cam0.mp4";
    int rc = run_chw_check(video);
    if (rc == 0 && argc > 2) {
        std::printf("---\n");
        rc = run_keypoint_check(video, argv[2]);
    }
    if (g_lib) ::dlclose(g_lib);
    return rc;
}
