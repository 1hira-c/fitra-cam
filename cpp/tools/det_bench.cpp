// det_bench — YOLOX 検出器単体の per-frame inference 時間ベンチ。
//
// Issue #9 番外編で、yolox-tiny FP32 を予算上限として S / M / X × FP16/FP32 の
// e2e latency を比較し、運用既定モデルを決めるための軽量ツール。
//
// 入力フレームは:
//   --frame PATH を指定すれば実フレーム (JPEG/PNG/MP4 1枚目)
//   未指定なら 1280x720 のグラデーション擬似フレーム
// で letterbox に流す。Yolox::infer() 全体 (前処理 + H2D + enqueue + sync + D2H)
// を 1 iter としてミリ秒で計る。
//
// Usage:
//   det_bench --engine PATH [--engine PATH ...]
//             [--frame PATH] [--iters N] [--warmup N] [--det-score F]
//
// 出力:
//   per engine: median / p50 / p90 / mean ms, persons-per-frame, input_size。

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <memory>
#include <numeric>
#include <string>
#include <string_view>
#include <vector>

#include <NvInfer.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "infer/trt_engine.hpp"
#include "infer/yolox.hpp"
#include "util/cuda_check.hpp"
#include "util/logging.hpp"

namespace {

class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity > Severity::kWARNING) return;
        using S = Severity;
        switch (severity) {
            case S::kINTERNAL_ERROR: FITRA_LOG_ERROR("[trt] INTERNAL: {}", msg); return;
            case S::kERROR:          FITRA_LOG_ERROR("[trt] {}",          msg); return;
            case S::kWARNING:        FITRA_LOG_WARN ("[trt] {}",          msg); return;
            case S::kINFO:           FITRA_LOG_INFO ("[trt] {}",          msg); return;
            case S::kVERBOSE:        FITRA_LOG_TRACE("[trt] {}",          msg); return;
        }
    }
};

void print_help() {
    std::puts(
        "det_bench — YOLOX engine per-frame latency benchmark\n"
        "\n"
        "Required:\n"
        "  --engine PATH         .engine to benchmark (repeatable)\n"
        "\n"
        "Optional:\n"
        "  --frame PATH          image (jpg/png) or video (first frame). Default: synthetic 1280x720.\n"
        "  --iters N             timed iterations per engine (default 200)\n"
        "  --warmup N            untimed warmup iters per engine (default 30)\n"
        "  --det-score F         YOLOX score threshold (default 0.5)\n");
}

cv::Mat load_frame_or_synth(const std::string& path) {
    if (!path.empty()) {
        std::filesystem::path p{path};
        if (!std::filesystem::exists(p)) {
            throw std::runtime_error("frame not found: " + path);
        }
        auto ext = p.extension().string();
        for (auto& c : ext) c = static_cast<char>(std::tolower(c));
        if (ext == ".mp4" || ext == ".mov" || ext == ".avi" || ext == ".mkv") {
            cv::VideoCapture cap{path};
            if (!cap.isOpened()) throw std::runtime_error("cannot open video: " + path);
            cv::Mat f;
            if (!cap.read(f) || f.empty()) {
                throw std::runtime_error("empty first frame: " + path);
            }
            return f;
        }
        cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
        if (img.empty()) throw std::runtime_error("cannot read image: " + path);
        return img;
    }
    // Synthetic gradient — same memory characteristics as a real BGR frame.
    cv::Mat f(720, 1280, CV_8UC3);
    for (int y = 0; y < f.rows; ++y) {
        auto* row = f.ptr<std::uint8_t>(y);
        for (int x = 0; x < f.cols; ++x) {
            row[3 * x + 0] = static_cast<std::uint8_t>((x * 255) / f.cols);
            row[3 * x + 1] = static_cast<std::uint8_t>((y * 255) / f.rows);
            row[3 * x + 2] = static_cast<std::uint8_t>(((x + y) * 255) / (f.cols + f.rows));
        }
    }
    return f;
}

double percentile_ms(std::vector<double>& v, double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    std::size_t idx = static_cast<std::size_t>(p * (v.size() - 1));
    return v[idx];
}

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> engine_paths;
    std::string frame_path;
    int iters  = 200;
    int warmup = 30;
    float det_score = 0.5f;

    for (int i = 1; i < argc; ++i) {
        std::string_view a{argv[i]};
        auto need = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing argument for %s\n", flag);
                std::exit(EXIT_FAILURE);
            }
            return argv[++i];
        };
        if      (a == "--help" || a == "-h") { print_help(); return EXIT_SUCCESS; }
        else if (a == "--engine")    engine_paths.emplace_back(need("--engine"));
        else if (a == "--frame")     frame_path = need("--frame");
        else if (a == "--iters")     iters  = std::atoi(need("--iters"));
        else if (a == "--warmup")    warmup = std::atoi(need("--warmup"));
        else if (a == "--det-score") det_score = std::stof(need("--det-score"));
        else {
            std::fprintf(stderr, "unknown arg: %s\n", argv[i]);
            print_help();
            return EXIT_FAILURE;
        }
    }
    if (engine_paths.empty()) {
        print_help();
        return EXIT_FAILURE;
    }

    try {
        TrtLogger tlog;
        std::unique_ptr<nvinfer1::IRuntime> rt{nvinfer1::createInferRuntime(tlog)};
        TRT_CHECK(rt != nullptr);

        cv::Mat frame = load_frame_or_synth(frame_path);
        FITRA_LOG_INFO("input frame: {}x{}", frame.cols, frame.rows);

        std::printf("%-70s  %6s  %8s  %8s  %8s  %8s  %6s\n",
                    "engine", "in", "median", "p50", "p90", "mean", "ppf");

        for (const auto& path : engine_paths) {
            FITRA_LOG_INFO("--- {}", path);
            auto eng = fitra::infer::TrtEngine::from_file(*rt, path, tlog);
            fitra::infer::Yolox::Options yopts;
            yopts.score_thr = det_score;
            fitra::infer::Yolox yolox{*eng, yopts};
            // input_size after auto-detect.
            const int in_size = eng->binding("input").dims.d[2];

            // Warmup.
            for (int w = 0; w < warmup; ++w) {
                (void)yolox.infer(frame);
            }

            std::vector<double> ms;
            ms.reserve(static_cast<std::size_t>(iters));
            std::size_t total_persons = 0;
            for (int it = 0; it < iters; ++it) {
                auto t0 = std::chrono::steady_clock::now();
                auto bboxes = yolox.infer(frame);
                auto t1 = std::chrono::steady_clock::now();
                ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
                total_persons += bboxes.size();
            }

            double mean = std::accumulate(ms.begin(), ms.end(), 0.0) / ms.size();
            auto ms_copy = ms;
            double p50  = percentile_ms(ms_copy, 0.50);
            double p90  = percentile_ms(ms_copy, 0.90);
            double med  = p50;
            double ppf  = static_cast<double>(total_persons) / iters;

            std::printf("%-70s  %6d  %7.2fms  %7.2fms  %7.2fms  %7.2fms  %6.2f\n",
                        std::filesystem::path(path).filename().string().c_str(),
                        in_size, med, p50, p90, mean, ppf);
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        FITRA_LOG_ERROR("fatal: {}", e.what());
        return EXIT_FAILURE;
    }
}
