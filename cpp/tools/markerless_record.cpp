// Dedicated batch raw capture. --record is required to open cameras/write files.
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <time.h>

#include "camera/markerless_recording.hpp"
#include "config/main_config.hpp"
#include "lift/calib_io.hpp"
#include "util/sha256.hpp"

namespace {
volatile std::sig_atomic_t interrupted = 0;
void signal_handler(int) { interrupted = 1; }

std::uint64_t now_ns() {
    timespec time{};
    if (::clock_gettime(CLOCK_MONOTONIC, &time) != 0)
        throw std::runtime_error("cannot read CLOCK_MONOTONIC");
    return static_cast<std::uint64_t>(time.tv_sec) * 1000000000ULL + time.tv_nsec;
}

double number(const std::string& input) {
    double value = 0;
    const auto result = std::from_chars(input.data(), input.data() + input.size(), value);
    if (result.ec != std::errc{} || result.ptr != input.data() + input.size() || !std::isfinite(value))
        throw std::runtime_error("invalid numeric argument");
    return value;
}

std::vector<fitra::camera::MarkerlessRecordingCamera> cameras(const fitra::config::MainOptions& opts) {
    using namespace fitra::camera;
    if (opts.width <= 0 || opts.height <= 0 || opts.width > 8192 || opts.height > 8192
        || opts.fps <= 0 || opts.fps > 240 || opts.n_buffers < 2 || opts.n_buffers > 64)
        throw std::runtime_error("invalid recording camera dimensions/fps/buffers");
    std::vector<MarkerlessRecordingCamera> out;
    std::set<std::string> devices;
    for (std::size_t slot = 0; slot < opts.cam_paths.size(); ++slot) {
        if (opts.cam_paths[slot].empty()) continue;
        const auto device = std::filesystem::weakly_canonical(opts.cam_paths[slot]).string();
        if (!devices.insert(device).second) throw std::runtime_error("duplicate camera device");
        MarkerlessRecordingCamera camera;
        camera.id = "cam" + std::to_string(out.size());
        camera.config_slot = slot;
        auto& in = camera.capture;
        in.device_path = opts.cam_paths[slot];
        in.width = opts.width;
        in.height = opts.height;
        in.cap_width = opts.cam_cap_width[slot];
        in.cap_height = opts.cam_cap_height[slot];
        in.fps = opts.fps;
        in.n_buffers = opts.n_buffers;
        const auto format = opts.cam_pixel_format[slot].empty() ? opts.pixel_format : opts.cam_pixel_format[slot];
        if (format != "mjpeg" && format != "nvjpeg" && format != "yuyv")
            throw std::runtime_error("unsupported recording pixel format");
        in.pixel_format = format == "yuyv" ? PixFmt::Yuyv : PixFmt::Mjpeg;
        const auto& exposure = opts.cam_exposure_mode[slot];
        if (!exposure.empty() && exposure != "auto" && exposure != "manual" && exposure != "assist")
            throw std::runtime_error("unsupported exposure mode");
        // No decode/assist feedback loop: preserve the configured initial
        // exposure/gain, and label this explicitly in the recording manifest.
        in.exposure_mode = exposure.empty() || exposure == "auto"
            ? V4l2Options::ExposureMode::Auto : V4l2Options::ExposureMode::Manual;
        in.exposure_us100 = opts.cam_exposure[slot];
        in.gain = opts.cam_gain[slot];
        if (in.capture_w() <= 0 || in.capture_h() <= 0 || in.capture_w() > 8192 || in.capture_h() > 8192
            || in.exposure_us100 < 0 || in.gain < -1)
            throw std::runtime_error("invalid capture size/exposure/gain");
        out.push_back(std::move(camera));
    }
    if (out.size() < 2 || out.size() > 3) throw std::runtime_error("recording needs 2..3 configured cameras");
    return out;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        std::string config, intrinsic, reference, output;
        bool record = false, json_plan = false;
        double height = 0, max_gib = 32, reserve_gib = 2;
        std::set<std::string> seen;
        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i];
            if (argument == "--help" || argument == "-h") {
                std::puts("markerless_record [--plan-json] [--config PATH] [--intrinsics PATH]\n"
                    "                  [--reference PATH] [--height-m N] [--out NEW_DIR] [--record]\n"
                    "                  [--max-gib 32] [--min-free-gib 2]\n"
                    "Default: print the 370-second Japanese shooting guide; no device/file mutation.\n"
                    "--config + --intrinsics add preflight; only the cameras config section is read.\n"
                    "--record explicitly starts raw capture.\n"
                    "No inference, VMT, upload, re-encoding or calibration application.\n"
                    "Stop the normal camera-owning process yourself before recording.\n"
                    "Assist exposure is held at configured initial manual exposure/gain.");
                return 0;
            }
            if (!seen.insert(argument).second) throw std::runtime_error("duplicate argument");
            if (argument == "--record") { record = true; continue; }
            if (argument == "--plan-json") { json_plan = true; continue; }
            if (i + 1 >= argc) throw std::runtime_error("missing argument value");
            const std::string value = argv[++i];
            if (argument == "--config") config = value;
            else if (argument == "--intrinsics") intrinsic = value;
            else if (argument == "--reference") reference = value;
            else if (argument == "--out") output = value;
            else if (argument == "--height-m") height = number(value);
            else if (argument == "--max-gib") max_gib = number(value);
            else if (argument == "--min-free-gib") reserve_gib = number(value);
            else throw std::runtime_error("unknown argument");
        }
        if (max_gib <= 0 || max_gib > 1024 || reserve_gib < 0 || reserve_gib > 1024
            || height < 0 || height > 2.5) throw std::runtime_error("argument outside supported range");
        const auto plan = fitra::camera::markerless_capture_plan();
        if (json_plan) {
            if (record) throw std::runtime_error("--plan-json cannot record");
            std::cout << fitra::camera::markerless_capture_plan_json() << '\n';
            return std::cout ? 0 : 1;
        }
        std::vector<fitra::camera::MarkerlessRecordingCamera> specs;
        if (!config.empty()) {
            fitra::config::MainOptions opts;
            fitra::config::load_camera_config(config, opts);
            specs = cameras(opts);
            if (intrinsic.empty()) throw std::runtime_error("camera preflight requires explicit --intrinsics");
            const auto calibration = fitra::lift::load_calibration(intrinsic);
            for (const auto& camera : specs) {
                bool matched = false;
                for (const auto& in : calibration.cameras) {
                    if (in.id != camera.id) continue;
                    matched = in.intrinsics.width == camera.capture.width && in.intrinsics.height == camera.capture.height;
                }
                if (!matched) throw std::runtime_error("intrinsic id/output size mismatch; select the runtime-matching intrinsic file");
                std::cout << camera.id << " " << camera.capture.device_path << " raw "
                    << camera.capture.capture_w() << 'x' << camera.capture.capture_h()
                    << " output " << camera.capture.width << 'x' << camera.capture.height
                    << " requested " << camera.capture.fps << " fps\n";
            }
            if (!reference.empty()) (void)fitra::lift::load_calibration(reference);
            std::cout << "intrinsics: " << intrinsic << " (snapshotted during recording)\n";
        }
        int total = 0;
        for (const auto& cue : plan) {
            std::cout << total << "s [" << cue.take << '/' << cue.id << "] "
                << cue.seconds << "秒: " << cue.prompt << '\n';
            total += cue.seconds;
        }
        std::cout << "合計 " << total << "秒。長さは初期収録予算で、十分性・精度の保証ではありません。\n";
        if (!record) {
            std::cout << "確認のみ。録画は --config PATH --out NEW_DIR --record を指定してください。\n";
            return 0;
        }
        if (config.empty() || output.empty()) throw std::runtime_error("--record requires --config and --out");
        fitra::camera::MarkerlessRecordingOptions options;
        options.out_dir = output;
        options.intrinsics = intrinsic;
        options.reference = reference;
        options.height_m = height;
        options.recorder_binary_sha256 = fitra::util::sha256_file("/proc/self/exe");
        options.max_payload_bytes = static_cast<std::uint64_t>(max_gib * 1073741824.0);
        options.min_free_bytes = static_cast<std::uint64_t>(reserve_gib * 1073741824.0);
        fitra::camera::MarkerlessRecording session(options, specs);
        std::vector<std::unique_ptr<fitra::camera::V4l2Capture>> captures;
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);
        bool complete = false;
        std::string reason = "interrupted";
        try {
            for (const auto& spec : specs) {
                captures.push_back(std::make_unique<fitra::camera::V4l2Capture>(spec.capture));
                captures.back()->start();
            }
            std::vector<std::uint64_t> last_input(specs.size(), now_ns());
            for (std::size_t step = 0; step < plan.size() && !interrupted; ++step) {
                const auto start = now_ns();
                session.cue(step, start);
                std::cout << '\a' << "\n[" << step + 1 << '/' << plan.size() << "] "
                    << plan[step].prompt << "（" << plan[step].seconds << "秒）" << std::endl;
                while (!interrupted && now_ns() - start < static_cast<std::uint64_t>(plan[step].seconds) * 1000000000ULL) {
                    bool any = false;
                    for (std::size_t i = 0; i < captures.size(); ++i) {
                        fitra::camera::Frame frame;
                        if (captures[i]->try_pop_latest(frame)) {
                            session.append(i, frame);
                            last_input[i] = now_ns();
                            any = true;
                        }
                        if (now_ns() - last_input[i] > 3000000000ULL)
                            throw std::runtime_error("camera stopped delivering frames");
                    }
                    if (!any) std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
            complete = !interrupted;
            reason = complete ? "protocol_finished" : "interrupted";
        } catch (const std::exception& error) {
            reason = error.what();
        }
        for (auto& capture : captures) capture->stop();
        session.finish(complete, reason);
        std::cout << "保存先: " << output << " (" << session.payload_bytes() << " raw bytes)\n"
            << "次に check_markerless_capture.py で時刻・欠落・区間とchecksumを確認してください。\n";
        if (!complete) std::cerr << "Recording incomplete: " << reason << '\n';
        return complete ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "markerless_record: " << error.what() << '\n';
        return 1;
    }
}
