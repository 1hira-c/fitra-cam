#include "app/mode_run.hpp"

#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <vector>

#include "app/camera_builder.hpp"
#include "app/idle_evaluator.hpp"
#include "app/idle_state.hpp"
#include "app/output_builder.hpp"
#include "app/pose_relay_builder.hpp"
#include "app/server_builder.hpp"
#include "app/stats_loop.hpp"
#include "app/threed_builder.hpp"
#include "app/trt_stack.hpp"
#include "corpus/event_corpus_recorder.hpp"
#include "util/logging.hpp"

namespace fitra::app {

namespace {

camera::PixFmt corpus_pixel_format(const config::MainOptions& opts,
                                   std::size_t slot) {
    const std::string& configured = opts.cam_pixel_format[slot].empty()
                                      ? opts.pixel_format
                                      : opts.cam_pixel_format[slot];
    if (configured == "yuyv") return camera::PixFmt::Yuyv;
    if (configured == "nvjpeg") return camera::PixFmt::Nvjpeg;
    return camera::PixFmt::Mjpeg;
}

std::vector<corpus::CameraSpec> corpus_camera_specs(
    const config::MainOptions& opts) {
    std::vector<corpus::CameraSpec> cameras;
    for (std::size_t slot = 0; slot < opts.cam_paths.size(); ++slot) {
        if (opts.cam_paths[slot].empty()) continue;
        corpus::CameraSpec camera;
        camera.index = cameras.size();
        camera.config_slot = slot;
        camera.device_path = opts.cam_paths[slot];
        camera.capture_width = opts.cam_cap_width[slot] > 0
                                 ? opts.cam_cap_width[slot] : opts.width;
        camera.capture_height = opts.cam_cap_height[slot] > 0
                                  ? opts.cam_cap_height[slot] : opts.height;
        camera.output_width = opts.width;
        camera.output_height = opts.height;
        camera.requested_fps = opts.fps;
        camera.pixel_format = corpus_pixel_format(opts, slot);
        cameras.push_back(std::move(camera));
    }
    return cameras;
}

corpus::RecorderOptions corpus_options(const config::MainOptions& opts) {
    const auto millis = [](double seconds) {
        return std::chrono::milliseconds{
            static_cast<std::chrono::milliseconds::rep>(
                std::llround(seconds * 1000.0))};
    };
    corpus::RecorderOptions out;
    out.spool_dir = opts.corpus_spool_dir;
    out.remote_dir = opts.corpus_remote_dir;
    out.pre_roll = millis(opts.corpus_pre_roll_s);
    out.post_roll = millis(opts.corpus_post_roll_s);
    out.segment = millis(opts.corpus_segment_s);
    out.upload_absent = millis(opts.corpus_upload_absent_s);
    out.presence_stale = millis(opts.corpus_presence_stale_s);
    out.queue_max_bytes = static_cast<std::size_t>(opts.corpus_queue_max_mib)
                        * 1024ULL * 1024ULL;
    out.ring_max_bytes = static_cast<std::size_t>(opts.corpus_ring_max_mib)
                       * 1024ULL * 1024ULL;
    out.spool_max_bytes = static_cast<std::uintmax_t>(
        opts.corpus_spool_max_gib * 1024.0 * 1024.0 * 1024.0);
    out.min_free_bytes = static_cast<std::uintmax_t>(
        opts.corpus_min_free_gib * 1024.0 * 1024.0 * 1024.0);
    out.remove_after_upload = opts.corpus_remove_after_upload;
    return out;
}

}  // namespace

int run_mode_run(const config::MainOptions& opts, FlowControl& flow) {
    auto trt = make_trt_stack(opts);
    // Consumer-presence state for idle/standby (issue #37). Declared first so it
    // outlives every component that reads or writes it. Run mode only — calib
    // modes never construct one, so their components are never throttled.
    app::IdleState idle_state;
    // Idle is force-disabled for the live benchmark path (--bench-fake-bbox):
    // it injects a synthetic bbox to keep inference saturated for ceiling
    // measurement, which has no real consumer (often --no-web) and would
    // otherwise standby after enter_after_s and collapse the measured fps.
    const bool idle_enabled = opts.idle_enabled && !opts.bench_fake_bbox
                           && !opts.corpus_enabled;
    if (opts.idle_enabled && opts.bench_fake_bbox) {
        FITRA_LOG_INFO("idle: disabled for --bench-fake-bbox (benchmark path)");
    }
    if (opts.idle_enabled && opts.corpus_enabled) {
        FITRA_LOG_INFO("idle: disabled while event corpus recording is enabled");
    }

    std::unique_ptr<corpus::EventCorpusRecorder> corpus_recorder;
    std::function<void(std::size_t, const camera::Frame&)> raw_frame_tap;
    if (opts.corpus_enabled) {
        auto camera_specs = corpus_camera_specs(opts);
        for (const auto& camera : camera_specs) {
            if (camera.pixel_format == camera::PixFmt::Yuyv) {
                FITRA_LOG_WARN(
                    "corpus: camera {} uses raw YUYV; expect very high SSD bandwidth/usage",
                    camera.index);
            }
        }
        corpus_recorder = std::make_unique<corpus::EventCorpusRecorder>(
            corpus_options(opts), std::move(camera_specs));
        corpus_recorder->start();
        raw_frame_tap = [recorder = corpus_recorder.get()](
                            std::size_t camera_index,
                            const camera::Frame& frame) noexcept {
            recorder->ingest_raw(camera_index, frame);
        };
    }
    auto cams = make_frame_sources(opts, trt.get(), nullptr,
                                   idle_enabled ? &idle_state.idle : nullptr,
                                   std::move(raw_frame_tap));
    const std::size_t n_cams = cams.sources.size();
    if (opts.enable_3d && n_cams < 2) {
        std::fprintf(stderr, "--enable-3d requires at least two cameras\n");
        return EXIT_FAILURE;
    }

    ThreeDSet threed;
    bool enable_3d = opts.enable_3d;
    if (enable_3d) {
        // The extrinsics calibration is a generated artifact resolved at run
        // time and may be missing/unreadable (e.g. the daemon crash-fell back
        // to Run before calibration completed, or a manual --enable-3d run on a
        // not-yet-calibrated rig). Degrade to 2D rather than throwing: a hard
        // failure here exits non-flow, the daemon counts it as a crash, and the
        // Run fallback crash-loops to give-up. This honors precheck's contract
        // "Run is the safe fallback; it tolerates a missing calib (2D only)"
        // (docs/design/pose-3d-calib-latest-resolution.md).
        try {
            threed = make_threed(opts, n_cams, opts.subject_height_m);
        } catch (const std::exception& e) {
            FITRA_LOG_WARN("3D disabled: calibration unavailable ({}); running 2D "
                           "only — calibrate extrinsics to enable 3D", e.what());
            enable_3d = false;
        }
    }

    pipeline::SnapshotBus bus{n_cams};
    auto driver = make_driver(std::move(cams.sources), *trt->rtmpose, bus, opts,
                              enable_3d ? &threed : nullptr);
    if (corpus_recorder) {
        driver->set_presence_tap(
            [recorder = corpus_recorder.get()](std::size_t camera_index,
                                                bool present,
                                                std::chrono::steady_clock::time_point
                                                    captured_at,
                                                std::uint64_t captured_mono_ns) noexcept {
                recorder->observe_presence(camera_index, present,
                                            captured_at,
                                            captured_mono_ns);
            });
    }
    // Idle/standby gate (issue #37): while idle the driver skips the 3D update
    // and throttles to idle_tick_hz. Run mode only — calib never sets it.
    driver->set_idle_gate(idle_enabled ? &idle_state.idle : nullptr,
                          opts.idle_tick_hz);
    driver->start();

    // Stop the driver worker on any scope exit (exception path); normal
    // shutdown calls driver->stop() explicitly below.
    struct DriverStop {
        pipeline::MultiCameraDriver* d;
        ~DriverStop() { if (d) d->stop(); }
    } driver_stop{driver.get()};

    // Start the TrackerExtractor before any consumer so the publishers and
    // the WebUI both see the same smoothed tracker stream.
    std::unique_ptr<tracking::TrackerExtractor> tracker_extractor;
    if (threed.bus3d && threed.tracker_bus) {
        tracker_extractor =
            make_tracker_extractor(opts, *threed.bus3d, *threed.tracker_bus,
                                   idle_enabled ? &idle_state.idle : nullptr,
                                   threed.tracker_axis_bus.get(),
                                   threed.tracker_axis_lineage_bus.get());
    }

    // Pose relay (input) + publishers/aligner (output). Publishers spin up
    // BEFORE the Crow server so /stats3d can hand out their stats blocks.
    auto relay = make_pose_relay(opts, opts.hmd_listen_enabled);
    auto outputs = make_run_outputs(opts, threed.bus3d.get(),
                                    threed.tracker_bus.get(),
                                    opts.hmd_listen_enabled
                                        ? relay.hmd_bus.get() : nullptr,
                                    relay.beacon
                                        ? &relay.beacon->endpoint_bus() : nullptr);

    // Stop outputs + relay + extractor on any scope exit. Must outlive the
    // server (so /stats3d never reads a dead pointer) and the driver (they
    // read buses the driver feeds).
    struct OutputStop {
        RunOutputs* outputs;
        PoseRelay*  relay;
        tracking::TrackerExtractor* tex;
        ~OutputStop() {
            if (outputs) outputs->stop();
            if (relay)   relay->stop();
            if (tex)     tex->stop();
        }
    } output_stop{&outputs, &relay, tracker_extractor.get()};

    auto server = make_server(opts, config::RunMode::Run, bus, threed.bus3d.get(),
                              &flow);
    if (server) {
        if (outputs.vmt_pub)   server->set_vmt_publisher(outputs.vmt_pub.get());
        if (relay.beacon)      server->set_discovery_beacon(relay.beacon.get());
        if (threed.tracker_bus) server->set_tracker_bus(threed.tracker_bus.get());
        if (threed.pose_gate_bus) server->set_pose_gate_bus(threed.pose_gate_bus.get());
        if (threed.fusion_pose_bus) server->set_fusion_pose_bus(threed.fusion_pose_bus.get());
        if (threed.tracker_axis_bus) {
            server->set_tracker_axis_bus(threed.tracker_axis_bus.get());
        }
        if (opts.hmd_listen_enabled) {
            server->set_hmd_pose_bus(relay.hmd_bus.get(), opts.hmd_stale_ms);
        }
        server->set_align_hmd_forward_m(
            static_cast<float>(opts.vmt_align_hmd_forward_m));
        if (outputs.aligner) server->set_continuous_aligner(outputs.aligner.get());
        server->set_idle_state(&idle_state, idle_enabled,
                               opts.idle_enter_after_s, opts.idle_tick_hz);
        server->start();
    }

    // Whether the HMD return channel is actually live: --hmd-listen-enabled may
    // have been requested but the receiver socket failed to bind (relay.receiver
    // == nullptr). Use the real state, not the flag — otherwise we'd report VR
    // as observable (vr_observable=true) while never seeing a peer, and a
    // VMT rig with a dead receiver would standby after enter_after_s
    // and stop emitting pose. With hmd_live=false the evaluator's safe default
    // keeps VR "present" so we never idle on an unobservable axis.
    const bool hmd_live = relay.receiver != nullptr;

    // Idle/standby evaluator: derives consumer presence from the WS client
    // count (maintained by the server) + HMD-pose freshness, and confirms
    // idle_state.idle with asymmetric hysteresis. hmd_bus is passed only when
    // the receiver is actually live (otherwise VR presence is unobservable).
    app::IdleEvaluator::Config idle_cfg;
    idle_cfg.enabled            = idle_enabled;
    idle_cfg.enter_after_s      = opts.idle_enter_after_s;
    idle_cfg.tick_hz            = opts.idle_tick_hz;
    idle_cfg.hmd_stale_ms       = opts.hmd_stale_ms;
    idle_cfg.has_vr_output      = opts.vmt_out;
    idle_cfg.hmd_listen_enabled = hmd_live;
    app::IdleEvaluator idle_eval{
        idle_state,
        hmd_live ? relay.hmd_bus.get() : nullptr,
        idle_cfg};
    idle_eval.start();

    run_stats_loop(*driver, opts.log_every_s, flow.stop);

    // Stop the evaluator before the relay/outputs it observes are torn down.
    idle_eval.stop();
    if (server) server->stop();
    outputs.stop();
    driver->stop();
    if (corpus_recorder) corpus_recorder->stop();
    return EXIT_SUCCESS;
}

}  // namespace fitra::app
