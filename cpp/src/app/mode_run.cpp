#include "app/mode_run.hpp"

#include <cstdio>
#include <cstdlib>
#include <memory>

#include "app/camera_builder.hpp"
#include "app/idle_evaluator.hpp"
#include "app/idle_state.hpp"
#include "app/output_builder.hpp"
#include "app/pose_relay_builder.hpp"
#include "app/server_builder.hpp"
#include "app/stats_loop.hpp"
#include "app/threed_builder.hpp"
#include "app/trt_stack.hpp"
#include "util/logging.hpp"

namespace fitra::app {

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
    const bool idle_enabled = opts.idle_enabled && !opts.bench_fake_bbox;
    if (opts.idle_enabled && opts.bench_fake_bbox) {
        FITRA_LOG_INFO("idle: disabled for --bench-fake-bbox (benchmark path)");
    }
    auto cams = make_frame_sources(opts, trt.get(), nullptr,
                                   idle_enabled ? &idle_state.idle : nullptr);
    const std::size_t n_cams = cams.sources.size();
    if (opts.enable_3d && n_cams < 2) {
        std::fprintf(stderr, "--enable-3d requires at least two cameras\n");
        return EXIT_FAILURE;
    }

    ThreeDSet threed;
    const bool enable_3d = opts.enable_3d;
    if (enable_3d) {
        threed = make_threed(opts, n_cams, opts.subject_height_m);
    }

    pipeline::SnapshotBus bus{n_cams};
    auto driver = make_driver(std::move(cams.sources), *trt->rtmpose, bus, opts,
                              enable_3d ? &threed : nullptr);
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
    std::unique_ptr<slimevr::TrackerExtractor> tracker_extractor;
    if (threed.bus3d && threed.tracker_bus) {
        tracker_extractor =
            make_tracker_extractor(opts, *threed.bus3d, *threed.tracker_bus,
                                   idle_enabled ? &idle_state.idle : nullptr);
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
        slimevr::TrackerExtractor* tex;
        ~OutputStop() {
            if (outputs) outputs->stop();
            if (relay)   relay->stop();
            if (tex)     tex->stop();
        }
    } output_stop{&outputs, &relay, tracker_extractor.get()};

    auto server = make_server(opts, config::RunMode::Run, bus, threed.bus3d.get(),
                              &flow);
    if (server) {
        if (outputs.slime_pub) server->set_native_publisher(outputs.slime_pub.get());
        if (outputs.vmt_pub)   server->set_vmt_publisher(outputs.vmt_pub.get());
        if (relay.beacon)      server->set_discovery_beacon(relay.beacon.get());
        if (threed.tracker_bus) server->set_tracker_bus(threed.tracker_bus.get());
        if (opts.hmd_listen_enabled) {
            server->set_hmd_pose_bus(relay.hmd_bus.get(), opts.hmd_stale_ms);
        }
        if (outputs.aligner) server->set_continuous_aligner(outputs.aligner.get());
        server->set_idle_state(&idle_state, idle_enabled,
                               opts.idle_enter_after_s, opts.idle_tick_hz);
        server->start();
    }

    // Whether the HMD return channel is actually live: --hmd-listen-enabled may
    // have been requested but the receiver socket failed to bind (relay.receiver
    // == nullptr). Use the real state, not the flag — otherwise we'd report VR
    // as observable (vr_observable=true) while never seeing a peer, and a
    // VMT/SlimeVR-out rig with a dead receiver would standby after enter_after_s
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
    idle_cfg.has_vr_output      = opts.vmt_out || opts.slimevr_out;
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
    return EXIT_SUCCESS;
}

}  // namespace fitra::app
