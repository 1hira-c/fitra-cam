#include "app/mode_run.hpp"

#include <cstdio>
#include <cstdlib>
#include <memory>

#include "app/camera_builder.hpp"
#include "app/output_builder.hpp"
#include "app/pose_relay_builder.hpp"
#include "app/server_builder.hpp"
#include "app/stats_loop.hpp"
#include "app/threed_builder.hpp"
#include "app/trt_stack.hpp"

namespace fitra::app {

int run_mode_run(const config::MainOptions& opts, FlowControl& flow) {
    auto trt = make_trt_stack(opts);
    auto cams = make_frame_sources(opts, trt.get(), nullptr);
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
            make_tracker_extractor(opts, *threed.bus3d, *threed.tracker_bus);
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
        server->start();
    }

    run_stats_loop(*driver, opts.log_every_s, flow.stop);

    if (server) server->stop();
    outputs.stop();
    driver->stop();
    return EXIT_SUCCESS;
}

}  // namespace fitra::app
