#include "app/mode_setup.hpp"

#include <chrono>
#include <cstdlib>
#include <thread>

#include "app/server_builder.hpp"
#include "pipeline/snapshot.hpp"
#include "util/logging.hpp"
#include "web/crow_server.hpp"

namespace fitra::app {

// First-run setup module. Builds only a Crow server — no FrameSource, no TRT
// stack, no 3D graph, no publishers — so it starts in well under a second and
// touches neither CUDA nor the GPU. The web surface (camera enumeration,
// config compose/save, /api/setup/proceed) is attached in M2/M3 via the
// CrowServer setup-route setter; here in M1 the module serves /api/state
// (mode="setup") and the /api/flow/switch route (when flow-managed), which is
// enough for the daemon to chain out of setup.
int run_mode_setup(const config::MainOptions& opts, FlowControl& flow) {
    // CrowServer needs a SnapshotBus reference; setup publishes no frames, so a
    // size-1 bus is constructed and left idle.
    pipeline::SnapshotBus bus{1};
    auto server = make_server(opts, config::RunMode::Setup, bus, nullptr, &flow);
    if (!server) {
        // --no-web in setup mode leaves nothing to drive the hand-off; treat as
        // a clean exit (the daemon stops rather than spinning).
        FITRA_LOG_WARN("setup: --no-web set — no web surface to configure from; exiting");
        return EXIT_SUCCESS;
    }

    server->start();
    FITRA_LOG_INFO("setup: serving WebUI on {}:{} — choose cameras, compose the "
                   "config, then proceed.", opts.host, opts.port);

    while (!flow.stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    server->stop();
    return EXIT_SUCCESS;
}

}  // namespace fitra::app
