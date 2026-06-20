#include "app/mode_setup.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <thread>

#include "app/daemon.hpp"          // initial_mode, profile_path
#include "app/server_builder.hpp"
#include "camera/setup_camera_manager.hpp"
#include "config/setup_config_store.hpp"
#include "pipeline/snapshot.hpp"
#include "util/logging.hpp"
#include "web/crow_server.hpp"

namespace fitra::app {

namespace {

// Decide the stage to hand off to once the config is composed, mirroring the
// daemon's auto initial-mode logic (artifact-driven), and write the union
// config + advance the flow. Returns false + a user-facing reason in `err`.
bool do_proceed(config::SetupConfigStore& store, FlowControl& flow,
                std::string& next_mode, std::string& err) {
    if (!store.validate_draft(err)) return false;
    config::MainOptions d = store.draft();
    if (d.cam_paths[0].empty()) {
        err = "configure at least cam0 before proceeding";
        return false;
    }
    std::error_code ec;
    const bool intrinsics_exists =
        !d.intrinsic_out.empty() && std::filesystem::exists(d.intrinsic_out, ec) && !ec;
    const bool extrinsics_exists =
        !d.calib.empty() && std::filesystem::exists(d.calib, ec) && !ec;
    bool profile_exists = true;
    {
        const std::string pp = profile_path(d);
        if (!pp.empty()) profile_exists = std::filesystem::exists(pp, ec) && !ec;
    }
    config::RunMode next =
        initial_mode(d, intrinsics_exists, extrinsics_exists, profile_exists);
    if (next == config::RunMode::Setup) {
        err = "configure cameras before proceeding";
        return false;
    }
    std::string perr;
    if (!config::precheck_mode_switch(d, next, perr)) {
        err = perr;
        return false;
    }
    if (!store.write_union(err)) return false;
    next_mode = config::run_mode_name(next);
    FITRA_LOG_INFO("setup: wrote {} — handing off to {}", store.union_path(), next_mode);
    if (flow.managed) flow.request_switch(next);
    else              flow.stop.store(true);
    return true;
}

}  // namespace

int run_mode_setup(const config::MainOptions& opts, const std::string& config_path,
                   FlowControl& flow) {
    // The store seeds from the loaded config and writes back to the same path
    // the daemon reloads on the next spawn.
    config::SetupConfigStore store{opts, config_path};

    pipeline::SnapshotBus bus{1};  // unused; CrowServer ctor needs one
    auto server = make_server(opts, config::RunMode::Setup, bus, nullptr, &flow);
    if (!server) {
        FITRA_LOG_WARN("setup: --no-web set — no web surface to configure from; exiting");
        return EXIT_SUCCESS;
    }
    camera::SetupCameraManager cameras;
    server->set_setup_handlers(
        &store,
        [&store, &flow](std::string& next_mode, std::string& err) {
            return do_proceed(store, flow, next_mode, err);
        });
    server->set_setup_camera_manager(&cameras);

    server->start();
    FITRA_LOG_INFO("setup: serving WebUI on {}:{} — choose cameras, compose the "
                   "config, then proceed.", opts.host, opts.port);
    if (config_path.empty()) {
        FITRA_LOG_WARN("setup: launched without --config — proceed cannot persist "
                       "the composed config (start the daemon with --config)");
    }

    while (!flow.stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    server->stop();
    return EXIT_SUCCESS;
}

}  // namespace fitra::app
