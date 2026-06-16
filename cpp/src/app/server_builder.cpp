#include "app/server_builder.hpp"

#include "app/paths.hpp"

namespace fitra::app {

std::unique_ptr<web::CrowServer> make_server(const config::MainOptions& opts,
                                             config::RunMode mode,
                                             pipeline::SnapshotBus& bus,
                                             pipeline::Skeleton3DBus* bus3d,
                                             FlowControl* flow) {
    if (opts.no_web) return nullptr;
    web::ServerOptions sopts;
    sopts.host = opts.host;
    sopts.port = opts.port;
    sopts.mode_label = config::run_mode_name(mode);
    sopts.flow_managed = flow != nullptr && flow->managed;
    sopts.static_dir = opts.static_dir.empty()
                        ? guess_static_dir().string()
                        : opts.static_dir;
    sopts.calib_static_dir = opts.calib_static_dir.empty()
                        ? guess_subject_calib_static_dir().string()
                        : opts.calib_static_dir;
    sopts.excal_static_dir = guess_extrinsic_calib_static_dir().string();
    sopts.incal_static_dir = guess_intrinsic_calib_static_dir().string();
    auto server = std::make_unique<web::CrowServer>(bus, bus3d, sopts);
    if (sopts.flow_managed) {
        // Capture a copy of opts so the precheck sees the same config the
        // respawned child will load (the daemon passes --config to every child).
        server->set_flow_switch_handler(
            [flow, opts](const std::string& mode_name, std::string& err) {
                config::RunMode next;
                if (!config::parse_run_mode_name(mode_name, next)) {
                    err = "unknown mode: " + mode_name
                          + " (expected run|calib-subject|calib-extrinsic"
                            "|calib-extrinsic-floor|calib-intrinsic)";
                    return false;
                }
                // Validate the target mode's config BEFORE respawning, so a
                // misconfiguration (e.g. missing floor_map) surfaces in the UI
                // here instead of the child dying at validate and the daemon
                // silently falling back to run.
                if (!config::precheck_mode_switch(opts, next, err)) return false;
                flow->request_switch(next);
                return true;
            });
    }
    return server;
}

}  // namespace fitra::app
