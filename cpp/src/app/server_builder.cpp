#include "app/server_builder.hpp"

#include "app/paths.hpp"

namespace fitra::app {

std::unique_ptr<web::CrowServer> make_server(const config::MainOptions& opts,
                                             config::RunMode mode,
                                             pipeline::SnapshotBus& bus,
                                             pipeline::Skeleton3DBus* bus3d) {
    if (opts.no_web) return nullptr;
    web::ServerOptions sopts;
    sopts.host = opts.host;
    sopts.port = opts.port;
    sopts.mode_label = config::run_mode_name(mode);
    sopts.static_dir = opts.static_dir.empty()
                        ? guess_static_dir().string()
                        : opts.static_dir;
    sopts.calib_static_dir = opts.calib_static_dir.empty()
                        ? guess_subject_calib_static_dir().string()
                        : opts.calib_static_dir;
    sopts.excal_static_dir = guess_extrinsic_calib_static_dir().string();
    return std::make_unique<web::CrowServer>(bus, bus3d, sopts);
}

}  // namespace fitra::app
