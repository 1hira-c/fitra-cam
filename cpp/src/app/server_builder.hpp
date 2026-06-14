#pragma once
//
// CrowServer construction with the mode label + static dirs resolved. The
// runner attaches its mode's sessions/publishers via the CrowServer setters
// and calls start() itself — the set of attachments IS the mode's web
// surface (unattached route groups are never registered → 404).

#include <memory>

#include "app/flow.hpp"
#include "config/main_config.hpp"
#include "pipeline/snapshot.hpp"
#include "web/crow_server.hpp"

namespace fitra::app {

// Returns nullptr when --no-web. bus3d may be null (2D-only / calib-extrinsic).
// When `flow` is daemon-managed, the server gets the /api/flow/switch handler
// (mode-switch requests stop the loop and surface as a flow exit code); the
// pointer must outlive the returned server.
std::unique_ptr<web::CrowServer> make_server(const config::MainOptions& opts,
                                             config::RunMode mode,
                                             pipeline::SnapshotBus& bus,
                                             pipeline::Skeleton3DBus* bus3d,
                                             FlowControl* flow);

}  // namespace fitra::app
