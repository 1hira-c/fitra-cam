#pragma once
//
// Mode-specific route groups: the subject-calib wizard (/subject-calib,
// /api/calib/*) and extrinsic calibration (/extrinsic-calib, /api/excal/*).
// CrowServer::start() registers a group only when its session is attached —
// in run mode neither group (static pages included) exists, so the paths fall
// through to the 404/405 of the static catchall
// (docs/design/pose-3d-calib-mode-separation.md). Split from crow_server.cpp
// so each mode's web surface is one greppable unit.
//
// Deps are captured by value at registration time; all of them are wired
// before start() and immutable afterwards, so this is equivalent to the old
// member reads at request time.

#include <functional>
#include <string>

#include <crow.h>

#include "pipeline/calibration_session.hpp"

namespace fitra::pipeline {
class ExtrinsicCalibSession;
class FloorCalibSession;
class IntrinsicCalibSession;
}
namespace fitra::vmt { class HmdPoseBus; class ControllerPoseBus; }
namespace fitra::config { class SetupConfigStore; }
namespace fitra::camera { class SetupCameraManager; }

namespace fitra::web::detail {

struct CalibRouteDeps {
    pipeline::CalibrationSession* session = nullptr;   // nullptr → no routes
    pipeline::CalibPreflight      defaults;
    std::string                   next_step;  // spliced into a successful approve
    std::string                   static_dir;
};
void register_calib_routes(crow::SimpleApp& app, const CalibRouteDeps& deps);

struct ExcalRouteDeps {
    pipeline::ExtrinsicCalibSession* session = nullptr;  // nullptr → no routes
    std::string             next_step;       // spliced into a successful solve
    vmt::HmdPoseBus*        hmd_bus = nullptr;
    double                  hmd_stale_ms = 200.0;
    vmt::ControllerPoseBus* controller_bus = nullptr;
    double                  controller_stale_ms = 200.0;
    std::string             controller_role;
    std::string             static_dir;
};
void register_excal_routes(crow::SimpleApp& app, const ExcalRouteDeps& deps);

// Floor-AprilTag calibration (案D). Reuses the same /extrinsic-calib page and
// /api/excal/{state,start,stop,solve} names as the controller path — the two
// run in separate processes (a flow-switch respawn), and the shared page
// branches on the state JSON's "method" field. There is no controller pose, so
// /api/excal/poses is not registered for the floor path.
struct FloorCalibRouteDeps {
    pipeline::FloorCalibSession* session = nullptr;  // nullptr → no routes
    std::string                  next_step;
    std::string                  static_dir;
};
void register_floor_calib_routes(crow::SimpleApp& app,
                                 const FloorCalibRouteDeps& deps);

// Intrinsic (ChArUco) calibration: /intrinsic-calib page + /api/incal/*.
struct IntrinsicCalibRouteDeps {
    pipeline::IntrinsicCalibSession* session = nullptr;  // nullptr → no routes
    std::string                      next_step;
    std::string                      static_dir;
};
void register_intrinsic_calib_routes(crow::SimpleApp& app,
                                     const IntrinsicCalibRouteDeps& deps);

// First-run setup module (docs/design/core-pipeline-setup-mode.md). Registered
// only in RunMode::Setup: camera enumeration + preview (when `cameras` is set,
// M3) and the editable config draft (when `store` is set). `on_proceed` writes
// the union config and advances the flow; it fills `next_mode` (a run-mode
// label) on success and `err` on failure.
struct SetupRouteDeps {
    config::SetupConfigStore*   store   = nullptr;   // nullptr → no config routes
    camera::SetupCameraManager* cameras = nullptr;   // nullptr → no camera routes
    std::function<bool(std::string& next_mode, std::string& err)> on_proceed;
    // Mode-aware validation for /api/config/validate. Falls back to
    // store->validate_draft (relaxed) when unset.
    std::function<bool(std::string& err)> on_validate;
};
void register_setup_mode_routes(crow::SimpleApp& app, const SetupRouteDeps& deps);

}  // namespace fitra::web::detail
