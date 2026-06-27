#include "app/mode_setup.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <thread>

#include "app/daemon.hpp"          // initial_mode, profile_path
#include "app/server_builder.hpp"
#include "camera/setup_camera_manager.hpp"
#include "lift/calib_io.hpp"        // clear_calib_latest
#include "lift/subject_profile.hpp" // subject_profile_compatible
#include "config/setup_config_store.hpp"
#include "pipeline/snapshot.hpp"
#include "util/logging.hpp"
#include "web/crow_server.hpp"

namespace fitra::app {

namespace {

// Subject-calib defaults seeded when routing into CalibSubject with none
// configured. Match the SubjectCalibPage defaults so the page reflects the same
// id the daemon's boot auto-start uses.
constexpr const char* kDefaultSubjectId = "subject01";
constexpr double      kDefaultSubjectHeightM = 1.70;

// Auto initial-mode for the composed draft, mirroring the daemon's
// artifact-driven picker.
config::RunMode derive_next_mode(const config::MainOptions& d) {
    std::error_code ec;
    // Resolved read paths: a generated artifact that is absent routes to its
    // calibration stage (see effective_*_path / pose-3d-calib-latest-resolution).
    const bool intrinsics_exists =
        !d.intrinsic_out.empty() && std::filesystem::exists(d.intrinsic_out, ec) && !ec;
    const std::string extr = config::effective_extrinsics_path(d);
    const bool extrinsics_exists =
        !extr.empty() && std::filesystem::exists(extr, ec) && !ec;
    const std::string pp = profile_path(d);
    // An empty path means no subject id is configured yet (the Setup wizard never
    // sets one). On a rig that already has intrinsics + extrinsics that means
    // subject calibration has not run — treat the profile as absent so we route
    // into CalibSubject rather than skipping it (#2). Also treat an existing but
    // schema-incompatible profile (wrong keypoint topology) as absent so it gets
    // recalibrated instead of crashing run (subject_profile_compatible).
    const bool profile_exists = !pp.empty() && lift::subject_profile_compatible(pp);
    return initial_mode(d, intrinsics_exists, extrinsics_exists, profile_exists);
}

// Persist the live draft and hand off to `next`, gating against the *live* draft
// rather than a stale bootstrap snapshot (#5). Shared by the auto-proceed path
// and the explicit WizardSteps switch.
bool compose_and_switch(config::SetupConfigStore& store, FlowControl& flow,
                        config::RunMode next, std::string& next_mode,
                        std::string& err) {
    // Range/enum sanity on the live draft. do_proceed used to run this on its
    // own, but an explicit WizardSteps jump (do_switch) reaches here too — so
    // gate both paths here, else a draft with an out-of-range value (bad port,
    // det_score, ...) would be written and only rejected when the spawned child
    // hits validate_options, dropping the daemon back to run.
    if (!store.validate_draft(err)) return false;
    config::MainOptions d = store.draft();
    // Leaving setup commits to the whole auto-chain: the spawned children re-read
    // this union config and the browser can no longer edit it. So when entering
    // ANY stage other than setup — even an intermediate calib stage — require the
    // cameras + inference engines the terminal stages (run, subject-calib) need,
    // and seed a subject id/height. Without the engines a later run/subject child
    // dies on missing --det-engine/--pose-engine with no way to fix it from the
    // UI; without a subject id the extrinsic stage keys on an empty subject_id
    // and routes straight to run, silently skipping subject calibration
    // (mode_calib_extrinsic.cpp's has_subject_stage). The relaxed validate_draft
    // alone would let such a half-config through.
    if (next != config::RunMode::Setup) {
        if (d.cam_paths[0].empty()) {
            err = "configure at least cam0 before proceeding";
            return false;
        }
        if (d.det_engine.empty() || d.pose_engine.empty()) {
            err = "set --det-engine and --pose-engine before proceeding "
                  "(subject calibration and run require them)";
            return false;
        }
        // Seed the canonical subject.* fields (the loader bridges them to
        // calib_subject_* when the child re-reads the config).
        bool seeded = false;
        if (d.subject_id.empty()) {
            d.subject_id = kDefaultSubjectId;
            seeded = true;
        }
        if (d.subject_height_m <= 0.0) {
            d.subject_height_m = kDefaultSubjectHeightM;
            seeded = true;
        }
        if (seeded) store.set_draft(d);
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

// Auto-proceed: derive the next stage from artifacts and hand off.
bool do_proceed(config::SetupConfigStore& store, FlowControl& flow,
                std::string& next_mode, std::string& err) {
    const config::MainOptions d = store.draft();
    const config::RunMode next = derive_next_mode(d);
    if (next == config::RunMode::Setup) {
        err = "configure cameras before proceeding";
        return false;
    }
    return compose_and_switch(store, flow, next, next_mode, err);
}

// Explicit WizardSteps switch: route the requested mode through the same
// compose + gate + write path as proceed, using the live draft (#5).
bool do_switch(config::SetupConfigStore& store, FlowControl& flow,
               const std::string& mode_name, std::string& err) {
    config::RunMode next;
    if (!config::parse_run_mode_name(mode_name, next)) {
        err = "unknown mode: " + mode_name;
        return false;
    }
    // The wizard's single "extrinsic" step is method-agnostic; honor the
    // configured extrinsic_calib.method so a floor rig lands in the floor variant
    // (matching the auto-chain's initial_mode pick) instead of spawning the
    // controller stage, which would fail its precheck on a floor config.
    if (next == config::RunMode::CalibExtrinsic ||
        next == config::RunMode::CalibExtrinsicFloor) {
        next = store.draft().excal_method == "floor"
                   ? config::RunMode::CalibExtrinsicFloor
                   : config::RunMode::CalibExtrinsic;
    }
    std::string next_mode;
    return compose_and_switch(store, flow, next, next_mode, err);
}

// Setup "validate" button: the relaxed range/enum pass, plus the cam0+engines
// requirement that validate_draft's daemon relaxation skips. A draft with
// cameras assigned but blank engines must not report a false OK (#4), since both
// run and subject calibration need them.
bool do_validate(config::SetupConfigStore& store, std::string& err) {
    if (!store.validate_draft(err)) return false;
    const config::MainOptions d = store.draft();
    if (!d.cam_paths[0].empty() && (d.det_engine.empty() || d.pose_engine.empty())) {
        err = "set --det-engine and --pose-engine (required to run inference "
              "and for subject calibration)";
        return false;
    }
    return true;
}

}  // namespace

int run_mode_setup(const config::MainOptions& opts, const std::string& config_path,
                   FlowControl& flow) {
    // The store seeds from the loaded config and writes back to the same path
    // the daemon reloads on the next spawn.
    config::SetupConfigStore store{opts, config_path};

    // Entering setup means "reconfigure from scratch": drop the calibration
    // `latest` pointers so the chain regenerates them (the timestamped history is
    // kept). No-op for explicit (non-latest.yaml) paths and for a fresh rig that
    // has no latest yet (docs/design/pose-3d-calib-latest-resolution.md).
    // Include floor_out: it defaults to the same latest.yaml as excal_out (the
    // second clear is then a harmless no-op), but a rig that pins a distinct
    // floor write target would otherwise keep a stale floor `latest` pointer.
    for (const auto& p : {opts.intrinsic_out, opts.excal_out, opts.floor_out}) {
        if (lift::clear_calib_latest(p)) {
            FITRA_LOG_INFO("setup: cleared calibration latest pointer {}", p);
        }
    }

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
        },
        [&store](std::string& err) { return do_validate(store, err); });
    server->set_setup_camera_manager(&cameras);
    // Override make_server's default flow-switch handler (which prechecks a stale
    // bootstrap snapshot) with one that composes + writes the live draft before
    // switching, so a WizardSteps jump uses the edited config (#5).
    if (flow.managed) {
        server->set_flow_switch_handler(
            [&store, &flow](const std::string& mode_name, std::string& err) {
                return do_switch(store, flow, mode_name, err);
            });
    }

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
