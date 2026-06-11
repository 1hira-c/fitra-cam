#include "app/mode_calib_extrinsic.hpp"

#include <cctype>
#include <charconv>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "app/camera_builder.hpp"
#include "app/excal_live_input.hpp"
#include "app/excal_runner.hpp"
#include "app/pose_relay_builder.hpp"
#include "app/server_builder.hpp"
#include "lift/calib_io.hpp"
#include "pipeline/excal_replay_input.hpp"
#include "pipeline/extrinsic_calib_session.hpp"
#include "util/logging.hpp"

namespace fitra::app {

namespace {

// Split a comma-separated list, trimming surrounding ASCII whitespace from
// each token. Used to parse --excal-faces "0, 1, 2".
std::vector<std::string> split_csv(const std::string& s) {
    std::vector<std::string> out;
    std::size_t start = 0;
    while (start <= s.size()) {
        std::size_t comma = s.find(',', start);
        std::size_t end = (comma == std::string::npos) ? s.size() : comma;
        std::size_t b = start, e = end;
        while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
        while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
        out.push_back(s.substr(b, e - b));
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return out;
}

// Builds the session from opts (intrinsics load + face-id parse + gate
// thresholds). Shared by the live and replay paths; returns nullptr with a
// message on stderr when the configuration is unusable.
std::unique_ptr<pipeline::ExtrinsicCalibSession>
build_excal_session(const config::MainOptions& opts, std::size_t n_cams) {
    const std::string intr_path = opts.excal_intrinsics.empty()
                                  ? opts.calib : opts.excal_intrinsics;
    FITRA_LOG_INFO("extrinsic-calib: loading intrinsics from {}", intr_path);
    pipeline::ExtrinsicCalibConfig ec;
    ec.intrinsics = lift::load_calibration(intr_path);
    if (ec.intrinsics.cameras.size() < n_cams) {
        std::fprintf(stderr,
            "extrinsic-calib: intrinsics file has %zu cameras, need >= %zu\n",
            ec.intrinsics.cameras.size(), n_cams);
        return nullptr;
    }
    // Parse "0,1,2" face ids; uniform tag size for the skeleton. Parse
    // strictly: a non-numeric token (typo, stray char) must fail loudly
    // rather than std::atoi-fold to face 0 and silently miscalibrate.
    for (const auto& tok : split_csv(opts.excal_faces)) {
        if (tok.empty()) continue;
        int face_id = 0;
        auto [ptr, ec_parse] =
            std::from_chars(tok.data(), tok.data() + tok.size(), face_id);
        if (ec_parse != std::errc{} || ptr != tok.data() + tok.size() ||
            face_id < 0) {
            std::fprintf(stderr,
                "extrinsic-calib: invalid --excal-faces token '%s' "
                "(expected non-negative integers, e.g. \"0,1,2\")\n",
                tok.c_str());
            return nullptr;
        }
        lift::MarkerFace f;
        f.face_id = face_id;
        f.tag_size_m = opts.excal_tag_size_m;
        ec.board.faces.push_back(f);
    }
    ec.lin_vel_max_mps        = opts.excal_lin_vel_max;
    ec.ang_vel_max_dps        = opts.excal_ang_vel_max;
    ec.burst_min              = opts.excal_burst_min;
    ec.min_samples_per_group  = opts.excal_min_samples;
    ec.out_path               = opts.excal_out;
    return std::make_unique<pipeline::ExtrinsicCalibSession>(std::move(ec));
}

// Unattended replay: a tools/excal_record session in, an extrinsics YAML
// out. No cameras, no SteamVR, no web server — collect→solve runs to
// completion and the exit code is the result.
int run_excal_replay(const config::MainOptions& opts, std::atomic<bool>& stop) {
    pipeline::ExcalReplayInput input{opts.excal_replay};
    const std::size_t n_cams = input.camera_count();
    if (input.size() == 0) {
        std::fprintf(stderr, "excal-replay: %s has no frames\n",
                     opts.excal_replay.c_str());
        return EXIT_FAILURE;
    }
    FITRA_LOG_INFO("excal-replay: {} frames across {} cameras from {}",
                   input.size(), n_cams, opts.excal_replay);

    auto session = build_excal_session(opts, n_cams);
    if (!session) return EXIT_FAILURE;

    session->start();
    run_excal_loop(input, *session, stop);

    FITRA_LOG_INFO("excal-replay: collected {} samples; solving...",
                   session->sample_count());
    std::string err;
    if (!session->solve_and_write(err)) {
        FITRA_LOG_ERROR("excal-replay: solve/write failed: {}", err);
        return EXIT_FAILURE;
    }
    FITRA_LOG_INFO("excal-replay: wrote extrinsics to {}", opts.excal_out);
    return EXIT_SUCCESS;
}

}  // namespace

int run_mode_calib_extrinsic(const config::MainOptions& opts,
                             std::atomic<bool>& stop) {
    if (!opts.excal_replay.empty()) {
        return run_excal_replay(opts, stop);
    }

    // Decode-only FrameSources (no YOLOX, no RTMPose prebake): the capture
    // loop only needs CPU BGR frames for AprilTag detection.
    auto cams = make_frame_sources(opts, nullptr, nullptr);
    const std::size_t n_cams = cams.sources.size();

    auto excal_session = build_excal_session(opts, n_cams);
    if (!excal_session) return EXIT_FAILURE;

    // Unified VMT pose relay receiver: the selected controller is the
    // calibration input; the HMD feeds the /extrinsic-calib scene.
    auto relay = make_pose_relay(opts, /*listen=*/true);

    const std::string guidance =
        "extrinsics written to " + opts.excal_out
        + ". Next: restart in subject-calib mode — ./main --calibrate"
          " --enable-3d --calib " + opts.excal_out
        + " --calib-subject-id <ID> --calib-subject-height-m <H> ...";

    // Auto-exit on solve: the YAML is the mode's whole output, so once it is
    // written there is nothing left to run. Fires from the Crow worker (or
    // the shutdown fallback below); the capture loop notices `stop` promptly,
    // after the solve response has been written.
    excal_session->set_on_solved([&guidance, &stop]() {
        FITRA_LOG_INFO("extrinsic-calib: solved. {}", guidance);
        stop.store(true);
    });

    // Inert snapshot bus: no driver feeds it; /stats serves an empty bundle.
    pipeline::SnapshotBus bus{n_cams};
    auto server = make_server(opts, config::RunMode::CalibExtrinsic, bus, nullptr);
    if (server) {
        server->set_extrinsic_calib_session(excal_session.get());
        server->set_extrinsic_calib_next_step(guidance);
        server->set_hmd_pose_bus(relay.hmd_bus.get(), opts.hmd_stale_ms);
        server->set_extrinsic_calib_pose_bus(
            relay.controller_bus.get(),
            vmt::tracked_pose_role_name(relay.controller_role),
            opts.excal_controller_stale_ms);
        server->start();
    }

    excal_session->start();
    FITRA_LOG_INFO("extrinsic-calib: collecting (faces={}, controller_role={}, "
                   "out={}). Solve from the web UI, or stop the process to "
                   "solve + write.",
                   opts.excal_faces, opts.excal_controller_role, opts.excal_out);

    ExcalLiveInput input{std::move(cams.sources), *relay.controller_bus,
                         opts.excal_controller_stale_ms};
    input.start();
    run_excal_loop(input, *excal_session, stop);
    input.stop();

    if (server) server->stop();
    relay.stop();

    if (excal_session->state() == pipeline::ExtrinsicCalibState::kSolved) {
        FITRA_LOG_INFO("extrinsic-calib: already solved; keeping {}", opts.excal_out);
        return EXIT_SUCCESS;
    }
    FITRA_LOG_INFO("extrinsic-calib: collected {} samples; solving...",
                   excal_session->sample_count());
    std::string err;
    if (!excal_session->solve_and_write(err)) {
        FITRA_LOG_ERROR("extrinsic-calib: solve/write failed: {}", err);
        return EXIT_FAILURE;
    }
    FITRA_LOG_INFO("extrinsic-calib: wrote extrinsics to {}", opts.excal_out);
    return EXIT_SUCCESS;
}

}  // namespace fitra::app
