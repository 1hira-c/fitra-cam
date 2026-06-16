#include "app/mode_calib_intrinsic.hpp"

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include "app/camera_builder.hpp"
#include "app/floor_live_input.hpp"
#include "app/intrinsic_calib_runner.hpp"
#include "app/server_builder.hpp"
#include "pipeline/excal_replay_input.hpp"
#include "pipeline/intrinsic_calib_session.hpp"
#include "pipeline/snapshot.hpp"
#include "util/logging.hpp"
#include "web/crow_server.hpp"

namespace fitra::app {

namespace {

std::unique_ptr<pipeline::IntrinsicCalibSession>
build_intrinsic_session(const config::MainOptions& opts, std::size_t n_cams) {
    pipeline::IntrinsicCalibConfig ic;
    ic.board.squares_x    = opts.charuco_squares_x;
    ic.board.squares_y    = opts.charuco_squares_y;
    ic.board.square_len_m = opts.charuco_square_len_m;
    ic.board.marker_len_m = opts.charuco_marker_len_m;
    ic.board.dictionary   = opts.charuco_dict;
    ic.distortion_model   = opts.intrinsic_model;
    ic.num_cams           = n_cams;
    ic.min_views          = opts.intrinsic_min_views;
    ic.min_corners        = opts.intrinsic_min_corners;
    ic.out_path           = opts.intrinsic_out;
    try {
        return std::make_unique<pipeline::IntrinsicCalibSession>(std::move(ic));
    } catch (const std::exception& e) {
        std::fprintf(stderr, "intrinsic-calib: %s\n", e.what());
        return nullptr;
    }
}

int run_intrinsic_replay(const config::MainOptions& opts, FlowControl& flow) {
    pipeline::ExcalReplayInput input{opts.intrinsic_replay};
    const std::size_t n_cams = input.camera_count();
    if (input.size() == 0) {
        std::fprintf(stderr, "intrinsic-replay: %s has no frames\n",
                     opts.intrinsic_replay.c_str());
        return EXIT_FAILURE;
    }
    FITRA_LOG_INFO("intrinsic-replay: {} frames across {} cameras ({} model)",
                   input.size(), n_cams, opts.intrinsic_model);

    auto session = build_intrinsic_session(opts, n_cams);
    if (!session) return EXIT_FAILURE;

    session->start();
    run_intrinsic_calib_loop(input, *session, flow.stop);

    std::string err;
    bool ok = session->solve_and_write(err);
    std::printf("intrinsic-calib report: %s\n", session->state_json().c_str());
    if (!ok) {
        FITRA_LOG_ERROR("intrinsic-replay: solve/write failed: {}", err);
        return EXIT_FAILURE;
    }
    FITRA_LOG_INFO("intrinsic-replay: wrote intrinsics to {}", opts.intrinsic_out);
    return EXIT_SUCCESS;
}

}  // namespace

int run_mode_calib_intrinsic(const config::MainOptions& opts, FlowControl& flow) {
    if (!opts.intrinsic_replay.empty()) {
        return run_intrinsic_replay(opts, flow);
    }

    auto cams = make_frame_sources(opts, nullptr, nullptr);
    const std::size_t n_cams = cams.sources.size();

    auto session = build_intrinsic_session(opts, n_cams);
    if (!session) return EXIT_FAILURE;

    // After intrinsics: the setup chain advances to extrinsic calibration. The
    // configured method (extrinsic_calib.method) picks controller vs floor.
    const config::RunMode next_after_solve =
        opts.floor_calib_enabled ? config::RunMode::CalibExtrinsicFloor
                                 : config::RunMode::CalibExtrinsic;
    const std::string guidance = flow.managed
        ? "intrinsics written to " + opts.intrinsic_out
          + ". Flow daemon switches to extrinsic calibration."
        : "intrinsics written to " + opts.intrinsic_out + ". Restart in the next mode.";
    session->set_on_solved([&flow, next_after_solve, guidance]() {
        FITRA_LOG_INFO("intrinsic-calib: solved. {}", guidance);
        if (flow.managed) flow.request_switch(next_after_solve);
        else              flow.stop.store(true);
    });

    // Crow server for web start/solve (the /intrinsic-calib page).
    pipeline::SnapshotBus bus{n_cams};
    auto server = make_server(opts, config::RunMode::CalibIntrinsic, bus,
                              nullptr, &flow);
    if (server) {
        server->set_intrinsic_calib_session(session.get());
        server->set_intrinsic_calib_next_step(guidance);
        server->start();
    }

    session->start();
    FITRA_LOG_INFO("intrinsic-calib: collecting ({} model, board {}x{}, out={}). "
                   "Show the ChArUco board to each camera; stop to solve + write.",
                   opts.intrinsic_model, opts.charuco_squares_x,
                   opts.charuco_squares_y, opts.intrinsic_out);

    FloorLiveInput input{std::move(cams.sources)};
    input.start();
    run_intrinsic_calib_loop(input, *session, flow.stop);
    input.stop();
    if (server) server->stop();

    if (session->state() == pipeline::IntrinsicCalibState::kSolved) {
        return EXIT_SUCCESS;
    }
    std::string err;
    bool ok = session->solve_and_write(err);
    std::printf("intrinsic-calib report: %s\n", session->state_json().c_str());
    if (!ok) {
        FITRA_LOG_ERROR("intrinsic-calib: solve/write failed: {}", err);
        return EXIT_FAILURE;
    }
    FITRA_LOG_INFO("intrinsic-calib: wrote intrinsics to {}", opts.intrinsic_out);
    return EXIT_SUCCESS;
}

}  // namespace fitra::app
