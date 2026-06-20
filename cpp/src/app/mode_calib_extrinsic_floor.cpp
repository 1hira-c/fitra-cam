#include "app/mode_calib_extrinsic_floor.hpp"

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>

#include "app/camera_builder.hpp"
#include "app/floor_calib_runner.hpp"
#include "app/floor_live_input.hpp"
#include "app/server_builder.hpp"
#include "lift/calib_io.hpp"
#include "lift/floor_tag_map.hpp"
#include "pipeline/excal_replay_input.hpp"
#include "pipeline/floor_calib_session.hpp"
#include "pipeline/snapshot.hpp"
#include "util/logging.hpp"
#include "web/crow_server.hpp"

namespace fitra::app {

namespace {

// Build the floor session from opts: PnP intrinsics + optional runtime
// intrinsics + the known tag map. Returns nullptr with a message on stderr.
std::unique_ptr<pipeline::FloorCalibSession>
build_floor_session(const config::MainOptions& opts, std::size_t n_cams) {
    const std::string intr_path =
        opts.floor_intrinsics.empty() ? opts.calib : opts.floor_intrinsics;
    FITRA_LOG_INFO("floor-calib: loading PnP intrinsics from {}", intr_path);

    pipeline::FloorCalibConfig fc;
    try {
        fc.intrinsics = lift::load_calibration(intr_path);
        if (!opts.floor_out_intrinsics.empty()) {
            fc.out_intrinsics = lift::load_calibration(opts.floor_out_intrinsics);
        }
        fc.map = lift::floor_tag_map_load(opts.floor_map);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "floor-calib: %s\n", e.what());
        return nullptr;
    }
    if (fc.intrinsics.cameras.size() < n_cams) {
        std::fprintf(stderr,
            "floor-calib: intrinsics file has %zu cameras, need >= %zu\n",
            fc.intrinsics.cameras.size(), n_cams);
        return nullptr;
    }
    fc.fisheye           = opts.floor_fisheye;
    fc.num_cams          = n_cams;  // actual capture count, not the file's camera count
    fc.burst_min         = opts.floor_burst_min;
    fc.max_pnp_reproj_px = opts.floor_max_reproj_px;
    fc.solver.max_reproj_px = opts.floor_max_reproj_px;
    fc.out_path          = opts.floor_out;
    FITRA_LOG_INFO("floor-calib: map has {} tags; output → {}",
                   fc.map.tags.size(), fc.out_path);
    return std::make_unique<pipeline::FloorCalibSession>(std::move(fc));
}

// Human + machine readable reprojection report (stdout).
void report_solution(const pipeline::FloorCalibSession& session) {
    const auto& sol = session.last_solution();
    std::ostringstream os;
    os << "{\"ok\":" << (sol.ok ? "true" : "false") << ",\"cameras\":[";
    for (std::size_t i = 0; i < sol.cameras.size(); ++i) {
        const auto& c = sol.cameras[i];
        if (i) os << ",";
        os << "{\"cam\":" << c.cam_index
           << ",\"n_tags\":" << c.n_tags
           << ",\"reproj_rms_px\":" << c.reproj_rms_px
           << ",\"planar_degenerate\":" << (c.planar_degenerate ? "true" : "false")
           << ",\"plane_thickness_m\":" << c.plane_thickness_m
           << ",\"solved\":" << (c.solved ? "true" : "false") << "}";
    }
    os << "]}";
    std::printf("floor-calib report: %s\n", os.str().c_str());
    for (const auto& c : sol.cameras) {
        FITRA_LOG_INFO("floor-calib: cam{} tags={} reproj={:.3f}px planar={}{}",
                       c.cam_index, c.n_tags, c.reproj_rms_px,
                       c.planar_degenerate ? "yes" : "no",
                       c.solved ? "" : " UNSOLVED");
    }
}

// Unattended replay: a tools/excal_record session in, an extrinsics YAML out.
int run_floor_replay(const config::MainOptions& opts, FlowControl& flow) {
    pipeline::ExcalReplayInput input{opts.floor_replay};
    const std::size_t n_cams = input.camera_count();
    if (input.size() == 0) {
        std::fprintf(stderr, "floor-replay: %s has no frames\n",
                     opts.floor_replay.c_str());
        return EXIT_FAILURE;
    }
    FITRA_LOG_INFO("floor-replay: {} frames across {} cameras from {}",
                   input.size(), n_cams, opts.floor_replay);

    auto session = build_floor_session(opts, n_cams);
    if (!session) return EXIT_FAILURE;

    session->start();
    run_floor_calib_loop(input, *session, flow.stop);

    FITRA_LOG_INFO("floor-replay: {} ready groups; solving...",
                   session->ready_group_count());
    std::string err;
    bool ok = session->solve_and_write(err);
    report_solution(*session);
    if (!ok) {
        FITRA_LOG_ERROR("floor-replay: solve/write failed: {}", err);
        return EXIT_FAILURE;
    }
    FITRA_LOG_INFO("floor-replay: wrote extrinsics to {}", opts.floor_out);
    return EXIT_SUCCESS;
}

}  // namespace

int run_mode_calib_extrinsic_floor(const config::MainOptions& opts,
                                   FlowControl& flow) {
    if (!opts.floor_replay.empty()) {
        return run_floor_replay(opts, flow);
    }

    // Decode-only FrameSources (AprilTag detection on CPU; no TRT).
    auto cams = make_frame_sources(opts, nullptr, nullptr);
    const std::size_t n_cams = cams.sources.size();

    auto session = build_floor_session(opts, n_cams);
    if (!session) return EXIT_FAILURE;

    const bool has_subject_stage = !opts.subject_id.empty();
    const config::RunMode next_after_solve =
        has_subject_stage ? config::RunMode::CalibSubject : config::RunMode::Run;
    const std::string guidance = flow.managed
        ? (has_subject_stage
              ? "extrinsics written to " + opts.floor_out
                + ". Flow daemon switches to subject-calib mode."
              : "extrinsics written to " + opts.floor_out
                + ". Flow daemon switches to run mode.")
        : "extrinsics written to " + opts.floor_out + ". Restart in the next mode.";
    session->set_on_solved([&flow, next_after_solve, guidance]() {
        FITRA_LOG_INFO("floor-calib: solved. {}", guidance);
        if (flow.managed) flow.request_switch(next_after_solve);
        else              flow.stop.store(true);
    });

    // Crow server for web start/solve (the /extrinsic-calib page, floor branch).
    pipeline::SnapshotBus bus{n_cams};
    auto server = make_server(opts, config::RunMode::CalibExtrinsicFloor, bus,
                              nullptr, &flow);
    if (server) {
        server->set_floor_calib_session(session.get());
        server->set_floor_calib_next_step(guidance);
        server->start();
    }

    session->start();
    FITRA_LOG_INFO("floor-calib: collecting (map={}, out={}). Solve from the web "
                   "UI, or stop the process to solve + write.",
                   opts.floor_map, opts.floor_out);

    FloorLiveInput input{std::move(cams.sources)};
    input.start();
    run_floor_calib_loop(input, *session, flow.stop);
    input.stop();

    if (server) server->stop();

    if (session->state() == pipeline::FloorCalibState::kSolved) {
        FITRA_LOG_INFO("floor-calib: already solved; keeping {}", opts.floor_out);
        return EXIT_SUCCESS;
    }
    FITRA_LOG_INFO("floor-calib: {} ready groups; solving...",
                   session->ready_group_count());
    std::string err;
    bool ok = session->solve_and_write(err);
    report_solution(*session);
    if (!ok) {
        FITRA_LOG_ERROR("floor-calib: solve/write failed: {}", err);
        return EXIT_FAILURE;
    }
    FITRA_LOG_INFO("floor-calib: wrote extrinsics to {}", opts.floor_out);
    return EXIT_SUCCESS;
}

}  // namespace fitra::app
