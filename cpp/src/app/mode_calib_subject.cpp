#include "app/mode_calib_subject.hpp"

#include <cstdio>
#include <cstdlib>
#include <memory>

#include "app/camera_builder.hpp"
#include "app/output_builder.hpp"
#include "app/paths.hpp"
#include "app/server_builder.hpp"
#include "app/stats_loop.hpp"
#include "app/threed_builder.hpp"
#include "app/trt_stack.hpp"
#include "pipeline/calibration_session.hpp"
#include "util/logging.hpp"

namespace fitra::app {

int run_mode_calib_subject(const config::MainOptions& opts_in, FlowControl& flow) {
    // Subject calibration's recording session + dump_keypoints_3d are two-camera
    // (cam0,cam1) by design. On a rig with more cameras, calibrate from the first
    // two — a 2-view bone-length profile is valid for an N-view run — rather than
    // refusing. Trim a local copy so everything downstream builds exactly two
    // sources; the extra cameras' calib entries are simply left unused.
    config::MainOptions opts = opts_in;
    std::size_t configured = 0;
    for (const auto& path : opts.cam_paths) {
        if (!path.empty()) ++configured;
    }
    if (configured < 2) {
        std::fprintf(stderr,
            "subject calibration needs at least 2 cameras (got %zu)\n", configured);
        return EXIT_FAILURE;
    }
    if (configured > 2) {
        FITRA_LOG_WARN("calibrate: rig has {} cameras; subject calibration uses "
                       "cam0+cam1 only (a 2-view bone-length profile is valid for "
                       "the N-view run)", configured);
        for (std::size_t i = 2; i < opts.cam_paths.size(); ++i)
            opts.cam_paths[i].clear();
    }

    // For the headless --calibrate path, prime the live IkSolver with the
    // calibration height up-front. Without this, the IK is unlocked at boot
    // and the 3D angle recognizer would have to wait for ~150 frames of
    // observational locking before it can judge pose holds.
    // subject_id/subject_height_m are the single source of truth (the old
    // calib_subject_height_m fallback collapsed into subject_height_m).
    const double subject_height_m = opts.subject_height_m;

    // Shared "calibration is recording" flag. When true:
    //   - FrameSource skips YOLOX + RTMPose pre-bake and retains BGR
    //   - CalibrationSession collects raw frames into the per-pose buffer
    // Wired from CalibrationSession::set_on_recording_active below.
    auto calib_recording_flag = std::make_shared<std::atomic<bool>>(false);

    auto trt = make_trt_stack(opts);
    auto cams = make_frame_sources(opts, trt.get(), calib_recording_flag);
    // After trimming to cam0+cam1 above, we expect two sources; fewer means a
    // camera failed to open (the two-camera recording session can't proceed).
    const std::size_t n_cams = cams.sources.size();
    if (n_cams != 2) {
        std::fprintf(stderr,
            "subject calibration needs 2 working cameras (opened %zu)\n", n_cams);
        return EXIT_FAILURE;
    }

    // 3D is mandatory here (validate_options enforces --enable-3d +
    // --calib); make_threed throws if the extrinsics YAML is unreadable.
    ThreeDSet threed = make_threed(opts, n_cams, subject_height_m);

    pipeline::SnapshotBus bus{n_cams};
    auto driver = make_driver(std::move(cams.sources), *trt->rtmpose, bus, opts,
                              &threed);
    driver->start();

    pipeline::CalibrationSession calib_session;
    pipeline::CalibPreflight calib_defaults;
    calib_session.set_fps_hint(static_cast<double>(opts.fps));
    calib_session.set_auto_approve(opts.calib_auto_approve);
    calib_session.set_auto_exit(opts.calib_auto_exit);
    calib_session.set_log([](const std::string& l) {
        std::fprintf(stderr, "[calib] %s\n", l.c_str());
    });
    // No in-process reinjection: the approved profile is a YAML artifact
    // consumed by the next run-mode boot (--subject-id).
    calib_session.set_on_approved([&opts, &flow](const lift::SubjectProfile& p) {
        if (flow.managed) {
            FITRA_LOG_INFO(
                "profile approved (id={}). Flow daemon switches to run mode.",
                p.subject_id);
        } else {
            FITRA_LOG_INFO(
                "profile approved (id={}). Restart in run mode to use it: "
                "./main --enable-3d --calib {} --subject-id {} ...",
                p.subject_id, opts.calib, p.subject_id);
        }
    });
    // Prime the live IK with the subject's height the moment preflight
    // succeeds, so the 3D angle recognizer has a sensible bone-length lock
    // from the very first frame of capture.
    calib_session.set_on_preflight(
        [&driver](const pipeline::CalibPreflight& p) {
            FITRA_LOG_INFO("priming IK with calibration height: {} m",
                           p.subject_height_m);
            driver->ik().apply_subject_height(p.subject_height_m);
        });
    calib_session.set_on_recording_active(
        [calib_recording_flag](bool active) {
            calib_recording_flag->store(active, std::memory_order_relaxed);
        });
    // --calib-auto-exit path. Under the flow daemon this is the auto-chain:
    // the exit request becomes a "switch to run" so the daemon respawns into
    // tracker output with the just-approved profile.
    calib_session.set_on_exit_requested([&flow]() {
        if (flow.managed) flow.request_switch(config::RunMode::Run);
        else              flow.stop.store(true);
    });

    calib_defaults.subject_id = "";
    calib_defaults.subjects_dir = opts.subjects_dir;
    // Resolve like every other read site: three_d.calib is empty by default and
    // resolves to the extrinsic write target (= calibrations/extrinsics/latest.yaml)
    // — passing the raw (empty) opts.calib made the preflight report
    // "calibration YAML not found:" (pose-3d-calib-latest-resolution.md).
    calib_defaults.calib_yaml   = config::effective_extrinsics_path(opts);
    calib_defaults.det_engine   = opts.det_engine;
    calib_defaults.pose_engine  = opts.pose_engine;
    calib_defaults.recording_frames_per_cam = opts.calib_frames_per_cam;
    calib_defaults.required_hold_sec        = opts.calib_hold_sec;
    calib_defaults.dump_tool_path = opts.calib_dump_tool.empty()
                                    ? guess_dump_tool_path().string()
                                    : opts.calib_dump_tool;

    driver->set_skeleton3d_tap(
        [s = &calib_session](const infer::Skeleton3D& skel, double drift) {
            s->on_skeleton3d(skel, drift);
        });
    driver->set_frame_tap(
        [s = &calib_session]
        (std::size_t cam, const cv::Mat& bgr, double ts) {
            s->on_frame(cam, bgr, ts);
        });

    // Stop the driver worker before the session it taps into goes out of
    // scope. Declared *after* calib_session so on unwind this destructor —
    // which calls driver->stop() — runs first, quiescing the tap callbacks
    // before the session they reference is destroyed.
    struct DriverStop {
        pipeline::MultiCameraDriver* d;
        ~DriverStop() { if (d) d->stop(); }
    } driver_stop{driver.get()};

    // TrackerExtractor for the wizard's WebUI 3D viz (no publishers in this
    // mode — setup modes emit no tracker output).
    auto tracker_extractor =
        make_tracker_extractor(opts, *threed.bus3d, *threed.tracker_bus);
    struct ExtractorStop {
        slimevr::TrackerExtractor* tex;
        ~ExtractorStop() { if (tex) tex->stop(); }
    } extractor_stop{tracker_extractor.get()};

    auto server = make_server(opts, config::RunMode::CalibSubject, bus,
                              threed.bus3d.get(), &flow);
    if (server) {
        server->set_calibration_session(&calib_session, calib_defaults);
        server->set_calibration_next_step(
            flow.managed
            ? "profile written. Flow daemon switches to run mode."
            : "profile written. Restart in run mode: ./main --enable-3d"
              " --calib " + opts.calib + " --subject-id "
              + opts.subject_id + " ...");
        server->set_tracker_bus(threed.tracker_bus.get());
        server->start();
    }

    // Boot-time auto preflight + start (--calibrate always set in this mode).
    {
        pipeline::CalibPreflight in = calib_defaults;
        in.subject_id = opts.subject_id;
        in.subject_height_m = opts.subject_height_m;
        std::string err;
        if (!calib_session.preflight(in, err)) {
            std::fprintf(stderr, "calibrate preflight failed: %s\n", err.c_str());
            return EXIT_FAILURE;
        }
        if (!calib_session.start(err)) {
            std::fprintf(stderr, "calibrate start failed: %s\n", err.c_str());
            return EXIT_FAILURE;
        }
        FITRA_LOG_INFO("calibration auto-start: subject={} height={} m",
                       opts.subject_id, opts.subject_height_m);
    }

    run_stats_loop(*driver, opts.log_every_s, flow.stop);

    if (server) server->stop();
    driver->stop();   // taps quiesce here — safe for the session to wind down
    return EXIT_SUCCESS;
}

}  // namespace fitra::app
