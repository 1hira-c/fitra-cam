#pragma once
//
// Crow HTTP + WebSocket server.
//
// Mirrors python/scripts/dual_rtmpose_web.py:
//   - GET /            serves web/dual_rtmpose/index.html
//   - GET /<path>      serves files under web/dual_rtmpose/
//   - GET /stats       returns the current bundle as JSON
//   - WS  /ws          broadcasts the bundle at ≤30 Hz to every client
//   - GET /stats3d     returns current 3D bundle or disabled JSON
//   - WS  /ws3d        broadcasts the 3D bundle when enabled
//
// The publisher loop runs on its own thread; Crow's worker pool handles
// the HTTP request and WS plumbing.

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "pipeline/calibration_session.hpp"
#include "pipeline/snapshot.hpp"

namespace fitra::slimevr {
class NativePublisher;   // fwd decl; full header included only in crow_server.cpp
class SlimeTrackerBus;   // tracker snapshot bus for /ws3d viz
}

namespace fitra::vmt {
class VmtPublisher;      // fwd decl; full header in crow_server.cpp
class HmdPoseBus;        // HMD pose source for auto-alignment routes
}

namespace fitra::web {

struct ServerOptions {
    std::string host = "0.0.0.0";
    int         port = 8000;
    std::string static_dir;        // absolute path to web/dual_rtmpose/
    double      publish_hz = 30.0;
    int         crow_threads = 2;

    // Directory that contains web/calibration/{index.html,app.js,...}.
    // When non-empty (and a CalibrationSession is attached) /calib serves
    // the wizard frontend and /api/calib/* exposes the orchestrator.
    std::string calib_static_dir;
};

class CrowServer {
public:
    CrowServer(pipeline::SnapshotBus& bus, ServerOptions opts);
    CrowServer(pipeline::SnapshotBus& bus,
               pipeline::Skeleton3DBus* bus3d,
               ServerOptions opts);
    ~CrowServer();

    CrowServer(const CrowServer&) = delete;
    CrowServer& operator=(const CrowServer&) = delete;

    // Attach a CalibrationSession + the static defaults used by /api/calib/preflight
    // (calib_yaml/det_engine/pose_engine/subjects_dir/dump_tool_path/recording_frames_per_cam).
    // Must be called before start().
    void set_calibration_session(pipeline::CalibrationSession* session,
                                 pipeline::CalibPreflight defaults);

    // Attach the SlimeVR native publisher so /stats3d includes its send
    // counters. Caller retains ownership; the pointer must outlive the
    // CrowServer or be cleared with set_native_publisher(nullptr) before
    // the publisher dies. The crow server only calls const observers (stats()).
    void set_native_publisher(slimevr::NativePublisher* publisher);

    // Attach the SlimeVR tracker bus so /ws3d's bundle includes a `trackers`
    // field with each tracker's role / pos / quat / valid / roll_confidence.
    // Caller retains ownership. Set to nullptr to disable (then the bundle
    // has no `trackers` field).
    void set_tracker_bus(slimevr::SlimeTrackerBus* tracker_bus);

    // Attach the VMT publisher so /stats3d (and the /ws3d bundle splice)
    // include its send counters under a top-level "vmt" key. Same ownership
    // rules as set_native_publisher. Setting nullptr removes the splice
    // (and the "vmt" key disappears from the JSON).
    void set_vmt_publisher(vmt::VmtPublisher* publisher);

    // Attach the HMD pose bus so the /api/vmt/alignment/auto/*
    // routes have an input. `stale_threshold_ms` is the value passed to
    // HmdPoseBus::snapshot() — packets older than that are considered
    // stale and the auto-alignment routes return StaleHmd.
    void set_hmd_pose_bus(vmt::HmdPoseBus* bus, double stale_threshold_ms);

    // Start listening + broadcasting on a background thread. Returns when
    // the server is bound and ready (best-effort; Crow's run() blocks).
    void start();
    void stop();

private:
    void publisher_loop();
    void register_calibration_routes_();

    pipeline::SnapshotBus& bus_;
    pipeline::Skeleton3DBus* bus3d_ = nullptr;
    ServerOptions          opts_;
    std::thread            server_thread_;
    std::thread            publisher_thread_;
    std::atomic<bool>      stop_{false};

    pipeline::CalibrationSession*  calib_session_   = nullptr;
    pipeline::CalibPreflight       calib_defaults_;
    slimevr::NativePublisher*      native_publisher_ = nullptr;
    slimevr::SlimeTrackerBus*      tracker_bus_     = nullptr;
    vmt::VmtPublisher*             vmt_publisher_   = nullptr;
    vmt::HmdPoseBus*               hmd_pose_bus_    = nullptr;
    double                         hmd_stale_ms_    = 200.0;

    struct Impl;
    std::unique_ptr<Impl>  impl_;
};

}  // namespace fitra::web
