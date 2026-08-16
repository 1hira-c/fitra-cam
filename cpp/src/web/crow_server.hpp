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
//   - GET /api/pose-gate returns the raw tri.skeleton position-only sample
//   - WS  /ws/pose-gate broadcasts new fitra_pose_gate_v1 samples
//   - GET /api/fusion-pose returns the D50 evidence-rich raw pose sample
//   - WS  /ws/fusion-pose preserves boundaries and accepts clock-sync pings
//   - GET /api/tracker-axis returns post-One-Euro anatomical axes
//   - WS  /ws/tracker-axis preserves boundaries and accepts clock-sync pings
//
// The publisher loop runs on its own thread; Crow's worker pool handles
// the HTTP request and WS plumbing.

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "app/idle_state.hpp"   // header-only consumer-presence state (idle mode)
#include "pipeline/calibration_session.hpp"
#include "pipeline/snapshot.hpp"

namespace fitra::pipeline {
class ExtrinsicCalibSession;   // fwd decl; full header included in crow_server.cpp
class FloorCalibSession;       // fwd decl; full header included in crow_server.cpp
class IntrinsicCalibSession;   // fwd decl; full header included in crow_server.cpp
class PoseGateBus;             // fwd decl; full header included in crow_server.cpp
class FusionPoseBus;
}

namespace fitra::tracking {
class TrackerBus;
class TrackerAxisBus;
}

namespace fitra::vmt {
class VmtPublisher;      // fwd decl; full header in crow_server.cpp
class HmdPoseBus;        // HMD pose source for auto-alignment routes
class ControllerPoseBus; // selected-controller pose source for excal scene
class ContinuousAligner; // always-on HMD-driven alignment refiner
class DiscoveryBeacon;   // zeroconf peer discovery (resolved peer in /stats3d)
}

namespace fitra::config { class SetupConfigStore; }   // setup-mode config draft
namespace fitra::camera { class SetupCameraManager; }  // setup-mode camera preview

namespace fitra::web {

struct ServerOptions {
    std::string host = "0.0.0.0";
    int         port = 8000;
    std::string static_dir;        // absolute path to web/dual_rtmpose/
    double      publish_hz = 30.0;
    int         crow_threads = 2;

    // RunMode label reported by GET /api/state ("run" / "calib-subject" /
    // "calib-extrinsic"); the frontends use it to show or hide the
    // calibration entry points.
    std::string mode_label = "run";

    // True when this module was spawned by the flow daemon (--flow-managed).
    // Reported by GET /api/state as "managed" so the frontends know whether
    // mode switches happen automatically (vs. manual restart guidance).
    bool flow_managed = false;

    // Directory that contains web/calibration/{index.html,app.js,...}.
    // When non-empty (and a CalibrationSession is attached) /calib serves
    // the wizard frontend and /api/calib/* exposes the orchestrator.
    std::string calib_static_dir;

    // Directory that contains web/extrinsic_calibration/{index.html,app.js,...}.
    // When non-empty (and an ExtrinsicCalibSession is attached) /extrinsic-calib
    // serves the collection frontend; /api/excal/* exposes the session.
    std::string excal_static_dir;

    // Directory that contains web/intrinsic_calibration/{index.html,app.js,...}.
    // When non-empty (and an IntrinsicCalibSession is attached) /intrinsic-calib
    // serves the collection frontend; /api/incal/* exposes the session.
    std::string incal_static_dir;
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

    // Attach the controller-marker extrinsic calibration session so /api/excal/*
    // exposes its state + start/stop/solve controls. Caller retains ownership;
    // the pointer must outlive the CrowServer. Must be called before start().
    void set_extrinsic_calib_session(pipeline::ExtrinsicCalibSession* session);

    // Attach the floor-AprilTag (案D) extrinsic calibration session. Reuses the
    // /api/excal/* route names backed by the floor session; never set together
    // with set_extrinsic_calib_session (the two modes run in separate
    // processes). Must be called before start().
    void set_floor_calib_session(pipeline::FloorCalibSession* session);
    // Guidance spliced into a successful floor /api/excal/solve as "next_step".
    void set_floor_calib_next_step(std::string guidance);

    // Attach the intrinsic (ChArUco) calibration session; /api/incal/* exposes
    // its state + start/stop/solve. Must be called before start().
    void set_intrinsic_calib_session(pipeline::IntrinsicCalibSession* session);
    void set_intrinsic_calib_next_step(std::string guidance);

    // Attach the setup-module handlers (RunMode::Setup only): the editable
    // config store (drives /api/config*) and an on-proceed callback that writes
    // the union config and advances the flow (drives /api/setup/proceed). A
    // camera manager for /api/cameras* is attached separately (M3). Caller
    // retains ownership; pointers must outlive the server. Before start().
    void set_setup_handlers(
        config::SetupConfigStore* store,
        std::function<bool(std::string& next_mode, std::string& err)> on_proceed,
        std::function<bool(std::string& err)> on_validate = {});
    // Attach the V4L2 camera manager for setup-mode enumeration + preview (M3).
    void set_setup_camera_manager(camera::SetupCameraManager* cameras);

    // Human-facing guidance spliced into a successful /api/excal/solve
    // response as "next_step" (the subject-calib restart command). Replaces
    // the old live-reinject solved callback: solve writes the YAML and the
    // process auto-exits; nothing is reloaded in-process. Must be called
    // before start().
    void set_extrinsic_calib_next_step(std::string guidance);

    // Symmetric guidance for the subject wizard: spliced into a successful
    // /api/calib/approve response as "next_step" (run-mode restart command,
    // or the flow-daemon auto-switch notice). Must be called before start().
    void set_calibration_next_step(std::string guidance);

    // Attach the flow-switch handler (daemon-managed modules only). When set,
    // POST /api/flow/switch {"mode": "run"|"calib-subject"|"calib-extrinsic"}
    // is registered; the handler returns false + fills `err` on an unknown
    // mode, otherwise records the request and stops the mode loop. Standalone
    // (non-managed) runs never call this, so the route stays unregistered
    // (GET 404 / POST 405 via the static catchall). Must be called before
    // start().
    using FlowSwitchFn = std::function<bool(const std::string& mode,
                                            std::string& err)>;
    void set_flow_switch_handler(FlowSwitchFn fn);

    // Attach the tracker bus so /ws3d's bundle includes a `trackers`
    // field with each tracker's role / pos / quat / valid / roll_confidence.
    // Caller retains ownership. Set to nullptr to disable (then the bundle
    // has no `trackers` field).
    void set_tracker_bus(tracking::TrackerBus* tracker_bus);

    // Attach the independent raw `tri.skeleton` position-only output. The
    // `/ws/pose-gate` stream is separate from `/ws3d` and never contains
    // tracker quaternions.
    void set_pose_gate_bus(pipeline::PoseGateBus* pose_gate_bus);

    // Attach the additive D50 producer.  This does not alter /ws/pose-gate.
    void set_fusion_pose_bus(pipeline::FusionPoseBus* fusion_pose_bus);

    // Attach the additive post-One-Euro D50 anatomical-axis producer.
    void set_tracker_axis_bus(tracking::TrackerAxisBus* tracker_axis_bus);

    // Attach the VMT publisher so /stats3d (and the /ws3d bundle splice)
    // include its send counters under a top-level "vmt" key. Same ownership
    // rules as set_tracker_bus. Setting nullptr removes the splice (and the
    // "vmt" key disappears from the JSON).
    void set_vmt_publisher(vmt::VmtPublisher* publisher);

    // Attach the HMD pose bus so the /api/vmt/alignment/auto/*
    // routes have an input. `stale_threshold_ms` is the value passed to
    // HmdPoseBus::snapshot() — packets older than that are considered
    // stale and the auto-alignment routes return StaleHmd.
    void set_hmd_pose_bus(vmt::HmdPoseBus* bus, double stale_threshold_ms);

    // HMD-on-face forward offset (m) applied by the one-shot T-pose / motion
    // auto-align routes when projecting the HMD onto the head axis. The
    // always-on ContinuousAligner takes the same value via its own config.
    void set_align_hmd_forward_m(float meters);

    // Attach the selected extrinsic-calibration controller pose bus so
    // /api/excal/poses can expose the live HMD + controller markers. The
    // ControllerPoseBus is already filtered to opts.excal_controller_role by
    // the tracked-pose receiver; `role` is the label to report to clients.
    void set_extrinsic_calib_pose_bus(vmt::ControllerPoseBus* bus,
                                      std::string role,
                                      double stale_threshold_ms);

    // Attach the continuous HMD-driven aligner so /stats3d reports its status
    // and /api/vmt/alignment/auto/continuous/* can toggle it at runtime. Same
    // ownership rules as set_vmt_publisher (caller retains, must outlive or be
    // cleared before destruction).
    void set_continuous_aligner(vmt::ContinuousAligner* aligner);

    // Attach the zeroconf discovery beacon so /stats3d reports the resolved
    // peer + the live peer list. Same ownership rules as set_vmt_publisher.
    void set_discovery_beacon(vmt::DiscoveryBeacon* beacon);

    // Attach the idle/standby shared state (issue #37). The /ws, /ws3d, and
    // /ws/pose-gate, /ws/fusion-pose and /ws/tracker-axis onopen/onclose
    // handlers then maintain its ws_client_count,
    // and /stats3d, the /ws3d bundle, and /api/state expose an `idle` status
    // object. `enabled` / `enter_after_s` / `tick_hz` are config echoes for the
    // status surface.
    // Caller retains ownership (the IdleState must outlive the server). Never
    // attached in calib modes.
    void set_idle_state(app::IdleState* state, bool enabled,
                        double enter_after_s, double tick_hz);

    // Start listening + broadcasting on a background thread. Returns when
    // the server is bound and ready (best-effort; Crow's run() blocks).
    void start();
    void stop();

private:
    void publisher_loop();
    void register_calibration_routes_();
    void register_extrinsic_calib_routes_();
    void register_floor_calib_routes_();
    void register_intrinsic_calib_routes_();
    void register_setup_mode_routes_();

    pipeline::SnapshotBus& bus_;
    pipeline::Skeleton3DBus* bus3d_ = nullptr;
    pipeline::PoseGateBus* pose_gate_bus_ = nullptr;
    pipeline::FusionPoseBus* fusion_pose_bus_ = nullptr;
    tracking::TrackerAxisBus* tracker_axis_bus_ = nullptr;
    ServerOptions          opts_;
    std::thread            server_thread_;
    std::thread            publisher_thread_;
    std::atomic<bool>      stop_{false};

    pipeline::CalibrationSession*  calib_session_   = nullptr;
    pipeline::CalibPreflight       calib_defaults_;
    std::string                    calib_next_step_;
    pipeline::ExtrinsicCalibSession* excal_session_ = nullptr;
    std::string                    excal_next_step_;
    pipeline::FloorCalibSession*   floor_session_ = nullptr;
    std::string                    floor_next_step_;
    pipeline::IntrinsicCalibSession* intrinsic_session_ = nullptr;
    std::string                    intrinsic_next_step_;
    config::SetupConfigStore*      setup_store_   = nullptr;
    camera::SetupCameraManager*    setup_cameras_ = nullptr;
    std::function<bool(std::string&, std::string&)> setup_on_proceed_;
    std::function<bool(std::string&)>               setup_on_validate_;
    FlowSwitchFn                   flow_switch_;
    tracking::TrackerBus*      tracker_bus_     = nullptr;
    vmt::VmtPublisher*             vmt_publisher_   = nullptr;
    vmt::HmdPoseBus*               hmd_pose_bus_    = nullptr;
    vmt::ControllerPoseBus*        excal_controller_pose_bus_ = nullptr;
    vmt::ContinuousAligner*        continuous_aligner_ = nullptr;
    vmt::DiscoveryBeacon*          discovery_beacon_ = nullptr;
    app::IdleState*                idle_state_      = nullptr;
    bool                           idle_enabled_      = false;
    double                         idle_enter_after_s_ = 10.0;
    double                         idle_tick_hz_       = 2.0;
    double                         hmd_stale_ms_    = 200.0;
    // HMD face lever-arm correction for the one-shot auto-align routes (see
    // auto_alignment.hpp::hmd_head_axis_xz). Set from --vmt-align-hmd-forward.
    float                          align_hmd_forward_m_ = 0.0f;
    double                         excal_controller_stale_ms_ = 200.0;
    std::string                    excal_controller_role_ = "right";

    struct Impl;
    std::unique_ptr<Impl>  impl_;
};

}  // namespace fitra::web
