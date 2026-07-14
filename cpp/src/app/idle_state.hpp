#pragma once
//
// Shared consumer-presence state for the idle/standby mode (issue #37,
// docs/design/core-pipeline-idle-standby.md).
//
// One IdleState is created per Run-mode process and shared (by raw pointer,
// owned by mode_run) with the components that write to or read from it,
// exactly like the calib_recording_flag pattern in camera_builder.cpp:
//
//   * CrowServer writes ws_client_count on /ws + /ws3d onopen/onclose.
//   * IdleEvaluator (the ~10Hz hysteresis thread) writes vr_peer_live /
//     vr_observable and confirms `idle`.
//   * FrameSource::decode_loop, MultiCameraDriver::loop, and TrackerExtractor
//     read `idle` (relaxed) to gate the heavy GPU work and reset smoothing on
//     resume.
//
// Calib modes never construct an IdleState — those components are handed a
// null pointer and so are never throttled (null => idle is always false).

#include <atomic>

namespace fitra::app {

struct IdleState {
    // Live WS viewer count, summed across /ws + /ws3d. Inc/dec by CrowServer.
    std::atomic<int>  ws_client_count{0};
    // A live VR peer is replying (HMD pose fresh) — derived by IdleEvaluator
    // from HmdPoseBus freshness.
    std::atomic<bool> vr_peer_live{false};
    // Whether VR presence is even observable. False when an output is enabled
    // (vmt_out) but no return signal is configured
    // (hmd_listen_enabled off): we then cannot tell whether a peer is live, so
    // the evaluator keeps VR "present" and never idles on the VR axis.
    std::atomic<bool> vr_observable{false};
    // The confirmed current standby state. Written only by IdleEvaluator.
    std::atomic<bool> idle{false};

    // True when at least one consumer is present right now (pre-hysteresis).
    bool any_consumer() const {
        return ws_client_count.load(std::memory_order_relaxed) > 0
            || vr_peer_live.load(std::memory_order_relaxed);
    }

    // Convenience reader for the gating sites (relaxed: the flag is a hint that
    // flips at most a few times a second; no ordering guarantees are needed).
    bool is_idle() const { return idle.load(std::memory_order_relaxed); }
};

}  // namespace fitra::app
