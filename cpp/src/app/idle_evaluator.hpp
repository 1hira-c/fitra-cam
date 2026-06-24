#pragma once
//
// Idle/standby evaluator (issue #37, docs/design/core-pipeline-idle-standby.md).
//
// A small ~10Hz thread that derives consumer presence from the shared
// IdleState + HmdPoseBus freshness and confirms the `idle` flag with
// asymmetric hysteresis. The decision math is split into header-only pure
// functions (idle_consumer_present / idle_vr_observable / IdleDecision::step)
// so it can be unit-tested without threads or clocks (test_idle_evaluator).

#include <atomic>
#include <memory>
#include <thread>

namespace fitra::vmt { class HmdPoseBus; }

namespace fitra::app {

struct IdleState;

// ---- Pure decision logic (unit-tested directly) --------------------------

// Whether VR presence is observable at all. Not observable when a VR output is
// enabled but no return channel is configured (we cannot tell if a peer is
// live), in which case the caller must keep VR "present" (safe default).
inline bool idle_vr_observable(bool has_vr_output, bool hmd_listen_enabled) {
    return !has_vr_output || hmd_listen_enabled;
}

// Folds the raw presence signals into a single "is any consumer present now?"
// predicate (pre-hysteresis), applying the VR-observability safe default: when
// VR presence is not observable but an output is enabled, treat VR as present
// so we never idle on an unobservable VR axis.
inline bool idle_consumer_present(bool ws_present, bool vr_peer_live,
                                  bool vr_observable, bool has_vr_output) {
    if (ws_present) return true;
    if (!vr_observable) return has_vr_output;
    return vr_peer_live;
}

// Asymmetric hysteresis timer. Idle engages only after consumers have been
// continuously absent for >= enter_after_s; it disengages on the first tick a
// consumer reappears (immediate resume, no exit delay).
struct IdleDecision {
    double absent_s = 0.0;
    bool   idle     = false;

    bool step(bool any_consumer, double dt_s, double enter_after_s) {
        if (any_consumer) {
            absent_s = 0.0;
            idle = false;
        } else {
            absent_s += (dt_s > 0.0 ? dt_s : 0.0);
            if (absent_s >= enter_after_s) idle = true;
        }
        return idle;
    }
};

// ---- Background evaluator thread -----------------------------------------

class IdleEvaluator {
public:
    struct Config {
        bool   enabled            = true;
        double enter_after_s      = 10.0;
        double tick_hz            = 2.0;    // informational (driver throttle)
        double hmd_stale_ms       = 200.0;
        bool   has_vr_output      = false;  // vmt_out || slimevr_out
        bool   hmd_listen_enabled = false;
    };

    // `state` and `hmd_bus` must outlive the evaluator. hmd_bus may be null
    // (no HMD listen): VR is then unobservable iff a VR output is enabled.
    IdleEvaluator(IdleState& state, const vmt::HmdPoseBus* hmd_bus, Config cfg);
    ~IdleEvaluator();

    IdleEvaluator(const IdleEvaluator&) = delete;
    IdleEvaluator& operator=(const IdleEvaluator&) = delete;

    void start();
    void stop();

private:
    void loop();

    IdleState&             state_;
    const vmt::HmdPoseBus*  hmd_bus_;
    Config                 cfg_;
    IdleDecision           decision_;
    std::thread            thread_;
    std::atomic<bool>      stop_{false};
};

}  // namespace fitra::app
