#pragma once
//
// Flow daemon contract (docs/design/pose-3d-flow-daemon.md): under
// `./main --daemon` each mode runs as a child "module" process that the
// daemon spawns. A module reports the next mode to run through its exit
// code; the daemon waits and spawns it. Modules know they are daemon-spawned
// via --flow-managed — standalone invocations keep the manual-restart
// contract (exit 0 + guidance log) and never emit flow exit codes.

#include <atomic>

#include "config/main_config.hpp"

namespace fitra::app {

// Module exit codes consumed by the daemon's wait loop. 0 is a clean stop
// (the daemon exits too); any other non-flow exit or a signal death is a
// crash. 80.. is outside the conventional 0/1/2 + 128+signal ranges.
inline constexpr int kExitFlowToRun                 = 80;
inline constexpr int kExitFlowToCalibSubject        = 81;
inline constexpr int kExitFlowToCalibExtrinsic      = 82;
inline constexpr int kExitFlowToCalibExtrinsicFloor = 83;

inline int flow_exit_code(config::RunMode m) {
    switch (m) {
        case config::RunMode::CalibSubject:        return kExitFlowToCalibSubject;
        case config::RunMode::CalibExtrinsic:      return kExitFlowToCalibExtrinsic;
        case config::RunMode::CalibExtrinsicFloor: return kExitFlowToCalibExtrinsicFloor;
        case config::RunMode::Run:                 break;
    }
    return kExitFlowToRun;
}

// Shared between main(), the mode runner, and the Crow flow-switch route: a
// switch request stops the current mode loop and records the requested next
// mode, which main() translates into the flow exit code after the runner
// returns cleanly.
struct FlowControl {
    std::atomic<bool>& stop;
    bool managed = false;            // spawned by the daemon (--flow-managed)
    std::atomic<int> next_mode{-1};  // -1 = none, else static_cast<int>(RunMode)

    void request_switch(config::RunMode m) {
        next_mode.store(static_cast<int>(m));
        stop.store(true);
    }
};

}  // namespace fitra::app
