#pragma once
//
// Flow daemon: `./main --daemon --config session.yaml` spawns one mode
// module at a time (same binary, mode flags appended) and chains modes via
// the module's exit code (app/flow.hpp, docs/design/pose-3d-flow-daemon.md).
// The daemon itself opens no sockets and never touches CUDA/TRT — modules
// own the whole pipeline and the :port web surface, so the browser keeps
// talking to the same address across mode switches.

#include <atomic>
#include <string>
#include <vector>

#include "config/main_config.hpp"

namespace fitra::app {

// ---- pure decision helpers (unit-tested in tools/test_flow_daemon.cpp) ----

// Spawn argv for a mode module (argv[0] excluded). All settings come from
// the union --config YAML; the daemon only appends mode flags. Publisher
// settings in the union YAML are negated for calib spawns (setup modes
// reject publishers in validate). run spawns get --subject-id only when the
// profile YAML already exists — the first boot has nothing to load yet.
std::vector<std::string> module_argv(config::RunMode mode,
                                     const config::MainOptions& opts,
                                     const std::string& config_path,
                                     bool profile_exists);

// Decision after a module exit. `exited_normally` is WIFEXITED, `exit_code`
// is WEXITSTATUS (ignored when !exited_normally). Flow exit codes spawn the
// requested mode and reset the failure streak; 0 is a clean stop; anything
// else is a crash → spawn run (the safe default the user can switch out of),
// or give up after kMaxConsecutiveFailures crashes without a normal exit
// in between.
inline constexpr int kMaxConsecutiveFailures = 3;

struct DaemonAction {
    enum class Kind { Spawn, CleanExit, GiveUp };
    Kind kind = Kind::CleanExit;
    config::RunMode mode = config::RunMode::Run;  // valid when kind == Spawn
    bool crashed = false;  // this Spawn is a crash fallback (backoff applies)
};

DaemonAction next_action(bool exited_normally, int exit_code,
                         int& consecutive_failures);

// First mode to spawn. --daemon-initial wins when not "auto"; auto picks the
// first stage whose input artifact is missing: no extrinsics YAML →
// calib-extrinsic, no subject profile → calib-subject, both present → run.
config::RunMode initial_mode(const config::MainOptions& opts,
                             bool extrinsics_exists,
                             bool profile_exists);

// Subject profile the run module would load:
// <subjects_dir>/<calib_subject_id>/latest_profile.yaml.
// Empty when calib_subject_id is empty (no profile stage configured).
std::string profile_path(const config::MainOptions& opts);

// ---- the daemon loop ----

// Blocks until a module exits cleanly (0), the crash streak gives up, or
// `stop` is set (SIGINT). `argv0` is the binary to spawn — the daemon's own
// argv[0]. `crash_backoff_ms` is the wait before a crash-fallback spawn
// (overridable for tests).
int run_daemon(const config::MainOptions& opts,
               const std::string& config_path,
               const char* argv0,
               std::atomic<bool>& stop,
               int crash_backoff_ms = 2000);

}  // namespace fitra::app
