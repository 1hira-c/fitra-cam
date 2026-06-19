#include "app/daemon.hpp"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <thread>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "app/flow.hpp"
#include "pipeline/calibration_session.hpp"
#include "util/logging.hpp"

namespace fitra::app {

namespace {

// Signal plumbing for the wait loop. A daemon-directed SIGINT/SIGTERM must do
// two things together: set the stop flag so the loop exits after the child is
// reaped, AND forward SIGINT to the current child so it shuts down (the daemon
// otherwise blocks in waitpid forever — a terminal Ctrl-C reaches the child
// only because it is delivered to the whole process group, which a lone
// `kill`/systemd stop is not). Both signals share one handler so SIGTERM
// (docker/systemd stop) and SIGINT behave identically.
std::atomic<pid_t> g_child_pid{0};
std::atomic<bool>* g_daemon_stop = nullptr;

void on_daemon_signal(int) {
    if (g_daemon_stop) g_daemon_stop->store(true);
    const pid_t child = g_child_pid.load();
    if (child > 0) ::kill(child, SIGINT);
}

std::string join_argv(const char* argv0, const std::vector<std::string>& args) {
    std::string line{argv0};
    for (const auto& a : args) {
        line += ' ';
        line += a;
    }
    return line;
}

}  // namespace

std::vector<std::string> module_argv(config::RunMode mode,
                                     const config::MainOptions& opts,
                                     const std::string& config_path,
                                     bool profile_exists) {
    std::vector<std::string> args;
    if (!config_path.empty()) {
        args.push_back("--config");
        args.push_back(config_path);
    }
    args.push_back("--flow-managed");
    switch (mode) {
        case config::RunMode::Setup:
            // GPU-less setup module: enumerate cameras + compose the config.
            // No --no-vmt-out (it constructs no publishers to negate).
            args.push_back("--setup");
            break;
        case config::RunMode::Run:
            if (profile_exists && !opts.calib_subject_id.empty()) {
                args.push_back("--subject-id");
                // Same sanitize the wizard applied when writing the profile,
                // so run reads the directory the wizard actually created.
                args.push_back(pipeline::CalibrationSession::sanitize_id(
                    opts.calib_subject_id));
            }
            break;
        case config::RunMode::CalibSubject:
            args.push_back("--calibrate");
            args.push_back("--calib-auto-exit");
            args.push_back("--no-vmt-out");
            args.push_back("--no-slimevr-out");
            break;
        case config::RunMode::CalibExtrinsic:
            args.push_back("--extrinsic-calib");
            args.push_back("--no-vmt-out");
            args.push_back("--no-slimevr-out");
            break;
        case config::RunMode::CalibExtrinsicFloor:
            // VR-free floor path: --floor-map / --floor-intrinsics come from
            // --config (extrinsic_calib.floor_*). --floor-calib forces the mode
            // regardless of the config's default method, so a flow switch into
            // floor works even when the file selects controller.
            args.push_back("--floor-calib");
            args.push_back("--no-vmt-out");
            args.push_back("--no-slimevr-out");
            break;
        case config::RunMode::CalibIntrinsic:
            // Intrinsic (ChArUco) calibration: board params + out come from
            // --config (intrinsic_calib.*). --calib-intrinsic forces the mode.
            args.push_back("--calib-intrinsic");
            args.push_back("--no-vmt-out");
            args.push_back("--no-slimevr-out");
            break;
    }
    return args;
}

DaemonAction next_action(bool exited_normally, int exit_code,
                         int& consecutive_failures) {
    DaemonAction act;
    if (exited_normally && exit_code == EXIT_SUCCESS) {
        consecutive_failures = 0;
        act.kind = DaemonAction::Kind::CleanExit;
        return act;
    }
    if (exited_normally) {
        config::RunMode next;
        bool is_flow = true;
        switch (exit_code) {
            case kExitFlowToRun:                 next = config::RunMode::Run; break;
            case kExitFlowToSetup:               next = config::RunMode::Setup; break;
            case kExitFlowToCalibSubject:        next = config::RunMode::CalibSubject; break;
            case kExitFlowToCalibExtrinsic:      next = config::RunMode::CalibExtrinsic; break;
            case kExitFlowToCalibExtrinsicFloor: next = config::RunMode::CalibExtrinsicFloor; break;
            case kExitFlowToCalibIntrinsic:      next = config::RunMode::CalibIntrinsic; break;
            default: is_flow = false; break;
        }
        if (is_flow) {
            consecutive_failures = 0;
            act.kind = DaemonAction::Kind::Spawn;
            act.mode = next;
            return act;
        }
    }
    // Crash (non-flow exit code or signal death): fall back to run mode so
    // the rig comes back in its useful state; the user can re-switch from
    // the viewer. A run mode that itself keeps crashing would loop forever —
    // give up after a streak.
    if (++consecutive_failures >= kMaxConsecutiveFailures) {
        act.kind = DaemonAction::Kind::GiveUp;
        return act;
    }
    act.kind = DaemonAction::Kind::Spawn;
    act.mode = config::RunMode::Run;
    act.crashed = true;
    return act;
}

config::RunMode initial_mode(const config::MainOptions& opts,
                             bool intrinsics_exists,
                             bool extrinsics_exists,
                             bool profile_exists) {
    config::RunMode m;
    if (opts.daemon_initial != "auto"
        && config::parse_run_mode_name(opts.daemon_initial, m)) {
        return m;
    }
    // First run / unconfigured rig: with no cameras in the union config there is
    // nothing to calibrate or run, so land in the Setup module — the browser
    // picks cameras + composes the config, then the chain proceeds. (When
    // cameras ARE configured we fall through to the artifact-driven stages.)
    if (opts.cam_paths[0].empty()) {
        return config::RunMode::Setup;
    }
    // Step 0 of setup: when C++ intrinsic calibration is enabled and its output
    // YAML is missing, calibrate intrinsics first. Disabled (the default) →
    // intrinsics are assumed provided externally and we skip to extrinsic.
    if (opts.intrinsic_step_enabled && !intrinsics_exists) {
        return config::RunMode::CalibIntrinsic;
    }
    if (!extrinsics_exists) {
        // The configured method (extrinsic_calib.method: floor) picks which
        // extrinsic stage to enter first.
        return opts.excal_method == "floor" ? config::RunMode::CalibExtrinsicFloor
                                            : config::RunMode::CalibExtrinsic;
    }
    if (!profile_exists)    return config::RunMode::CalibSubject;
    return config::RunMode::Run;
}

std::string profile_path(const config::MainOptions& opts) {
    if (opts.calib_subject_id.empty()) return {};
    // The wizard sanitizes the id before writing <subjects_dir>/<id>/..., so
    // the daemon must look at (and pass on) the same sanitized form — an id
    // like "alice.v1" or a non-ASCII name otherwise yields a path the wizard
    // never wrote, and run never gets --subject-id.
    const std::string id =
        pipeline::CalibrationSession::sanitize_id(opts.calib_subject_id);
    return (std::filesystem::path{opts.subjects_dir}
            / id / "latest_profile.yaml").string();
}

int run_daemon(const config::MainOptions& opts,
               const std::string& config_path,
               const char* argv0,
               std::atomic<bool>& stop,
               int crash_backoff_ms) {
    if (config_path.empty()) {
        FITRA_LOG_WARN("[daemon] no --config given — modules spawn with code "
                       "defaults only (daemon mode does not forward other CLI "
                       "flags)");
    }
    // The calib-extrinsic stage writes excal_out; run/calib-subject read
    // three_d.calib. When they differ the chain hands over a stale file.
    if (!opts.calib.empty() && opts.excal_out != opts.calib) {
        FITRA_LOG_WARN("[daemon] extrinsic_calib.out ({}) != three_d.calib ({})"
                       " — a fresh extrinsic solve will not be picked up by "
                       "the next stage", opts.excal_out, opts.calib);
    }

    const auto profile_now = [&opts]() {
        // Empty id = no profile stage configured; treat as present so auto
        // initial mode (and run argv synthesis) skips it.
        if (opts.calib_subject_id.empty()) return true;
        // error_code overload: a permission/encoding error must not throw
        // out of a long-running daemon.
        std::error_code ec;
        return std::filesystem::exists(profile_path(opts), ec) && !ec;
    };
    std::error_code ec;
    const bool extrinsics_exists =
        !opts.calib.empty() && std::filesystem::exists(opts.calib, ec) && !ec;
    const bool intrinsics_exists =
        !opts.intrinsic_out.empty() &&
        std::filesystem::exists(opts.intrinsic_out, ec) && !ec;

    config::RunMode mode =
        initial_mode(opts, intrinsics_exists, extrinsics_exists, profile_now());
    FITRA_LOG_INFO("[daemon] initial mode: {} (intrinsics {}, extrinsics {}, profile {})",
                   config::run_mode_name(mode),
                   intrinsics_exists ? "present" : "missing",
                   extrinsics_exists ? "present" : "missing",
                   profile_now() ? "present" : "missing");

    // Pre-flight the chosen initial mode's config the same way the flow-switch
    // route does — otherwise a misconfigured calib stage (e.g. method: floor
    // with no floor_map) spawns a child that dies at validate, next_action sees
    // a non-flow exit and treats it as a crash, and the daemon silently falls
    // back to run. Surface the reason and start in run instead (the rig comes up
    // usable; the user fixes the config and re-switches from the viewer).
    {
        std::string perr;
        if (!config::precheck_mode_switch(opts, mode, perr)) {
            FITRA_LOG_ERROR("[daemon] initial mode {} is misconfigured: {} — starting "
                            "in run mode; fix the config and re-switch from the viewer",
                            config::run_mode_name(mode), perr);
            mode = config::RunMode::Run;
        }
    }

    // Own both signals: main() leaves them to us for the daemon path so the
    // handler can forward to the child and set stop in one place. Restore the
    // previous dispositions and clear the stop pointer on every exit path —
    // otherwise a signal after run_daemon returns (e.g. between tests) would
    // dereference the now-dangling &stop.
    struct SignalGuard {
        void (*old_int)(int);
        void (*old_term)(int);
        explicit SignalGuard(std::atomic<bool>* s) {
            g_daemon_stop = s;
            old_int  = std::signal(SIGINT, on_daemon_signal);
            old_term = std::signal(SIGTERM, on_daemon_signal);
        }
        ~SignalGuard() {
            std::signal(SIGINT, old_int);
            std::signal(SIGTERM, old_term);
            g_daemon_stop = nullptr;
        }
    } signal_guard{&stop};

    int consecutive_failures = 0;
    while (!stop.load()) {
        const auto args = module_argv(mode, opts, config_path, profile_now());
        FITRA_LOG_INFO("[daemon] spawning: {}", join_argv(argv0, args));

        // Build argv BEFORE fork: the child between fork and exec must avoid
        // heap allocation (not async-signal-safe). c_str() on the existing
        // strings allocates nothing.
        std::vector<char*> cargv;
        cargv.reserve(args.size() + 2);
        cargv.push_back(const_cast<char*>(argv0));
        for (const auto& a : args) cargv.push_back(const_cast<char*>(a.c_str()));
        cargv.push_back(nullptr);

        const pid_t pid = ::fork();
        if (pid < 0) {
            std::perror("[daemon] fork");
            return EXIT_FAILURE;
        }
        if (pid == 0) {
            // execvp searches PATH when argv0 has no slash (installed binary)
            // and behaves like execv for a path-qualified argv0.
            ::execvp(argv0, cargv.data());
            std::perror("[daemon] execvp");
            ::_exit(127);
        }
        g_child_pid.store(pid);

        int wstatus = 0;
        pid_t reaped;
        do {
            reaped = ::waitpid(pid, &wstatus, 0);
        } while (reaped < 0 && errno == EINTR);
        g_child_pid.store(0);
        if (reaped < 0) {
            std::perror("[daemon] waitpid");
            return EXIT_FAILURE;
        }

        const bool exited = WIFEXITED(wstatus);
        const int  code   = exited ? WEXITSTATUS(wstatus) : -1;
        if (!exited) {
            FITRA_LOG_ERROR("[daemon] module died on signal {}",
                            WIFSIGNALED(wstatus) ? WTERMSIG(wstatus) : 0);
        }

        const auto act = next_action(exited, code, consecutive_failures);
        switch (act.kind) {
            case DaemonAction::Kind::CleanExit:
                FITRA_LOG_INFO("[daemon] module exited cleanly — stopping");
                return EXIT_SUCCESS;
            case DaemonAction::Kind::GiveUp:
                FITRA_LOG_ERROR("[daemon] {} consecutive module failures — "
                                "giving up", consecutive_failures);
                return EXIT_FAILURE;
            case DaemonAction::Kind::Spawn:
                break;
        }
        if (stop.load()) break;  // switch requested while we were stopping

        if (act.crashed) {
            FITRA_LOG_WARN("[daemon] module crashed (exit {}); restarting run "
                           "mode in {} ms (failure {}/{})",
                           code, crash_backoff_ms,
                           consecutive_failures, kMaxConsecutiveFailures);
            const auto deadline = std::chrono::steady_clock::now()
                                  + std::chrono::milliseconds(crash_backoff_ms);
            while (!stop.load()
                   && std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        } else {
            FITRA_LOG_INFO("[daemon] module requested next mode: {}",
                           config::run_mode_name(act.mode));
        }
        mode = act.mode;
    }
    return EXIT_SUCCESS;
}

}  // namespace fitra::app
