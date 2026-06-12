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
#include "util/logging.hpp"

namespace fitra::app {

namespace {

// Child pid for the SIGTERM forwarder. The terminal's Ctrl-C reaches the
// child by itself (same process group); SIGTERM (systemd, docker stop) goes
// to the daemon only, so forward it as the module's usual SIGINT shutdown.
std::atomic<pid_t> g_child_pid{0};

void forward_term(int) {
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
        case config::RunMode::Run:
            if (profile_exists && !opts.calib_subject_id.empty()) {
                args.push_back("--subject-id");
                args.push_back(opts.calib_subject_id);
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
            case kExitFlowToRun:            next = config::RunMode::Run; break;
            case kExitFlowToCalibSubject:   next = config::RunMode::CalibSubject; break;
            case kExitFlowToCalibExtrinsic: next = config::RunMode::CalibExtrinsic; break;
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
                             bool extrinsics_exists,
                             bool profile_exists) {
    config::RunMode m;
    if (opts.daemon_initial != "auto"
        && config::parse_run_mode_name(opts.daemon_initial, m)) {
        return m;
    }
    if (!extrinsics_exists) return config::RunMode::CalibExtrinsic;
    if (!profile_exists)    return config::RunMode::CalibSubject;
    return config::RunMode::Run;
}

std::string profile_path(const config::MainOptions& opts) {
    if (opts.calib_subject_id.empty()) return {};
    return (std::filesystem::path{opts.subjects_dir}
            / opts.calib_subject_id / "latest_profile.yaml").string();
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
        return std::filesystem::exists(profile_path(opts));
    };
    const bool extrinsics_exists =
        !opts.calib.empty() && std::filesystem::exists(opts.calib);

    config::RunMode mode = initial_mode(opts, extrinsics_exists, profile_now());
    FITRA_LOG_INFO("[daemon] initial mode: {} (extrinsics {}, profile {})",
                   config::run_mode_name(mode),
                   extrinsics_exists ? "present" : "missing",
                   profile_now() ? "present" : "missing");

    std::signal(SIGTERM, forward_term);

    int consecutive_failures = 0;
    while (!stop.load()) {
        const auto args = module_argv(mode, opts, config_path, profile_now());
        FITRA_LOG_INFO("[daemon] spawning: {}", join_argv(argv0, args));

        const pid_t pid = ::fork();
        if (pid < 0) {
            std::perror("[daemon] fork");
            return EXIT_FAILURE;
        }
        if (pid == 0) {
            // Child: exec the module. No heap discipline needed — execv
            // replaces the image, _exit(127) only on exec failure.
            std::vector<char*> cargv;
            cargv.push_back(const_cast<char*>(argv0));
            for (const auto& a : args) cargv.push_back(const_cast<char*>(a.c_str()));
            cargv.push_back(nullptr);
            ::execv(argv0, cargv.data());
            std::perror("[daemon] execv");
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
