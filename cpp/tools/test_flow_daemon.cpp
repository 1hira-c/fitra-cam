// test_flow_daemon — unit tests for the flow daemon's pure decision helpers
// (module_argv / next_action / initial_mode) plus an end-to-end spawn-loop
// test that runs run_daemon() against a stub shell script standing in for
// the module binary. No cameras, no GPU, no sockets.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include "app/daemon.hpp"
#include "app/flow.hpp"

namespace {

using fitra::app::DaemonAction;
using fitra::app::initial_mode;
using fitra::app::module_argv;
using fitra::app::next_action;
using fitra::app::profile_path;
using fitra::app::run_daemon;
using fitra::config::MainOptions;
using fitra::config::RunMode;

void check(bool cond, const std::string& msg) {
    if (!cond) throw std::runtime_error("ASSERT FAILED: " + msg);
}

bool has_arg(const std::vector<std::string>& args, const std::string& a) {
    for (const auto& s : args) if (s == a) return true;
    return false;
}

std::string join(const std::vector<std::string>& args) {
    std::string s;
    for (const auto& a : args) { s += a; s += ' '; }
    return s;
}

void test_module_argv() {
    MainOptions opts;
    opts.calib_subject_id = "subj";

    // run, profile present: config + managed + subject-id.
    auto run_args = module_argv(RunMode::Run, opts, "/tmp/session.yaml", true);
    check(run_args.size() == 5, "run argv size: " + join(run_args));
    check(run_args[0] == "--config" && run_args[1] == "/tmp/session.yaml",
          "run argv starts with --config PATH");
    check(run_args[2] == "--flow-managed", "run argv has --flow-managed");
    check(run_args[3] == "--subject-id" && run_args[4] == "subj",
          "run argv carries --subject-id from calib_subject_id");
    check(!has_arg(run_args, "--no-vmt-out"),
          "run argv keeps the YAML publishers");

    // run, profile missing (first boot): no --subject-id.
    auto run_fresh = module_argv(RunMode::Run, opts, "/tmp/session.yaml", false);
    check(!has_arg(run_fresh, "--subject-id"),
          "run argv omits --subject-id when the profile does not exist yet");

    // run, no subject configured: no --subject-id either.
    MainOptions no_subj;
    auto run_nosubj = module_argv(RunMode::Run, no_subj, "/tmp/s.yaml", true);
    check(!has_arg(run_nosubj, "--subject-id"),
          "run argv omits --subject-id when calib_subject_id is empty");

    // calib-subject: mode flag + auto-exit + publisher negations.
    auto subj_args = module_argv(RunMode::CalibSubject, opts,
                                 "/tmp/session.yaml", false);
    check(has_arg(subj_args, "--calibrate"),       "subject argv: --calibrate");
    check(has_arg(subj_args, "--calib-auto-exit"), "subject argv: --calib-auto-exit");
    check(has_arg(subj_args, "--no-vmt-out"),      "subject argv: --no-vmt-out");
    check(has_arg(subj_args, "--no-slimevr-out"),  "subject argv: --no-slimevr-out");
    check(has_arg(subj_args, "--flow-managed"),    "subject argv: --flow-managed");
    check(!has_arg(subj_args, "--subject-id"),
          "subject argv never carries --subject-id (a stale profile must not "
          "be loaded during recalibration)");

    // calib-extrinsic: mode flag + publisher negations.
    auto excal_args = module_argv(RunMode::CalibExtrinsic, opts,
                                  "/tmp/session.yaml", false);
    check(has_arg(excal_args, "--extrinsic-calib"), "excal argv: --extrinsic-calib");
    check(has_arg(excal_args, "--no-vmt-out"),      "excal argv: --no-vmt-out");
    check(has_arg(excal_args, "--no-slimevr-out"),  "excal argv: --no-slimevr-out");
    check(!has_arg(excal_args, "--calib-auto-exit"),
          "excal argv: solve auto-exits by itself");

    // No --config: only the mode flags.
    auto bare = module_argv(RunMode::Run, no_subj, "", true);
    check(bare.size() == 1 && bare[0] == "--flow-managed",
          "no-config run argv is just --flow-managed: " + join(bare));
}

void test_next_action() {
    int fails = 0;

    // Flow exit codes spawn the requested mode and clear the streak.
    fails = 2;
    auto a = next_action(true, fitra::app::kExitFlowToCalibSubject, fails);
    check(a.kind == DaemonAction::Kind::Spawn && a.mode == RunMode::CalibSubject,
          "81 -> spawn calib-subject");
    check(!a.crashed && fails == 0, "flow exit clears the failure streak");
    a = next_action(true, fitra::app::kExitFlowToRun, fails);
    check(a.kind == DaemonAction::Kind::Spawn && a.mode == RunMode::Run,
          "80 -> spawn run");
    a = next_action(true, fitra::app::kExitFlowToCalibExtrinsic, fails);
    check(a.kind == DaemonAction::Kind::Spawn && a.mode == RunMode::CalibExtrinsic,
          "82 -> spawn calib-extrinsic");

    // Clean exit stops the daemon.
    fails = 2;
    a = next_action(true, EXIT_SUCCESS, fails);
    check(a.kind == DaemonAction::Kind::CleanExit, "0 -> clean exit");
    check(fails == 0, "clean exit clears the failure streak");

    // Crashes (non-flow exit, exec failure, signal death) fall back to run
    // with the streak counting up; the third in a row gives up.
    fails = 0;
    a = next_action(true, EXIT_FAILURE, fails);
    check(a.kind == DaemonAction::Kind::Spawn && a.mode == RunMode::Run
          && a.crashed && fails == 1, "exit 1 -> crash fallback to run");
    a = next_action(false, -1, fails);
    check(a.kind == DaemonAction::Kind::Spawn && a.crashed && fails == 2,
          "signal death -> crash fallback to run");
    a = next_action(true, 127, fails);
    check(a.kind == DaemonAction::Kind::GiveUp && fails == 3,
          "third consecutive crash -> give up");

    // A flow exit in between resets the streak.
    fails = 2;
    (void)next_action(true, fitra::app::kExitFlowToRun, fails);
    check(fails == 0, "flow exit resets streak");
    a = next_action(true, EXIT_FAILURE, fails);
    check(a.kind == DaemonAction::Kind::Spawn && fails == 1,
          "streak restarts from 1 after a reset");
}

void test_initial_mode() {
    MainOptions opts;  // daemon_initial defaults to "auto"

    check(initial_mode(opts, false, false) == RunMode::CalibExtrinsic,
          "auto: no extrinsics -> calib-extrinsic");
    check(initial_mode(opts, true, false) == RunMode::CalibSubject,
          "auto: extrinsics ok, no profile -> calib-subject");
    check(initial_mode(opts, true, true) == RunMode::Run,
          "auto: both artifacts -> run");

    opts.daemon_initial = "calib-extrinsic";
    check(initial_mode(opts, true, true) == RunMode::CalibExtrinsic,
          "explicit --daemon-initial wins over artifacts");
    opts.daemon_initial = "run";
    check(initial_mode(opts, false, false) == RunMode::Run,
          "explicit run wins even with artifacts missing");

    MainOptions p;
    p.subjects_dir = "/data/subjects";
    p.calib_subject_id = "alice";
    check(profile_path(p) == "/data/subjects/alice/latest_profile.yaml",
          "profile_path layout");
    p.calib_subject_id.clear();
    check(profile_path(p).empty(), "profile_path empty without a subject id");
}

// End-to-end spawn loop: run_daemon() against a stub script that logs its
// argv and exits with a scripted sequence of codes. Locks fork/exec/waitpid,
// the exit-code chaining, the crash backoff path, and the give-up streak.
struct StubDir {
    std::filesystem::path dir, script, log, count;

    explicit StubDir(const std::string& name, const std::string& exit_codes) {
        dir = std::filesystem::temp_directory_path() / "fitra_test_flow_daemon" / name;
        std::filesystem::remove_all(dir);
        std::filesystem::create_directories(dir);
        script = dir / "stub_module.sh";
        log    = dir / "argv.log";
        count  = dir / "count";
        // The stub appends its argv to the log and exits with the N-th code
        // of `exit_codes` (space-separated; the last repeats).
        std::ofstream f(script);
        f << "#!/usr/bin/env bash\n"
          << "set -u\n"
          << "echo \"$@\" >> '" << log.string() << "'\n"
          << "n=0\n"
          << "[ -f '" << count.string() << "' ] && n=$(cat '" << count.string() << "')\n"
          << "echo $((n + 1)) > '" << count.string() << "'\n"
          << "codes=(" << exit_codes << ")\n"
          << "idx=$(( n < ${#codes[@]} ? n : ${#codes[@]} - 1 ))\n"
          << "exit \"${codes[$idx]}\"\n";
        f.close();
        ::chmod(script.c_str(), 0755);
    }

    std::vector<std::string> log_lines() const {
        std::vector<std::string> lines;
        std::ifstream f(log);
        std::string line;
        while (std::getline(f, line)) lines.push_back(line);
        return lines;
    }
};

void test_run_daemon_chain() {
    // excal (81 -> subject) -> subject (80 -> run) -> run (0 -> clean exit).
    StubDir stub("chain", "81 80 0");
    MainOptions opts;
    opts.daemon = true;
    opts.daemon_initial = "calib-extrinsic";
    opts.calib_subject_id = "subj";
    // Profile "exists": point subjects_dir at the stub dir and create it, so
    // the run spawn carries --subject-id.
    opts.subjects_dir = stub.dir.string();
    std::filesystem::create_directories(stub.dir / "subj");
    std::ofstream(stub.dir / "subj" / "latest_profile.yaml") << "x: 1\n";

    std::atomic<bool> stop{false};
    int rc = run_daemon(opts, "/tmp/session.yaml", stub.script.c_str(), stop,
                        /*crash_backoff_ms=*/10);
    check(rc == EXIT_SUCCESS, "chain daemon exits 0 after the clean module exit");

    auto lines = stub.log_lines();
    check(lines.size() == 3, "three modules spawned");
    check(lines[0].find("--extrinsic-calib") != std::string::npos
          && lines[0].find("--no-vmt-out") != std::string::npos,
          "spawn 1 is calib-extrinsic: " + lines[0]);
    check(lines[1].find("--calibrate") != std::string::npos
          && lines[1].find("--calib-auto-exit") != std::string::npos,
          "spawn 2 is calib-subject: " + lines[1]);
    check(lines[2].find("--subject-id subj") != std::string::npos
          && lines[2].find("--calibrate") == std::string::npos
          && lines[2].find("--extrinsic-calib") == std::string::npos,
          "spawn 3 is run with --subject-id: " + lines[2]);
    for (const auto& l : lines) {
        check(l.find("--config /tmp/session.yaml") != std::string::npos,
              "every spawn passes --config: " + l);
        check(l.find("--flow-managed") != std::string::npos,
              "every spawn passes --flow-managed: " + l);
    }
}

void test_run_daemon_crash_fallback_and_give_up() {
    // First module crashes (1) -> run respawn; run keeps crashing -> give up
    // after kMaxConsecutiveFailures total.
    StubDir stub("crash", "1 1 1 1");
    MainOptions opts;
    opts.daemon = true;
    opts.daemon_initial = "run";

    std::atomic<bool> stop{false};
    int rc = run_daemon(opts, "", stub.script.c_str(), stop,
                        /*crash_backoff_ms=*/10);
    check(rc == EXIT_FAILURE, "crash streak makes the daemon give up with 1");
    check(stub.log_lines().size()
          == static_cast<std::size_t>(fitra::app::kMaxConsecutiveFailures),
          "exactly kMaxConsecutiveFailures spawns before giving up");
}

// A daemon-directed SIGINT/SIGTERM must stop the loop cleanly (rc 0): set the
// stop flag AND forward SIGINT to the running child so waitpid returns. This
// stub installs its own INT trap and exits 0, then idles until signalled —
// it touches `ready` first so the test can signal only after the child (and
// thus the daemon's signal handlers) are up.
void test_run_daemon_signal_clean_stop() {
    auto dir = std::filesystem::temp_directory_path()
               / "fitra_test_flow_daemon" / "signal";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    auto script = dir / "idle_module.sh";
    auto ready  = dir / "ready";
    {
        std::ofstream f(script);
        f << "#!/usr/bin/env bash\n"
          << "trap 'exit 0' INT\n"
          << "touch '" << ready.string() << "'\n"
          << "while true; do sleep 0.1; done\n";
    }
    ::chmod(script.c_str(), 0755);

    MainOptions opts;
    opts.daemon = true;
    opts.daemon_initial = "run";

    std::atomic<bool> stop{false};
    int rc = -999;
    std::thread th([&]() {
        rc = run_daemon(opts, "", script.c_str(), stop, /*crash_backoff_ms=*/10);
    });

    // Wait for the child to come up (bounded), then deliver SIGTERM to our own
    // process — run_daemon's handler forwards SIGINT to the child.
    bool up = false;
    for (int i = 0; i < 200; ++i) {
        if (std::filesystem::exists(ready)) { up = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    check(up, "signal stub child came up (ready file)");
    ::kill(::getpid(), SIGTERM);

    th.join();
    check(rc == EXIT_SUCCESS,
          "SIGTERM stops the daemon cleanly (rc 0, no crash-respawn): rc="
          + std::to_string(rc));

    // Restore the default disposition so a stray signal later in the run does
    // not hit run_daemon's now-dangling handler state.
    std::signal(SIGTERM, SIG_DFL);
    std::signal(SIGINT, SIG_DFL);
}

struct TestCase {
    const char* name;
    void (*fn)();
};

const TestCase kTests[] = {
    {"module_argv",                       test_module_argv},
    {"next_action",                       test_next_action},
    {"initial_mode",                      test_initial_mode},
    {"run_daemon_chain",                  test_run_daemon_chain},
    {"run_daemon_crash_fallback_give_up", test_run_daemon_crash_fallback_and_give_up},
    {"run_daemon_signal_clean_stop",      test_run_daemon_signal_clean_stop},
};

}  // namespace

int main() {
    int failed = 0;
    for (const auto& t : kTests) {
        try {
            t.fn();
            std::printf("[PASS] %s\n", t.name);
        } catch (const std::exception& e) {
            std::printf("[FAIL] %s: %s\n", t.name, e.what());
            ++failed;
        }
    }
    if (failed) {
        std::printf("\n%d test(s) failed\n", failed);
        return EXIT_FAILURE;
    }
    std::printf("\nall %zu tests passed\n", sizeof(kTests) / sizeof(kTests[0]));
    return EXIT_SUCCESS;
}
