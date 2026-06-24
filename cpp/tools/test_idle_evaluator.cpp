// Pure-logic unit tests for the idle/standby evaluator (issue #37):
//   * IdleDecision::step       — asymmetric hysteresis (delayed enter, instant resume)
//   * idle_vr_observable       — VR-presence observability
//   * idle_consumer_present    — presence fold-in + VR safe default
// No threads, clocks, or I/O — the header-only predicates are exercised directly.

#include "app/idle_evaluator.hpp"

#include <cstdio>

namespace {

using fitra::app::IdleDecision;
using fitra::app::idle_consumer_present;
using fitra::app::idle_vr_observable;

int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++g_fail; } \
} while (0)

// Idle engages only after consumers are absent for >= enter_after_s.
void test_enter_requires_dwell() {
    IdleDecision d;
    const double enter = 10.0;
    const double dt = 1.0;
    // 9s of absence: not yet idle.
    for (int i = 0; i < 9; ++i) CHECK(d.step(/*any_consumer=*/false, dt, enter) == false);
    // The tick that crosses 10s flips idle.
    CHECK(d.step(false, dt, enter) == true);
    // Stays idle while still absent.
    CHECK(d.step(false, dt, enter) == true);
}

// A consumer present at any tick keeps idle false and zeroes the dwell timer.
void test_presence_blocks_and_resets() {
    IdleDecision d;
    const double enter = 5.0;
    CHECK(d.step(false, 4.0, enter) == false);     // 4s absent
    CHECK(d.step(true,  1.0, enter) == false);     // consumer back -> reset
    CHECK(d.absent_s == 0.0);
    // Must accumulate the full dwell again from zero (not 1 more second).
    CHECK(d.step(false, 4.0, enter) == false);
    CHECK(d.step(false, 1.0, enter) == true);
}

// Resume is immediate: the first tick a consumer reappears drops idle.
void test_resume_is_instant() {
    IdleDecision d;
    const double enter = 3.0;
    CHECK(d.step(false, 3.0, enter) == true);      // enter idle
    CHECK(d.step(true,  0.1, enter) == false);     // one tick -> resume
}

// enter_after_s == 0 means idle the moment consumers are absent.
void test_zero_dwell_enters_immediately() {
    IdleDecision d;
    CHECK(d.step(false, 0.016, 0.0) == true);
}

void test_vr_observable() {
    // No VR output: VR presence is vacuously observable.
    CHECK(idle_vr_observable(/*has_vr_output=*/false, /*hmd_listen=*/false) == true);
    CHECK(idle_vr_observable(false, true) == true);
    // VR output WITH a return channel: observable.
    CHECK(idle_vr_observable(true, true) == true);
    // VR output WITHOUT a return channel: NOT observable (the safe-default case).
    CHECK(idle_vr_observable(true, false) == false);
}

void test_consumer_present_fold_in() {
    // A WS viewer always counts, regardless of the VR axis.
    CHECK(idle_consumer_present(/*ws=*/true, /*vr_live=*/false,
                                /*vr_observable=*/true, /*has_vr_output=*/false) == true);
    // Observable VR: presence tracks vr_peer_live.
    CHECK(idle_consumer_present(false, true,  true, true)  == true);
    CHECK(idle_consumer_present(false, false, true, true)  == false);
    // No consumers at all -> absent.
    CHECK(idle_consumer_present(false, false, true, false) == false);
    // SAFE DEFAULT: VR output enabled but unobservable (no HMD listen) -> treat
    // VR as present even though vr_peer_live is false, so we never idle on an
    // axis we cannot see.
    CHECK(idle_consumer_present(false, false, /*vr_observable=*/false,
                                /*has_vr_output=*/true) == true);
}

}  // namespace

int main() {
    test_enter_requires_dwell();
    test_presence_blocks_and_resets();
    test_resume_is_instant();
    test_zero_dwell_enters_immediately();
    test_vr_observable();
    test_consumer_present_fold_in();
    if (g_fail) {
        std::fprintf(stderr, "test_idle_evaluator: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_idle_evaluator: OK\n");
    return 0;
}
