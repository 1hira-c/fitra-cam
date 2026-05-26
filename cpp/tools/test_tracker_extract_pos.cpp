// test_tracker_extract_pos — exercise apply_pos_smoothing.
//
// TrackerExtractor maintains a position EMA (mirroring the existing per-tracker
// quat smoothing). We verify:
//   1. pos_smooth=1.0 passes raw curr.pos through unchanged
//   2. pos_smooth=0.5 converges toward the target after a few frames
//   3. valid=false holds prev_pos (curr.pos snaps to prev, prev untouched)
//   4. dropout/recovery does NOT push the cached position back through (0,0,0)
//      — important because prev_pos is the only "last good" cache available
//      to the publisher during a brief occlusion.
//
// All checks use cv::Vec3f and the same shape of helpers as
// test_tracker_extract.

#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

#include "slimevr/tracker_extract.hpp"

namespace {

constexpr float kEps = 1.0e-5f;

using fitra::slimevr::SlimeTracker;
using fitra::slimevr::TrackerRole;
using fitra::slimevr::apply_pos_smoothing;
using fitra::slimevr::kTrackerCount;

void check(bool cond, const std::string& msg) {
    if (!cond) throw std::runtime_error(msg);
}

void check_close(float got, float want, const std::string& label, float eps = kEps) {
    if (std::abs(got - want) > eps) {
        char buf[200];
        std::snprintf(buf, sizeof(buf), "%s: got=%.6f want=%.6f", label.c_str(), got, want);
        throw std::runtime_error(buf);
    }
}

void check_vec3(const cv::Vec3f& got, const cv::Vec3f& want, const std::string& label,
                float eps = kEps) {
    for (int i = 0; i < 3; ++i) check_close(got[i], want[i], label + "[" + std::to_string(i) + "]", eps);
}

// All-valid trackers at a uniform pos for the smoothing tests. role values
// don't matter for apply_pos_smoothing.
std::array<SlimeTracker, kTrackerCount> make_trackers(const cv::Vec3f& pos, bool valid) {
    std::array<SlimeTracker, kTrackerCount> ts{};
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        ts[i].role  = static_cast<TrackerRole>(i);
        ts[i].pos   = pos;
        ts[i].valid = valid;
    }
    return ts;
}

void test_no_smooth_passthrough() {
    auto ts = make_trackers({1.0f, 2.0f, 3.0f}, /*valid=*/true);
    std::array<cv::Vec3f, kTrackerCount> prev{};  // zero-init
    apply_pos_smoothing(ts, prev, /*base_alpha=*/1.0f);
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        check_vec3(ts[i].pos, {1.0f, 2.0f, 3.0f}, "no-smooth.curr[" + std::to_string(i) + "]");
        check_vec3(prev[i],   {1.0f, 2.0f, 3.0f}, "no-smooth.prev[" + std::to_string(i) + "]");
    }
}

void test_smooth_convergence() {
    // Target = (4, 0, 0), prev starts at (0,0,0), alpha=0.5.
    // 1 frame:  prev = 2.0     ; rel error = 50%
    // 2 frames: prev = 3.0     ; rel error = 25%
    // 6 frames: prev = 4*(1-0.5^6) = 3.9375; rel error ≈ 1.6% (< 5% threshold)
    std::array<cv::Vec3f, kTrackerCount> prev{};
    const cv::Vec3f target{4.0f, 0.0f, 0.0f};
    for (int frame = 0; frame < 6; ++frame) {
        auto ts = make_trackers(target, /*valid=*/true);
        apply_pos_smoothing(ts, prev, /*base_alpha=*/0.5f);
    }
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        const float err = std::abs(prev[i][0] - target[0]) / std::abs(target[0]);
        check(err < 0.05f, "convergence rel err > 5% at i=" + std::to_string(i));
    }
}

void test_invalid_holds_prev() {
    // Seed prev to a known good pos, then submit an invalid frame at a
    // wildly different raw pos. Smoothed curr should be the prev (not the
    // raw), and prev itself should stay unchanged.
    std::array<cv::Vec3f, kTrackerCount> prev{};
    for (auto& p : prev) p = cv::Vec3f{0.5f, 0.6f, 0.7f};

    auto ts = make_trackers({99.0f, -99.0f, 0.0f}, /*valid=*/false);
    apply_pos_smoothing(ts, prev, /*base_alpha=*/0.5f);

    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        check_vec3(ts[i].pos, {0.5f, 0.6f, 0.7f}, "invalid.curr[" + std::to_string(i) + "]");
        check_vec3(prev[i],   {0.5f, 0.6f, 0.7f}, "invalid.prev[" + std::to_string(i) + "]");
    }
}

void test_dropout_recovery() {
    // Realistic occlusion timeline:
    //   frame 1-3: valid pos (1,0,0), prev converges
    //   frame 4-5: invalid (occluded) — prev must NOT drift back to 0
    //   frame 6-8: valid pos (1.5,0,0) — converges from the held prev,
    //              not from (0,0,0)
    std::array<cv::Vec3f, kTrackerCount> prev{};
    const float alpha = 0.5f;

    // Step 1: valid at (1,0,0)
    for (int f = 0; f < 3; ++f) {
        auto ts = make_trackers({1.0f, 0.0f, 0.0f}, /*valid=*/true);
        apply_pos_smoothing(ts, prev, alpha);
    }
    // After 3 frames with alpha=0.5 toward 1.0 from 0: prev = 1 - 0.5^3 = 0.875
    check_close(prev[0][0], 0.875f, "dropout.step1.prev[0].x");

    // Step 2: invalid (occluded) — must hold prev unchanged
    for (int f = 0; f < 2; ++f) {
        auto ts = make_trackers({999.0f, 999.0f, 999.0f}, /*valid=*/false);
        apply_pos_smoothing(ts, prev, alpha);
    }
    check_close(prev[0][0], 0.875f, "dropout.step2.prev held");
    check_close(prev[0][1], 0.0f,   "dropout.step2.prev y held");

    // Step 3: recover at (1.5,0,0) — convergence starts from 0.875, NOT 0
    auto ts = make_trackers({1.5f, 0.0f, 0.0f}, /*valid=*/true);
    apply_pos_smoothing(ts, prev, alpha);
    // After 1 frame with alpha=0.5: prev = 0.875 + 0.5*(1.5 - 0.875) = 1.1875.
    // If the cache had snapped back to 0, we'd see 0.75 instead — the test
    // distinguishes "held" from "reset to 0".
    check_close(prev[0][0], 1.1875f, "dropout.phase3.prev recovers from held");
}

}  // namespace

int main() {
    try {
        test_no_smooth_passthrough();
        test_smooth_convergence();
        test_invalid_holds_prev();
        test_dropout_recovery();
        std::puts("test_tracker_extract_pos ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_tracker_extract_pos failed: %s\n", e.what());
        return 1;
    }
}
