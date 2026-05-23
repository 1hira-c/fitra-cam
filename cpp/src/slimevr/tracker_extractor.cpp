#include "slimevr/tracker_extractor.hpp"

#include <algorithm>
#include <chrono>

#include "lift/keypoint_format.hpp"

namespace fitra::slimevr {

namespace {

const infer::Skeleton3D* pick_skeleton(const pipeline::Skeleton3DSnapshot& snap) {
    if (snap.persons.empty()) return nullptr;
    return &snap.persons.front();
}

}  // namespace

TrackerExtractor::TrackerExtractor(pipeline::Skeleton3DBus& skeleton_bus,
                                   SlimeTrackerBus&         tracker_bus,
                                   TrackerExtractorOptions  opts)
    : skel_bus_(skeleton_bus), tracker_bus_(tracker_bus), opts_(opts) {
    for (auto& q : prev_quat_) q = cv::Vec4f{1.0f, 0.0f, 0.0f, 0.0f};
}

TrackerExtractor::~TrackerExtractor() {
    stop();
}

void TrackerExtractor::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) return;
    stop_.store(false, std::memory_order_relaxed);
    thread_ = std::thread([this]() { run_loop(); });
}

void TrackerExtractor::stop() {
    if (!running_.load()) return;
    stop_.store(true, std::memory_order_relaxed);
    if (thread_.joinable()) thread_.join();
    running_.store(false);
}

void TrackerExtractor::run_loop() {
    using clk = std::chrono::steady_clock;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, opts_.extract_rate_hz));
    const auto period_d = std::chrono::duration_cast<clk::duration>(period);

    auto next = clk::now() + period_d;

    while (!stop_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_until(next);
        next += period_d;

        auto snap = skel_bus_.snapshot();
        if (!snap.stats.enabled) continue;
        const infer::Skeleton3D* sk = pick_skeleton(snap);
        if (!sk) continue;
        if (fitra::lift::active_keypoint_format() != fitra::lift::KeypointFormat::Halpe26) {
            // extract_trackers asserts Halpe26 internally; skip silently if the
            // pipeline is in COCO17 mode so we don't crash the extractor thread.
            continue;
        }

        auto trackers = extract_trackers(*sk);
        apply_quat_smoothing(trackers, prev_quat_, opts_.quat_smooth);
        tracker_bus_.publish(trackers);
    }
}

}  // namespace fitra::slimevr
