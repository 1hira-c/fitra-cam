// TrackerExtractor producer cadence tests.
//
// Event-driven mode must consume each Skeleton3DBus update once. In particular,
// wait_for_update() timeouts must not re-run the smoothing/filter stack against
// the same stale Skeleton3DSnapshot, because that fabricates duplicate input
// frames and distorts velocity-dependent filters.

#include <chrono>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>
#include <thread>

#include "pipeline/snapshot.hpp"
#include "slimevr/tracker_extractor.hpp"

namespace {

void check(bool cond, const std::string& msg) {
    if (!cond) throw std::runtime_error(msg);
}

fitra::pipeline::Skeleton3DSnapshot make_source_snapshot(std::uint64_t seq) {
    fitra::pipeline::Skeleton3DSnapshot snap;
    snap.seq = seq;
    snap.stats.enabled = true;
    snap.stats.tri_fps = 30.0;
    return snap;
}

template <typename Pred>
bool wait_until(Pred pred, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return pred();
}

fitra::slimevr::SlimeTrackerSnapshot
wait_for_source_seq(fitra::slimevr::SlimeTrackerBus& bus,
                    std::uint64_t source_update_seq) {
    fitra::slimevr::SlimeTrackerSnapshot snap;
    const bool ok = wait_until([&] {
        snap = bus.snapshot();
        return snap.has_data && snap.stream.source_update_seq == source_update_seq;
    }, std::chrono::milliseconds(500));
    check(ok, "timed out waiting for tracker source update seq " +
                  std::to_string(source_update_seq));
    return snap;
}

void test_event_driven_skips_timeout_duplicates() {
    fitra::pipeline::Skeleton3DBus skel_bus;
    fitra::slimevr::SlimeTrackerBus tracker_bus;

    fitra::slimevr::TrackerExtractorOptions opts;
    opts.extract_rate_hz = 200.0;  // short timeout keeps the regression quick
    opts.event_driven = true;
    opts.one_euro = false;
    opts.st_filter = false;
    opts.stats_window = 4;

    fitra::slimevr::TrackerExtractor extractor(skel_bus, tracker_bus, opts);
    extractor.start();

    skel_bus.update(make_source_snapshot(/*seq=*/101));
    const auto first = wait_for_source_seq(tracker_bus, /*source_update_seq=*/1);
    const auto first_bus_seq = first.seq;

    // Several wait_for_update() timeouts pass here. The extractor should count
    // them as duplicate opportunities, but must not publish another tracker
    // frame until Skeleton3DBus::update() provides a fresh source snapshot.
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    const auto after_timeout = tracker_bus.snapshot();
    check(after_timeout.seq == first_bus_seq,
          "event-driven timeout republished a stale tracker snapshot");

    skel_bus.update(make_source_snapshot(/*seq=*/102));
    const auto second = wait_for_source_seq(tracker_bus, /*source_update_seq=*/2);
    check(second.seq == first_bus_seq + 1,
          "fresh source update should produce exactly one additional tracker publish");
    check(second.stream.mode == "event", "tracker stream mode must be event");
    check(second.stream.source_pose_seq == 102,
          "tracker stream must expose the source Skeleton3DSnapshot seq");
    check(second.stream.duplicate_ticks > first.stream.duplicate_ticks,
          "event-driven timeouts should be visible as duplicate_ticks");
    check(second.stream.source_stale == false,
          "fresh source update must not be reported as stale");

    extractor.stop();
}

}  // namespace

int main() {
    try {
        test_event_driven_skips_timeout_duplicates();
        std::printf("[ok] event-driven extractor skips timeout duplicates\n");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what());
        return 1;
    }
}
