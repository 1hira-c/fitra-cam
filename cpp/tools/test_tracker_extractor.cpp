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

void check_all_invalid(const fitra::slimevr::SlimeTrackerSnapshot& snap,
                       const std::string& label) {
    for (std::size_t i = 0; i < snap.trackers.size(); ++i) {
        check(!snap.trackers[i].valid,
              label + ": tracker " + std::to_string(i) + " should be invalid");
    }
}

fitra::slimevr::TrackerExtractorOptions fast_event_options() {
    fitra::slimevr::TrackerExtractorOptions opts;
    opts.extract_rate_hz = 200.0;  // short timeout keeps thread tests quick
    opts.stale_clear_after_ms = 40;
    opts.event_driven = true;
    opts.one_euro = false;
    opts.st_filter = false;
    opts.stats_window = 4;
    return opts;
}

void test_event_driven_skips_timeout_duplicates() {
    fitra::pipeline::Skeleton3DBus skel_bus;
    fitra::slimevr::SlimeTrackerBus tracker_bus;

    fitra::slimevr::TrackerExtractorOptions opts = fast_event_options();
    opts.stale_clear_after_ms = 250;  // keep this test below stale-clear threshold
    opts.event_driven = true;

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
    check(second.stream.mode == fitra::slimevr::SlimeTrackerStreamMode::Event,
          "tracker stream mode must be event");
    check(second.stream.source_pose_seq == 102,
          "tracker stream must expose the source Skeleton3DSnapshot seq");
    check(second.stream.suppressed_wakeups > first.stream.suppressed_wakeups,
          "event-driven timeouts should be visible as suppressed_wakeups");
    check(second.stream.refiltered_duplicates == 0,
          "event-driven mode must not report fixed-rate refiltered duplicates");
    check(second.stream.source_stale == false,
          "fresh source update must not be reported as stale");

    extractor.stop();
}

void test_event_driven_initial_stall_publishes_stale_diagnostics() {
    fitra::pipeline::Skeleton3DBus skel_bus;
    fitra::slimevr::SlimeTrackerBus tracker_bus;

    fitra::slimevr::TrackerExtractor extractor(
        skel_bus, tracker_bus, fast_event_options());
    extractor.start();

    fitra::slimevr::SlimeTrackerSnapshot stale;
    const bool ok = wait_until([&] {
        stale = tracker_bus.snapshot();
        return stale.has_data && stale.stream.source_stale;
    }, std::chrono::milliseconds(500));
    check(ok, "initial upstream stall should publish tracker_stream diagnostics");
    check(stale.stream.source_update_seq == 0,
          "initial stale publish must not invent a source update seq");
    check(stale.stream.stale_clears == 1,
          "initial stale publish should count one stale clear");
    check_all_invalid(stale, "initial stale publish");

    extractor.stop();
}

void test_event_driven_stale_clear_is_single_shot_and_recovers() {
    fitra::pipeline::Skeleton3DBus skel_bus;
    fitra::slimevr::SlimeTrackerBus tracker_bus;

    fitra::slimevr::TrackerExtractor extractor(
        skel_bus, tracker_bus, fast_event_options());
    extractor.start();

    skel_bus.update(make_source_snapshot(/*seq=*/201));
    const auto first = wait_for_source_seq(tracker_bus, /*source_update_seq=*/1);

    fitra::slimevr::SlimeTrackerSnapshot stale;
    const bool stale_ok = wait_until([&] {
        stale = tracker_bus.snapshot();
        return stale.has_data && stale.seq == first.seq + 1 && stale.stream.source_stale;
    }, std::chrono::milliseconds(500));
    check(stale_ok, "quiet source should publish exactly one stale clear");
    check(stale.stream.stale_clears == first.stream.stale_clears + 1,
          "stale clear counter should increment once");
    check(stale.stream.source_update_seq == first.stream.source_update_seq,
          "stale clear should not advance source update seq");
    check_all_invalid(stale, "stale clear");

    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    const auto still_stale = tracker_bus.snapshot();
    check(still_stale.seq == stale.seq, "stale clear must be single-shot");

    skel_bus.update(make_source_snapshot(/*seq=*/202));
    const auto recovered = wait_for_source_seq(tracker_bus, /*source_update_seq=*/2);
    check(recovered.seq == stale.seq + 1,
          "fresh source after stale clear should publish one recovery frame");
    check(recovered.stream.source_stale == false,
          "fresh source after stale clear must clear source_stale");

    extractor.stop();
}

void test_fixed_mode_reports_refiltered_duplicates_and_stale_flag() {
    fitra::pipeline::Skeleton3DBus skel_bus;
    fitra::slimevr::SlimeTrackerBus tracker_bus;

    auto opts = fast_event_options();
    opts.event_driven = false;
    opts.stale_clear_after_ms = 20;
    fitra::slimevr::TrackerExtractor extractor(skel_bus, tracker_bus, opts);
    extractor.start();

    skel_bus.update(make_source_snapshot(/*seq=*/301));
    (void)wait_for_source_seq(tracker_bus, /*source_update_seq=*/1);

    fitra::slimevr::SlimeTrackerSnapshot snap;
    const bool ok = wait_until([&] {
        snap = tracker_bus.snapshot();
        return snap.has_data &&
               snap.stream.refiltered_duplicates > 0 &&
               snap.stream.source_stale;
    }, std::chrono::milliseconds(500));
    check(ok, "fixed mode should report duplicate refilters and stale source age");
    check(snap.stream.mode == fitra::slimevr::SlimeTrackerStreamMode::Fixed,
          "tracker stream mode must be fixed");
    check(snap.stream.suppressed_wakeups == 0,
          "fixed mode should not report event suppressed wakeups");

    extractor.stop();
}

}  // namespace

int main() {
    try {
        test_event_driven_skips_timeout_duplicates();
        std::printf("[ok] event-driven extractor skips timeout duplicates\n");
        test_event_driven_initial_stall_publishes_stale_diagnostics();
        std::printf("[ok] event-driven initial stall publishes stale diagnostics\n");
        test_event_driven_stale_clear_is_single_shot_and_recovers();
        std::printf("[ok] event-driven stale clear is single-shot and recovers\n");
        test_fixed_mode_reports_refiltered_duplicates_and_stale_flag();
        std::printf("[ok] fixed mode reports refiltered duplicates and stale flag\n");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what());
        return 1;
    }
}
