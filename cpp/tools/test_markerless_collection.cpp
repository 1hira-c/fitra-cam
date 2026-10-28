#include "markerless_test_scene.hpp"
#include "lift/markerless_collection.hpp"

#include <cstdio>
#include <limits>
#include <string>

namespace {
using namespace fitra::lift;
int failures = 0;
#define CHECK(condition) do { if (!(condition)) { \
    std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #condition); \
    ++failures; } } while (0)

void bounded_ownership_and_duplicates() {
    auto scene = markerless_test::make_scene({});
    MarkerlessCollectionOptions options;
    options.max_frame_sets = 3;
    MarkerlessCollector collector(scene.intrinsics, options);
    // An external owner may reuse its calibration and frame storage.
    scene.intrinsics.cameras[0].intrinsics.K.at<double>(0, 0) = 0;
    std::string reason;
    auto first = scene.samples[0];
    CHECK(collector.ingest(first, reason) == MarkerlessIngest::Accepted);
    const double x = first.frames[0].keypoints[0].pixel.x;
    first.frames[0].keypoints[0].pixel.x += 10;
    CHECK(collector.frame_sets()[0].frames[0].keypoints[0].pixel.x == x);
    CHECK(collector.ingest(scene.samples[0], reason) == MarkerlessIngest::Rejected);
    CHECK(!reason.empty());
    CHECK(collector.frame_sets().size() == 1);
    for (std::size_t i = 1; i < 6; ++i)
        CHECK(collector.ingest(scene.samples[i], reason) == MarkerlessIngest::Accepted);
    CHECK(collector.frame_sets().size() == 3);
    CHECK(collector.frame_sets().front().sample_id == scene.samples[3].sample_id);
    CHECK(collector.stats().evicted == 3);
    CHECK(collector.stats().accepted == 6);
    // Eviction must not allow old frames back into the current session.
    CHECK(collector.ingest(scene.samples[0], reason) == MarkerlessIngest::Reset);
    CHECK(collector.frame_sets().empty());
    CHECK(collector.stats().resets == 1);
}

void boundaries_clear_history() {
    const auto scene = markerless_test::make_scene({});
    for (int scenario = 0; scenario < 7; ++scenario) {
        MarkerlessCollector collector(scene.intrinsics);
        std::string reason;
        CHECK(collector.ingest(scene.samples[0], reason) == MarkerlessIngest::Accepted);
        auto next = scene.samples[1];
        if (scenario == 0) next.continuity_id = "new-person-generation";
        if (scenario == 1) ++next.frames[1].stream_epoch;
        if (scenario == 2) {
            for (auto& frame : next.frames)
                frame.timestamp_kind = MarkerlessTimestampKind::KernelMonotonicEof;
        }
        if (scenario == 3) --next.frames[0].width;
        if (scenario == 4) next.frames[0].person_count = 0;
        if (scenario == 5) next.frames[0].capture_mono_ns = 1;
        if (scenario == 6)
            next.frames[0].timestamp_kind = MarkerlessTimestampKind::Unknown;
        CHECK(collector.ingest(next, reason) == MarkerlessIngest::Reset);
        CHECK(collector.frame_sets().empty());
        CHECK(collector.stats().resets == 1);
        CHECK(collector.ingest(scene.samples[2], reason) == MarkerlessIngest::Accepted);
        CHECK(collector.frame_sets().size() == 1);
        CHECK(collector.frame_sets().front().sample_id == scene.samples[2].sample_id);
    }
}

void clock_skew_and_validation() {
    auto scene = markerless_test::make_scene({});
    MarkerlessCollector collector(scene.intrinsics);
    std::string reason;
    auto sample = scene.samples[0];
    for (auto& frame : sample.frames) frame.capture_mono_ns = 1000000000;
    sample.frames.back().capture_mono_ns += 20000000;
    CHECK(collector.ingest(sample, reason) == MarkerlessIngest::Accepted);
    CHECK(collector.stats().max_accepted_skew_ms == 20.0);
    auto bad = scene.samples[1];
    for (auto& frame : bad.frames) frame.capture_mono_ns = 2000000000;
    bad.frames.back().capture_mono_ns += 20000001;
    CHECK(collector.ingest(bad, reason) == MarkerlessIngest::Reset);
    CHECK(collector.frame_sets().empty());

    auto valid = scene.samples[2];
    CHECK(collector.ingest(valid, reason) == MarkerlessIngest::Accepted);
    auto repeated = scene.samples[3];
    repeated.frames[0].source_seq = valid.frames[0].source_seq;
    CHECK(collector.ingest(repeated, reason) == MarkerlessIngest::Rejected);
    CHECK(collector.frame_sets().size() == 1);

    auto invalid = scene.samples[4];
    invalid.frames[0].keypoints[0].pixel.x = std::numeric_limits<double>::quiet_NaN();
    CHECK(!validate_markerless_inputs(scene.intrinsics, {invalid}).empty());
    auto duplicated = scene.samples[5];
    duplicated.frames[0].keypoints.push_back(duplicated.frames[0].keypoints[0]);
    CHECK(!validate_markerless_inputs(scene.intrinsics, {duplicated}).empty());
    auto mixed = scene.samples[6];
    mixed.frames[0].timestamp_kind = MarkerlessTimestampKind::KernelMonotonicSoe;
    CHECK(!validate_markerless_inputs(scene.intrinsics, {mixed}).empty());
    CHECK(validate_markerless_inputs(scene.intrinsics, scene.samples).empty());
}

void invalid_configuration() {
    auto scene = markerless_test::make_scene({});
    bool threw = false;
    try {
        MarkerlessCollectionOptions options;
        options.max_frame_sets = 0;
        MarkerlessCollector collector(scene.intrinsics, options);
    } catch (const std::exception&) { threw = true; }
    CHECK(threw);
    scene.intrinsics.cameras[1].id = scene.intrinsics.cameras[0].id;
    CHECK(!validate_markerless_inputs(scene.intrinsics, {}).empty());
}
}  // namespace

int main() {
    bounded_ownership_and_duplicates();
    boundaries_clear_history();
    clock_skew_and_validation();
    invalid_configuration();
    std::printf("markerless collection: %d failures\n", failures);
    return failures ? 1 : 0;
}
