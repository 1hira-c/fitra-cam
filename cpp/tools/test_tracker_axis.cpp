#include "tracking/tracker_axis.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include <crow.h>

#include "lift/keypoint_format.hpp"
#include "lift/skeleton_def.hpp"
#include "pipeline/fusion_pose.hpp"
#include "tracking/tracker_extract.hpp"

namespace {

int g_fail = 0;
#define CHECK(cond) do { \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL %s:%d %s\n", \
                     __FILE__, __LINE__, #cond); \
        ++g_fail; \
    } \
} while (0)

using fitra::pipeline::FusionPoseEventType;
using fitra::pipeline::FusionPoseSourceState;
using fitra::pipeline::TrackerAxisLineage;
using fitra::pipeline::TrackerAxisSourceJoint;
using fitra::tracking::TrackerAxisBus;
using fitra::tracking::TrackerPose;
using fitra::tracking::TrackerRole;

constexpr std::array<const char*, 6> kRoles{{
    "chest", "hips", "left_upper_leg", "right_upper_leg",
    "left_lower_leg", "right_lower_leg",
}};

std::array<TrackerPose, fitra::tracking::kTrackerCount> trackers() {
    std::array<TrackerPose, fitra::tracking::kTrackerCount> out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i].role = static_cast<TrackerRole>(i);
        out[i].valid = true;
        out[i].quat_wxyz = {1.0f, 0.0f, 0.0f, 0.0f};
    }
    return out;
}

TrackerAxisLineage lineage(std::uint64_t seq = 1) {
    TrackerAxisLineage out;
    out.source_sample_seq = seq;
    out.event_type = FusionPoseEventType::Pose;
    out.source_state = FusionPoseSourceState::Fresh;
    out.source_reason = "fresh";
    out.stream_id = "stream-a";
    out.subject_track_id = "subject-a";
    out.coordinate_epoch = 3;
    out.continuity_epoch = 5;
    out.source_publish_mono_ns = 900'000'000 + seq;
    out.capture.oldest_mono_ns = 800'000'000 + seq;
    out.capture.newest_mono_ns = 806'000'000 + seq;
    out.capture.semantics =
        fitra::camera::V4l2TimestampSemantics::MonotonicSoe;
    out.observed.fill(true);
    return out;
}

bool integer_number(const crow::json::rvalue& value) {
    return value.t() == crow::json::type::Number &&
           value.nt() != crow::json::num_type::Floating_point &&
           value.nt() !=
               crow::json::num_type::Double_precision_floating_point;
}

bool exact_keys(const crow::json::rvalue& value,
                std::vector<std::string> expected) {
    if (value.t() != crow::json::type::Object) return false;
    auto actual = value.keys();
    std::sort(actual.begin(), actual.end());
    std::sort(expected.begin(), expected.end());
    return actual == expected;
}

void test_exact_schema_and_axis_sign() {
    TrackerAxisBus bus{"stream-a", 3};
    const auto frame = bus.publish(trackers(), lineage());
    CHECK(frame.fresh);
    const auto root = crow::json::load(bus.make_json());
    CHECK(static_cast<bool>(root));
    if (!root) return;
    CHECK(std::string{root["protocol_version"].s()} ==
          "fitra_tracker_axis_v1");
    CHECK(std::string{root["source_state"].s()} == "fresh");
    CHECK(exact_keys(root, {
        "protocol_version", "delivery_seq", "source_sample_seq",
        "source_state", "stream_id", "subject_track_id",
        "coordinate_epoch", "continuity_epoch", "source_publish_mono_ns",
        "capture", "axes",
    }));
    for (const char* field : {
             "delivery_seq", "source_sample_seq", "coordinate_epoch",
             "continuity_epoch", "source_publish_mono_ns"}) {
        CHECK(integer_number(root[field]));
        CHECK(root[field].u() > 0);
    }
    CHECK(root["capture"].t() == crow::json::type::Object);
    CHECK(exact_keys(root["capture"], {
        "oldest_mono_ns", "newest_mono_ns", "timestamp_semantics",
    }));
    CHECK(integer_number(root["capture"]["oldest_mono_ns"]));
    CHECK(integer_number(root["capture"]["newest_mono_ns"]));
    CHECK(std::string{root["capture"]["timestamp_semantics"].s()} ==
          "monotonic_soe");
    CHECK(!root.has("boundary"));
    CHECK(!root.has("quat_wxyz"));
    CHECK(root["axes"].t() == crow::json::type::List);
    CHECK(root["axes"].size() == kRoles.size());
    for (std::size_t i = 0; i < kRoles.size(); ++i) {
        const auto& axis = root["axes"][i];
        CHECK(exact_keys(root["axes"][i], {
            "role", "availability", "observed_this_frame", "axis",
        }));
        CHECK(std::string{axis["role"].s()} == kRoles[i]);
        CHECK(std::string{axis["availability"].s()} == "fresh");
        CHECK(axis["observed_this_frame"].b());
        CHECK(axis["axis"].t() == crow::json::type::List);
        CHECK(axis["axis"].size() == 3);
        double norm2 = 0.0;
        for (std::size_t k = 0; k < 3; ++k) {
            CHECK(axis["axis"][k].t() == crow::json::type::Number);
            const double v = axis["axis"][k].d();
            CHECK(std::isfinite(v));
            norm2 += v * v;
        }
        CHECK(std::fabs(std::sqrt(norm2) - 1.0) < 1e-6);
    }
    // Identity orientation: torso body-right is -local X; leg direction is
    // proximal-to-distal local +Z.
    CHECK(std::fabs(root["axes"][0]["axis"][0].d() + 1.0) < 1e-6);
    CHECK(std::fabs(root["axes"][1]["axis"][0].d() + 1.0) < 1e-6);
    for (std::size_t i = 2; i < kRoles.size(); ++i) {
        CHECK(std::fabs(root["axes"][i]["axis"][2].d() - 1.0) < 1e-6);
    }
}

void test_predict_only_shape() {
    TrackerAxisBus bus{"stream-a", 3};
    auto source = lineage();
    source.observed[static_cast<std::size_t>(
        TrackerAxisSourceJoint::RightShoulder)] = false;
    bus.publish(trackers(), source);
    const auto root = crow::json::load(bus.make_json());
    CHECK(static_cast<bool>(root));
    if (!root) return;
    const auto& chest = root["axes"][0];
    CHECK(std::string{chest["availability"].s()} == "unavailable");
    CHECK(!chest["observed_this_frame"].b());
    CHECK(chest["axis"].t() == crow::json::type::Null);
    CHECK(std::string{root["axes"][1]["availability"].s()} == "fresh");
}

void test_invalid_post_filter_axis_rejected() {
    TrackerAxisBus bus{"stream-a", 3};
    auto values = trackers();
    values[static_cast<std::size_t>(TrackerRole::Waist)].quat_wxyz[0] =
        std::numeric_limits<float>::quiet_NaN();
    bus.publish(values, lineage());
    const auto root = crow::json::load(bus.make_json());
    CHECK(static_cast<bool>(root));
    if (!root) return;
    CHECK(std::string{root["axes"][1]["availability"].s()} ==
          "unavailable");
    CHECK(!root["axes"][1]["observed_this_frame"].b());
    CHECK(root["axes"][1]["axis"].t() == crow::json::type::Null);
}

void test_long_sitting_legs_stay_horizontal_after_first_smoothing_sample() {
    using namespace fitra::lift;
    set_active_keypoint_format(KeypointFormat::Halpe26);
    fitra::infer::Skeleton3D skeleton;
    skeleton.kp_count = 26;

    // Long sitting: both straight legs extend in world +Y at one height.
    // Exercise extraction, the first One-Euro sample, and the public wire
    // basis instead of testing only hand-written quaternions.
    auto set_joint = [&](std::size_t joint, float x, float y, float z) {
        auto& value = skeleton.joints[joint];
        value.x = x;
        value.y = y;
        value.z = z;
        value.score = 1.0f;
        value.valid = true;
    };
    set_joint(kHalpeLeftHip, 0.10f, 0.00f, 0.10f);
    set_joint(kHalpeLeftKnee, 0.10f, 0.40f, 0.10f);
    set_joint(kHalpeLeftAnkle, 0.10f, 0.80f, 0.10f);
    set_joint(kHalpeRightHip, -0.10f, 0.00f, 0.10f);
    set_joint(kHalpeRightKnee, -0.10f, 0.40f, 0.10f);
    set_joint(kHalpeRightAnkle, -0.10f, 0.80f, 0.10f);

    auto values = fitra::tracking::extract_trackers(skeleton);
    std::array<cv::Vec4f, fitra::tracking::kTrackerCount> previous{};
    previous.fill(cv::Vec4f{1.0f, 0.0f, 0.0f, 0.0f});
    fitra::tracking::QuatSmoothingContext smoothing;
    fitra::tracking::apply_quat_smoothing(
        values, previous, smoothing, fitra::tracking::OneEuroParams{},
        1.0f / 60.0f, 1.0f / 60.0f);

    TrackerAxisBus bus{"stream-a", 3};
    const auto frame = bus.publish(values, lineage());
    CHECK(frame.fresh);
    for (const auto role : {
             fitra::tracking::TrackerAxisRole::LeftUpperLeg,
             fitra::tracking::TrackerAxisRole::RightUpperLeg,
             fitra::tracking::TrackerAxisRole::LeftLowerLeg,
             fitra::tracking::TrackerAxisRole::RightLowerLeg}) {
        const auto& value = frame.axes[static_cast<std::size_t>(role)];
        CHECK(value.availability ==
                  fitra::tracking::TrackerAxisAvailability::Fresh);
        CHECK(value.axis.has_value());
        if (value.axis) {
            CHECK(std::fabs((*value.axis)[0]) < 1.0e-5);
            CHECK(std::fabs((*value.axis)[1] - 1.0) < 1.0e-5);
            CHECK(std::fabs((*value.axis)[2]) < 1.0e-5);
        }
    }
}

void test_timestamp_fail_closed() {
    TrackerAxisBus bus{"stream-a", 3};
    auto source = lineage();
    source.capture.semantics =
        fitra::camera::V4l2TimestampSemantics::Unavailable;
    source.capture.oldest_mono_ns.reset();
    source.capture.newest_mono_ns.reset();
    bus.publish(trackers(), source);
    const auto root = crow::json::load(bus.make_json());
    CHECK(static_cast<bool>(root));
    if (!root) return;
    CHECK(std::string{root["source_state"].s()} == "boundary");
    CHECK(std::string{root["boundary"].s()} == "unsupported_timestamp");
    CHECK(exact_keys(root, {
        "protocol_version", "delivery_seq", "source_sample_seq",
        "source_state", "stream_id", "subject_track_id",
        "coordinate_epoch", "continuity_epoch", "source_publish_mono_ns",
        "boundary",
    }));
    CHECK(!root.has("capture"));
    CHECK(!root.has("axes"));

    TrackerAxisBus eof_bus{"stream-a", 3};
    auto eof_source = lineage();
    eof_source.capture.semantics =
        fitra::camera::V4l2TimestampSemantics::MonotonicEof;
    eof_bus.publish(trackers(), eof_source);
    const auto eof_root = crow::json::load(eof_bus.make_json());
    CHECK(static_cast<bool>(eof_root));
    if (eof_root) {
        CHECK(std::string{eof_root["source_state"].s()} == "fresh");
        CHECK(std::string{eof_root["capture"]["timestamp_semantics"].s()} ==
              "monotonic_eof");
    }
}

void test_boundary_order() {
    TrackerAxisBus bus{"stream-a", 3, 16};
    bus.publish(trackers(), lineage(1));
    (void)bus.drain_pending_json();

    auto changed = lineage(2);
    changed.stream_id = "stream-b";
    changed.subject_track_id = "subject-b";
    changed.coordinate_epoch = 4;
    changed.continuity_epoch = 6;
    bus.publish(trackers(), changed);
    const auto docs = bus.drain_pending_json();
    CHECK(docs.size() == 5);
    constexpr std::array<const char*, 4> expected{{
        "stream_changed", "subject_changed", "coordinate_changed",
        "continuity_reset",
    }};
    std::uint64_t previous_delivery = 0;
    for (std::size_t i = 0; i < docs.size(); ++i) {
        const auto root = crow::json::load(docs[i]);
        CHECK(static_cast<bool>(root));
        if (!root) continue;
        CHECK(root["delivery_seq"].u() > previous_delivery);
        previous_delivery = root["delivery_seq"].u();
        if (i < expected.size()) {
            CHECK(std::string{root["source_state"].s()} == "boundary");
            CHECK(std::string{root["boundary"].s()} == expected[i]);
            CHECK(!root.has("capture"));
            CHECK(!root.has("axes"));
        } else {
            CHECK(std::string{root["source_state"].s()} == "fresh");
        }
    }
}

void test_overflow_continuity_reset() {
    TrackerAxisBus bus{"stream-a", 3, 2};
    auto first = lineage(1);
    first.event_type = FusionPoseEventType::Boundary;
    first.source_state = FusionPoseSourceState::Unavailable;
    first.source_reason = "person_lost";
    bus.publish(trackers(), first);

    auto second = first;
    second.source_sample_seq = 2;
    second.source_publish_mono_ns += 1;
    second.source_state = FusionPoseSourceState::UnsupportedTopology;
    bus.publish(trackers(), second);

    auto third = second;
    third.source_sample_seq = 3;
    third.source_publish_mono_ns += 1;
    third.coordinate_epoch = 4;
    third.source_state = FusionPoseSourceState::EpochChanged;
    bus.publish(trackers(), third);

    const auto docs = bus.drain_pending_json();
    CHECK(docs.size() == 1);
    if (!docs.empty()) {
        const auto root = crow::json::load(docs.front());
        CHECK(static_cast<bool>(root));
        if (root) {
            CHECK(std::string{root["boundary"].s()} == "continuity_reset");
            CHECK(root["continuity_epoch"].u() > first.continuity_epoch);
        }
    }
}

void test_lineage_handoff_preserves_boundaries() {
    fitra::pipeline::TrackerAxisLineageBus bus{2};
    auto first = lineage(1);
    first.event_type = FusionPoseEventType::Boundary;
    first.source_state = FusionPoseSourceState::Unavailable;
    first.source_reason = "first";
    bus.publish(first);
    auto second = first;
    second.source_sample_seq = 2;
    second.source_publish_mono_ns += 1;
    second.source_reason = "second";
    bus.publish(second);
    auto ordered = bus.drain_boundaries();
    CHECK(ordered.size() == 2);
    if (ordered.size() == 2) {
        CHECK(ordered[0].source_sample_seq == 1);
        CHECK(ordered[1].source_sample_seq == 2);
    }

    bus.publish(first);
    bus.publish(second);
    auto third = second;
    third.source_sample_seq = 3;
    third.source_publish_mono_ns += 1;
    third.source_reason = "third";
    const auto collapsed = bus.publish(third);
    CHECK(collapsed.source_state == FusionPoseSourceState::ContinuityReset);
    CHECK(collapsed.continuity_epoch > third.continuity_epoch);
    const auto reset = bus.drain_boundaries();
    CHECK(reset.size() == 1);
    if (!reset.empty()) {
        CHECK(reset.front().source_state ==
              FusionPoseSourceState::ContinuityReset);
    }
}

void test_stale_snapshot_cannot_reopen_after_boundary() {
    TrackerAxisBus bus{"stream-a", 3};
    auto boundary = lineage(2);
    boundary.event_type = FusionPoseEventType::Boundary;
    boundary.source_state = FusionPoseSourceState::Unavailable;
    boundary.source_reason = "person_lost";
    bus.publish(trackers(), boundary);
    const auto before = bus.snapshot();
    CHECK(!before.fresh);
    bus.publish(trackers(), lineage(1));
    const auto after = bus.snapshot();
    CHECK(!after.fresh);
    CHECK(after.delivery_seq == before.delivery_seq);
    CHECK(after.source_sample_seq == before.source_sample_seq);
}

void test_clock_pong() {
    const auto root = crow::json::load(fitra::pipeline::make_clock_sync_pong(
        7, 11, 13, 17));
    CHECK(static_cast<bool>(root));
    if (!root) return;
    CHECK(std::string{root["type"].s()} == "clock_sync_pong");
    for (const char* field : {
             "nonce", "client_send_mono_ns", "server_receive_mono_ns",
             "server_send_mono_ns"}) {
        CHECK(integer_number(root[field]));
    }
    CHECK(root["nonce"].u() == 7);
    CHECK(root["client_send_mono_ns"].u() == 11);
}

}  // namespace

int main() {
    test_exact_schema_and_axis_sign();
    test_predict_only_shape();
    test_invalid_post_filter_axis_rejected();
    test_long_sitting_legs_stay_horizontal_after_first_smoothing_sample();
    test_timestamp_fail_closed();
    test_boundary_order();
    test_overflow_continuity_reset();
    test_lineage_handoff_preserves_boundaries();
    test_stale_snapshot_cannot_reopen_after_boundary();
    test_clock_pong();
    if (g_fail) {
        std::fprintf(stderr, "test_tracker_axis: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_tracker_axis: OK\n");
    return 0;
}
