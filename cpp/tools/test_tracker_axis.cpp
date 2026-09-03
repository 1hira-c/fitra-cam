#include "tracking/tracker_axis.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <crow.h>

#include "lift/floor_contact_stabilizer.hpp"
#include "lift/keypoint_format.hpp"
#include "lift/skeleton_def.hpp"
#include "pipeline/fusion_pose.hpp"
#include "pipeline/lifecycle_filter_history.hpp"
#include "pipeline/snapshot.hpp"
#include "tracking/tracker_extract.hpp"
#include "tracking/tracker_extractor.hpp"

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

void set_joint(fitra::infer::Skeleton3D& skeleton, std::size_t joint,
               float x, float y, float z) {
    auto& value = skeleton.joints[joint];
    value.x = x;
    value.y = y;
    value.z = z;
    value.score = 1.0f;
    value.valid = true;
}

fitra::infer::Skeleton3D lifecycle_skeleton() {
    fitra::infer::Skeleton3D skeleton;
    skeleton.kp_count = 26;
    set_joint(skeleton, 19, 0.0f, 0.00f, 0.90f);
    set_joint(skeleton, 11, 0.1f, 0.00f, 0.90f);
    set_joint(skeleton, 12, -0.1f, 0.00f, 0.90f);
    set_joint(skeleton, 18, 0.0f, 0.00f, 1.45f);
    set_joint(skeleton, 5, 0.18f, 0.00f, 1.42f);
    set_joint(skeleton, 6, -0.18f, 0.00f, 1.42f);
    set_joint(skeleton, 7, 0.45f, 0.02f, 1.42f);
    set_joint(skeleton, 8, -0.45f, 0.02f, 1.42f);
    set_joint(skeleton, 9, 0.72f, 0.05f, 1.27f);
    set_joint(skeleton, 10, -0.72f, 0.05f, 1.27f);
    set_joint(skeleton, 13, 0.1f, 0.10f, 0.45f);
    set_joint(skeleton, 14, -0.1f, 0.10f, 0.45f);
    set_joint(skeleton, 15, 0.1f, 0.05f, 0.05f);
    set_joint(skeleton, 16, -0.1f, 0.05f, 0.05f);
    set_joint(skeleton, 20, 0.1f, 0.17f, 0.00f);
    set_joint(skeleton, 21, -0.1f, 0.17f, 0.00f);
    set_joint(skeleton, 22, 0.07f, 0.17f, 0.00f);
    set_joint(skeleton, 23, -0.07f, 0.17f, 0.00f);
    set_joint(skeleton, 24, 0.1f, 0.02f, 0.00f);
    set_joint(skeleton, 25, -0.1f, 0.02f, 0.00f);
    return skeleton;
}

fitra::infer::Skeleton3D rotate_z_and_translate(
    fitra::infer::Skeleton3D skeleton, float shift_x) {
    for (std::size_t i = 0; i < skeleton.kp_count; ++i) {
        auto& joint = skeleton.joints[i];
        if (!joint.valid) continue;
        const float x = joint.x;
        const float y = joint.y;
        joint.x = -y + shift_x;
        joint.y = x;
    }
    return skeleton;
}

void shift_left_foot(fitra::infer::Skeleton3D& skeleton, float dx) {
    for (const std::size_t joint : {
             fitra::lift::kHalpeLeftAnkle,
             fitra::lift::kHalpeLeftBigToe,
             fitra::lift::kHalpeLeftSmallToe,
             fitra::lift::kHalpeLeftHeel}) {
        skeleton.joints[joint].x += dx;
    }
}

bool wait_for_axis(TrackerAxisBus& bus, std::uint64_t source_sample_seq,
                   bool fresh) {
    for (int i = 0; i < 200; ++i) {
        const auto frame = bus.snapshot();
        if (frame.source_sample_seq == source_sample_seq &&
            frame.fresh == fresh) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
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

void test_extractor_resets_all_history_before_lifecycle_fresh() {
    using fitra::pipeline::FusionPoseEventType;
    using fitra::pipeline::FusionPoseSourceState;
    using fitra::pipeline::Skeleton3DSnapshot;
    using fitra::tracking::ExtractContext;
    using fitra::tracking::TrackerExtractor;
    using fitra::tracking::TrackerExtractorOptions;

    fitra::lift::set_active_keypoint_format(
        fitra::lift::KeypointFormat::Halpe26);
    fitra::pipeline::Skeleton3DBus skeleton_bus;
    fitra::tracking::TrackerBus tracker_bus;
    TrackerAxisBus axis_bus{"stream-a", 3};
    fitra::pipeline::TrackerAxisLineageBus lineage_bus;
    TrackerExtractorOptions opts;
    opts.event_driven = true;
    opts.extract_rate_hz = 500.0;
    TrackerExtractor extractor{
        skeleton_bus, tracker_bus, opts, &axis_bus, &lineage_bus};
    extractor.start();

    const auto subject_a = lifecycle_skeleton();
    auto lineage_a = lineage(1);
    lineage_a = lineage_bus.publish(lineage_a);
    Skeleton3DSnapshot snap_a;
    snap_a.persons.push_back(subject_a);
    snap_a.tracker_axis_lineage = lineage_a;
    skeleton_bus.update(snap_a);
    CHECK(wait_for_axis(axis_bus, 1, true));

    auto switched = lineage(2);
    switched.event_type = FusionPoseEventType::Boundary;
    switched.source_state = FusionPoseSourceState::PersonSwitched;
    switched.source_reason = "person_switched";
    switched.subject_track_id = "subject-b";
    lineage_bus.publish(switched);

    const auto subject_b = rotate_z_and_translate(subject_a, 2.0f);
    auto lineage_b = lineage(3);
    lineage_b.subject_track_id = "subject-b";
    lineage_b = lineage_bus.publish(lineage_b);
    Skeleton3DSnapshot snap_b;
    snap_b.persons.push_back(subject_b);
    snap_b.tracker_axis_lineage = lineage_b;
    skeleton_bus.update(snap_b);
    CHECK(wait_for_axis(axis_bus, 3, true));

    ExtractContext expected_ctx;
    const auto expected_trackers =
        fitra::tracking::extract_trackers_with_floor_corrections(
            subject_b, {}, &expected_ctx, opts.foot_pos_mode,
            opts.chest_height_frac, opts.waist_height_frac,
            opts.limb_extension);
    TrackerAxisBus expected_axis_bus{"stream-a", 3};
    const auto expected_axes = expected_axis_bus.publish(
        expected_trackers, lineage_b);
    const auto actual_axes = axis_bus.snapshot();
    CHECK(actual_axes.fresh);
    for (std::size_t i = 0; i < actual_axes.axes.size(); ++i) {
        CHECK(actual_axes.axes[i].axis.has_value() ==
              expected_axes.axes[i].axis.has_value());
        if (!actual_axes.axes[i].axis || !expected_axes.axes[i].axis) continue;
        for (std::size_t component = 0; component < 3; ++component) {
            CHECK(std::fabs((*actual_axes.axes[i].axis)[component] -
                            (*expected_axes.axes[i].axis)[component]) < 1.0e-5);
        }
    }

    const auto smoothed_b = tracker_bus.snapshot();
    for (const auto role : {TrackerRole::Waist,
                            TrackerRole::LeftUpperLeg}) {
        const auto i = static_cast<std::size_t>(role);
        CHECK(smoothed_b.trackers[i].valid);
        for (int component = 0; component < 3; ++component) {
            CHECK(std::fabs(smoothed_b.trackers[i].pos[component] -
                            expected_trackers[i].pos[component]) < 1.0e-5f);
        }
    }

    // Let the boundary sidecar outrun the latest-only skeleton snapshot. The
    // old subject-b snapshot must not reseed smoothing or its FootAnchor while
    // the new Fresh snapshot is still in flight.
    auto lost = lineage(4);
    lost.event_type = FusionPoseEventType::Boundary;
    lost.source_state = FusionPoseSourceState::Unavailable;
    lost.source_reason = "person_lost";
    lost.subject_track_id = "subject-b";
    lineage_bus.publish(lost);
    auto reacquired = lineage(5);
    reacquired.event_type = FusionPoseEventType::Boundary;
    reacquired.source_state = FusionPoseSourceState::Reacquired;
    reacquired.source_reason = "reacquired";
    reacquired.subject_track_id = "subject-c";
    lineage_bus.publish(reacquired);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    auto subject_c = rotate_z_and_translate(subject_a, -2.0f);
    for (const std::size_t joint : {15u, 20u, 22u, 24u}) {
        subject_c.joints[joint].valid = false;
    }
    auto lineage_c = lineage(6);
    lineage_c.subject_track_id = "subject-c";
    lineage_c.observed[static_cast<std::size_t>(
        TrackerAxisSourceJoint::LeftAnkle)] = false;
    lineage_c = lineage_bus.publish(lineage_c);
    Skeleton3DSnapshot snap_c;
    snap_c.persons.push_back(subject_c);
    snap_c.tracker_axis_lineage = lineage_c;
    skeleton_bus.update(snap_c);
    CHECK(wait_for_axis(axis_bus, 6, true));
    const auto trackers_c = tracker_bus.snapshot();
    CHECK(!trackers_c.trackers[static_cast<std::size_t>(
        TrackerRole::LeftFoot)].valid);

    // Even if an upstream bug/drop presents lifecycle-field changes on a
    // Fresh snapshot without a queued boundary, reset before smoothing. The
    // TrackerAxisBus will synthesize the ordered stream/subject/coordinate/
    // continuity boundaries from the same lineage.
    const auto subject_d = rotate_z_and_translate(subject_a, 4.0f);
    auto lineage_d = lineage(7);
    lineage_d.stream_id = "stream-b";
    lineage_d.subject_track_id = "subject-d";
    lineage_d.coordinate_epoch = 4;
    lineage_d.continuity_epoch = 6;
    lineage_d = lineage_bus.publish(lineage_d);
    Skeleton3DSnapshot snap_d;
    snap_d.persons.push_back(subject_d);
    snap_d.tracker_axis_lineage = lineage_d;
    skeleton_bus.update(snap_d);
    CHECK(wait_for_axis(axis_bus, 7, true));
    ExtractContext expected_d_ctx;
    const auto expected_d =
        fitra::tracking::extract_trackers_with_floor_corrections(
            subject_d, {}, &expected_d_ctx, opts.foot_pos_mode,
            opts.chest_height_frac, opts.waist_height_frac,
            opts.limb_extension);
    const auto trackers_d = tracker_bus.snapshot();
    const auto waist = static_cast<std::size_t>(TrackerRole::Waist);
    CHECK(trackers_d.trackers[waist].valid);
    for (int component = 0; component < 3; ++component) {
        CHECK(std::fabs(trackers_d.trackers[waist].pos[component] -
                        expected_d[waist].pos[component]) < 1.0e-5f);
    }

    extractor.stop();
}

void test_floor_history_is_clean_before_new_lifecycle_axis() {
    using fitra::lift::FloorContactReport;
    using fitra::lift::FloorContactStabilizer;
    using fitra::pipeline::Skeleton3DSnapshot;
    using fitra::tracking::ExtractContext;
    using fitra::tracking::TrackerExtractor;
    using fitra::tracking::TrackerExtractorOptions;

    constexpr double kDt = 1.0 / 30.0;
    fitra::lift::set_active_keypoint_format(
        fitra::lift::KeypointFormat::Halpe26);

    // Seed the old lifecycle with a latched contact and a non-zero XY
    // correction. Without the boundary reset below, the new subject's first
    // frame takes the exit-grace path and inherits this anchor/correction.
    FloorContactStabilizer floor;
    auto old_seed = lifecycle_skeleton();
    (void)floor.update(old_seed, kDt);
    auto old_contact = lifecycle_skeleton();
    (void)floor.update(old_contact, kDt);
    auto old_corrected = lifecycle_skeleton();
    shift_left_foot(old_corrected, 0.02f);
    FloorContactReport last_floor_report = floor.update(old_corrected, kDt);
    CHECK(last_floor_report.feet[0].contact);
    CHECK(last_floor_report.feet[0].corrected);
    CHECK(cv::norm(last_floor_report.feet[0].correction_m) > 1.0e-3f);
    fitra::lift::SkeletonKalman kalman;
    (void)kalman.update(old_corrected, kDt);
    bool has_last_3d_update = true;

    fitra::pipeline::Skeleton3DBus skeleton_bus;
    fitra::tracking::TrackerBus tracker_bus;
    TrackerAxisBus axis_bus{"stream-a", 3};
    fitra::pipeline::TrackerAxisLineageBus lineage_bus;
    TrackerExtractorOptions opts;
    opts.event_driven = true;
    opts.extract_rate_hz = 500.0;
    TrackerExtractor extractor{
        skeleton_bus, tracker_bus, opts, &axis_bus, &lineage_bus};
    extractor.start();

    auto old_lineage = lineage(1);
    old_lineage = lineage_bus.publish(old_lineage);
    Skeleton3DSnapshot old_snapshot;
    old_snapshot.stats.enabled = true;
    old_snapshot.stats.floor_corrections_m[0] =
        last_floor_report.feet[0].correction_m;
    old_snapshot.stats.floor_corrections_m[1] =
        last_floor_report.feet[1].correction_m;
    old_snapshot.persons.push_back(old_corrected);
    old_snapshot.tracker_axis_lineage = old_lineage;
    skeleton_bus.update(old_snapshot);
    CHECK(wait_for_axis(axis_bus, 1, true));

    // Exercise the same production helper called by MultiCameraDriver before
    // the boundary measurement reaches Kalman or FloorContactStabilizer.
    CHECK(fitra::pipeline::reset_lifecycle_filter_history_if_boundary(
        fitra::pipeline::PoseGateSourceState::PersonSwitched,
        kalman, floor, last_floor_report, has_last_3d_update));
    CHECK(!has_last_3d_update);
    CHECK(!last_floor_report.feet[0].contact);
    CHECK(cv::norm(last_floor_report.feet[0].correction_m) == 0.0f);

    const auto new_raw = rotate_z_and_translate(
        lifecycle_skeleton(), 2.0f);
    auto boundary_skeleton = kalman.update(new_raw, kDt);
    const auto boundary_report = floor.update(boundary_skeleton, kDt);
    CHECK(!boundary_report.feet[0].contact);
    CHECK(!boundary_report.feet[0].corrected);
    CHECK(std::fabs(
              boundary_skeleton.joints[fitra::lift::kHalpeLeftAnkle].x -
              new_raw.joints[fitra::lift::kHalpeLeftAnkle].x) < 1.0e-6f);

    auto boundary = lineage(2);
    boundary.event_type = FusionPoseEventType::Boundary;
    boundary.source_state = FusionPoseSourceState::PersonSwitched;
    boundary.source_reason = "person_switched";
    boundary.subject_track_id = "subject-b";
    boundary = lineage_bus.publish(boundary);
    Skeleton3DSnapshot boundary_snapshot;
    boundary_snapshot.stats.enabled = true;
    boundary_snapshot.stats.floor_corrections_m[0] =
        boundary_report.feet[0].correction_m;
    boundary_snapshot.stats.floor_corrections_m[1] =
        boundary_report.feet[1].correction_m;
    boundary_snapshot.persons.push_back(boundary_skeleton);
    boundary_snapshot.tracker_axis_lineage = boundary;
    skeleton_bus.update(boundary_snapshot);
    CHECK(wait_for_axis(axis_bus, 2, false));

    auto fresh_skeleton = kalman.update(new_raw, kDt);
    const auto fresh_report = floor.update(fresh_skeleton, kDt);

    // Separately constructed filters are the clean-state oracle for the same
    // boundary + first-Fresh measurements.
    fitra::lift::SkeletonKalman clean_kalman;
    FloorContactStabilizer clean_floor;
    auto clean_boundary = clean_kalman.update(new_raw, kDt);
    (void)clean_floor.update(clean_boundary, kDt);
    auto clean_fresh = clean_kalman.update(new_raw, kDt);
    const auto clean_report = clean_floor.update(clean_fresh, kDt);
    const auto joint = fitra::lift::kHalpeLeftAnkle;
    const cv::Vec3f actual_ankle{
        fresh_skeleton.joints[joint].x,
        fresh_skeleton.joints[joint].y,
        fresh_skeleton.joints[joint].z};
    const cv::Vec3f expected_ankle{
        clean_fresh.joints[joint].x,
        clean_fresh.joints[joint].y,
        clean_fresh.joints[joint].z};
    for (int component = 0; component < 3; ++component) {
        CHECK(std::fabs(actual_ankle[component] - expected_ankle[component]) <
              1.0e-6f);
        CHECK(std::fabs(fresh_report.feet[0].correction_m[component] -
                        clean_report.feet[0].correction_m[component]) <
              1.0e-6f);
    }

    auto fresh_lineage = lineage(3);
    fresh_lineage.subject_track_id = "subject-b";
    fresh_lineage = lineage_bus.publish(fresh_lineage);
    Skeleton3DSnapshot fresh_snapshot;
    fresh_snapshot.stats.enabled = true;
    fresh_snapshot.stats.floor_corrections_m[0] =
        fresh_report.feet[0].correction_m;
    fresh_snapshot.stats.floor_corrections_m[1] =
        fresh_report.feet[1].correction_m;
    fresh_snapshot.persons.push_back(fresh_skeleton);
    fresh_snapshot.tracker_axis_lineage = fresh_lineage;
    skeleton_bus.update(fresh_snapshot);
    CHECK(wait_for_axis(axis_bus, 3, true));

    ExtractContext expected_ctx;
    const auto expected_trackers =
        fitra::tracking::extract_trackers_with_floor_corrections(
            clean_fresh,
            {clean_report.feet[0].correction_m,
             clean_report.feet[1].correction_m},
            &expected_ctx, opts.foot_pos_mode, opts.chest_height_frac,
            opts.waist_height_frac, opts.limb_extension);
    TrackerAxisBus expected_axis_bus{"stream-a", 3};
    const auto expected_axes = expected_axis_bus.publish(
        expected_trackers, fresh_lineage);
    const auto actual_axes = axis_bus.snapshot();
    for (const auto role : {
             fitra::tracking::TrackerAxisRole::LeftLowerLeg,
             fitra::tracking::TrackerAxisRole::RightLowerLeg}) {
        const std::size_t i = static_cast<std::size_t>(role);
        CHECK(actual_axes.axes[i].axis.has_value());
        CHECK(expected_axes.axes[i].axis.has_value());
        if (!actual_axes.axes[i].axis || !expected_axes.axes[i].axis) continue;
        for (std::size_t component = 0; component < 3; ++component) {
            CHECK(std::fabs((*actual_axes.axes[i].axis)[component] -
                            (*expected_axes.axes[i].axis)[component]) <
                  1.0e-5);
        }
    }

    extractor.stop();
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
    test_extractor_resets_all_history_before_lifecycle_fresh();
    test_floor_history_is_clean_before_new_lifecycle_axis();
    test_clock_pong();
    if (g_fail) {
        std::fprintf(stderr, "test_tracker_axis: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_tracker_axis: OK\n");
    return 0;
}
