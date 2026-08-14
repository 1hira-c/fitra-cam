#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <iterator>
#include <linux/videodev2.h>
#include <stdexcept>
#include <string>
#include <vector>

#include <crow.h>

#include "lift/skeleton_def.hpp"
#include "pipeline/fusion_pose.hpp"
#include "pipeline/pose_gate.hpp"

namespace {

using fitra::camera::V4l2CaptureTimestamp;
using fitra::camera::V4l2TimestampSemantics;
using fitra::lift::TriangulatedSkeleton;
using fitra::pipeline::FusionCaptureInterval;
using fitra::pipeline::FusionPoseEventType;
using fitra::pipeline::FusionPoseSourceState;
using fitra::pipeline::PoseGateAvailability;
using fitra::pipeline::PoseGateFrame;
using fitra::pipeline::PoseGateJoint;
using fitra::pipeline::PoseGateSourceState;

constexpr std::array<const char*, 8> kJointNames{{
    "hips", "neck", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
}};

void check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

crow::json::rvalue parse_json(const std::string& json) {
    auto parsed = crow::json::load(json);
    check(static_cast<bool>(parsed), "fusion output is not valid JSON");
    return parsed;
}

void check_nonnegative_integer(const crow::json::rvalue& value,
                               const std::string& name) {
    check(value.t() == crow::json::type::Number,
          name + " must be a JSON number");
    check(value.nt() == crow::json::num_type::Signed_integer ||
              value.nt() == crow::json::num_type::Unsigned_integer,
          name + " must be an integer JSON number");
    if (value.nt() == crow::json::num_type::Signed_integer) {
        check(value.i() >= 0, name + " must be non-negative");
    }
}

void check_number(const crow::json::rvalue& value,
                  const std::string& name) {
    check(value.t() == crow::json::type::Number,
          name + " must be a JSON number");
    check(std::isfinite(value.d()), name + " must be finite");
}

void check_exact_joint_keys(const crow::json::rvalue& joints) {
    check(joints.t() == crow::json::type::Object,
          "joints must be a JSON object");
    auto keys = joints.keys();
    std::sort(keys.begin(), keys.end());
    std::vector<std::string> expected;
    expected.reserve(kJointNames.size());
    for (const char* name : kJointNames) expected.emplace_back(name);
    std::sort(expected.begin(), expected.end());
    check(keys == expected, "joints must contain exactly the eight D50 keys");
}

void check_fresh_joint_wire(const crow::json::rvalue& joint,
                            const std::string& name) {
    check(joint.t() == crow::json::type::Object,
          name + " must be an object");
    check(joint["availability"].t() == crow::json::type::String &&
              std::string{joint["availability"].s()} == "Fresh",
          name + " availability must be Fresh");
    const auto& position = joint["position_m"];
    check(position.t() == crow::json::type::List && position.size() == 3,
          name + " position_m must be a three-number list");
    for (std::size_t i = 0; i < position.size(); ++i) {
        check_number(position[i], name + ".position_m");
    }
    check_number(joint["keypoint_score"], name + ".keypoint_score");
    check_nonnegative_integer(joint["inlier_view_count"],
                              name + ".inlier_view_count");
    check_number(joint["mean_reproj_error_px"],
                 name + ".mean_reproj_error_px");
    check_number(joint["max_ray_angle_deg"],
                 name + ".max_ray_angle_deg");
}

void check_unavailable_joint_wire(const crow::json::rvalue& joint,
                                  const std::string& name) {
    check(joint.t() == crow::json::type::Object,
          name + " must be an object");
    check(joint["availability"].t() == crow::json::type::String &&
              std::string{joint["availability"].s()} == "Unavailable",
          name + " availability must be Unavailable");
    for (const char* field : {
             "position_m", "keypoint_score", "inlier_view_count",
             "mean_reproj_error_px", "max_ray_angle_deg"}) {
        check(joint[field].t() == crow::json::type::Null,
              name + "." + field + " must be null");
    }
}

void check_boundary_wire(const std::string& json,
                         const std::string& expected_state) {
    const auto root = parse_json(json);
    check(std::string{root["event_type"].s()} == "boundary",
          "lifecycle sample must be a boundary");
    check(std::string{root["source_state"].s()} == expected_state,
          "boundary source_state mismatch");
    const auto& capture = root["capture"];
    check(capture["oldest_mono_ns"].t() == crow::json::type::Null &&
              capture["newest_mono_ns"].t() == crow::json::type::Null &&
              capture["span_ms"].t() == crow::json::type::Null &&
              std::string{capture["timestamp_semantics"].s()} == "unavailable",
          "boundary capture evidence must be unavailable");
    const auto& joints = root["joints"];
    check_exact_joint_keys(joints);
    for (const char* name : kJointNames) {
        check_unavailable_joint_wire(joints[name], name);
    }
}

void set_joint(fitra::infer::Skeleton3D& skeleton, std::size_t index,
               float x, float y, float z, float score = 0.9f) {
    auto& joint = skeleton.joints[index];
    joint.x = x;
    joint.y = y;
    joint.z = z;
    joint.score = score;
    joint.valid = true;
}

TriangulatedSkeleton make_tri(float shift_x = 0.0f) {
    TriangulatedSkeleton tri;
    tri.skeleton.kp_count = 26;
    const std::size_t indices[] = {
        fitra::lift::kHalpeHipCenter, fitra::lift::kHalpeNeck,
        fitra::lift::kHalpeLeftHip, fitra::lift::kHalpeRightHip,
        fitra::lift::kHalpeLeftKnee, fitra::lift::kHalpeRightKnee,
        fitra::lift::kHalpeLeftAnkle, fitra::lift::kHalpeRightAnkle,
    };
    for (std::size_t i = 0; i < std::size(indices); ++i) {
        set_joint(tri.skeleton, indices[i], shift_x + 0.01f * i,
                  0.0f, 0.8f + 0.05f * i);
        tri.view_count[indices[i]] = 3;
        tri.reproj_error_px[indices[i]] = 0.8f;
        tri.max_ray_angle_deg[indices[i]] = 12.5f;
    }
    return tri;
}

void test_v4l2_timestamp_semantics() {
    const auto soe = fitra::camera::interpret_v4l2_timestamp(
        12, 345, V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
                 V4L2_BUF_FLAG_TSTAMP_SRC_SOE);
    check(soe.semantics == V4l2TimestampSemantics::MonotonicSoe,
          "SOE flag was not preserved");
    check(soe.mono_ns == 12'000'345'000ULL,
          "kernel timeval conversion is wrong");

    const auto eof = fitra::camera::interpret_v4l2_timestamp(
        13, 999'999, V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
                     V4L2_BUF_FLAG_TSTAMP_SRC_EOF);
    check(eof.semantics == V4l2TimestampSemantics::MonotonicEof,
          "EOF flag was not preserved");
    check(eof.mono_ns == 13'999'999'000ULL,
          "EOF timestamp conversion is wrong");

    for (const auto& invalid : {
             fitra::camera::interpret_v4l2_timestamp(12, 345, 0),
             fitra::camera::interpret_v4l2_timestamp(
                 12, 345, V4L2_BUF_FLAG_TIMESTAMP_COPY),
             fitra::camera::interpret_v4l2_timestamp(
                 12, 1'000'000, V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC),
             fitra::camera::interpret_v4l2_timestamp(
                 -1, 0, V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC),
             fitra::camera::interpret_v4l2_timestamp(
                 20'000'000'000LL, 0,
                 V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)}) {
        check(invalid.semantics == V4l2TimestampSemantics::Unavailable &&
                  !invalid.mono_ns,
              "unknown/copy/invalid V4L2 timestamp must be unavailable");
    }
}

void test_capture_interval_semantics() {
    const auto same = fitra::pipeline::make_fusion_capture_interval({
        V4l2CaptureTimestamp{1'000'000'000ULL,
                             V4l2TimestampSemantics::MonotonicSoe},
        V4l2CaptureTimestamp{1'006'000'000ULL,
                             V4l2TimestampSemantics::MonotonicSoe},
    });
    check(same.oldest_mono_ns == 1'000'000'000ULL &&
              same.newest_mono_ns == 1'006'000'000ULL &&
              same.semantics == V4l2TimestampSemantics::MonotonicSoe,
          "matching camera timestamp semantics must retain the interval");

    const auto mixed = fitra::pipeline::make_fusion_capture_interval({
        V4l2CaptureTimestamp{1, V4l2TimestampSemantics::MonotonicSoe},
        V4l2CaptureTimestamp{2, V4l2TimestampSemantics::MonotonicEof},
    });
    check(!mixed.oldest_mono_ns && !mixed.newest_mono_ns &&
              mixed.semantics == V4l2TimestampSemantics::Unavailable,
          "mixed SOE/EOF semantics must invalidate the complete interval");

    const auto unknown = fitra::pipeline::make_fusion_capture_interval({
        V4l2CaptureTimestamp{1, V4l2TimestampSemantics::MonotonicSoe},
        V4l2CaptureTimestamp{},
    });
    check(!unknown.oldest_mono_ns && !unknown.newest_mono_ns &&
              unknown.semantics == V4l2TimestampSemantics::Unavailable,
          "one unknown camera timestamp must invalidate the complete interval");
}

void test_wire_and_pose_gate_compatibility() {
    fitra::pipeline::PoseGateBus gate{"shared-stream", 77};
    const auto tri = make_tri();
    const PoseGateFrame lifecycle = gate.observe(tri, 1'100'000'000ULL);
    const std::string legacy_before = gate.make_json();

    fitra::pipeline::FusionPoseBus fusion{
        gate.stream_id(), gate.coordinate_epoch()};
    FusionCaptureInterval capture;
    capture.oldest_mono_ns = 1'000'000'000ULL;
    capture.newest_mono_ns = 1'006'000'000ULL;
    capture.semantics = V4l2TimestampSemantics::MonotonicSoe;
    const auto frame = fusion.observe(tri, capture, lifecycle);

    check(frame.event_type == FusionPoseEventType::Pose &&
              frame.source_state == FusionPoseSourceState::Fresh,
          "normal fusion observation must be a Fresh pose");
    check(frame.stream_id == lifecycle.stream_id &&
              frame.subject_track_id == lifecycle.subject_track_id &&
              frame.coordinate_epoch == lifecycle.coordinate_epoch,
          "fusion wire must share the PoseGate opaque lifecycle identity");

    const std::string json = fusion.make_json();
    const auto root = parse_json(json);
    check(root["protocol_version"].t() == crow::json::type::String &&
              std::string{root["protocol_version"].s()} ==
                  "fitra_fusion_pose_v1",
          "protocol_version mismatch");
    check(std::string{root["position_space"].s()} == "fitra_world_z_up_m",
          "position_space mismatch");
    for (const char* field : {
             "sample_seq", "coordinate_epoch", "continuity_epoch",
             "source_publish_mono_ns"}) {
        check_nonnegative_integer(root[field], field);
    }

    const auto& capture_json = root["capture"];
    check(capture_json.t() == crow::json::type::Object,
          "capture must be an object");
    check_nonnegative_integer(capture_json["oldest_mono_ns"],
                              "capture.oldest_mono_ns");
    check_nonnegative_integer(capture_json["newest_mono_ns"],
                              "capture.newest_mono_ns");
    check_number(capture_json["span_ms"], "capture.span_ms");
    check(capture_json["oldest_mono_ns"].u() == 1'000'000'000ULL &&
              capture_json["newest_mono_ns"].u() == 1'006'000'000ULL &&
              std::abs(capture_json["span_ms"].d() - 6.0) < 1.0e-9,
          "capture interval values or units are wrong");
    check(std::string{capture_json["timestamp_semantics"].s()} ==
              "monotonic_soe",
          "capture timestamp semantics mismatch");
    check(!root.has("capture_oldest_mono_ns") &&
              !root.has("capture_newest_mono_ns"),
          "legacy flat fusion capture fields must not leak into the wire");

    const auto& joints = root["joints"];
    check_exact_joint_keys(joints);
    for (const char* name : kJointNames) {
        check_fresh_joint_wire(joints[name], name);
    }
    check(json.find("quat") == std::string::npos,
          "fusion pose wire must remain position-only");
    check(gate.make_json() == legacy_before,
          "observing the additive fusion bus changed PoseGate v1 bytes");
}

void test_unavailable_joint_and_timestamp_shape() {
    fitra::pipeline::PoseGateBus gate{"null-shape"};
    auto tri = make_tri();
    tri.skeleton.joints[fitra::lift::kHalpeLeftKnee].valid = false;
    const auto lifecycle = gate.observe(tri, 2'000'000'000ULL);
    fitra::pipeline::FusionPoseBus fusion{gate.stream_id()};
    fusion.observe(tri, {}, lifecycle);

    const auto root = parse_json(fusion.make_json());
    const auto& capture = root["capture"];
    check(capture["oldest_mono_ns"].t() == crow::json::type::Null &&
              capture["newest_mono_ns"].t() == crow::json::type::Null &&
              capture["span_ms"].t() == crow::json::type::Null &&
              std::string{capture["timestamp_semantics"].s()} == "unavailable",
          "unavailable capture semantics must have a complete null shape");
    check_unavailable_joint_wire(root["joints"]["left_knee"], "left_knee");
    check_fresh_joint_wire(root["joints"]["right_knee"], "right_knee");
}

void test_latest_pose_and_ordered_boundaries() {
    fitra::pipeline::PoseGateBus gate{"queue-stream"};
    fitra::pipeline::FusionPoseBus fusion{gate.stream_id()};
    const auto tri0 = make_tri();
    const auto first = gate.observe(tri0, 3'000'000'000ULL);
    fusion.observe(tri0, {}, first);
    const auto tri1 = make_tri(0.1f);
    const auto second = gate.observe(tri1, 3'033'000'000ULL);
    fusion.observe(tri1, {}, second);

    auto pending = fusion.drain_pending_json();
    check(pending.size() == 1,
          "ordinary Fresh poses must collapse to the latest sample");
    check(parse_json(pending[0])["sample_seq"].u() == 2,
          "latest pose sample_seq mismatch");

    TriangulatedSkeleton empty;
    empty.skeleton.kp_count = 26;
    const auto lost = gate.observe(empty, 3'066'000'000ULL);
    const auto lost_frame = fusion.observe(empty, {}, lost);
    check(lost_frame.event_type == FusionPoseEventType::Boundary &&
              lost_frame.source_state == FusionPoseSourceState::Unavailable,
          "matched empty observation must produce an Unavailable boundary");

    const auto reacquired = gate.observe(tri1, 3'099'000'000ULL);
    fusion.observe(tri1, {}, reacquired);
    const auto fresh = gate.observe(tri1, 3'132'000'000ULL);
    fusion.observe(tri1, {}, fresh);

    pending = fusion.drain_pending_json();
    check(pending.size() == 3,
          "Unavailable, Reacquired and following Fresh must all be delivered");
    check_boundary_wire(pending[0], "Unavailable");
    check_boundary_wire(pending[1], "Reacquired");
    check(std::string{parse_json(pending[2])["event_type"].s()} == "pose",
          "Fresh pose must follow the ordered recovery boundaries");
    for (const auto& message : pending) {
        check(message.find("PersonLost") == std::string::npos,
              "PersonLost is not part of the adapter source_state contract");
    }
}

void test_stream_subject_coordinate_and_continuity_cut_hold() {
    fitra::pipeline::PoseGateBus old_gate{"stream-old", 11};
    fitra::pipeline::FusionPoseBus fusion{old_gate.stream_id(), 11};
    const auto old_tri = make_tri();
    const auto old_lifecycle = old_gate.observe(old_tri, 4'000'000'000ULL);
    fusion.observe(old_tri, {}, old_lifecycle);

    // Runtime reconstruction normally creates a new PoseGate/FusionPose pair.
    // If a lifecycle owner is replaced in-process, the same change is detected
    // at observe() and converted into an explicit ContinuityReset before the
    // first new-stream pose. The pending old-stream latest pose is discarded.
    fitra::pipeline::PoseGateBus new_gate{"stream-new", 22};
    const auto new_tri = make_tri(0.2f);
    const auto new_lifecycle = new_gate.observe(new_tri, 4'033'000'000ULL);
    check(new_lifecycle.subject_track_id != old_lifecycle.subject_track_id,
          "runtime recreation must use a new subject identity");
    fusion.observe(new_tri, {}, new_lifecycle);

    const auto pending = fusion.drain_pending_json();
    check(pending.size() == 2,
          "stream replacement must retain only reset + new pose");
    check_boundary_wire(pending[0], "ContinuityReset");
    const auto reset = parse_json(pending[0]);
    const auto pose = parse_json(pending[1]);
    check(std::string{reset["stream_id"].s()} == "stream-new" &&
              std::string{reset["source_reason"].s()} == "stream_id_changed" &&
              reset["coordinate_epoch"].u() == 22 &&
              reset["continuity_epoch"].u() == 2,
          "stream reset must expose the new runtime invalidation keys");
    check(std::string{pose["event_type"].s()} == "pose" &&
              std::string{pose["stream_id"].s()} == "stream-new" &&
              std::string{pose["subject_track_id"].s()} ==
                  new_lifecycle.subject_track_id &&
              pose["coordinate_epoch"].u() == 22 &&
              pose["continuity_epoch"].u() == 2,
          "new pose must not inherit old stream/subject/coordinate/continuity");
    check(pending[0].find("stream-old") == std::string::npos &&
              pending[1].find("stream-old") == std::string::npos,
          "old-stream latest pose survived the runtime boundary");
}

void test_subject_and_coordinate_boundaries_cut_latest() {
    fitra::pipeline::PoseGateBus gate{"identity-boundaries", 31};
    fitra::pipeline::FusionPoseBus fusion{gate.stream_id(), 31};
    const auto first_tri = make_tri();
    const auto first = gate.observe(first_tri, 5'000'000'000ULL);
    fusion.observe(first_tri, {}, first);

    const auto switched_tri = make_tri(2.0f);
    const auto switched = gate.observe(switched_tri, 5'033'000'000ULL);
    check(switched.source_state == PoseGateSourceState::PersonSwitched,
          "test setup did not create a person-switch lifecycle");
    fusion.observe(switched_tri, {}, switched);
    auto pending = fusion.drain_pending_json();
    check(pending.size() == 1,
          "person switch must discard the pending old-subject pose");
    check_boundary_wire(pending[0], "PersonSwitched");

    const auto after_switch = gate.observe(switched_tri, 5'066'000'000ULL);
    fusion.observe(switched_tri, {}, after_switch);
    gate.set_coordinate_epoch(32);
    const auto epoch = gate.observe(switched_tri, 5'099'000'000ULL);
    check(epoch.source_state == PoseGateSourceState::EpochChanged,
          "test setup did not create an epoch boundary");
    fusion.observe(switched_tri, {}, epoch);
    pending = fusion.drain_pending_json();
    check(pending.size() == 1,
          "coordinate change must discard the pending old-epoch pose");
    check_boundary_wire(pending[0], "EpochChanged");
    const auto root = parse_json(pending[0]);
    check(root["coordinate_epoch"].u() == 32 &&
              std::string{root["subject_track_id"].s()} == "none",
          "epoch boundary retained old coordinate/subject evidence");
}

void test_overflow_increments_continuity() {
    fitra::pipeline::FusionPoseBus fusion{"overflow-stream", 9, 2};
    PoseGateFrame boundary;
    boundary.stream_id = "overflow-stream";
    boundary.subject_track_id = "subject-old";
    boundary.coordinate_epoch = 9;
    boundary.source_state = PoseGateSourceState::Unavailable;
    for (const char* reason : {"loss-a", "loss-b", "loss-c"}) {
        boundary.source_reason = reason;
        fusion.publish_boundary(boundary);
    }

    check(fusion.continuity_epoch() == 2,
          "boundary FIFO overflow must increment continuity_epoch");
    const auto pending = fusion.drain_pending_json();
    check(pending.size() == 2,
          "overflow must cut the old queue and retain reset + incoming boundary");
    check_boundary_wire(pending[0], "ContinuityReset");
    check_boundary_wire(pending[1], "Unavailable");
    const auto reset = parse_json(pending[0]);
    const auto incoming = parse_json(pending[1]);
    check(std::string{reset["source_reason"].s()} ==
              "boundary_queue_overflow" &&
              reset["continuity_epoch"].u() == 2 &&
              std::string{incoming["source_reason"].s()} == "loss-c" &&
              incoming["continuity_epoch"].u() == 2,
          "overflow reset evidence is missing or out of order");
}

void test_clock_sync_response() {
    const auto pong = fitra::pipeline::make_clock_sync_pong(
        17, 1'000, 2'000, 2'100);
    check(pong ==
              "{\"type\":\"clock_sync_pong\",\"nonce\":17,"
              "\"client_send_mono_ns\":1000,"
              "\"server_receive_mono_ns\":2000,"
              "\"server_send_mono_ns\":2100}",
          "clock-sync response does not echo the complete contract");
    const auto root = parse_json(pong);
    for (const char* field : {
             "nonce", "client_send_mono_ns", "server_receive_mono_ns",
             "server_send_mono_ns"}) {
        check_nonnegative_integer(root[field], field);
    }
    check(root["server_receive_mono_ns"].u() <=
              root["server_send_mono_ns"].u(),
          "clock-sync server timestamps are reversed");
}

}  // namespace

int main() {
    try {
        test_v4l2_timestamp_semantics();
        test_capture_interval_semantics();
        test_wire_and_pose_gate_compatibility();
        test_unavailable_joint_and_timestamp_shape();
        test_latest_pose_and_ordered_boundaries();
        test_stream_subject_coordinate_and_continuity_cut_hold();
        test_subject_and_coordinate_boundaries_cut_latest();
        test_overflow_increments_continuity();
        test_clock_sync_response();
        std::puts("test_fusion_pose ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_fusion_pose failed: %s\n", e.what());
        return 1;
    }
}
