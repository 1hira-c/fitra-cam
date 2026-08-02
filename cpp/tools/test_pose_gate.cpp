#include <array>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

#include "infer/types.hpp"
#include "lift/skeleton_def.hpp"
#include "pipeline/pose_gate.hpp"

namespace {

using fitra::infer::Skeleton3D;
using fitra::lift::TriangulatedSkeleton;
using fitra::pipeline::PoseGateAvailability;
using fitra::pipeline::PoseGateFrame;
using fitra::pipeline::PoseGateJoint;
using fitra::pipeline::PoseGateSourceState;

void check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

std::size_t index(PoseGateJoint joint) {
    return static_cast<std::size_t>(joint);
}

void set_joint(Skeleton3D& skeleton, std::size_t joint, float x, float y,
               float z) {
    skeleton.joints[joint].x = x;
    skeleton.joints[joint].y = y;
    skeleton.joints[joint].z = z;
    skeleton.joints[joint].score = 0.9f;
    skeleton.joints[joint].valid = true;
}

TriangulatedSkeleton make_tri(double shift_x = 0.0) {
    TriangulatedSkeleton tri;
    tri.skeleton.kp_count = 26;
    set_joint(tri.skeleton, fitra::lift::kHalpeHipCenter,
              static_cast<float>(shift_x), 0.0f, 0.9f);
    set_joint(tri.skeleton, fitra::lift::kHalpeNeck,
              static_cast<float>(shift_x), 0.0f, 1.4f);
    set_joint(tri.skeleton, fitra::lift::kHalpeLeftHip,
              static_cast<float>(shift_x - 0.15), 0.0f, 0.9f);
    set_joint(tri.skeleton, fitra::lift::kHalpeRightHip,
              static_cast<float>(shift_x + 0.15), 0.0f, 0.9f);
    set_joint(tri.skeleton, fitra::lift::kHalpeLeftKnee,
              static_cast<float>(shift_x - 0.15), 0.0f, 0.5f);
    set_joint(tri.skeleton, fitra::lift::kHalpeRightKnee,
              static_cast<float>(shift_x + 0.15), 0.0f, 0.5f);
    set_joint(tri.skeleton, fitra::lift::kHalpeLeftAnkle,
              static_cast<float>(shift_x - 0.15), 0.0f, 0.1f);
    set_joint(tri.skeleton, fitra::lift::kHalpeRightAnkle,
              static_cast<float>(shift_x + 0.15), 0.0f, 0.1f);

    for (const std::size_t joint : {
             fitra::lift::kHalpeHipCenter, fitra::lift::kHalpeNeck,
             fitra::lift::kHalpeLeftHip, fitra::lift::kHalpeRightHip,
             fitra::lift::kHalpeLeftKnee, fitra::lift::kHalpeRightKnee,
             fitra::lift::kHalpeLeftAnkle, fitra::lift::kHalpeRightAnkle}) {
        tri.view_count[joint] = 2;
        tri.reproj_error_px[joint] = 0.5f;
    }
    return tri;
}

void check_all_fresh(const PoseGateFrame& frame) {
    for (const auto& joint : frame.joints) {
        check(joint.availability == PoseGateAvailability::Fresh,
              "expected all gate joints to be Fresh");
        check(joint.position_m.has_value(),
              "Fresh gate joint must carry position_m");
        check(joint.keypoint_score.has_value(),
              "Fresh gate joint must carry keypoint_score");
        check(joint.view_count.has_value(),
              "Fresh gate joint must carry view_count");
        check(joint.reproj_error.has_value(),
              "Fresh gate joint must carry reproj_error");
    }
}

void check_all_unavailable(const PoseGateFrame& frame) {
    for (const auto& joint : frame.joints) {
        check(joint.availability == PoseGateAvailability::Unavailable,
              "expected all gate joints to be Unavailable");
        check(!joint.position_m.has_value(),
              "Unavailable gate joint must not hold position_m");
        check(!joint.keypoint_score.has_value(),
              "Unavailable gate joint must not hold keypoint_score");
        check(!joint.view_count.has_value(),
              "Unavailable gate joint must not hold view_count");
        check(!joint.reproj_error.has_value(),
              "Unavailable gate joint must not hold reproj_error");
    }
}

void test_normal_and_json_contract() {
    fitra::pipeline::PoseGateBus bus{"stream-test"};
    const auto frame = bus.observe(make_tri(), 1'000'000'000);

    check(frame.protocol_version == "fitra_pose_gate_v1",
          "wrong pose gate protocol version");
    check(frame.stream_id == "stream-test", "stream id was not preserved");
    check(frame.subject_track_id != "none" && frame.subject_track_id != "0" &&
              frame.subject_track_id != "1",
          "subject track id must be opaque, not an array index");
    check(frame.coordinate_epoch == 1, "unexpected initial coordinate epoch");
    check(frame.content_mono_ns == 1'000'000'000,
          "content monotonic timestamp was not preserved");
    check(frame.source_state == PoseGateSourceState::Fresh,
          "normal observation must be Fresh");
    check_all_fresh(frame);

    const std::string json = bus.make_json();
    for (const char* field : {
             "\"protocol_version\":\"fitra_pose_gate_v1\"",
             "\"stream_id\":\"stream-test\"",
             "\"content_mono_ns\":1000000000",
             "\"stage\":\"tri.skeleton\"",
             "\"postprocess\":\"none\"",
             "\"position_space\":\"fitra_world_z_up_m\"",
             "\"hips\"", "\"neck\"", "\"left_hip\"",
             "\"right_hip\"", "\"left_knee\"", "\"right_knee\"",
             "\"left_ankle\"", "\"right_ankle\""}) {
        check(json.find(field) != std::string::npos,
              std::string{"missing required JSON contract field: "} + field);
    }
    check(json.find("quat") == std::string::npos,
          "pose gate JSON must not contain tracker quaternion");
    check(json.find("/ws3d") == std::string::npos,
          "pose gate JSON must not identify the postprocessed /ws3d surface");
}

void test_joint_missing_is_not_held() {
    fitra::pipeline::PoseGateBus bus{"missing-test"};
    auto tri = make_tri();
    tri.skeleton.joints[fitra::lift::kHalpeLeftKnee].valid = false;
    const auto frame = bus.observe(tri, 2'000'000'000);

    check(frame.source_state == PoseGateSourceState::Fresh,
          "a partial raw observation still has a valid source frame");
    const auto& missing = frame.joints[index(PoseGateJoint::LeftKnee)];
    check(missing.availability == PoseGateAvailability::Unavailable,
          "missing raw joint must be Unavailable");
    check(!missing.position_m && !missing.keypoint_score &&
              !missing.view_count && !missing.reproj_error,
          "missing raw joint must not inherit quality or position values");
    check(frame.joints[index(PoseGateJoint::RightKnee)].availability ==
              PoseGateAvailability::Fresh,
          "an available sibling joint must remain Fresh");
}

void test_reconnect_gets_new_track() {
    fitra::pipeline::PoseGateBus bus{"reconnect-test"};
    const auto first = bus.observe(make_tri(), 3'000'000'000);
    const auto boundary = bus.publish_unavailable(
        3'100'000'000, PoseGateSourceState::Unavailable, "reconnect");
    const auto reacquired = bus.observe(make_tri(), 3'200'000'000);

    check_all_unavailable(boundary);
    check(reacquired.source_state == PoseGateSourceState::Reacquired,
          "reconnect must be marked Reacquired");
    check(reacquired.subject_track_id != first.subject_track_id,
          "reconnect must not reuse the previous subject track");
    check_all_fresh(reacquired);
}

void test_person_switch_gets_new_track() {
    fitra::pipeline::PoseGateBus bus{"switch-test"};
    const auto first = bus.observe(make_tri(), 4'000'000'000);
    const auto switched = bus.observe(make_tri(2.0), 4'033'000'000);

    check(switched.source_state == PoseGateSourceState::PersonSwitched,
          "large raw subject jump must be marked PersonSwitched");
    check(switched.subject_track_id != first.subject_track_id,
          "person switch must not reuse the previous subject track");
    check_all_fresh(switched);
    check((*switched.joints[index(PoseGateJoint::Hips)].position_m)[0] == 2.0,
          "person switch frame must contain the new raw position");
}

void test_epoch_change_has_boundary() {
    fitra::pipeline::PoseGateBus bus{"epoch-test"};
    const auto first = bus.observe(make_tri(), 5'000'000'000);
    bus.set_coordinate_epoch(2);
    const auto boundary = bus.observe(make_tri(), 5'033'000'000);
    const auto reacquired = bus.observe(make_tri(), 5'066'000'000);

    check(boundary.coordinate_epoch == 2,
          "epoch boundary must expose the new coordinate epoch");
    check(boundary.source_state == PoseGateSourceState::EpochChanged,
          "epoch change must be explicit in source_state");
    check(boundary.subject_track_id == "none",
          "epoch boundary must not retain the old subject id");
    check_all_unavailable(boundary);
    check(reacquired.coordinate_epoch == 2,
          "reacquired frame must keep the new coordinate epoch");
    check(reacquired.source_state == PoseGateSourceState::Reacquired,
          "first frame after an epoch boundary must reacquire");
    check(reacquired.subject_track_id != first.subject_track_id,
          "epoch change must invalidate the previous subject track");
    check_all_fresh(reacquired);
}

void test_unsupported_inputs_are_explicit() {
    fitra::pipeline::PoseGateBus bus{"unsupported-test"};
    const auto multi = bus.observe(make_tri(), 6'000'000'000, false);
    check(multi.source_state == PoseGateSourceState::UnsupportedMultiPerson,
          "M0 multi-person input must not be silently treated as one subject");
    check_all_unavailable(multi);

    auto coco = make_tri();
    coco.skeleton.kp_count = 17;
    const auto topology = bus.observe(coco, 6'033'000'000, true);
    check(topology.source_state == PoseGateSourceState::UnsupportedTopology,
          "COCO17 input must not fabricate the Halpe neck/hip center");
    check_all_unavailable(topology);
}

}  // namespace

int main() {
    try {
        test_normal_and_json_contract();
        test_joint_missing_is_not_held();
        test_reconnect_gets_new_track();
        test_person_switch_gets_new_track();
        test_epoch_change_has_boundary();
        test_unsupported_inputs_are_explicit();
        std::puts("test_pose_gate ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_pose_gate failed: %s\n", e.what());
        return 1;
    }
}
