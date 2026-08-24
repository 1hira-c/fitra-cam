#include "pipeline/tracker_axis_lineage.hpp"

#include <algorithm>

namespace fitra::pipeline {

namespace {

bool fresh(const FusionPoseFrame& frame, FusionPoseJoint joint) {
    return frame.joints[static_cast<std::size_t>(joint)].observed_this_frame;
}

}  // namespace

TrackerAxisLineage make_tracker_axis_lineage(const FusionPoseFrame& frame) {
    TrackerAxisLineage out;
    out.source_sample_seq = frame.sample_seq;
    out.event_type = frame.event_type;
    out.source_state = frame.source_state;
    out.source_reason = frame.source_reason;
    out.stream_id = frame.stream_id;
    out.subject_track_id = frame.subject_track_id;
    out.coordinate_epoch = frame.coordinate_epoch;
    out.continuity_epoch = frame.continuity_epoch;
    out.source_publish_mono_ns = frame.source_publish_mono_ns;
    out.capture = frame.capture;
    out.observed[static_cast<std::size_t>(TrackerAxisSourceJoint::LeftShoulder)] =
        fresh(frame, FusionPoseJoint::LeftShoulder);
    out.observed[static_cast<std::size_t>(TrackerAxisSourceJoint::RightShoulder)] =
        fresh(frame, FusionPoseJoint::RightShoulder);
    out.observed[static_cast<std::size_t>(TrackerAxisSourceJoint::LeftHip)] =
        fresh(frame, FusionPoseJoint::LeftHip);
    out.observed[static_cast<std::size_t>(TrackerAxisSourceJoint::RightHip)] =
        fresh(frame, FusionPoseJoint::RightHip);
    out.observed[static_cast<std::size_t>(TrackerAxisSourceJoint::LeftKnee)] =
        fresh(frame, FusionPoseJoint::LeftKnee);
    out.observed[static_cast<std::size_t>(TrackerAxisSourceJoint::RightKnee)] =
        fresh(frame, FusionPoseJoint::RightKnee);
    out.observed[static_cast<std::size_t>(TrackerAxisSourceJoint::LeftAnkle)] =
        fresh(frame, FusionPoseJoint::LeftAnkle);
    out.observed[static_cast<std::size_t>(TrackerAxisSourceJoint::RightAnkle)] =
        fresh(frame, FusionPoseJoint::RightAnkle);
    return out;
}

namespace {

std::string boundary_key(const TrackerAxisLineage& lineage) {
    return std::to_string(static_cast<int>(lineage.source_state)) + "|" +
           lineage.source_reason + "|" + lineage.stream_id + "|" +
           lineage.subject_track_id + "|" +
           std::to_string(lineage.coordinate_epoch) + "|" +
           std::to_string(lineage.continuity_epoch);
}

}  // namespace

TrackerAxisLineageBus::TrackerAxisLineageBus(
    std::size_t boundary_capacity)
    : boundary_capacity_{std::max<std::size_t>(boundary_capacity, 2)} {}

TrackerAxisLineage TrackerAxisLineageBus::publish(
    TrackerAxisLineage lineage) {
    std::lock_guard<std::mutex> lock{mu_};
    continuity_epoch_ = std::max(continuity_epoch_, lineage.continuity_epoch);
    lineage.continuity_epoch = continuity_epoch_;
    if (lineage.event_type != FusionPoseEventType::Boundary) {
        boundary_active_ = false;
        active_boundary_key_.clear();
        return lineage;
    }
    const auto key = boundary_key(lineage);
    if (boundary_active_ && key == active_boundary_key_) return lineage;
    if (boundaries_.size() >= boundary_capacity_) {
        boundaries_.clear();
        ++continuity_epoch_;
        lineage.source_state = FusionPoseSourceState::ContinuityReset;
        lineage.source_reason = "lineage_boundary_queue_overflow";
        lineage.subject_track_id = "none";
        lineage.continuity_epoch = continuity_epoch_;
    }
    boundaries_.push_back(lineage);
    boundary_active_ = true;
    active_boundary_key_ = boundary_key(lineage);
    return lineage;
}

std::vector<TrackerAxisLineage>
TrackerAxisLineageBus::drain_boundaries() {
    std::lock_guard<std::mutex> lock{mu_};
    std::vector<TrackerAxisLineage> out;
    out.reserve(boundaries_.size());
    while (!boundaries_.empty()) {
        out.push_back(std::move(boundaries_.front()));
        boundaries_.pop_front();
    }
    return out;
}

std::size_t TrackerAxisLineageBus::pending_boundary_count() const {
    std::lock_guard<std::mutex> lock{mu_};
    return boundaries_.size();
}

}  // namespace fitra::pipeline
