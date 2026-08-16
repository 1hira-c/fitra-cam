#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "pipeline/fusion_pose.hpp"

namespace fitra::pipeline {

// Compact provenance sidecar carried from the raw triangulation seam to the
// post-One-Euro TrackerPose producer.  It deliberately contains no position,
// quality, or identity-derived geometry: TrackerAxis only needs the lifecycle,
// capture interval, and proof that every anatomical source joint was measured
// in this exact multi-camera match.
enum class TrackerAxisSourceJoint : std::size_t {
    LeftShoulder = 0,
    RightShoulder,
    LeftHip,
    RightHip,
    LeftKnee,
    RightKnee,
    LeftAnkle,
    RightAnkle,
    Count,
};

struct TrackerAxisLineage {
    std::uint64_t source_sample_seq = 0;
    FusionPoseEventType event_type = FusionPoseEventType::Boundary;
    FusionPoseSourceState source_state = FusionPoseSourceState::Unavailable;
    std::string source_reason = "not_started";
    std::string stream_id;
    std::string subject_track_id = "none";
    std::uint64_t coordinate_epoch = 1;
    std::uint64_t continuity_epoch = 1;
    std::uint64_t source_publish_mono_ns = 0;
    FusionCaptureInterval capture{};
    std::array<bool,
               static_cast<std::size_t>(TrackerAxisSourceJoint::Count)>
        observed{};
};

TrackerAxisLineage make_tracker_axis_lineage(const FusionPoseFrame& frame);

// Boundary-preserving handoff across the latest-only Skeleton3DBus. Fresh
// lineage still rides the exact postprocessed skeleton snapshot; only ordered
// lifecycle records are queued here. Overflow collapses to one explicit
// ContinuityReset and advances the effective continuity epoch.
class TrackerAxisLineageBus {
public:
    explicit TrackerAxisLineageBus(std::size_t boundary_capacity = 32);

    TrackerAxisLineage publish(TrackerAxisLineage lineage);
    std::vector<TrackerAxisLineage> drain_boundaries();
    std::size_t pending_boundary_count() const;

private:
    mutable std::mutex mu_;
    const std::size_t boundary_capacity_;
    std::uint64_t continuity_epoch_ = 1;
    std::deque<TrackerAxisLineage> boundaries_;
    bool boundary_active_ = false;
    std::string active_boundary_key_;
};

}  // namespace fitra::pipeline
