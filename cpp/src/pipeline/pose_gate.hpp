#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "lift/triangulator.hpp"

namespace fitra::pipeline {

inline constexpr const char* kPoseGateProtocolVersion = "fitra_pose_gate_v1";
inline constexpr const char* kPoseGatePositionSpace = "fitra_world_z_up_m";

enum class PoseGateAvailability {
    Fresh,
    Unavailable,
};

enum class PoseGateSourceState {
    Fresh,
    Reacquired,
    PersonSwitched,
    Unavailable,
    EpochChanged,
    UnsupportedTopology,
    UnsupportedMultiPerson,
};

enum class PoseGateJoint {
    Hips,
    Neck,
    LeftHip,
    RightHip,
    LeftKnee,
    RightKnee,
    LeftAnkle,
    RightAnkle,
    Count,
};

const char* pose_gate_source_state_name(PoseGateSourceState state);
const char* pose_gate_joint_name(PoseGateJoint joint);

struct PoseGateJointValue {
    PoseGateAvailability availability = PoseGateAvailability::Unavailable;
    std::optional<std::array<double, 3>> position_m;
    std::optional<double> keypoint_score;
    std::optional<int> view_count;
    std::optional<double> reproj_error;
};

struct PoseGateFrame {
    std::uint64_t sample_seq = 0;
    std::string protocol_version = kPoseGateProtocolVersion;
    std::string stream_id;
    std::string subject_track_id = "none";
    std::uint64_t coordinate_epoch = 1;
    // Empty only for the initial not-started snapshot or an idle boundary.
    // Every frame built from a camera observation has a non-null value.
    std::optional<std::uint64_t> content_mono_ns;
    PoseGateSourceState source_state = PoseGateSourceState::Unavailable;
    std::string source_reason = "not_started";
    std::array<PoseGateJointValue,
               static_cast<std::size_t>(PoseGateJoint::Count)> joints{};
};

// Owns the latest fusion-facing sample and the small amount of lifecycle state
// needed to ensure that an unavailable/changed subject cannot inherit a held
// value downstream. The producer side is the 3D pipeline thread; readers are
// Crow HTTP/WS threads.
class PoseGateBus {
public:
    explicit PoseGateBus(std::string stream_id = {},
                         std::uint64_t coordinate_epoch = 1);

    // Convert the raw TriangulatedSkeleton immediately after triangulation.
    // No Kalman, IK, floor correction, smoothing, or tracker extraction is
    // reachable from this method.
    PoseGateFrame observe(const lift::TriangulatedSkeleton& tri,
                          std::optional<std::uint64_t> content_mono_ns,
                          bool single_subject = true);

    // Publish a boundary/transport sample with every joint unavailable. In
    // particular, this is used for sync misses, idle, and reacquisition gaps.
    PoseGateFrame publish_unavailable(
        std::optional<std::uint64_t> content_mono_ns,
        PoseGateSourceState state = PoseGateSourceState::Unavailable,
        std::string reason = {});

    // A coordinate change invalidates the current subject lifecycle. The next
    // observe() first publishes an EpochChanged/all-Unavailable boundary frame,
    // then the following observation can acquire a new track ID.
    void set_coordinate_epoch(std::uint64_t epoch);
    std::uint64_t coordinate_epoch() const;
    std::string stream_id() const;

    PoseGateFrame snapshot() const;
    std::string make_json() const;

    // Used by the WS publisher to avoid repeatedly sending one Fresh sample
    // when no new triangulation has arrived.
    bool make_json_if_new(std::uint64_t& last_sent_seq, std::string& out) const;

private:
    using Point = std::array<double, 3>;

    PoseGateFrame make_base_locked(
        std::optional<std::uint64_t> content_mono_ns) const;
    PoseGateFrame unavailable_locked(
        std::optional<std::uint64_t> content_mono_ns,
        PoseGateSourceState state,
        const std::string& reason);
    void commit_locked(PoseGateFrame frame);
    std::string make_track_id_locked();
    bool looks_like_person_switch_locked(
        const PoseGateFrame& current,
        std::optional<std::uint64_t> content_mono_ns) const;

    mutable std::mutex mu_;
    std::string stream_id_;
    std::uint64_t coordinate_epoch_ = 1;
    std::uint64_t sample_seq_ = 0;
    PoseGateFrame snapshot_;

    bool track_active_ = false;
    bool ever_observed_ = false;
    bool had_unavailable_ = false;
    bool epoch_boundary_pending_ = false;
    std::string track_id_;
    std::uint64_t track_nonce_ = 0;
    std::optional<Point> previous_hips_;
    std::optional<double> previous_torso_length_m_;
    std::optional<std::uint64_t> previous_content_mono_ns_;
};

}  // namespace fitra::pipeline
