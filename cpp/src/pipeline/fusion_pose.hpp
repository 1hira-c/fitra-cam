#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "camera/v4l2_capture.hpp"
#include "lift/triangulator.hpp"
#include "pipeline/pose_gate.hpp"

namespace fitra::pipeline {

inline constexpr const char* kFusionPoseProtocolVersion =
    "fitra_fusion_pose_v1";

enum class FusionPoseEventType {
    Pose,
    Boundary,
};

enum class FusionPoseSourceState {
    Fresh,
    Reacquired,
    PersonSwitched,
    Unavailable,
    EpochChanged,
    UnsupportedTopology,
    UnsupportedMultiPerson,
    ContinuityReset,
};

enum class FusionPoseJoint {
    Hips,
    Neck,
    LeftHip,
    RightHip,
    LeftKnee,
    RightKnee,
    LeftAnkle,
    RightAnkle,
    LeftShoulder,
    RightShoulder,
    Count,
};

struct FusionCaptureInterval {
    std::optional<std::uint64_t> oldest_mono_ns;
    std::optional<std::uint64_t> newest_mono_ns;
    camera::V4l2TimestampSemantics semantics =
        camera::V4l2TimestampSemantics::Unavailable;
};

struct FusionPoseJointValue {
    PoseGateAvailability availability = PoseGateAvailability::Unavailable;
    std::optional<std::array<double, 3>> position_m;
    std::optional<double> keypoint_score;
    std::optional<int> inlier_view_count;
    std::optional<double> mean_reproj_error_px;
    std::optional<double> max_ray_angle_deg;
};

struct FusionPoseFrame {
    std::uint64_t sample_seq = 0;
    std::string protocol_version = kFusionPoseProtocolVersion;
    FusionPoseEventType event_type = FusionPoseEventType::Boundary;
    std::string stream_id;
    std::string subject_track_id = "none";
    std::uint64_t coordinate_epoch = 1;
    std::uint64_t continuity_epoch = 1;
    FusionPoseSourceState source_state = FusionPoseSourceState::Unavailable;
    std::string source_reason = "not_started";
    std::uint64_t source_publish_mono_ns = 0;
    FusionCaptureInterval capture{};
    std::array<FusionPoseJointValue,
               static_cast<std::size_t>(FusionPoseJoint::Count)> joints{};
};

const char* fusion_pose_event_type_name(FusionPoseEventType type);
const char* fusion_pose_source_state_name(FusionPoseSourceState state);
const char* fusion_pose_joint_name(FusionPoseJoint joint);

FusionCaptureInterval make_fusion_capture_interval(
    const std::vector<camera::V4l2CaptureTimestamp>& timestamps);

// Independent producer for the D50 contract.  The legacy PoseGate frame is
// supplied by the existing lifecycle owner, so both surfaces share opaque
// stream/subject identity without changing PoseGate v1 serialization.
class FusionPoseBus {
public:
    explicit FusionPoseBus(std::string stream_id,
                           std::uint64_t coordinate_epoch = 1,
                           std::size_t boundary_capacity = 32);

    FusionPoseFrame observe(const lift::TriangulatedSkeleton& tri,
                            const FusionCaptureInterval& capture,
                            const PoseGateFrame& lifecycle);
    FusionPoseFrame publish_boundary(const PoseGateFrame& lifecycle);

    FusionPoseFrame snapshot() const;
    std::string make_json() const;

    // Drain every ordered boundary and at most one latest pose.  The returned
    // strings are already in sample_seq order and are intended for one global
    // broadcast pass to all currently connected clients.
    std::vector<std::string> drain_pending_json();

    std::uint64_t continuity_epoch() const;
    std::size_t pending_boundary_count() const;

private:
    FusionPoseFrame make_base_locked(const PoseGateFrame& lifecycle,
                                     FusionPoseEventType event_type) const;
    FusionPoseFrame make_boundary_locked(const PoseGateFrame& lifecycle,
                                         FusionPoseSourceState state,
                                         std::string reason);
    void commit_pose_locked(FusionPoseFrame frame);
    void commit_boundary_locked(FusionPoseFrame frame);
    void insert_continuity_reset_locked(const PoseGateFrame& lifecycle,
                                        std::string reason);
    void synchronize_stream_locked(const PoseGateFrame& lifecycle);
    bool duplicate_active_boundary_locked(const FusionPoseFrame& frame) const;

    mutable std::mutex mu_;
    std::string stream_id_;
    std::uint64_t coordinate_epoch_ = 1;
    std::uint64_t continuity_epoch_ = 1;
    std::uint64_t sample_seq_ = 0;
    const std::size_t boundary_capacity_;
    FusionPoseFrame snapshot_;
    std::optional<FusionPoseFrame> latest_pose_;
    std::deque<FusionPoseFrame> boundaries_;
    bool boundary_active_ = false;
    std::string active_boundary_key_;
    std::string last_subject_track_id_ = "none";
};

// Pure serializer for the WS clock-sync response.  Parsing and connection I/O
// remain in CrowServer; tests can verify the wire and timestamp ordering here.
std::string make_clock_sync_pong(std::uint64_t nonce,
                                 std::uint64_t client_send_mono_ns,
                                 std::uint64_t server_receive_mono_ns,
                                 std::uint64_t server_send_mono_ns);

}  // namespace fitra::pipeline
