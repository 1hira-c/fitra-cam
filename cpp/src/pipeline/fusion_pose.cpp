#include "pipeline/fusion_pose.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include "lift/skeleton_def.hpp"
#include "util/clock.hpp"

namespace fitra::pipeline {

namespace {

constexpr std::array<std::size_t,
                     static_cast<std::size_t>(FusionPoseJoint::Count)>
    kHalpeFusionIndices{{
        lift::kHalpeHipCenter,
        lift::kHalpeNeck,
        lift::kHalpeLeftHip,
        lift::kHalpeRightHip,
        lift::kHalpeLeftKnee,
        lift::kHalpeRightKnee,
        lift::kHalpeLeftAnkle,
        lift::kHalpeRightAnkle,
        lift::kHalpeLeftShoulder,
        lift::kHalpeRightShoulder,
    }};

void append_json_string(std::string& out, const std::string& value) {
    out += '"';
    for (unsigned char ch : value) {
        switch (ch) {
            case '\\': out += "\\\\"; break;
            case '"':  out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (ch < 0x20) {
                    char buf[7];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", ch);
                    out += buf;
                } else {
                    out += static_cast<char>(ch);
                }
                break;
        }
    }
    out += '"';
}

void append_number(std::string& out, double value) {
    if (!std::isfinite(value)) {
        out += "null";
        return;
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.9g", value);
    out += buf;
}

void append_optional_number(std::string& out,
                            const std::optional<double>& value) {
    if (!value) {
        out += "null";
        return;
    }
    append_number(out, *value);
}

void append_optional_uint64(std::string& out,
                            const std::optional<std::uint64_t>& value) {
    if (!value) {
        out += "null";
        return;
    }
    out += std::to_string(*value);
}

constexpr float kRawKeypointScoreMin = 0.3f;

bool finite_joint(const infer::Joint3D& joint) {
    return joint.valid && std::isfinite(joint.x) && std::isfinite(joint.y) &&
           std::isfinite(joint.z) && std::isfinite(joint.score);
}

bool raw_observed(const lift::TriangulatedSkeleton& tri, std::size_t source_idx) {
    if (source_idx >= tri.skeleton.joints.size() ||
        source_idx >= tri.skeleton.kp_count) {
        return false;
    }
    const auto& source = tri.skeleton.joints[source_idx];
    return finite_joint(source) && source.score >= kRawKeypointScoreMin &&
           tri.view_count[source_idx] > 0 &&
           std::isfinite(tri.reproj_error_px[source_idx]) &&
           tri.reproj_error_px[source_idx] >= 0.0f &&
           std::isfinite(tri.max_ray_angle_deg[source_idx]) &&
           tri.max_ray_angle_deg[source_idx] >= 0.0f;
}

std::optional<std::array<double, 3>> filtered_position(
    const infer::Skeleton3D* skeleton, std::size_t source_idx) {
    if (!skeleton || source_idx >= skeleton->joints.size() ||
        source_idx >= skeleton->kp_count) {
        return std::nullopt;
    }
    const auto& joint = skeleton->joints[source_idx];
    if (!joint.valid || !std::isfinite(joint.x) || !std::isfinite(joint.y) ||
        !std::isfinite(joint.z)) {
        return std::nullopt;
    }
    return std::array<double, 3>{joint.x, joint.y, joint.z};
}

std::array<double, 3> midpoint(const std::array<double, 3>& left,
                               const std::array<double, 3>& right) {
    return {(left[0] + right[0]) * 0.5,
            (left[1] + right[1]) * 0.5,
            (left[2] + right[2]) * 0.5};
}

void fill_raw_joint(const lift::TriangulatedSkeleton& tri,
                    std::size_t source_idx,
                    FusionPoseJointValue& dst) {
    if (!raw_observed(tri, source_idx)) return;
    const auto& source = tri.skeleton.joints[source_idx];
    dst.availability = PoseGateAvailability::Fresh;
    dst.observed_this_frame = true;
    dst.position_m = std::array<double, 3>{source.x, source.y, source.z};
    dst.keypoint_score = source.score;
    dst.inlier_view_count = tri.view_count[source_idx];
    dst.mean_reproj_error_px = tri.reproj_error_px[source_idx];
    dst.max_ray_angle_deg = tri.max_ray_angle_deg[source_idx];
}

std::string serialize(const FusionPoseFrame& frame) {
    std::string out;
    out.reserve(2600);
    out += "{\"protocol_version\":";
    append_json_string(out, frame.protocol_version);
    out += ",\"sample_seq\":" + std::to_string(frame.sample_seq);
    out += ",\"event_type\":";
    append_json_string(out, fusion_pose_event_type_name(frame.event_type));
    out += ",\"stream_id\":";
    append_json_string(out, frame.stream_id);
    out += ",\"subject_track_id\":";
    append_json_string(out, frame.subject_track_id);
    out += ",\"coordinate_epoch\":" + std::to_string(frame.coordinate_epoch);
    out += ",\"continuity_epoch\":" + std::to_string(frame.continuity_epoch);
    out += ",\"source_state\":";
    append_json_string(out, fusion_pose_source_state_name(frame.source_state));
    out += ",\"source_reason\":";
    append_json_string(out, frame.source_reason);
    out += ",\"source_publish_mono_ns\":" +
           std::to_string(frame.source_publish_mono_ns);
    out += ",\"capture\":{\"oldest_mono_ns\":";
    append_optional_uint64(out, frame.capture.oldest_mono_ns);
    out += ",\"newest_mono_ns\":";
    append_optional_uint64(out, frame.capture.newest_mono_ns);
    out += ",\"span_ms\":";
    if (frame.capture.oldest_mono_ns && frame.capture.newest_mono_ns &&
        *frame.capture.newest_mono_ns >= *frame.capture.oldest_mono_ns) {
        append_number(out,
            static_cast<double>(*frame.capture.newest_mono_ns -
                                *frame.capture.oldest_mono_ns) / 1.0e6);
    } else {
        out += "null";
    }
    out += ",\"timestamp_semantics\":";
    append_json_string(out, camera::v4l2_timestamp_semantics_name(
                                frame.capture.semantics));
    out += '}';
    out += ",\"provenance\":{\"pipeline\":\"fitra-cam\""
           ",\"stage\":\"tri.skeleton\""
           ",\"position_source\":\"multi_view_triangulation\""
           ",\"postprocess\":\"none\""
           ",\"kalman\":false,\"ik\":false,\"floor_contact\":false}";
    out += ",\"filtered_position_provenance\":{\"stage\":\"post_kalman_ik\""
           ",\"position_source\":\"skeleton3d_snapshot\""
           ",\"floor_contact\":false,\"root_transform\":false}";
    out += ",\"position_space\":\"fitra_world_z_up_m\",\"joints\":{";
    for (std::size_t i = 0;
         i < static_cast<std::size_t>(FusionPoseJoint::Count); ++i) {
        if (i) out += ',';
        append_json_string(out,
                           fusion_pose_joint_name(static_cast<FusionPoseJoint>(i)));
        out += ":{";
        const auto& joint = frame.joints[i];
        out += "\"position_m\":";
        if (joint.position_m) {
            out += '[';
            append_number(out, (*joint.position_m)[0]); out += ',';
            append_number(out, (*joint.position_m)[1]); out += ',';
            append_number(out, (*joint.position_m)[2]); out += ']';
        } else {
            out += "null";
        }
        out += ",\"filtered_position_m\":";
        if (joint.filtered_position_m) {
            out += '[';
            append_number(out, (*joint.filtered_position_m)[0]); out += ',';
            append_number(out, (*joint.filtered_position_m)[1]); out += ',';
            append_number(out, (*joint.filtered_position_m)[2]); out += ']';
        } else {
            out += "null";
        }
        out += ",\"availability\":";
        append_json_string(out,
            joint.availability == PoseGateAvailability::Fresh
                ? "Fresh" : "Unavailable");
        out += ",\"observed_this_frame\":";
        out += joint.observed_this_frame ? "true" : "false";
        out += ",\"keypoint_score\":";
        append_optional_number(out, joint.keypoint_score);
        out += ",\"inlier_view_count\":";
        if (joint.inlier_view_count) out += std::to_string(*joint.inlier_view_count);
        else out += "null";
        out += ",\"mean_reproj_error_px\":";
        append_optional_number(out, joint.mean_reproj_error_px);
        out += ",\"max_ray_angle_deg\":";
        append_optional_number(out, joint.max_ray_angle_deg);
        out += '}';
    }
    out += "}}";
    return out;
}

std::string boundary_key(const FusionPoseFrame& frame) {
    return std::to_string(static_cast<int>(frame.source_state)) + "|" +
           frame.source_reason + "|" + frame.stream_id + "|" +
           frame.subject_track_id + "|" +
           std::to_string(frame.coordinate_epoch);
}

}  // namespace

const char* fusion_pose_event_type_name(FusionPoseEventType type) {
    return type == FusionPoseEventType::Pose ? "pose" : "boundary";
}

const char* fusion_pose_source_state_name(FusionPoseSourceState state) {
    switch (state) {
        case FusionPoseSourceState::Fresh: return "Fresh";
        case FusionPoseSourceState::Reacquired: return "Reacquired";
        case FusionPoseSourceState::PersonSwitched: return "PersonSwitched";
        case FusionPoseSourceState::Unavailable: return "Unavailable";
        case FusionPoseSourceState::EpochChanged: return "EpochChanged";
        case FusionPoseSourceState::UnsupportedTopology:
            return "UnsupportedTopology";
        case FusionPoseSourceState::UnsupportedMultiPerson:
            return "UnsupportedMultiPerson";
        case FusionPoseSourceState::ContinuityReset: return "ContinuityReset";
    }
    return "Unavailable";
}

const char* fusion_pose_joint_name(FusionPoseJoint joint) {
    switch (joint) {
        case FusionPoseJoint::Hips:          return "hips";
        case FusionPoseJoint::Neck:          return "neck";
        case FusionPoseJoint::LeftHip:       return "left_hip";
        case FusionPoseJoint::RightHip:      return "right_hip";
        case FusionPoseJoint::LeftKnee:      return "left_knee";
        case FusionPoseJoint::RightKnee:     return "right_knee";
        case FusionPoseJoint::LeftAnkle:     return "left_ankle";
        case FusionPoseJoint::RightAnkle:    return "right_ankle";
        case FusionPoseJoint::LeftShoulder:  return "left_shoulder";
        case FusionPoseJoint::RightShoulder: return "right_shoulder";
        case FusionPoseJoint::Count:         break;
    }
    return "unknown";
}

FusionCaptureInterval make_fusion_capture_interval(
    const std::vector<camera::V4l2CaptureTimestamp>& timestamps) {
    FusionCaptureInterval out;
    if (timestamps.empty()) return out;
    out.semantics = timestamps.front().semantics;
    if (!timestamps.front().mono_ns ||
        out.semantics == camera::V4l2TimestampSemantics::Unavailable) {
        return {};
    }
    out.oldest_mono_ns = timestamps.front().mono_ns;
    out.newest_mono_ns = timestamps.front().mono_ns;
    for (const auto& timestamp : timestamps) {
        if (!timestamp.mono_ns || timestamp.semantics != out.semantics ||
            timestamp.semantics ==
                camera::V4l2TimestampSemantics::Unavailable) {
            return {};
        }
        out.oldest_mono_ns = std::min(*out.oldest_mono_ns, *timestamp.mono_ns);
        out.newest_mono_ns = std::max(*out.newest_mono_ns, *timestamp.mono_ns);
    }
    return out;
}

namespace {

FusionPoseSourceState map_lifecycle_state(const PoseGateFrame& lifecycle) {
    switch (lifecycle.source_state) {
        case PoseGateSourceState::Fresh:
            return FusionPoseSourceState::Fresh;
        case PoseGateSourceState::Reacquired:
            return FusionPoseSourceState::Reacquired;
        case PoseGateSourceState::PersonSwitched:
            return FusionPoseSourceState::PersonSwitched;
        case PoseGateSourceState::EpochChanged:
            return FusionPoseSourceState::EpochChanged;
        case PoseGateSourceState::UnsupportedTopology:
            return FusionPoseSourceState::UnsupportedTopology;
        case PoseGateSourceState::UnsupportedMultiPerson:
            return FusionPoseSourceState::UnsupportedMultiPerson;
        case PoseGateSourceState::Unavailable:
            return FusionPoseSourceState::Unavailable;
    }
    return FusionPoseSourceState::Unavailable;
}

}  // namespace

FusionPoseBus::FusionPoseBus(std::string stream_id,
                             std::uint64_t coordinate_epoch,
                             std::size_t boundary_capacity)
    : stream_id_{std::move(stream_id)},
      coordinate_epoch_{coordinate_epoch},
      boundary_capacity_{std::max<std::size_t>(boundary_capacity, 2)} {
    if (stream_id_.empty()) {
        throw std::invalid_argument("fusion pose stream id must not be empty");
    }
    if (coordinate_epoch_ == 0) {
        throw std::invalid_argument("fusion pose coordinate epoch must be non-zero");
    }
    snapshot_.stream_id = stream_id_;
    snapshot_.coordinate_epoch = coordinate_epoch_;
    snapshot_.continuity_epoch = continuity_epoch_;
}

FusionPoseFrame FusionPoseBus::make_base_locked(
    const PoseGateFrame& lifecycle,
    FusionPoseEventType event_type) const {
    FusionPoseFrame frame;
    frame.event_type = event_type;
    frame.stream_id = stream_id_;
    frame.subject_track_id = lifecycle.subject_track_id;
    frame.coordinate_epoch = lifecycle.coordinate_epoch;
    frame.continuity_epoch = continuity_epoch_;
    frame.source_state = map_lifecycle_state(lifecycle);
    frame.source_reason = lifecycle.source_reason;
    return frame;
}

FusionPoseFrame FusionPoseBus::make_boundary_locked(
    const PoseGateFrame& lifecycle,
    FusionPoseSourceState state,
    std::string reason) {
    auto frame = make_base_locked(lifecycle, FusionPoseEventType::Boundary);
    frame.source_state = state;
    frame.source_reason = std::move(reason);
    return frame;
}

void FusionPoseBus::commit_pose_locked(FusionPoseFrame frame) {
    frame.sample_seq = ++sample_seq_;
    frame.continuity_epoch = continuity_epoch_;
    frame.source_publish_mono_ns = fitra::util::monotonic_ns();
    snapshot_ = frame;
    latest_pose_ = std::move(frame);
    boundary_active_ = false;
    active_boundary_key_.clear();
}

void FusionPoseBus::insert_continuity_reset_locked(
    const PoseGateFrame& lifecycle,
    std::string reason) {
    boundaries_.clear();
    latest_pose_.reset();
    ++continuity_epoch_;
    auto overflow = make_boundary_locked(
        lifecycle, FusionPoseSourceState::ContinuityReset,
        std::move(reason));
    overflow.subject_track_id = "none";
    overflow.sample_seq = ++sample_seq_;
    overflow.continuity_epoch = continuity_epoch_;
    overflow.source_publish_mono_ns = fitra::util::monotonic_ns();
    snapshot_ = overflow;
    boundaries_.push_back(std::move(overflow));
    boundary_active_ = true;
    active_boundary_key_ = boundary_key(snapshot_);
}

void FusionPoseBus::synchronize_stream_locked(
    const PoseGateFrame& lifecycle) {
    if (lifecycle.stream_id.empty() || lifecycle.stream_id == stream_id_) return;
    stream_id_ = lifecycle.stream_id;
    coordinate_epoch_ = lifecycle.coordinate_epoch;
    last_subject_track_id_ = "none";
    insert_continuity_reset_locked(lifecycle, "stream_id_changed");
}

bool FusionPoseBus::duplicate_active_boundary_locked(
    const FusionPoseFrame& frame) const {
    return boundary_active_ && active_boundary_key_ == boundary_key(frame);
}

void FusionPoseBus::commit_boundary_locked(FusionPoseFrame frame) {
    if (duplicate_active_boundary_locked(frame)) return;
    if (boundaries_.size() >= boundary_capacity_) {
        PoseGateFrame lifecycle;
        lifecycle.stream_id = frame.stream_id;
        lifecycle.subject_track_id = frame.subject_track_id;
        lifecycle.coordinate_epoch = frame.coordinate_epoch;
        lifecycle.source_state = PoseGateSourceState::Unavailable;
        lifecycle.source_reason = frame.source_reason;
        insert_continuity_reset_locked(lifecycle, "boundary_queue_overflow");
    }
    frame.sample_seq = ++sample_seq_;
    frame.continuity_epoch = continuity_epoch_;
    frame.source_publish_mono_ns = fitra::util::monotonic_ns();
    snapshot_ = frame;
    boundaries_.push_back(std::move(frame));
    boundary_active_ = true;
    active_boundary_key_ = boundary_key(snapshot_);
    latest_pose_.reset();
}

FusionPoseFrame FusionPoseBus::observe(
    const lift::TriangulatedSkeleton& tri,
    const FusionCaptureInterval& capture,
    const PoseGateFrame& lifecycle,
    const infer::Skeleton3D* filtered_skeleton) {
    std::lock_guard<std::mutex> lock{mu_};
    synchronize_stream_locked(lifecycle);
    coordinate_epoch_ = lifecycle.coordinate_epoch;

    const bool identity_changed =
        last_subject_track_id_ != "none" &&
        lifecycle.subject_track_id != "none" &&
        last_subject_track_id_ != lifecycle.subject_track_id;
    if (lifecycle.source_state != PoseGateSourceState::Fresh || identity_changed) {
        auto state = map_lifecycle_state(lifecycle);
        auto reason = lifecycle.source_reason;
        if (identity_changed && state == FusionPoseSourceState::Fresh) {
            state = FusionPoseSourceState::PersonSwitched;
            reason = "subject_track_id_changed";
        }
        auto frame = make_boundary_locked(lifecycle, state, std::move(reason));
        commit_boundary_locked(std::move(frame));
        last_subject_track_id_ = lifecycle.subject_track_id;
        return snapshot_;
    }

    auto frame = make_base_locked(lifecycle, FusionPoseEventType::Pose);
    frame.capture = capture;
    for (std::size_t i = 0; i < kHalpeFusionIndices.size(); ++i) {
        if (i == static_cast<std::size_t>(FusionPoseJoint::Hips)) continue;
        const std::size_t source_idx = kHalpeFusionIndices[i];
        auto& dst = frame.joints[i];
        fill_raw_joint(tri, source_idx, dst);
        dst.filtered_position_m = filtered_position(filtered_skeleton, source_idx);
    }

    // HALPE26 has no independent camera hip-center observation.  Keep the
    // legacy ten-joint shape, but derive Hips only from the same-capture raw
    // left/right hip pair and aggregate its evidence conservatively.
    auto& hips = frame.joints[static_cast<std::size_t>(FusionPoseJoint::Hips)];
    const auto& left_hip =
        frame.joints[static_cast<std::size_t>(FusionPoseJoint::LeftHip)];
    const auto& right_hip =
        frame.joints[static_cast<std::size_t>(FusionPoseJoint::RightHip)];
    const bool raw_hips_observed =
        left_hip.observed_this_frame && right_hip.observed_this_frame &&
        left_hip.position_m && right_hip.position_m;
    if (raw_hips_observed) {
        hips.availability = PoseGateAvailability::Fresh;
        hips.observed_this_frame = true;
        hips.position_m = midpoint(*left_hip.position_m, *right_hip.position_m);
        hips.keypoint_score =
            std::min(*left_hip.keypoint_score, *right_hip.keypoint_score);
        hips.inlier_view_count =
            std::min(*left_hip.inlier_view_count, *right_hip.inlier_view_count);
        hips.mean_reproj_error_px = std::max(
            *left_hip.mean_reproj_error_px, *right_hip.mean_reproj_error_px);
        hips.max_ray_angle_deg =
            std::min(*left_hip.max_ray_angle_deg, *right_hip.max_ray_angle_deg);
    }
    if (left_hip.filtered_position_m && right_hip.filtered_position_m) {
        hips.filtered_position_m = midpoint(*left_hip.filtered_position_m,
                                            *right_hip.filtered_position_m);
    }
    commit_pose_locked(std::move(frame));
    last_subject_track_id_ = lifecycle.subject_track_id;
    return snapshot_;
}

FusionPoseFrame FusionPoseBus::publish_boundary(
    const PoseGateFrame& lifecycle) {
    std::lock_guard<std::mutex> lock{mu_};
    synchronize_stream_locked(lifecycle);
    coordinate_epoch_ = lifecycle.coordinate_epoch;
    auto frame = make_boundary_locked(lifecycle, map_lifecycle_state(lifecycle),
                                      lifecycle.source_reason);
    commit_boundary_locked(std::move(frame));
    if (lifecycle.subject_track_id != "none") {
        last_subject_track_id_ = lifecycle.subject_track_id;
    }
    return snapshot_;
}

FusionPoseFrame FusionPoseBus::snapshot() const {
    std::lock_guard<std::mutex> lock{mu_};
    return snapshot_;
}

std::string FusionPoseBus::make_json() const {
    std::lock_guard<std::mutex> lock{mu_};
    return serialize(snapshot_);
}

std::vector<std::string> FusionPoseBus::drain_pending_json() {
    std::lock_guard<std::mutex> lock{mu_};
    std::vector<std::string> out;
    out.reserve(boundaries_.size() + (latest_pose_ ? 1 : 0));
    while (!boundaries_.empty() || latest_pose_) {
        if (boundaries_.empty()) {
            out.push_back(serialize(*latest_pose_));
            latest_pose_.reset();
        } else if (!latest_pose_ ||
                   boundaries_.front().sample_seq < latest_pose_->sample_seq) {
            out.push_back(serialize(boundaries_.front()));
            boundaries_.pop_front();
        } else {
            out.push_back(serialize(*latest_pose_));
            latest_pose_.reset();
        }
    }
    return out;
}

std::uint64_t FusionPoseBus::continuity_epoch() const {
    std::lock_guard<std::mutex> lock{mu_};
    return continuity_epoch_;
}

std::size_t FusionPoseBus::pending_boundary_count() const {
    std::lock_guard<std::mutex> lock{mu_};
    return boundaries_.size();
}

std::string make_clock_sync_pong(
    std::uint64_t nonce,
    std::uint64_t client_send_mono_ns,
    std::uint64_t server_receive_mono_ns,
    std::uint64_t server_send_mono_ns) {
    return "{\"type\":\"clock_sync_pong\",\"nonce\":" +
           std::to_string(nonce) + ",\"client_send_mono_ns\":" +
           std::to_string(client_send_mono_ns) +
           ",\"server_receive_mono_ns\":" +
           std::to_string(server_receive_mono_ns) +
           ",\"server_send_mono_ns\":" +
           std::to_string(server_send_mono_ns) + "}";
}

}  // namespace fitra::pipeline
