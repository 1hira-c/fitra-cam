#include "tracking/tracker_axis.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include "camera/v4l2_capture.hpp"
#include "util/clock.hpp"

namespace fitra::tracking {

namespace {

using SourceJoint = pipeline::TrackerAxisSourceJoint;

constexpr std::array<TrackerRole,
                     static_cast<std::size_t>(TrackerAxisRole::Count)>
    kTrackerRoles{{
        TrackerRole::Chest,
        TrackerRole::Waist,
        TrackerRole::LeftUpperLeg,
        TrackerRole::RightUpperLeg,
        TrackerRole::LeftLowerLeg,
        TrackerRole::RightLowerLeg,
    }};

constexpr std::array<std::array<SourceJoint, 2>,
                     static_cast<std::size_t>(TrackerAxisRole::Count)>
    kRequiredJoints{{
        {SourceJoint::LeftShoulder, SourceJoint::RightShoulder},
        {SourceJoint::LeftHip, SourceJoint::RightHip},
        {SourceJoint::LeftHip, SourceJoint::LeftKnee},
        {SourceJoint::RightHip, SourceJoint::RightKnee},
        {SourceJoint::LeftKnee, SourceJoint::LeftAnkle},
        {SourceJoint::RightKnee, SourceJoint::RightAnkle},
    }};

void append_json_string(std::string& out, const std::string& value) {
    out += '"';
    for (const unsigned char ch : value) {
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
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.9g", value);
    out += buf;
}

std::string serialize(const TrackerAxisFrame& frame) {
    std::string out;
    out.reserve(1500);
    out += "{\"protocol_version\":\"fitra_tracker_axis_v1\"";
    out += ",\"delivery_seq\":" + std::to_string(frame.delivery_seq);
    out += ",\"source_sample_seq\":" +
           std::to_string(frame.source_sample_seq);
    out += ",\"source_state\":\"";
    out += frame.fresh ? "fresh\"" : "boundary\"";
    out += ",\"stream_id\":";
    append_json_string(out, frame.stream_id);
    out += ",\"subject_track_id\":";
    append_json_string(out, frame.subject_track_id);
    out += ",\"coordinate_epoch\":" +
           std::to_string(frame.coordinate_epoch);
    out += ",\"continuity_epoch\":" +
           std::to_string(frame.continuity_epoch);
    out += ",\"source_publish_mono_ns\":" +
           std::to_string(frame.source_publish_mono_ns);
    if (!frame.fresh) {
        out += ",\"boundary\":";
        append_json_string(out, tracker_axis_boundary_name(frame.boundary));
        out += '}';
        return out;
    }

    out += ",\"capture\":{\"oldest_mono_ns\":" +
           std::to_string(*frame.capture.oldest_mono_ns);
    out += ",\"newest_mono_ns\":" +
           std::to_string(*frame.capture.newest_mono_ns);
    out += ",\"timestamp_semantics\":";
    append_json_string(out, camera::v4l2_timestamp_semantics_name(
                                frame.capture.semantics));
    out += "},\"axes\":[";
    for (std::size_t i = 0; i < frame.axes.size(); ++i) {
        if (i) out += ',';
        const auto& axis = frame.axes[i];
        out += "{\"role\":";
        append_json_string(out, tracker_axis_role_name(axis.role));
        out += ",\"availability\":\"";
        out += axis.availability == TrackerAxisAvailability::Fresh
                   ? "fresh\"" : "unavailable\"";
        out += ",\"observed_this_frame\":";
        out += axis.observed_this_frame ? "true" : "false";
        out += ",\"axis\":";
        if (!axis.axis) {
            out += "null";
        } else {
            out += '[';
            append_number(out, (*axis.axis)[0]); out += ',';
            append_number(out, (*axis.axis)[1]); out += ',';
            append_number(out, (*axis.axis)[2]); out += ']';
        }
        out += '}';
    }
    out += "]}";
    return out;
}

bool accepted_capture(const pipeline::FusionCaptureInterval& capture) {
    const bool semantics_ok =
        capture.semantics == camera::V4l2TimestampSemantics::MonotonicSoe ||
        capture.semantics == camera::V4l2TimestampSemantics::MonotonicEof;
    return semantics_ok && capture.oldest_mono_ns && capture.newest_mono_ns &&
           *capture.newest_mono_ns >= *capture.oldest_mono_ns;
}

bool finite(double value) {
    return std::isfinite(value);
}

std::optional<std::array<double, 3>> rotate_basis(
    const cv::Vec4f& quat, TrackerAxisRole role) {
    double w = quat[0];
    double x = quat[1];
    double y = quat[2];
    double z = quat[3];
    if (!finite(w) || !finite(x) || !finite(y) || !finite(z)) {
        return std::nullopt;
    }
    const double qnorm = std::sqrt(w*w + x*x + y*y + z*z);
    if (!(qnorm > 1e-9) || !finite(qnorm)) return std::nullopt;
    w /= qnorm; x /= qnorm; y /= qnorm; z /= qnorm;

    const bool torso = role == TrackerAxisRole::Chest ||
                       role == TrackerAxisRole::Hips;
    const double vx = torso ? 1.0 : 0.0;
    const double vy = 0.0;
    const double vz = torso ? 0.0 : 1.0;
    // Unit-quaternion vector rotation, expanded to avoid matrix allocation.
    const double tx = 2.0 * (y*vz - z*vy);
    const double ty = 2.0 * (z*vx - x*vz);
    const double tz = 2.0 * (x*vy - y*vx);
    double ox = vx + w*tx + (y*tz - z*ty);
    double oy = vy + w*ty + (z*tx - x*tz);
    double oz = vz + w*tz + (x*ty - y*tx);
    if (torso) { ox = -ox; oy = -oy; oz = -oz; }
    const double norm = std::sqrt(ox*ox + oy*oy + oz*oz);
    if (!(norm > 1e-9) || !finite(norm)) return std::nullopt;
    ox /= norm; oy /= norm; oz /= norm;
    if (!finite(ox) || !finite(oy) || !finite(oz)) return std::nullopt;
    return std::array<double, 3>{ox, oy, oz};
}

bool observed(const pipeline::TrackerAxisLineage& lineage,
              SourceJoint joint) {
    return lineage.observed[static_cast<std::size_t>(joint)];
}

TrackerAxisBoundary map_boundary(
    const pipeline::TrackerAxisLineage& lineage) {
    switch (lineage.source_state) {
        case pipeline::FusionPoseSourceState::PersonSwitched:
            return TrackerAxisBoundary::SubjectChanged;
        case pipeline::FusionPoseSourceState::EpochChanged:
            return TrackerAxisBoundary::CoordinateChanged;
        case pipeline::FusionPoseSourceState::ContinuityReset:
            return TrackerAxisBoundary::ContinuityReset;
        case pipeline::FusionPoseSourceState::UnsupportedTopology:
        case pipeline::FusionPoseSourceState::UnsupportedMultiPerson:
            return TrackerAxisBoundary::SourceEnded;
        case pipeline::FusionPoseSourceState::Unavailable:
            if (lineage.source_reason == "idle" ||
                lineage.source_reason == "source_ended") {
                return TrackerAxisBoundary::SourceEnded;
            }
            return TrackerAxisBoundary::PersonLost;
        case pipeline::FusionPoseSourceState::Reacquired:
        case pipeline::FusionPoseSourceState::Fresh:
            return TrackerAxisBoundary::PersonLost;
    }
    return TrackerAxisBoundary::SourceEnded;
}

std::string boundary_key(const TrackerAxisFrame& frame) {
    return std::to_string(static_cast<int>(frame.boundary)) + "|" +
           frame.stream_id + "|" + frame.subject_track_id + "|" +
           std::to_string(frame.coordinate_epoch) + "|" +
           std::to_string(frame.continuity_epoch);
}

}  // namespace

const char* tracker_axis_role_name(TrackerAxisRole role) {
    switch (role) {
        case TrackerAxisRole::Chest: return "chest";
        case TrackerAxisRole::Hips: return "hips";
        case TrackerAxisRole::LeftUpperLeg: return "left_upper_leg";
        case TrackerAxisRole::RightUpperLeg: return "right_upper_leg";
        case TrackerAxisRole::LeftLowerLeg: return "left_lower_leg";
        case TrackerAxisRole::RightLowerLeg: return "right_lower_leg";
        case TrackerAxisRole::Count: break;
    }
    return "unknown";
}

const char* tracker_axis_boundary_name(TrackerAxisBoundary boundary) {
    switch (boundary) {
        case TrackerAxisBoundary::PersonLost: return "person_lost";
        case TrackerAxisBoundary::StreamChanged: return "stream_changed";
        case TrackerAxisBoundary::SubjectChanged: return "subject_changed";
        case TrackerAxisBoundary::CoordinateChanged:
            return "coordinate_changed";
        case TrackerAxisBoundary::ContinuityReset: return "continuity_reset";
        case TrackerAxisBoundary::SourceEnded: return "source_ended";
        case TrackerAxisBoundary::UnsupportedTimestamp:
            return "unsupported_timestamp";
    }
    return "source_ended";
}

TrackerAxisBus::TrackerAxisBus(std::string stream_id,
                               std::uint64_t coordinate_epoch,
                               std::size_t boundary_capacity)
    : boundary_capacity_{std::max<std::size_t>(boundary_capacity, 2)},
      continuity_epoch_{1},
      last_stream_id_{stream_id},
      last_coordinate_epoch_{coordinate_epoch},
      last_source_stream_id_{stream_id} {
    if (stream_id.empty()) {
        throw std::invalid_argument("tracker axis stream id must not be empty");
    }
    if (coordinate_epoch == 0) {
        throw std::invalid_argument(
            "tracker axis coordinate epoch must be non-zero");
    }
    snapshot_.delivery_seq = ++delivery_seq_;
    snapshot_.source_sample_seq = 1;
    snapshot_.fresh = false;
    snapshot_.stream_id = std::move(stream_id);
    snapshot_.subject_track_id = "none";
    snapshot_.coordinate_epoch = coordinate_epoch;
    snapshot_.continuity_epoch = continuity_epoch_;
    snapshot_.source_publish_mono_ns = fitra::util::monotonic_ns();
    snapshot_.boundary = TrackerAxisBoundary::SourceEnded;
}

TrackerAxisFrame TrackerAxisBus::make_base_locked(
    const pipeline::TrackerAxisLineage& lineage, bool fresh_frame) const {
    TrackerAxisFrame frame;
    frame.source_sample_seq = lineage.source_sample_seq;
    frame.fresh = fresh_frame;
    frame.stream_id = lineage.stream_id;
    frame.subject_track_id = lineage.subject_track_id;
    frame.coordinate_epoch = lineage.coordinate_epoch;
    frame.continuity_epoch = continuity_epoch_;
    frame.source_publish_mono_ns = lineage.source_publish_mono_ns;
    frame.capture = lineage.capture;
    for (std::size_t i = 0; i < frame.axes.size(); ++i) {
        frame.axes[i].role = static_cast<TrackerAxisRole>(i);
    }
    return frame;
}

TrackerAxisFrame TrackerAxisBus::make_boundary_locked(
    const pipeline::TrackerAxisLineage& lineage,
    TrackerAxisBoundary boundary) const {
    auto frame = make_base_locked(lineage, false);
    frame.boundary = boundary;
    return frame;
}

bool TrackerAxisBus::duplicate_active_boundary_locked(
    const TrackerAxisFrame& frame) const {
    return boundary_active_ && active_boundary_key_ == boundary_key(frame);
}

void TrackerAxisBus::collapse_overflow_locked(
    const pipeline::TrackerAxisLineage& lineage) {
    boundaries_.clear();
    latest_fresh_.reset();
    continuity_epoch_ = std::max(continuity_epoch_, lineage.continuity_epoch);
    ++continuity_epoch_;
    auto reset = make_boundary_locked(
        lineage, TrackerAxisBoundary::ContinuityReset);
    reset.subject_track_id = "none";
    reset.continuity_epoch = continuity_epoch_;
    reset.delivery_seq = ++delivery_seq_;
    snapshot_ = reset;
    boundaries_.push_back(std::move(reset));
    boundary_active_ = true;
    active_boundary_key_ = boundary_key(snapshot_);
}

bool TrackerAxisBus::commit_boundary_locked(TrackerAxisFrame frame) {
    if (duplicate_active_boundary_locked(frame)) return true;
    if (boundaries_.size() >= boundary_capacity_) {
        pipeline::TrackerAxisLineage overflow;
        overflow.source_sample_seq = frame.source_sample_seq;
        overflow.event_type = pipeline::FusionPoseEventType::Boundary;
        overflow.source_state =
            pipeline::FusionPoseSourceState::ContinuityReset;
        overflow.source_reason = "boundary_queue_overflow";
        overflow.stream_id = frame.stream_id;
        overflow.subject_track_id = frame.subject_track_id;
        overflow.coordinate_epoch = frame.coordinate_epoch;
        overflow.continuity_epoch = frame.continuity_epoch;
        overflow.source_publish_mono_ns = frame.source_publish_mono_ns;
        overflow.capture = frame.capture;
        collapse_overflow_locked(overflow);
        return false;
    }
    frame.delivery_seq = ++delivery_seq_;
    frame.continuity_epoch = continuity_epoch_;
    snapshot_ = frame;
    boundaries_.push_back(std::move(frame));
    latest_fresh_.reset();
    boundary_active_ = true;
    active_boundary_key_ = boundary_key(snapshot_);
    return true;
}

void TrackerAxisBus::commit_fresh_locked(TrackerAxisFrame frame) {
    frame.delivery_seq = ++delivery_seq_;
    frame.continuity_epoch = continuity_epoch_;
    snapshot_ = frame;
    latest_fresh_ = std::move(frame);
    boundary_active_ = false;
    active_boundary_key_.clear();
}

TrackerAxisFrame TrackerAxisBus::publish(
    const std::array<TrackerPose, kTrackerCount>& trackers,
    const std::optional<pipeline::TrackerAxisLineage>& maybe_lineage) {
    std::lock_guard<std::mutex> lock{mu_};
    if (!maybe_lineage) return snapshot_;
    const auto& lineage = *maybe_lineage;
    if (lineage.source_sample_seq == 0 || lineage.stream_id.empty() ||
        lineage.coordinate_epoch == 0 || lineage.continuity_epoch == 0 ||
        lineage.source_publish_mono_ns == 0) {
        return snapshot_;
    }
    // A boundary can arrive through the sidecar after this loop captured an
    // older latest skeleton. Never let that stale snapshot re-open Fresh.
    if (lineage.stream_id == last_source_stream_id_ &&
        lineage.source_sample_seq <= last_source_sample_seq_) {
        return snapshot_;
    }

    continuity_epoch_ = std::max(continuity_epoch_, lineage.continuity_epoch);
    if (have_source_) {
        const std::array<std::pair<bool, TrackerAxisBoundary>, 4> changes{{
            {lineage.stream_id != last_stream_id_,
             TrackerAxisBoundary::StreamChanged},
            {lineage.subject_track_id != "none" &&
                 lineage.subject_track_id != last_subject_track_id_,
             TrackerAxisBoundary::SubjectChanged},
            {lineage.coordinate_epoch != last_coordinate_epoch_,
             TrackerAxisBoundary::CoordinateChanged},
            {lineage.continuity_epoch != last_source_continuity_epoch_,
             TrackerAxisBoundary::ContinuityReset},
        }};
        for (const auto& [changed, boundary] : changes) {
            if (changed && !commit_boundary_locked(
                               make_boundary_locked(lineage, boundary))) {
                last_stream_id_ = lineage.stream_id;
                last_subject_track_id_ = lineage.subject_track_id;
                last_coordinate_epoch_ = lineage.coordinate_epoch;
                last_source_continuity_epoch_ = lineage.continuity_epoch;
                last_source_stream_id_ = lineage.stream_id;
                last_source_sample_seq_ = lineage.source_sample_seq;
                return snapshot_;
            }
        }
    }

    last_stream_id_ = lineage.stream_id;
    last_subject_track_id_ = lineage.subject_track_id;
    last_coordinate_epoch_ = lineage.coordinate_epoch;
    last_source_continuity_epoch_ = lineage.continuity_epoch;
    last_source_stream_id_ = lineage.stream_id;
    last_source_sample_seq_ = lineage.source_sample_seq;
    have_source_ = true;

    if (lineage.event_type != pipeline::FusionPoseEventType::Pose ||
        lineage.source_state != pipeline::FusionPoseSourceState::Fresh) {
        commit_boundary_locked(
            make_boundary_locked(lineage, map_boundary(lineage)));
        return snapshot_;
    }
    if (!accepted_capture(lineage.capture)) {
        commit_boundary_locked(make_boundary_locked(
            lineage, TrackerAxisBoundary::UnsupportedTimestamp));
        return snapshot_;
    }

    auto frame = make_base_locked(lineage, true);
    for (std::size_t i = 0; i < frame.axes.size(); ++i) {
        auto& dst = frame.axes[i];
        const auto& required = kRequiredJoints[i];
        const bool raw_observed = observed(lineage, required[0]) &&
                                  observed(lineage, required[1]);
        const auto& tracker = trackers[static_cast<std::size_t>(kTrackerRoles[i])];
        if (!raw_observed || !tracker.valid) continue;
        auto axis = rotate_basis(tracker.quat_wxyz, dst.role);
        if (!axis) continue;
        dst.availability = TrackerAxisAvailability::Fresh;
        dst.observed_this_frame = true;
        dst.axis = std::move(axis);
    }
    commit_fresh_locked(std::move(frame));
    return snapshot_;
}

TrackerAxisFrame TrackerAxisBus::snapshot() const {
    std::lock_guard<std::mutex> lock{mu_};
    return snapshot_;
}

std::string TrackerAxisBus::make_json() const {
    std::lock_guard<std::mutex> lock{mu_};
    return serialize(snapshot_);
}

std::vector<std::string> TrackerAxisBus::drain_pending_json() {
    std::lock_guard<std::mutex> lock{mu_};
    std::vector<TrackerAxisFrame> pending;
    pending.reserve(boundaries_.size() + (latest_fresh_ ? 1 : 0));
    while (!boundaries_.empty()) {
        pending.push_back(std::move(boundaries_.front()));
        boundaries_.pop_front();
    }
    if (latest_fresh_) {
        pending.push_back(std::move(*latest_fresh_));
        latest_fresh_.reset();
    }
    std::sort(pending.begin(), pending.end(),
              [](const auto& a, const auto& b) {
                  return a.delivery_seq < b.delivery_seq;
              });
    std::vector<std::string> out;
    out.reserve(pending.size());
    for (const auto& frame : pending) out.push_back(serialize(frame));
    return out;
}

std::uint64_t TrackerAxisBus::continuity_epoch() const {
    std::lock_guard<std::mutex> lock{mu_};
    return continuity_epoch_;
}

std::size_t TrackerAxisBus::pending_boundary_count() const {
    std::lock_guard<std::mutex> lock{mu_};
    return boundaries_.size();
}

}  // namespace fitra::tracking
