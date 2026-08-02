#include "pipeline/pose_gate.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "lift/skeleton_def.hpp"
#include "util/clock.hpp"

namespace fitra::pipeline {

namespace {

constexpr std::array<std::size_t,
                     static_cast<std::size_t>(PoseGateJoint::Count)>
    kHalpeGateIndices{{
        lift::kHalpeHipCenter,
        lift::kHalpeNeck,
        lift::kHalpeLeftHip,
        lift::kHalpeRightHip,
        lift::kHalpeLeftKnee,
        lift::kHalpeRightKnee,
        lift::kHalpeLeftAnkle,
        lift::kHalpeRightAnkle,
    }};

std::string make_opaque_id(const char* prefix, std::uint64_t nonce) {
    static std::atomic<std::uint64_t> sequence{0};
    const std::uint64_t serial = sequence.fetch_add(1, std::memory_order_relaxed) + 1;
    std::ostringstream out;
    out << prefix << "-" << std::hex << fitra::util::monotonic_ns()
        << "-" << nonce << "-" << serial;
    return out.str();
}

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

double distance(const std::array<double, 3>& a,
                const std::array<double, 3>& b) {
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::optional<std::array<double, 3>> fresh_position(const PoseGateJointValue& j) {
    if (j.availability != PoseGateAvailability::Fresh || !j.position_m) return std::nullopt;
    return j.position_m;
}

double torso_length(const PoseGateFrame& frame) {
    const auto hips = fresh_position(
        frame.joints[static_cast<std::size_t>(PoseGateJoint::Hips)]);
    const auto neck = fresh_position(
        frame.joints[static_cast<std::size_t>(PoseGateJoint::Neck)]);
    if (!hips || !neck) return 0.0;
    return distance(*hips, *neck);
}

std::string serialize(const PoseGateFrame& frame) {
    std::string out;
    out.reserve(2400);
    out += "{\"protocol_version\":";
    append_json_string(out, frame.protocol_version);
    out += ",\"sample_seq\":" + std::to_string(frame.sample_seq);
    out += ",\"stream_id\":";
    append_json_string(out, frame.stream_id);
    out += ",\"subject_track_id\":";
    append_json_string(out, frame.subject_track_id);
    out += ",\"coordinate_epoch\":" + std::to_string(frame.coordinate_epoch);
    out += ",\"content_mono_ns\":";
    if (frame.content_mono_ns) out += std::to_string(*frame.content_mono_ns);
    else out += "null";
    out += ",\"provenance\":{\"pipeline\":\"fitra-cam\""
           ",\"stage\":\"tri.skeleton\""
           ",\"position_source\":\"multi_view_triangulation\""
           ",\"postprocess\":\"none\""
           ",\"kalman\":false,\"ik\":false,\"floor_contact\":false}";
    out += ",\"source_state\":";
    append_json_string(out, pose_gate_source_state_name(frame.source_state));
    out += ",\"source_reason\":";
    append_json_string(out, frame.source_reason);
    out += ",\"position_space\":";
    append_json_string(out, kPoseGatePositionSpace);
    out += ",\"joints\":{";
    for (std::size_t i = 0; i < static_cast<std::size_t>(PoseGateJoint::Count); ++i) {
        if (i) out += ',';
        append_json_string(out, pose_gate_joint_name(static_cast<PoseGateJoint>(i)));
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
        out += ",\"availability\":";
        append_json_string(out, joint.availability == PoseGateAvailability::Fresh
                                  ? "Fresh" : "Unavailable");
        out += ",\"keypoint_score\":";
        append_optional_number(out, joint.keypoint_score);
        out += ",\"view_count\":";
        if (joint.view_count) out += std::to_string(*joint.view_count);
        else out += "null";
        out += ",\"reproj_error\":";
        append_optional_number(out, joint.reproj_error);
        out += '}';
    }
    out += "}}";
    return out;
}

}  // namespace

const char* pose_gate_source_state_name(PoseGateSourceState state) {
    switch (state) {
        case PoseGateSourceState::Fresh:                return "Fresh";
        case PoseGateSourceState::Reacquired:           return "Reacquired";
        case PoseGateSourceState::PersonSwitched:       return "PersonSwitched";
        case PoseGateSourceState::Unavailable:          return "Unavailable";
        case PoseGateSourceState::EpochChanged:         return "EpochChanged";
        case PoseGateSourceState::UnsupportedTopology:  return "UnsupportedTopology";
        case PoseGateSourceState::UnsupportedMultiPerson:
            return "UnsupportedMultiPerson";
    }
    return "Unavailable";
}

const char* pose_gate_joint_name(PoseGateJoint joint) {
    switch (joint) {
        case PoseGateJoint::Hips:       return "hips";
        case PoseGateJoint::Neck:       return "neck";
        case PoseGateJoint::LeftHip:    return "left_hip";
        case PoseGateJoint::RightHip:   return "right_hip";
        case PoseGateJoint::LeftKnee:   return "left_knee";
        case PoseGateJoint::RightKnee:  return "right_knee";
        case PoseGateJoint::LeftAnkle:  return "left_ankle";
        case PoseGateJoint::RightAnkle: return "right_ankle";
        case PoseGateJoint::Count:      break;
    }
    return "unknown";
}

PoseGateBus::PoseGateBus(std::string stream_id,
                         std::uint64_t coordinate_epoch)
    : stream_id_{stream_id.empty() ? make_opaque_id("stream", 0)
                                   : std::move(stream_id)},
      coordinate_epoch_{coordinate_epoch} {
    if (coordinate_epoch_ == 0) {
        throw std::invalid_argument("pose gate coordinate epoch must be non-zero");
    }
    snapshot_.protocol_version = kPoseGateProtocolVersion;
    snapshot_.stream_id = stream_id_;
    snapshot_.coordinate_epoch = coordinate_epoch_;
    snapshot_.source_state = PoseGateSourceState::Unavailable;
    snapshot_.source_reason = "not_started";
}

PoseGateFrame PoseGateBus::make_base_locked(
    std::optional<std::uint64_t> content_mono_ns) const {
    PoseGateFrame frame;
    frame.protocol_version = kPoseGateProtocolVersion;
    frame.stream_id = stream_id_;
    frame.subject_track_id = track_active_ ? track_id_ : "none";
    frame.coordinate_epoch = coordinate_epoch_;
    frame.content_mono_ns = content_mono_ns;
    return frame;
}

std::string PoseGateBus::make_track_id_locked() {
    return make_opaque_id("subject", ++track_nonce_);
}

PoseGateFrame PoseGateBus::unavailable_locked(
    std::optional<std::uint64_t> content_mono_ns,
    PoseGateSourceState state,
    const std::string& reason) {
    PoseGateFrame frame = make_base_locked(content_mono_ns);
    frame.source_state = state;
    frame.source_reason = reason.empty()
        ? pose_gate_source_state_name(state) : reason;
    if (state == PoseGateSourceState::EpochChanged ||
        state == PoseGateSourceState::UnsupportedTopology ||
        state == PoseGateSourceState::UnsupportedMultiPerson) {
        frame.subject_track_id = "none";
    }
    for (auto& joint : frame.joints) {
        joint = PoseGateJointValue{};
    }
    track_active_ = false;
    track_id_.clear();
    previous_hips_.reset();
    previous_torso_length_m_.reset();
    previous_content_mono_ns_.reset();
    had_unavailable_ = true;
    commit_locked(frame);
    return frame;
}

void PoseGateBus::commit_locked(PoseGateFrame frame) {
    frame.sample_seq = ++sample_seq_;
    snapshot_ = std::move(frame);
}

bool PoseGateBus::looks_like_person_switch_locked(
    const PoseGateFrame& current,
    std::optional<std::uint64_t> content_mono_ns) const {
    const auto current_hips = fresh_position(
        current.joints[static_cast<std::size_t>(PoseGateJoint::Hips)]);
    const double current_torso = torso_length(current);
    if (!current_hips || !previous_hips_ || !current_torso ||
        !previous_torso_length_m_) {
        return false;
    }

    double dt_s = 0.0;
    if (content_mono_ns && previous_content_mono_ns_ &&
        *content_mono_ns > *previous_content_mono_ns_) {
        dt_s = static_cast<double>(*content_mono_ns - *previous_content_mono_ns_)
             / 1.0e9;
    }
    // A normal person can move faster after a dropped frame, but an abrupt
    // multi-camera identity change should still break the hold. Keep this
    // threshold conservative; this is a reset gate, not a motion predictor.
    const double max_jump_m = 0.75 + std::min(1.5, 3.0 * dt_s);
    const double scale_ratio = current_torso / *previous_torso_length_m_;
    return distance(*current_hips, *previous_hips_) > max_jump_m
        || scale_ratio < 0.55 || scale_ratio > 1.80;
}

PoseGateFrame PoseGateBus::observe(
    const lift::TriangulatedSkeleton& tri,
    std::optional<std::uint64_t> content_mono_ns,
    bool single_subject) {
    std::lock_guard<std::mutex> lock{mu_};

    if (epoch_boundary_pending_) {
        epoch_boundary_pending_ = false;
        return unavailable_locked(content_mono_ns,
                                  PoseGateSourceState::EpochChanged,
                                  "coordinate_epoch_changed");
    }
    if (!single_subject) {
        return unavailable_locked(content_mono_ns,
                                  PoseGateSourceState::UnsupportedMultiPerson,
                                  "m0_requires_single_subject");
    }
    if (tri.skeleton.kp_count <= lift::kHalpeHipCenter) {
        return unavailable_locked(content_mono_ns,
                                  PoseGateSourceState::UnsupportedTopology,
                                  "halpe26_required");
    }

    PoseGateFrame current = make_base_locked(content_mono_ns);
    bool any_fresh = false;
    for (std::size_t i = 0; i < kHalpeGateIndices.size(); ++i) {
        const std::size_t source_idx = kHalpeGateIndices[i];
        const auto& source = tri.skeleton.joints[source_idx];
        auto& dst = current.joints[i];
        if (!source.valid || !std::isfinite(source.x) ||
            !std::isfinite(source.y) || !std::isfinite(source.z) ||
            !std::isfinite(source.score) || tri.view_count[source_idx] <= 0 ||
            !std::isfinite(tri.reproj_error_px[source_idx])) {
            continue;
        }
        dst.availability = PoseGateAvailability::Fresh;
        dst.position_m = std::array<double, 3>{
            static_cast<double>(source.x), static_cast<double>(source.y),
            static_cast<double>(source.z)};
        dst.keypoint_score = static_cast<double>(source.score);
        dst.view_count = tri.view_count[source_idx];
        dst.reproj_error = static_cast<double>(tri.reproj_error_px[source_idx]);
        any_fresh = true;
    }
    if (!any_fresh) {
        return unavailable_locked(content_mono_ns,
                                  PoseGateSourceState::Unavailable,
                                  "no_triangulated_gate_joint");
    }

    const bool has_anchor =
        current.joints[static_cast<std::size_t>(PoseGateJoint::Hips)].availability
            == PoseGateAvailability::Fresh
        && current.joints[static_cast<std::size_t>(PoseGateJoint::Neck)].availability
            == PoseGateAvailability::Fresh;
    if (!track_active_ && !has_anchor) {
        return unavailable_locked(content_mono_ns,
                                  PoseGateSourceState::Unavailable,
                                  "no_stable_subject_anchor");
    }

    PoseGateSourceState state = PoseGateSourceState::Fresh;
    if (track_active_ && looks_like_person_switch_locked(current, content_mono_ns)) {
        track_active_ = false;
        track_id_.clear();
        previous_hips_.reset();
        previous_torso_length_m_.reset();
        previous_content_mono_ns_.reset();
        state = PoseGateSourceState::PersonSwitched;
    }
    if (!track_active_) {
        track_active_ = true;
        track_id_ = make_track_id_locked();
        current.subject_track_id = track_id_;
        if (state != PoseGateSourceState::PersonSwitched &&
            ever_observed_ && had_unavailable_) {
            state = PoseGateSourceState::Reacquired;
        }
        ever_observed_ = true;
        had_unavailable_ = false;
    } else {
        current.subject_track_id = track_id_;
    }
    current.source_state = state;
    current.source_reason = pose_gate_source_state_name(state);

    const auto hips = fresh_position(
        current.joints[static_cast<std::size_t>(PoseGateJoint::Hips)]);
    const double torso = torso_length(current);
    if (hips && torso > 0.0) {
        previous_hips_ = *hips;
        previous_torso_length_m_ = torso;
        previous_content_mono_ns_ = content_mono_ns;
    }
    commit_locked(current);
    return current;
}

PoseGateFrame PoseGateBus::publish_unavailable(
    std::optional<std::uint64_t> content_mono_ns,
    PoseGateSourceState state,
    std::string reason) {
    std::lock_guard<std::mutex> lock{mu_};
    return unavailable_locked(content_mono_ns, state, reason);
}

void PoseGateBus::set_coordinate_epoch(std::uint64_t epoch) {
    if (epoch == 0) throw std::invalid_argument("pose gate coordinate epoch must be non-zero");
    std::lock_guard<std::mutex> lock{mu_};
    if (coordinate_epoch_ == epoch) return;
    coordinate_epoch_ = epoch;
    epoch_boundary_pending_ = true;
    track_active_ = false;
    track_id_.clear();
    previous_hips_.reset();
    previous_torso_length_m_.reset();
    previous_content_mono_ns_.reset();
}

std::uint64_t PoseGateBus::coordinate_epoch() const {
    std::lock_guard<std::mutex> lock{mu_};
    return coordinate_epoch_;
}

std::string PoseGateBus::stream_id() const {
    std::lock_guard<std::mutex> lock{mu_};
    return stream_id_;
}

PoseGateFrame PoseGateBus::snapshot() const {
    std::lock_guard<std::mutex> lock{mu_};
    return snapshot_;
}

std::string PoseGateBus::make_json() const {
    std::lock_guard<std::mutex> lock{mu_};
    return serialize(snapshot_);
}

bool PoseGateBus::make_json_if_new(std::uint64_t& last_sent_seq,
                                   std::string& out) const {
    std::lock_guard<std::mutex> lock{mu_};
    if (snapshot_.sample_seq == 0 || snapshot_.sample_seq == last_sent_seq) {
        return false;
    }
    out = serialize(snapshot_);
    last_sent_seq = snapshot_.sample_seq;
    return true;
}

}  // namespace fitra::pipeline
