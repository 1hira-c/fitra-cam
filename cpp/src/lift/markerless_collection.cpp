#include "lift/markerless_collection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace fitra::lift {
namespace {

constexpr std::size_t kMaxFrameSets = 1024;

std::string validate_intrinsics(const CalibrationSet& input) {
    if (input.cameras.size() < 2 || input.cameras.size() > 3)
        return "markerless geometry requires two or three cameras";
    auto intrinsics = input;
    std::set<std::string> ids;
    for (auto& camera : intrinsics.cameras) {
        if (!ids.insert(camera.id).second) return "duplicate camera id";
        // The previous extrinsics are not an input to this estimation.
        camera.has_extrinsics = false;
    }
    try {
        validate_calibration(intrinsics);
    } catch (const std::exception& error) {
        return error.what();
    }
    for (const auto& camera : intrinsics.cameras) {
        const auto& in = camera.intrinsics;
        if (std::abs(in.K.at<double>(0, 1)) > 1e-12
            || std::abs(in.K.at<double>(1, 0)) > 1e-12
            || std::abs(in.K.at<double>(2, 0)) > 1e-12
            || std::abs(in.K.at<double>(2, 1)) > 1e-12
            || std::abs(in.K.at<double>(2, 2) - 1.0) > 1e-12)
            return "unsupported camera matrix convention";
        const auto coefficients = in.dist.total();
        if (in.dist.rows != 1 && in.dist.cols != 1)
            return "distortion coefficients must be a vector";
        if (!in.is_fisheye() && coefficients != 4 && coefficients != 5
            && coefficients != 8 && coefficients != 12 && coefficients != 14)
            return "unsupported pinhole distortion coefficient count";
    }
    return {};
}

bool known_clock(MarkerlessTimestampKind kind) {
    return kind == MarkerlessTimestampKind::KernelMonotonicSoe
        || kind == MarkerlessTimestampKind::KernelMonotonicEof
        || kind == MarkerlessTimestampKind::SyntheticMonotonic;
}

}  // namespace

std::string validate_markerless_inputs(const CalibrationSet& intrinsics,
                                       const std::vector<MarkerlessFrameSet>& sets,
                                       double max_sync_skew_ms) {
    if (const auto error = validate_intrinsics(intrinsics); !error.empty()) return error;
    if (!std::isfinite(max_sync_skew_ms) || max_sync_skew_ms < 0.0
        || max_sync_skew_ms > 1000.0) return "invalid synchronization window";
    if (sets.size() > kMaxFrameSets) return "too many markerless frame sets";
    std::map<std::string, const Intrinsics*> cameras;
    for (const auto& camera : intrinsics.cameras) cameras.emplace(camera.id, &camera.intrinsics);
    std::map<std::string, const MarkerlessCameraFrame*> previous;
    MarkerlessTimestampKind clock = MarkerlessTimestampKind::Unknown;
    for (std::size_t sample = 0; sample < sets.size(); ++sample) {
        const auto& set = sets[sample];
        if (set.continuity_id.empty()) return "missing continuity id";
        if (sample && set.continuity_id != sets.front().continuity_id)
            return "continuity changed within observation set";
        if (sample && set.sample_id <= sets[sample - 1].sample_id)
            return "sample id is duplicated or regressed";
        if (set.frames.size() != cameras.size()) return "incomplete camera frame set";
        std::set<std::string> seen;
        auto oldest = std::numeric_limits<std::int64_t>::max();
        std::int64_t newest = 0;
        for (const auto& frame : set.frames) {
            const auto found = cameras.find(frame.camera_id);
            if (found == cameras.end()) return "unknown camera id";
            if (!seen.insert(frame.camera_id).second) return "duplicate camera in frame set";
            const auto& in = *found->second;
            if (frame.width != in.width || frame.height != in.height)
                return "frame resolution does not match intrinsics";
            if (frame.person_count != 1) return "exactly one person is required";
            if (frame.capture_mono_ns <= 0) return "capture timestamp is missing or invalid";
            if (!known_clock(frame.timestamp_kind)) return "unsupported timestamp semantics";
            if (clock == MarkerlessTimestampKind::Unknown) clock = frame.timestamp_kind;
            if (frame.timestamp_kind != clock) return "mixed timestamp semantics";
            if (const auto last = previous.find(frame.camera_id); last != previous.end()) {
                if (frame.stream_epoch != last->second->stream_epoch)
                    return "stream epoch changed within observation set";
                if (frame.source_seq <= last->second->source_seq)
                    return "source sequence is duplicated or regressed";
                if (frame.capture_mono_ns <= last->second->capture_mono_ns)
                    return "capture timestamp is duplicated or regressed";
            }
            if (frame.keypoints.size() > 26) return "too many Halpe26 keypoints";
            std::set<int> joints;
            for (const auto& point : frame.keypoints) {
                if (point.joint_index < 0 || point.joint_index >= 26
                    || !joints.insert(point.joint_index).second)
                    return "invalid or duplicated Halpe26 joint index";
                if (!std::isfinite(point.pixel.x) || !std::isfinite(point.pixel.y)
                    || !MarkerlessKeypoint::valid_score(point.score))
                    return "invalid keypoint value";
                if (point.pixel.x < 0.0 || point.pixel.x >= in.width
                    || point.pixel.y < 0.0 || point.pixel.y >= in.height)
                    return "keypoint is outside the calibrated image";
            }
            oldest = std::min(oldest, frame.capture_mono_ns);
            newest = std::max(newest, frame.capture_mono_ns);
            previous[frame.camera_id] = &frame;
        }
        if (static_cast<double>(newest - oldest) / 1e6 > max_sync_skew_ms)
            return "capture skew exceeds synchronization window";
    }
    return {};
}

MarkerlessCollector::MarkerlessCollector(CalibrationSet intrinsics,
                                         MarkerlessCollectionOptions options)
    : intrinsics_(std::move(intrinsics)), options_(options) {
    if (!options_.max_frame_sets || options_.max_frame_sets > kMaxFrameSets)
        throw std::invalid_argument("invalid markerless collection capacity");
    if (auto error = validate_markerless_inputs(intrinsics_, {}, options_.max_sync_skew_ms);
        !error.empty()) throw std::invalid_argument(error);
    for (auto& camera : intrinsics_.cameras) {
        camera.intrinsics.K = camera.intrinsics.K.clone();
        camera.intrinsics.dist = camera.intrinsics.dist.clone();
        camera.has_extrinsics = false;
    }
}

void MarkerlessCollector::reset() {
    sets_.clear();
    last_frames_.clear();
    continuity_id_.clear();
    have_sample_ = false;
    ++stats_.resets;
}

MarkerlessIngest MarkerlessCollector::ingest(const MarkerlessFrameSet& set,
                                            std::string& reason) {
    reason = validate_markerless_inputs(intrinsics_, {set}, options_.max_sync_skew_ms);
    if (!reason.empty()) {
        ++stats_.rejected;
        if (have_sample_) {
            reset();
            return MarkerlessIngest::Reset;
        }
        return MarkerlessIngest::Rejected;
    }
    bool boundary = have_sample_ && (set.continuity_id != continuity_id_
                                    || set.sample_id < last_sample_id_);
    bool duplicate = have_sample_ && set.sample_id == last_sample_id_;
    for (const auto& frame : set.frames) {
        const auto found = last_frames_.find(frame.camera_id);
        if (found == last_frames_.end()) continue;
        const auto& last = found->second;
        boundary |= frame.stream_epoch != last.stream_epoch
                 || frame.timestamp_kind != last.timestamp_kind
                 || frame.source_seq < last.source_seq
                 || frame.capture_mono_ns < last.capture_mono_ns;
        duplicate |= frame.source_seq == last.source_seq
                  || frame.capture_mono_ns == last.capture_mono_ns;
    }
    if (boundary) {
        reason = "continuity, stream or source clock boundary; collection reset";
        reset();
        ++stats_.rejected;
        return MarkerlessIngest::Reset;
    }
    if (duplicate) {
        reason = "duplicate sample or source frame";
        ++stats_.rejected;
        return MarkerlessIngest::Rejected;
    }
    if (sets_.size() == options_.max_frame_sets) {
        sets_.erase(sets_.begin());
        ++stats_.evicted;
    }
    sets_.push_back(set);
    continuity_id_ = set.continuity_id;
    last_sample_id_ = set.sample_id;
    have_sample_ = true;
    auto oldest = set.frames.front().capture_mono_ns;
    auto newest = oldest;
    for (const auto& frame : set.frames) {
        last_frames_[frame.camera_id] = frame;
        oldest = std::min(oldest, frame.capture_mono_ns);
        newest = std::max(newest, frame.capture_mono_ns);
    }
    stats_.max_accepted_skew_ms = std::max(stats_.max_accepted_skew_ms,
                                         static_cast<double>(newest - oldest) / 1e6);
    ++stats_.accepted;
    return MarkerlessIngest::Accepted;
}

}  // namespace fitra::lift
