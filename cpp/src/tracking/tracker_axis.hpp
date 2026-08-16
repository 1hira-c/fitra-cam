#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "pipeline/tracker_axis_lineage.hpp"
#include "tracking/tracker_extract.hpp"

namespace fitra::tracking {

inline constexpr const char* kTrackerAxisProtocolVersion =
    "fitra_tracker_axis_v1";

enum class TrackerAxisRole : std::size_t {
    Chest = 0,
    Hips,
    LeftUpperLeg,
    RightUpperLeg,
    LeftLowerLeg,
    RightLowerLeg,
    Count,
};

enum class TrackerAxisAvailability {
    Fresh,
    Unavailable,
};

enum class TrackerAxisBoundary {
    PersonLost,
    StreamChanged,
    SubjectChanged,
    CoordinateChanged,
    ContinuityReset,
    SourceEnded,
    UnsupportedTimestamp,
};

struct TrackerAxisValue {
    TrackerAxisRole role = TrackerAxisRole::Chest;
    TrackerAxisAvailability availability =
        TrackerAxisAvailability::Unavailable;
    bool observed_this_frame = false;
    std::optional<std::array<double, 3>> axis;
};

struct TrackerAxisFrame {
    std::uint64_t delivery_seq = 0;
    std::uint64_t source_sample_seq = 0;
    bool fresh = false;
    std::string stream_id;
    std::string subject_track_id = "none";
    std::uint64_t coordinate_epoch = 1;
    std::uint64_t continuity_epoch = 1;
    std::uint64_t source_publish_mono_ns = 0;
    pipeline::FusionCaptureInterval capture{};
    std::array<TrackerAxisValue,
               static_cast<std::size_t>(TrackerAxisRole::Count)> axes{};
    TrackerAxisBoundary boundary = TrackerAxisBoundary::SourceEnded;
};

const char* tracker_axis_role_name(TrackerAxisRole role);
const char* tracker_axis_boundary_name(TrackerAxisBoundary boundary);

// Single producer for the additive D50 tracker-axis wire.  Normal documents
// are latest-only; lifecycle documents use an ordered bounded queue.  The
// input trackers must be the post-One-Euro TrackerExtractor output.
class TrackerAxisBus {
public:
    explicit TrackerAxisBus(std::string stream_id,
                            std::uint64_t coordinate_epoch = 1,
                            std::size_t boundary_capacity = 32);

    TrackerAxisFrame publish(
        const std::array<TrackerPose, kTrackerCount>& trackers,
        const std::optional<pipeline::TrackerAxisLineage>& lineage);

    TrackerAxisFrame snapshot() const;
    std::string make_json() const;
    std::vector<std::string> drain_pending_json();

    std::uint64_t continuity_epoch() const;
    std::size_t pending_boundary_count() const;

private:
    TrackerAxisFrame make_base_locked(
        const pipeline::TrackerAxisLineage& lineage, bool fresh) const;
    TrackerAxisFrame make_boundary_locked(
        const pipeline::TrackerAxisLineage& lineage,
        TrackerAxisBoundary boundary) const;
    bool commit_boundary_locked(TrackerAxisFrame frame);
    void commit_fresh_locked(TrackerAxisFrame frame);
    void collapse_overflow_locked(
        const pipeline::TrackerAxisLineage& lineage);
    bool duplicate_active_boundary_locked(
        const TrackerAxisFrame& frame) const;

    mutable std::mutex mu_;
    const std::size_t boundary_capacity_;
    std::uint64_t delivery_seq_ = 0;
    std::uint64_t continuity_epoch_ = 1;
    TrackerAxisFrame snapshot_;
    std::optional<TrackerAxisFrame> latest_fresh_;
    std::deque<TrackerAxisFrame> boundaries_;
    bool boundary_active_ = false;
    std::string active_boundary_key_;
    bool have_source_ = false;
    std::string last_stream_id_;
    std::string last_subject_track_id_ = "none";
    std::uint64_t last_coordinate_epoch_ = 1;
    std::uint64_t last_source_continuity_epoch_ = 1;
    std::string last_source_stream_id_;
    std::uint64_t last_source_sample_seq_ = 0;
};

}  // namespace fitra::tracking
