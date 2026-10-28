#pragma once

#include <map>
#include <string>
#include <vector>

#include "lift/markerless_extrinsic.hpp"

namespace fitra::lift {

// Shared by the direct numerical API and the collector/replay boundary.
// A valid set has exactly one frame for each configured camera. Missing joint
// observations are allowed; missing cameras, ambiguous persons or clocks are not.
// Empty input is structurally valid; the solver enforces its sample minimum.
std::string validate_markerless_inputs(
    const CalibrationSet& intrinsics,
    const std::vector<MarkerlessFrameSet>& frame_sets,
    double max_sync_skew_ms = 20.0);

enum class MarkerlessIngest { Accepted, Rejected, Reset };

struct MarkerlessCollectionOptions {
    std::size_t max_frame_sets = 240;
    double max_sync_skew_ms = 20.0;
};

struct MarkerlessCollectionStats {
    std::size_t accepted = 0;
    std::size_t rejected = 0;
    std::size_t resets = 0;
    std::size_t evicted = 0;
    double max_accepted_skew_ms = 0.0;
};

// Single-owner, bounded input seam for future live collection and today's
// offline replay. Does not open cameras, run inference or infer person identity.
class MarkerlessCollector {
public:
    explicit MarkerlessCollector(CalibrationSet intrinsics,
                                 MarkerlessCollectionOptions options = {});
    MarkerlessIngest ingest(const MarkerlessFrameSet& frame_set,
                            std::string& reason);
    void reset();
    const std::vector<MarkerlessFrameSet>& frame_sets() const { return sets_; }
    const MarkerlessCollectionStats& stats() const { return stats_; }

private:
    CalibrationSet intrinsics_;
    MarkerlessCollectionOptions options_;
    std::vector<MarkerlessFrameSet> sets_;
    MarkerlessCollectionStats stats_;
    std::map<std::string, MarkerlessCameraFrame> last_frames_;
    std::string continuity_id_;
    std::uint64_t last_sample_id_ = 0;
    bool have_sample_ = false;
};

}  // namespace fitra::lift
