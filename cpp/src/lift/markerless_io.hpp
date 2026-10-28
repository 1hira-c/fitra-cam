#pragma once

// Diagnostic observation replay only. No CalibrationSet/extrinsics writer is
// exposed: relative-camera poses have neither metric scale nor floor alignment.
#include <cstddef>
#include <iosfwd>
#include <string>
#include <vector>

#include "lift/markerless_collection.hpp"

namespace fitra::lift {

inline constexpr std::size_t markerless_max_jsonl_line_bytes = 256 * 1024;
inline constexpr std::size_t markerless_max_jsonl_bytes = 32 * 1024 * 1024;

struct MarkerlessReplay {
    std::vector<MarkerlessFrameSet> frame_sets;
    MarkerlessCollectionStats collection;
    std::size_t records = 0;
    std::size_t rejected_records = 0;
    // Bounded explanatory messages; counters still include every rejection.
    std::vector<std::string> diagnostics;
    bool input_clean() const { return rejected_records == 0; }
};

// Throws on malformed JSON, unsupported schema/topology, invalid field types,
// input size limits or I/O errors. Semantically rejected/reset samples instead
// appear in the returned counters, and cannot be mistaken for a clean replay.
MarkerlessReplay read_markerless_observations(
    std::istream& input, const CalibrationSet& intrinsics,
    const MarkerlessCollectionOptions& options = {});

// Writes exactly the replay schema, preserving integer source IDs/timestamps.
// No intrinsics, derived 3D, predicted or held values are serialized here.
void write_markerless_observations(
    std::ostream& output, const std::vector<MarkerlessFrameSet>& frame_sets);

std::string markerless_relative_report_json(
    const MarkerlessRelativeResult& geometry, const MarkerlessReplay& replay);

// O_EXCL protects existing files and symlinks, including calibration pointers.
// Only a new .json path is accepted. A partial file is removed on write failure.
void write_markerless_report_new(const std::string& path,
                                 const std::string& report);

}  // namespace fitra::lift
