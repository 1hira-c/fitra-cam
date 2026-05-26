#pragma once
//
// Latest SlimeVR tracker snapshot bus.
//
// Produced by TrackerExtractor (extract_trackers + apply_quat_smoothing on the
// Skeleton3DBus output, owning the prev_quat smoothing state). Consumed by:
//   * NativePublisher (Firmware UDP serialize + send) when --slimevr-out is on
//   * snapshot.cpp::Skeleton3DBus::make_bundle_json() when populating the
//     `trackers` field of the WS bundle for the Three.js viewer
//
// One-writer / many-reader; locked atomic value-copy is sufficient (60 Hz of
// 10 trackers × ~40 bytes is negligible contention).

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include "slimevr/tracker_extract.hpp"

namespace fitra::slimevr {

// Per-tracker rolling stats published alongside the smoothed
// trackers themselves. Computed by TrackerExtractor over a fixed window
// (default 2 s of frames at the extractor's tick rate, e.g. 120 frames at
// 60 Hz). Exposed to the WebUI so the user can see which tracker is the
// noise source when something jitters or snaps.
//
// Indices match SlimeTracker indices in `trackers[]` (= TrackerRole enum
// order = sensor_id).
struct SlimeTrackerStats {
    // Quaternion angular velocity in rad/s, percentiles over the window.
    // delta_rad = 2 * acos(|dot(prev_smoothed, curr_smoothed)|), then /dt.
    std::array<float, kTrackerCount> angular_velocity_rad_s_p50{};
    std::array<float, kTrackerCount> angular_velocity_rad_s_p95{};

    // Mean roll_confidence over the window (after smoothing weight modulation).
    std::array<float, kTrackerCount> roll_confidence_avg{};

    // Fraction of frames (0..1) in the smoothstep leakage zone
    // (0 < roll_confidence < 1). This is the regime where unstable up-vector
    // measurements pull the quat with low weight — known to accumulate drift
    // over many frames. High values flag the tracker as a drift candidate.
    std::array<float, kTrackerCount> leakage_pct{};

    // Fraction of frames (0..1) the tracker was held (= invalid input, prev
    // quat reused). Distinct from leakage: freeze is a hard hold; leakage is
    // a weak update.
    std::array<float, kTrackerCount> freeze_pct{};

    // Current consecutive freeze run length in ms, and max observed since
    // process start. dropout_count counts valid→invalid transitions.
    std::array<int, kTrackerCount>   freeze_current_ms{};
    std::array<int, kTrackerCount>   freeze_max_ms{};
    std::array<std::uint64_t, kTrackerCount> dropout_count{};

    // Window size (in frames) used for the rolling percentiles + averages.
    // Reported so the WebUI can label the stats meaningfully.
    int window_frames = 0;
};

struct SlimeTrackerSnapshot {
    std::uint64_t                              seq = 0;
    std::chrono::system_clock::time_point      ts{};
    std::array<SlimeTracker, kTrackerCount>    trackers{};
    SlimeTrackerStats                          stats{};
    bool                                       has_data = false;
};

class SlimeTrackerBus {
public:
    SlimeTrackerBus() = default;

    void publish(const std::array<SlimeTracker, kTrackerCount>& trackers,
                 const SlimeTrackerStats&                       stats);

    SlimeTrackerSnapshot snapshot() const;

private:
    mutable std::mutex   mu_;
    SlimeTrackerSnapshot snapshot_{};
};

// Render the bus snapshot as a JSON fragment ready to inject into
// Skeleton3DBus::make_bundle_json's `extra_fields_json` parameter.
//
// Output shape (no leading or trailing comma; valid as a top-level field):
//   "trackers":[
//     {"role":"LeftUpperArm","pos":[x,y,z],"quat_wxyz":[w,x,y,z],
//      "valid":true,"roll_confidence":0.95},
//     ...   // exactly kTrackerCount entries, role-ordered
//   ]
//
// Empty bus (no publish yet) returns `"trackers":[]` so the WebUI receives a
// well-formed key even before TrackerExtractor has run once.
std::string make_tracker_bundle_fragment(const SlimeTrackerBus& bus);

// Human-readable role name (PascalCase, no spaces). Stable wire string for
// the WebUI to switch on. Used by make_tracker_bundle_fragment but also
// available to /stats3d builders.
const char* tracker_role_name(TrackerRole role);

}  // namespace fitra::slimevr
