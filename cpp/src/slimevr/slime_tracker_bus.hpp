#pragma once
//
// Phase 13 M1: latest SlimeVR tracker snapshot bus.
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

struct SlimeTrackerSnapshot {
    std::uint64_t                              seq = 0;
    std::chrono::system_clock::time_point      ts{};
    std::array<SlimeTracker, kTrackerCount>    trackers{};
    bool                                       has_data = false;
};

class SlimeTrackerBus {
public:
    SlimeTrackerBus() = default;

    void publish(const std::array<SlimeTracker, kTrackerCount>& trackers);

    SlimeTrackerSnapshot snapshot() const;

private:
    mutable std::mutex   mu_;
    SlimeTrackerSnapshot snapshot_{};
};

// Phase 13 M1: render the bus snapshot as a JSON fragment ready to inject
// into Skeleton3DBus::make_bundle_json's `extra_fields_json` parameter.
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
