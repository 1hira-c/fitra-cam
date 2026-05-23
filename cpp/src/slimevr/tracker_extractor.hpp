#pragma once
//
// Phase 13 M1: standalone tracker extraction service.
//
// Reads the Skeleton3DBus on a paced loop, runs extract_trackers +
// apply_quat_smoothing (owning the prev_quat state), and publishes the
// smoothed `SlimeTracker[]` to a SlimeTrackerBus.
//
// Architecturally this is the single producer of `SlimeTracker` snapshots.
// Both the SlimeVR Firmware UDP publisher (NativePublisher) and the Three.js
// WebUI viewer (via Skeleton3DBus::make_bundle_json picking up the trackers
// snapshot) consume from the same bus, so they always see the same smoothed
// values — no double-extract, no divergent prev_quat history.
//
// Runs whenever there is a 3D skeleton in the Skeleton3DBus (regardless of
// --slimevr-out): the WebUI orientation viz needs trackers even when the
// Firmware UDP publisher is disabled.

#include <array>
#include <atomic>
#include <thread>

#include <opencv2/core.hpp>

#include "pipeline/snapshot.hpp"
#include "slimevr/slime_tracker_bus.hpp"
#include "slimevr/tracker_extract.hpp"

namespace fitra::slimevr {

struct TrackerExtractorOptions {
    double extract_rate_hz = 60.0;   // produce snapshots at this cadence
    float  quat_smooth     = 0.5f;   // apply_quat_smoothing base alpha
};

class TrackerExtractor {
public:
    TrackerExtractor(pipeline::Skeleton3DBus&    skeleton_bus,
                     SlimeTrackerBus&            tracker_bus,
                     TrackerExtractorOptions     opts);
    ~TrackerExtractor();

    TrackerExtractor(const TrackerExtractor&) = delete;
    TrackerExtractor& operator=(const TrackerExtractor&) = delete;

    void start();
    void stop();

    const TrackerExtractorOptions& options() const { return opts_; }

private:
    void run_loop();

    pipeline::Skeleton3DBus&            skel_bus_;
    SlimeTrackerBus&                    tracker_bus_;
    TrackerExtractorOptions             opts_;

    std::thread                         thread_;
    std::atomic<bool>                   stop_{false};
    std::atomic<bool>                   running_{false};

    // Quaternion smoothing history; owned here so the bus consumers never see
    // raw (unsmoothed) extractions and so the prev_quat history is preserved
    // across consumers (no double history).
    std::array<cv::Vec4f, kTrackerCount> prev_quat_{};
};

}  // namespace fitra::slimevr
