#pragma once
//
// Standalone tracker extraction service.
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
#include <vector>

#include <opencv2/core.hpp>

#include "pipeline/snapshot.hpp"
#include "slimevr/slime_tracker_bus.hpp"
#include "slimevr/tracker_extract.hpp"

namespace fitra::slimevr {

struct TrackerExtractorOptions {
    double extract_rate_hz   = 60.0;   // produce snapshots at this cadence
    float  quat_smooth       = 0.5f;   // apply_quat_smoothing base alpha
    // Position EMA base alpha (apply_pos_smoothing). Independent
    // from quat_smooth so the operator can dial pos and quat damping
    // separately — pos jitter and roll jitter have different noise sources.
    float  pos_smooth        = 0.5f;
    // Rolling window (in frames) used for percentile stats.
    // 120 frames ≈ 2 s at 60 Hz — long enough to catch a sustained drift but
    // short enough to react to scene changes.
    int    stats_window      = 120;
    // Event-driven mode: instead of producing at a fixed `extract_rate_hz`,
    // block on the Skeleton3DBus and react to each new triangulation result
    // (one smoothing step per real 3D frame). Removes the extractor's fixed-
    // cadence latency hop. A timeout fallback (extract_rate_hz period) still
    // fires so stale trackers are cleared when the 3D bus goes quiet. Default
    // off to preserve the validated fixed-rate behavior; opt in for minimum
    // capture->send latency.
    bool   event_driven      = false;
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

    // Position EMA history. Same single-history invariant as prev_quat_.
    // Initialized to (0,0,0) in the ctor; first valid frame snaps to the
    // real position (the velocity gate in apply_pos_smoothing is suppressed
    // until pos_ctx_.initialized[i] flips).
    std::array<cv::Vec3f, kTrackerCount> prev_pos_{};

    // FK fallback state for extract_trackers. Holds per-foot anchors
    // (knee→ankle direction + tibia length, ankle→toe direction + foot
    // length) learned from fully measured frames; foot trackers fall back
    // to the anchor when the KP drops in a single frame.
    ExtractContext extract_ctx_{};

    // Hip cache + per-tracker initialization flags + frame dt for the
    // position smoother. See PosSmoothingContext docstring.
    PosSmoothingContext pos_ctx_{};

    // Per-tracker rolling stats state.
    //
    // We keep:
    //   * ring buffer of angular velocity samples (rad/s) for p50/p95
    //   * running sums for the windowed means (confidence, leakage frac,
    //     freeze frac) — implemented as `samples_seen` cum sum + window
    //     subtraction is overkill, so we recompute the mean from the same
    //     ring buffers per publish (small N keeps this cheap)
    //   * freeze run counter + max + dropout edge counter (lifetime, not
    //     windowed)
    //
    // Each per-tracker ring uses a single vector sized to `stats_window`,
    // with an index that wraps. We accept the std::vector heap allocation
    // up-front so the publish loop is allocation-free.
    struct PerTrackerStats {
        std::vector<float> ang_vel_ring;       // rad/s
        std::vector<float> conf_ring;          // smoothing weight in [0,1]
        std::vector<std::uint8_t> leakage_ring;   // 1 if 0 < conf < 1
        std::vector<std::uint8_t> freeze_ring;    // 1 if tracker was held (input invalid)
        std::size_t        head = 0;           // next write index
        std::size_t        fill = 0;           // count of valid samples [0..ring.size()]
        // Reused scratch buffer for nth_element percentile computation. Sized
        // to stats_window in the ctor so the publish loop's assign()+nth_element
        // sequence never allocates.
        std::vector<float> percentile_scratch;

        // Dropout / freeze edge tracking. `prev_was_valid_seen` starts false
        // so the very first sample never registers as a valid→invalid edge
        // (avoids inflating dropout_count for trackers that start invalid).
        bool               prev_was_valid_seen = false;
        bool               prev_was_valid      = false;
        int                freeze_current_ms   = 0;
        int                freeze_max_ms       = 0;
        std::uint64_t      dropout_count       = 0;
    };
    std::array<PerTrackerStats, kTrackerCount> stats_{};

    // Per-tracker quaternion from the PREVIOUS publish (post-smoothing), used
    // to compute the next angular velocity sample. Distinct from prev_quat_
    // which is the apply_quat_smoothing internal history (also post-smoothing
    // but updated before stats see the new frame).
    std::array<cv::Vec4f, kTrackerCount> last_emitted_quat_{};
    bool                                  have_last_emitted_ = false;
};

}  // namespace fitra::slimevr
