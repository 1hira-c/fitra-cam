#pragma once
//
// Continuous (always-on) HMD-driven VMT alignment.
//
// The one-shot solvers in auto_alignment.{hpp,cpp} (solve_tpose / solve_motion)
// are user-triggered: press a button, hold a pose / walk for N seconds, solve
// once. This module keeps the same 2D Procrustes core (solve_motion) but feeds
// it from a background sampler that runs from start-up and refines the live
// alignment semi-continuously:
//
//   * Each tick pairs the HMD pose (HmdPoseBus, already in VMT frame) with the
//     most reliably reported body point taken from the raw 3D skeleton
//     (Skeleton3DBus): head_top when its keypoint score is high enough, else
//     the chest midpoint (neck/hip_center). Head is preferred because it shares
//     the HMD's physical location, so the rigid 2D Procrustes has almost no
//     lever-arm residual; chest is the robust fallback when the head keypoint
//     is unreliable (subject facing away, occlusion).
//   * A sample-quality weight gates admission. Its dominant term is the
//     verticality of the spine/neck bone (neck->hip_center): when the subject
//     stands upright the head sits directly above the hips and the head<->HMD
//     correspondence is geometrically cleanest. Joint confidence and HMD speed
//     (quasi-static is cleaner) round out the weight.
//   * Admitted samples land in a spatially-bucketed reservoir keyed on the HMD
//     xz cell, keeping the best sample per cell. This keeps the Procrustes
//     input well spread (non-collinear) and bounded, and the per-cell time
//     decay lets the solution track slow SLAM drift.
//   * Every resolve_period_s, if enough cells are occupied, solve_motion runs
//     and a clamped EMA of (yaw, x, z) is pushed into VmtPublisher::set_alignment
//     (same channel as the manual UI / one-shot solvers; last write wins). Y is
//     never touched — the head/chest height offset stays on the manual slider,
//     consistent with the one-shot path.
//
// The pure helpers (ramp / verticality_score / SampleReservoir / make_sample /
// blend_alignment) take explicit inputs and timestamps so they unit-test
// without threads or a clock. ContinuousAligner wraps them in the polling loop.

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "vmt/auto_alignment.hpp"      // AutoAlignmentResult/Status, MotionSample
#include "vmt/hmd_pose_receiver.hpp"   // HmdPose, HmdPoseBus
#include "vmt/vmt_protocol.hpp"        // VmtAlignment

namespace fitra::pipeline { class Skeleton3DBus; }

namespace fitra::vmt {

class VmtPublisher;  // fwd decl; full header pulled in the .cpp

// Which body landmark fed a correspondence sample.
enum class CorrSource : std::uint8_t { None = 0, Head, Chest };
const char* corr_source_name(CorrSource s);

struct ContinuousAlignerConfig {
    bool   enabled          = false;  // main flips this to the user's --vmt-continuous-align

    double sample_hz        = 15.0;   // background poll rate
    double resolve_period_s = 2.0;    // re-solve cadence
    float  blend_alpha      = 0.2f;   // EMA weight toward each fresh solve

    // ---- sample quality ----
    float  head_conf_thresh = 0.5f;   // head_top.score >= this -> use head, else chest
    float  vert_full_deg    = 15.0f;  // spine tilt <= this -> verticality weight 1
    float  vert_zero_deg    = 40.0f;  // spine tilt >= this -> verticality weight 0
    float  vel_full_mps     = 0.30f;  // HMD speed <= this -> velocity weight 1
    float  vel_zero_mps     = 1.50f;  // HMD speed >= this -> velocity weight 0
    float  quality_thresh   = 0.25f;  // min combined quality to admit a sample

    // ---- reservoir ----
    float  cell_size_m      = 0.30f;  // HMD xz bucket size
    int    max_cells        = 64;     // cap; lowest-quality cells evicted past this
    double sample_ttl_s     = 60.0;   // cell expiry (tracks slow drift)
    int    min_cells        = 8;      // need this many occupied cells to solve

    // ---- solve gate + per-update step clamp ----
    float  residual_max_m   = 0.15f;  // reject a solve whose mean residual exceeds this
    float  max_yaw_step_deg = 2.0f;   // clamp |Δyaw| applied per resolve
    float  max_pos_step_m   = 0.05f;  // clamp |Δ(x,z)| applied per resolve
};

// Smooth Hermite ramp. Returns 0 at `zero_at`, 1 at `full_at`, monotonic in
// between regardless of which bound is numerically larger.
float ramp(float v, float zero_at, float full_at);

// Verticality of a world-frame bone (Z-up). 1.0 when parallel to ±Z (upright),
// decaying via `ramp` on the tilt angle (acos|unit.z|). Returns 0 for a
// degenerate (near-zero-length) bone.
float verticality_score(const cv::Vec3f& bone_world, float full_deg, float zero_deg);

// One curated correspondence sample. xz are in the VMT Driver frame.
struct AlignSample {
    float      hmd_x   = 0.0f, hmd_z  = 0.0f;
    float      body_x  = 0.0f, body_z = 0.0f;
    float      quality = 0.0f;
    double     t_s     = 0.0;
    CorrSource source  = CorrSource::None;
};

// World-frame skeleton joints needed to build a sample (Z-up, meters).
struct SampleInputs {
    cv::Vec3f head_top{};   bool head_valid = false;  float head_score = 0.0f;
    cv::Vec3f neck{};       bool neck_valid = false;  float neck_score = 0.0f;
    cv::Vec3f hip_center{}; bool hip_valid  = false;  float hip_score  = 0.0f;
};

// Build a correspondence sample. Picks head vs chest, computes the quality
// weight (verticality · joint-conf · velocity), and converts the body point to
// the VMT frame. `hmd_speed_mps` feeds the velocity term (pass 0 on the first
// frame). Returns source==None when neck/hip_center are missing (no usable
// point / no verticality reference).
AlignSample make_sample(const SampleInputs& in,
                        const HmdPose&      hmd,
                        float               hmd_speed_mps,
                        double              t_s,
                        const ContinuousAlignerConfig& cfg);

// Spatially-bucketed reservoir keyed on HMD xz cells: keeps the best-quality
// sample per cell, expires stale cells, caps total cell count.
class SampleReservoir {
public:
    explicit SampleReservoir(const ContinuousAlignerConfig& cfg) : cfg_{cfg} {}

    // Admit a sample. Replaces the cell's sample iff the cell is empty or the
    // newcomer is at least as good (within kRefreshFactor, so comparable-quality
    // recent samples refresh position/time and the cell tracks drift). Returns
    // true if the reservoir changed.
    bool admit(const AlignSample& s);

    // Drop cells whose sample is older than sample_ttl_s relative to now_s,
    // then evict the lowest-quality cells down to max_cells.
    void prune(double now_s);

    std::vector<MotionSample> motion_samples() const;
    int  occupied_cells() const { return static_cast<int>(cells_.size()); }
    int  head_count() const;
    int  chest_count() const;
    void clear() { cells_.clear(); }

private:
    std::int64_t key_of(float x, float z) const;

    ContinuousAlignerConfig          cfg_;
    std::map<std::int64_t, AlignSample> cells_;
};

// Blend `target` toward `current` by `alpha`, clamping the per-update step
// (translation magnitude and |Δyaw|). Y is copied from `current` (height stays
// manual); yaw is blended on the circle.
VmtAlignment blend_alignment(const VmtAlignment& current,
                             const VmtAlignment& target,
                             float alpha,
                             float max_yaw_step_deg,
                             float max_pos_step_m);

// Background driver: owns the polling thread, taps the buses, drives the
// reservoir + solve + clamped EMA into the publisher. Read-only consumer of the
// shared 3D skeleton state, same as the publishers.
class ContinuousAligner {
public:
    struct Status {
        bool                running        = false;
        bool                enabled        = false;
        int                 occupied_cells = 0;
        int                 n_samples      = 0;
        int                 head_samples   = 0;
        int                 chest_samples  = 0;
        float               last_residual_m = 0.0f;
        AutoAlignmentStatus last_status    = AutoAlignmentStatus::NoHmd;
        std::uint64_t       resolves       = 0;
        std::uint64_t       updates        = 0;
    };

    ContinuousAligner(pipeline::Skeleton3DBus& skel_bus,
                      HmdPoseBus&              hmd_bus,
                      VmtPublisher&            publisher,
                      double                   hmd_stale_ms,
                      ContinuousAlignerConfig  cfg);
    ~ContinuousAligner();

    ContinuousAligner(const ContinuousAligner&) = delete;
    ContinuousAligner& operator=(const ContinuousAligner&) = delete;

    // Launch the polling thread. Body runs only while enabled() is true.
    bool start();
    void stop();  // idempotent

    void set_enabled(bool on) { enabled_.store(on); }
    bool enabled() const { return enabled_.load(); }

    Status status() const;
    const ContinuousAlignerConfig& config() const { return cfg_; }

private:
    void loop();

    pipeline::Skeleton3DBus& skel_bus_;
    HmdPoseBus&              hmd_bus_;
    VmtPublisher&            publisher_;
    double                   hmd_stale_ms_;
    ContinuousAlignerConfig  cfg_;

    std::thread        thread_;
    std::atomic<bool>  stop_{false};
    std::atomic<bool>  enabled_{false};

    mutable std::mutex status_mu_;
    Status             status_{};
};

}  // namespace fitra::vmt
