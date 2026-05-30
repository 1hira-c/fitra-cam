#include "vmt/continuous_aligner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "infer/types.hpp"            // Skeleton3D / Joint3D
#include "lift/skeleton_def.hpp"      // not strictly needed; landmark indices below
#include "pipeline/snapshot.hpp"      // Skeleton3DBus
#include "vmt/vmt_publisher.hpp"      // VmtPublisher

namespace fitra::vmt {

namespace {

constexpr float kPi = 3.14159265358979323846f;

// Halpe26 landmark indices (see lift/skeleton_def.hpp).
constexpr int kHeadTop   = 17;
constexpr int kNeck      = 18;
constexpr int kHipCenter = 19;

// A recent sample within this fraction of the incumbent's quality still
// overwrites the cell, refreshing its position/time so the reservoir tracks
// slow drift rather than pinning forever to one old high-quality frame.
constexpr float kRefreshFactor = 0.9f;

inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

inline float wrap180(float deg) {
    return std::remainder(deg, 360.0f);
}

}  // namespace

const char* corr_source_name(CorrSource s) {
    switch (s) {
        case CorrSource::None:  return "none";
        case CorrSource::Head:  return "head";
        case CorrSource::Chest: return "chest";
    }
    return "unknown";
}

float ramp(float v, float zero_at, float full_at) {
    const float denom = full_at - zero_at;
    if (denom == 0.0f) return v == zero_at ? 0.0f : (v == full_at ? 1.0f : 0.0f);
    float t = (v - zero_at) / denom;
    t = clampf(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);  // smoothstep
}

float verticality_score(const cv::Vec3f& bone_world, float full_deg, float zero_deg) {
    const double len = std::sqrt(static_cast<double>(
        bone_world[0] * bone_world[0] +
        bone_world[1] * bone_world[1] +
        bone_world[2] * bone_world[2]));
    if (len < 1e-4) return 0.0f;
    const float cos_tilt = static_cast<float>(std::fabs(bone_world[2]) / len);
    const float tilt_deg = std::acos(clampf(cos_tilt, 0.0f, 1.0f)) * (180.0f / kPi);
    return ramp(tilt_deg, zero_deg, full_deg);
}

AlignSample make_sample(const SampleInputs& in,
                        const HmdPose&      hmd,
                        float               hmd_speed_mps,
                        double              t_s,
                        const ContinuousAlignerConfig& cfg) {
    AlignSample s;
    s.t_s = t_s;

    // Verticality reference + chest fallback both require neck & hip_center.
    if (!in.neck_valid || !in.hip_valid) {
        s.source = CorrSource::None;
        return s;
    }

    // Spine/neck bone (terminates at the neck): the upright-pose indicator.
    const cv::Vec3f spine = in.neck - in.hip_center;
    const float vert = verticality_score(spine, cfg.vert_full_deg, cfg.vert_zero_deg);
    const float vel  = ramp(hmd_speed_mps, cfg.vel_zero_mps, cfg.vel_full_mps);

    cv::Vec3f body_world;
    float conf;
    if (in.head_valid && in.head_score >= cfg.head_conf_thresh) {
        s.source   = CorrSource::Head;
        body_world = in.head_top;
        conf       = in.head_score;
    } else {
        s.source   = CorrSource::Chest;
        body_world = (in.neck + in.hip_center) * 0.5f;
        conf       = std::min(in.neck_score, in.hip_score);
    }

    const VmtPos body_vmt =
        world_pos_to_vmt(body_world[0], body_world[1], body_world[2]);
    s.body_x  = body_vmt.x;
    s.body_z  = body_vmt.z;
    s.hmd_x   = hmd.x;
    s.hmd_z   = hmd.z;
    s.quality = clampf(conf, 0.0f, 1.0f) * vert * vel;
    return s;
}

std::int64_t SampleReservoir::key_of(float x, float z) const {
    const float cell = cfg_.cell_size_m > 1e-3f ? cfg_.cell_size_m : 0.3f;
    const std::int64_t ix = static_cast<std::int64_t>(std::floor(x / cell));
    const std::int64_t iz = static_cast<std::int64_t>(std::floor(z / cell));
    // Pack two 32-bit cell coords into one key. Room-scale coords keep |ix|,
    // |iz| well within int32, so the shift never overflows in practice.
    return (ix << 32) ^ (iz & 0xffffffffLL);
}

bool SampleReservoir::admit(const AlignSample& s) {
    if (s.source == CorrSource::None) return false;
    const std::int64_t k = key_of(s.hmd_x, s.hmd_z);
    auto it = cells_.find(k);
    if (it == cells_.end()) {
        cells_.emplace(k, s);
        return true;
    }
    if (s.quality >= it->second.quality * kRefreshFactor) {
        it->second = s;
        return true;
    }
    return false;
}

void SampleReservoir::prune(double now_s) {
    for (auto it = cells_.begin(); it != cells_.end();) {
        if (now_s - it->second.t_s > cfg_.sample_ttl_s) {
            it = cells_.erase(it);
        } else {
            ++it;
        }
    }
    const std::size_t cap = static_cast<std::size_t>(std::max(1, cfg_.max_cells));
    while (cells_.size() > cap) {
        auto worst = cells_.begin();
        for (auto it = std::next(cells_.begin()); it != cells_.end(); ++it) {
            if (it->second.quality < worst->second.quality) worst = it;
        }
        cells_.erase(worst);
    }
}

std::vector<MotionSample> SampleReservoir::motion_samples() const {
    std::vector<MotionSample> out;
    out.reserve(cells_.size());
    for (const auto& [k, s] : cells_) {
        out.push_back({s.hmd_x, s.hmd_z, s.body_x, s.body_z});
    }
    return out;
}

int SampleReservoir::head_count() const {
    int n = 0;
    for (const auto& [k, s] : cells_) if (s.source == CorrSource::Head) ++n;
    return n;
}

int SampleReservoir::chest_count() const {
    int n = 0;
    for (const auto& [k, s] : cells_) if (s.source == CorrSource::Chest) ++n;
    return n;
}

LockState update_lock_state(LockState prev,
                            const VmtAlignment& current,
                            const VmtAlignment& target,
                            float residual_m,
                            const ContinuousAlignerConfig& cfg) {
    const float dx = target.x - current.x;
    const float dz = target.z - current.z;
    const float pos_err = std::sqrt(dx * dx + dz * dz);
    const float yaw_err = std::fabs(wrap180(target.yaw_deg - current.yaw_deg));

    LockState out = prev;
    if (prev.locked) {
        // Large divergence (e.g. VMT re-centering): drop back to coarse.
        if (pos_err > cfg.unlock_pos_err_m || yaw_err > cfg.unlock_yaw_err_deg) {
            out.locked = false;
            out.streak = 0;
        }
        return out;
    }

    const bool converged = residual_m <= cfg.residual_max_m &&
                           pos_err <= cfg.lock_pos_tol_m &&
                           yaw_err <= cfg.lock_yaw_tol_deg;
    if (converged) {
        out.streak = prev.streak + 1;
        if (out.streak >= cfg.lock_streak) out.locked = true;
    } else {
        out.streak = 0;
    }
    return out;
}

VmtAlignment blend_alignment(const VmtAlignment& current,
                             const VmtAlignment& target,
                             float alpha,
                             float max_yaw_step_deg,
                             float max_pos_step_m) {
    VmtAlignment out;
    out.y = current.y;  // height stays manual

    // Translation: EMA step, clamped by 2D magnitude so direction is preserved.
    float dx = alpha * (target.x - current.x);
    float dz = alpha * (target.z - current.z);
    const float mag = std::sqrt(dx * dx + dz * dz);
    if (mag > max_pos_step_m && mag > 0.0f) {
        const float scale = max_pos_step_m / mag;
        dx *= scale;
        dz *= scale;
    }
    out.x = current.x + dx;
    out.z = current.z + dz;

    // Yaw: EMA on the shortest arc, clamped per update.
    float dyaw = wrap180(target.yaw_deg - current.yaw_deg) * alpha;
    dyaw = clampf(dyaw, -max_yaw_step_deg, max_yaw_step_deg);
    out.yaw_deg = wrap180(current.yaw_deg + dyaw);
    return out;
}

ContinuousAligner::ContinuousAligner(pipeline::Skeleton3DBus& skel_bus,
                                     HmdPoseBus&              hmd_bus,
                                     VmtPublisher&            publisher,
                                     double                   hmd_stale_ms,
                                     ContinuousAlignerConfig  cfg)
    : skel_bus_{skel_bus},
      hmd_bus_{hmd_bus},
      publisher_{publisher},
      hmd_stale_ms_{hmd_stale_ms},
      cfg_{cfg} {
    enabled_.store(cfg.enabled);
}

ContinuousAligner::~ContinuousAligner() {
    stop();
}

bool ContinuousAligner::start() {
    if (thread_.joinable()) return true;
    stop_.store(false);
    thread_ = std::thread{&ContinuousAligner::loop, this};
    return true;
}

void ContinuousAligner::stop() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
}

ContinuousAligner::Status ContinuousAligner::status() const {
    std::lock_guard<std::mutex> g{status_mu_};
    return status_;
}

void ContinuousAligner::loop() {
    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    auto now_s = [&]() {
        return std::chrono::duration<double>(clock::now() - t0).count();
    };

    SampleReservoir reservoir{cfg_};
    const double sample_hz = cfg_.sample_hz > 1e-3 ? cfg_.sample_hz : 15.0;
    const auto period =
        std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(1.0 / sample_hz));
    auto next_tick = clock::now();
    double last_resolve = -1e18;
    LockState lock;  // starts unlocked -> coarse clamps for fast cold-start

    bool   have_prev = false;
    float  prev_hx = 0.0f, prev_hz = 0.0f;
    double prev_t = 0.0;

    {
        std::lock_guard<std::mutex> g{status_mu_};
        status_.running = true;
    }

    while (!stop_.load()) {
        next_tick += period;
        std::this_thread::sleep_until(next_tick);
        if (stop_.load()) break;

        const bool en = enabled_.load();
        {
            std::lock_guard<std::mutex> g{status_mu_};
            status_.enabled = en;
        }
        if (!en) {
            have_prev = false;
            lock = {};  // re-acquire from coarse when toggled back on
            continue;
        }

        const double t = now_s();
        auto h = hmd_bus_.snapshot(hmd_stale_ms_);
        if (!(h.have_any && !h.stale && h.pose.valid)) {
            have_prev = false;
            continue;
        }

        auto snap = skel_bus_.snapshot();
        if (snap.persons.empty()) continue;
        const auto& skel = snap.persons.front();

        SampleInputs in;
        const auto& jh = skel.joints[kHeadTop];
        const auto& jn = skel.joints[kNeck];
        const auto& jc = skel.joints[kHipCenter];
        in.head_top  = {jh.x, jh.y, jh.z}; in.head_valid = jh.valid; in.head_score = jh.score;
        in.neck      = {jn.x, jn.y, jn.z}; in.neck_valid = jn.valid; in.neck_score = jn.score;
        in.hip_center= {jc.x, jc.y, jc.z}; in.hip_valid  = jc.valid; in.hip_score  = jc.score;

        float speed = 0.0f;
        if (have_prev && t > prev_t) {
            const float dx = h.pose.x - prev_hx;
            const float dz = h.pose.z - prev_hz;
            speed = std::sqrt(dx * dx + dz * dz) / static_cast<float>(t - prev_t);
        }
        prev_hx = h.pose.x;
        prev_hz = h.pose.z;
        prev_t = t;
        have_prev = true;

        const AlignSample s = make_sample(in, h.pose, speed, t, cfg_);
        if (s.source != CorrSource::None && s.quality >= cfg_.quality_thresh) {
            reservoir.admit(s);
        }
        reservoir.prune(t);

        if (t - last_resolve < cfg_.resolve_period_s) continue;
        last_resolve = t;

        const int cells = reservoir.occupied_cells();
        AutoAlignmentResult r;
        bool applied = false;
        if (cells >= cfg_.min_cells) {
            auto samples = reservoir.motion_samples();
            r = solve_motion(samples, cfg_.min_cells);
            if (r.status == AutoAlignmentStatus::Ok &&
                r.residual_m <= cfg_.residual_max_m) {
                const VmtAlignment cur = publisher_.alignment();
                lock = update_lock_state(lock, cur, r.alignment, r.residual_m, cfg_);
                // Coarse (unlocked) until the solve settles onto live -> fast
                // cold-start; fine clamps once locked -> jump-free drift tracking.
                const float a  = lock.locked ? cfg_.blend_alpha        : cfg_.coarse_blend_alpha;
                const float ys = lock.locked ? cfg_.max_yaw_step_deg   : cfg_.coarse_max_yaw_step_deg;
                const float ps = lock.locked ? cfg_.max_pos_step_m     : cfg_.coarse_max_pos_step_m;
                const VmtAlignment next = blend_alignment(cur, r.alignment, a, ys, ps);
                publisher_.set_alignment(next);
                applied = true;
            }
        } else {
            r.status    = AutoAlignmentStatus::NotEnoughSamples;
            r.n_samples = cells;
        }

        std::lock_guard<std::mutex> g{status_mu_};
        status_.locked          = lock.locked;
        status_.occupied_cells  = cells;
        status_.n_samples       = r.n_samples;
        status_.head_samples    = reservoir.head_count();
        status_.chest_samples   = reservoir.chest_count();
        status_.last_status     = r.status;
        status_.last_residual_m = r.residual_m;
        ++status_.resolves;
        if (applied) ++status_.updates;
    }

    std::lock_guard<std::mutex> g{status_mu_};
    status_.running = false;
}

}  // namespace fitra::vmt
