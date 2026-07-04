#include "slimevr/st_filter.hpp"

#include <algorithm>
#include <cmath>

namespace fitra::slimevr {

namespace {

// Hermite smoothstep: 0 at x ≤ low, 1 at x ≥ high, C¹ in between. Mirrors the
// tracker_extract smoothstep01 (kept local so this module is self-contained;
// the slim-down at the end of the track can unify them).
inline float smoothstep01(float x, float low, float high) {
    if (x <= low)  return 0.0f;
    if (x >= high) return 1.0f;
    float t = (x - low) / (high - low);
    return t * t * (3.0f - 2.0f * t);
}

inline float vnorm(const cv::Vec3f& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// ---- quaternion (wxyz) helpers for the twist path ------------------------- #
inline cv::Vec4f qnorm(const cv::Vec4f& q) {
    float n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1.0e-9f) return cv::Vec4f{1, 0, 0, 0};
    return cv::Vec4f{q[0]/n, q[1]/n, q[2]/n, q[3]/n};
}
inline cv::Vec4f qconj(const cv::Vec4f& q) {
    return cv::Vec4f{q[0], -q[1], -q[2], -q[3]};
}
inline cv::Vec4f qmul(const cv::Vec4f& a, const cv::Vec4f& b) {
    return cv::Vec4f{
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]};
}

// Build the seeded per-group config once. Values are M-C2 starting points
// (metres / m/s for position, radians / rad/s for the twist), to be swept into
// tuned defaults by the M-C1 harness in M-C4. The ~3 cm "discard fine motion"
// target lands at d_full; d_core sits a few σ of the static jitter below it.
StFilterConfig make_default_config() {
    StFilterConfig cfg{};

    auto& pos = cfg.pos;
    //                     d_core d_full a_rest a_norm v_high v_rej   lag_cap
    pos[static_cast<std::size_t>(StGroup::Waist)]    = {{0.008f, 0.025f, 0.12f, 0.45f,  4.0f,  8.0f}, 0.10f};
    pos[static_cast<std::size_t>(StGroup::Chest)]    = {{0.010f, 0.030f, 0.15f, 0.50f,  4.0f,  8.0f}, 0.10f};
    pos[static_cast<std::size_t>(StGroup::UpperArm)] = {{0.012f, 0.035f, 0.15f, 0.55f,  8.0f, 16.0f}, 0.10f};
    pos[static_cast<std::size_t>(StGroup::UpperLeg)] = {{0.010f, 0.030f, 0.15f, 0.50f,  6.0f, 12.0f}, 0.10f};
    pos[static_cast<std::size_t>(StGroup::LowerLeg)] = {{0.012f, 0.035f, 0.12f, 0.50f, 10.0f, 20.0f}, 0.12f};
    pos[static_cast<std::size_t>(StGroup::Foot)]     = {{0.012f, 0.035f, 0.10f, 0.45f, 12.0f, 24.0f}, 0.15f};

    // Twist regime (radians / rad/s): ~3° deadband, ~10° ramp end, strong rest.
    const StRegime roll_seed{0.052f, 0.175f, 0.10f, 0.50f, 4.0f, 12.0f};
    for (auto& r : cfg.roll) r = roll_seed;

    // Roll regime scope: ARM ONLY (M-C4/M-C5). The M-C4 kick clip (leg roll
    // genuinely observable, roll_conf 0.95-0.99) + adversarial verification
    // showed the twist regime gives NO benefit on legs (rel_dps unchanged/worse
    // vs One Euro) — the earlier leg roll_rms "wins" were a swing-leakage metric
    // artifact. The genuine, real-hardware-confirmed benefit is arm twist only
    // (l_upper_arm roll_rms -12.6% vs One Euro). Legs kept OUT of the twist
    // regime so they don't add rel_dps / widen the threshold-snap surface for
    // zero gain; the POSITION regime still applies to legs/feet (it drives their
    // orientation stability via steadier swing). Chest / waist / foot roll stays
    // pinned. See docs/design/pose-3d-spatiotemporal-filter.md M-C4/M-C5.
    auto& hr = cfg.has_roll;
    hr[static_cast<std::size_t>(StGroup::Waist)]    = false;
    hr[static_cast<std::size_t>(StGroup::Chest)]    = false;
    hr[static_cast<std::size_t>(StGroup::UpperArm)] = true;
    hr[static_cast<std::size_t>(StGroup::UpperLeg)] = false;  // M-C4: no leg-twist benefit
    hr[static_cast<std::size_t>(StGroup::LowerLeg)] = false;  // M-C4: no leg-twist benefit
    hr[static_cast<std::size_t>(StGroup::Foot)]     = false;

    return cfg;
}

}  // namespace

StGroup st_group_for(TrackerRole role) {
    switch (role) {
        case TrackerRole::LeftUpperArm:
        case TrackerRole::RightUpperArm: return StGroup::UpperArm;
        case TrackerRole::Chest:         return StGroup::Chest;
        case TrackerRole::Waist:         return StGroup::Waist;
        case TrackerRole::LeftUpperLeg:
        case TrackerRole::RightUpperLeg: return StGroup::UpperLeg;
        case TrackerRole::LeftLowerLeg:
        case TrackerRole::RightLowerLeg: return StGroup::LowerLeg;
        case TrackerRole::LeftFoot:
        case TrackerRole::RightFoot:     return StGroup::Foot;
        case TrackerRole::Count:         break;
    }
    return StGroup::Waist;  // unreachable; keep a safe default
}

const StFilterConfig& default_st_config() {
    static const StFilterConfig cfg = make_default_config();
    return cfg;
}

const StPosParams& st_pos_params(const StFilterConfig& cfg, TrackerRole role) {
    return cfg.pos[static_cast<std::size_t>(st_group_for(role))];
}
const StRegime& st_roll_params(const StFilterConfig& cfg, TrackerRole role) {
    return cfg.roll[static_cast<std::size_t>(st_group_for(role))];
}
bool st_has_roll(const StFilterConfig& cfg, TrackerRole role) {
    return cfg.has_roll[static_cast<std::size_t>(st_group_for(role))];
}

float st_alpha_d(float d, const StRegime& r) {
    if (d <= r.d_core) return r.alpha_rest;
    if (d >= r.d_full) return r.alpha_normal;
    const float t = smoothstep01(d, r.d_core, r.d_full);
    return r.alpha_rest + t * (r.alpha_normal - r.alpha_rest);
}

float st_vel_gate(float v, const StRegime& r) {
    return 1.0f - smoothstep01(v, r.v_high, r.v_reject);
}

float st_rate_adjust_alpha(float base_alpha, float dt_s, float nominal_dt_s) {
    base_alpha = std::clamp(base_alpha, 0.0f, 1.0f);
    if (base_alpha <= 0.0f) return 0.0f;
    if (base_alpha >= 1.0f) return 1.0f;
    if (!(nominal_dt_s > 0.0f) || !(dt_s > 0.0f)) return base_alpha;
    if (dt_s == nominal_dt_s) return base_alpha;
    const float ratio = dt_s / nominal_dt_s;
    return 1.0f - std::pow(1.0f - base_alpha, ratio);
}

cv::Vec3f st_pos_step(const cv::Vec3f& held,
                      const cv::Vec3f& target,
                      const cv::Vec3f& last_raw,
                      float dt_s, float nominal_dt_s,
                      const StPosParams& p) {
    const cv::Vec3f to_target = target - held;
    const float d  = vnorm(to_target);
    const float dt = std::max(1.0e-3f, dt_s);
    const float v  = vnorm(target - last_raw) / dt;

    const float gate    = st_vel_gate(v, p.regime);
    const float alpha_d = st_alpha_d(d, p.regime);
    const float alpha   = st_rate_adjust_alpha(alpha_d, dt_s, nominal_dt_s) * gate;

    cv::Vec3f out = held + alpha * to_target;

    // Spatial lag cap: while the measurement is trusted (gate > 0), never let
    // the output trail the raw target by more than lag_cap along the held→target
    // ray. A rejected measurement (gate ~0) is a suspected glitch, so it does
    // NOT get pulled toward — the cap would defeat the outlier hold.
    if (gate > 1.0e-3f) {
        const cv::Vec3f behind = target - out;
        if (vnorm(behind) > p.lag_cap_m && d > 1.0e-9f) {
            const cv::Vec3f dir = to_target * (1.0f / d);  // unit(target − held)
            out = target - p.lag_cap_m * dir;
        }
    }
    return out;
}

float st_twist_angle(const cv::Vec4f& prev_wxyz, const cv::Vec4f& curr_wxyz) {
    cv::Vec4f delta = qmul(qconj(qnorm(prev_wxyz)), qnorm(curr_wxyz));
    if (delta[0] < 0.0f) delta = cv::Vec4f{-delta[0], -delta[1], -delta[2], -delta[3]};
    // Twist about local +Z is the (w, 0, 0, z) component; its rotation angle is
    // 2·atan2(z, w) (the xy scale cancels inside atan2).
    return 2.0f * std::atan2(delta[3], delta[0]);
}

float st_twist_alpha(float d_roll, float v_roll, float roll_confidence,
                     float dt_s, float nominal_dt_s, const StRegime& r) {
    const float alpha_d = st_alpha_d(std::abs(d_roll), r);
    const float gate    = st_vel_gate(std::abs(v_roll), r);
    const float alpha   = st_rate_adjust_alpha(alpha_d, dt_s, nominal_dt_s);
    return std::clamp(alpha * roll_confidence * gate, 0.0f, 1.0f);
}

void apply_pos_st_filter(std::array<SlimeTracker, kTrackerCount>& curr,
                         StPosState& st, const StFilterConfig& cfg,
                         float dt_s, float nominal_dt_s) {
    constexpr std::size_t kWaist = static_cast<std::size_t>(TrackerRole::Waist);

    // --- 1. Waist in world (the limb-relative reference) --------------------
    const bool had_ref = st.waist_seen;
    {
        SlimeTracker& c = curr[kWaist];
        const StPosParams& p = st_pos_params(cfg, TrackerRole::Waist);
        if (c.valid) {
            if (!st.steady[kWaist]) {
                st.held[kWaist]     = c.pos;   // snap on first / post-dropout
                st.last_raw[kWaist] = c.pos;
                st.steady[kWaist]   = true;
            } else {
                st.held[kWaist] = st_pos_step(st.held[kWaist], c.pos,
                                              st.last_raw[kWaist], dt_s, nominal_dt_s, p);
                st.last_raw[kWaist] = c.pos;
            }
            st.waist_seen = true;
        } else {
            st.steady[kWaist] = false;  // hold; recovery re-snaps
        }
        c.pos = st.held[kWaist];  // world (held when invalid; publisher skips invalid)
    }

    const bool have_ref = st.waist_seen;
    // The frame the reference first appears the limb frame origin jumps (~world
    // → waist-relative), so force a re-snap so the shift isn't read as motion.
    const bool ref_appeared = have_ref && !had_ref;
    const cv::Vec3f ref = have_ref ? st.held[kWaist] : cv::Vec3f{0.0f, 0.0f, 0.0f};

    // --- 2. Limbs in the waist-relative frame -------------------------------
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        if (i == kWaist) continue;
        SlimeTracker& c = curr[i];
        const StPosParams& p = st_pos_params(cfg, static_cast<TrackerRole>(i));
        if (ref_appeared) st.steady[i] = false;

        if (c.valid) {
            const cv::Vec3f rel_target = c.pos - ref;
            if (!st.steady[i]) {
                st.held[i]     = rel_target;   // snap into the relative frame
                st.last_raw[i] = rel_target;
                st.steady[i]   = true;
            } else {
                st.held[i] = st_pos_step(st.held[i], rel_target,
                                         st.last_raw[i], dt_s, nominal_dt_s, p);
                st.last_raw[i] = rel_target;
            }
        } else {
            st.steady[i] = false;  // hold the relative offset (drags with the waist)
        }
        c.pos = st.held[i] + ref;  // back to world
    }
}

void fill_st_twist_overrides(const std::array<SlimeTracker, kTrackerCount>& curr,
                             const std::array<cv::Vec4f, kTrackerCount>& prev_quat,
                             StTwistState& st, const StFilterConfig& cfg,
                             float dt_s, float nominal_dt_s,
                             std::array<float, kTrackerCount>& out_override) {
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        out_override[i] = -1.0f;  // sentinel: keep apply_quat_smoothing's own ta
        const TrackerRole role = static_cast<TrackerRole>(i);
        if (!st_has_roll(cfg, role)) continue;
        if (!curr[i].valid) { st.steady[i] = false; continue; }

        const cv::Vec4f raw = curr[i].quat_wxyz;
        if (!st.steady[i]) {
            // First / post-dropout: let the existing path snap; seed v_roll state.
            st.last_raw_quat[i] = raw;
            st.steady[i]        = true;
            continue;
        }
        const StRegime& r    = st_roll_params(cfg, role);
        const float d_roll   = st_twist_angle(prev_quat[i], raw);              // held→target
        const float v_roll   = st_twist_angle(st.last_raw_quat[i], raw)
                               / std::max(1.0e-3f, dt_s);                       // raw consecutive
        out_override[i] = st_twist_alpha(d_roll, v_roll, curr[i].roll_confidence,
                                         dt_s, nominal_dt_s, r);
        st.last_raw_quat[i] = raw;
    }
}

}  // namespace fitra::slimevr
