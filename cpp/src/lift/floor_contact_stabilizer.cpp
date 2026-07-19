#include "lift/floor_contact_stabilizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "lift/skeleton_def.hpp"

namespace fitra::lift {

namespace {

constexpr float kSoleOutlierPenetrationM = 0.005f;

bool in_range(const infer::Skeleton3D& skel, std::size_t j) {
    return skel.kp_count > 0
        && j < static_cast<std::size_t>(skel.kp_count)
        && j < skel.joints.size();
}

bool valid_joint(const infer::Skeleton3D& skel, std::size_t j) {
    if (!in_range(skel, j) || !skel.joints[j].valid) return false;
    const auto& point = skel.joints[j];
    return std::isfinite(point.x)
        && std::isfinite(point.y)
        && std::isfinite(point.z);
}

cv::Vec3f point_of(const infer::Joint3D& j) {
    return {j.x, j.y, j.z};
}

bool nonzero(const cv::Vec3f& v) {
    constexpr float kEps = 1.0e-7f;
    return std::abs(v[0]) > kEps || std::abs(v[1]) > kEps || std::abs(v[2]) > kEps;
}

bool apply_translation(infer::Skeleton3D& skel,
                       const HalpeFootJoints& def,
                       const cv::Vec3f& delta) {
    if (!nonzero(delta)) return false;
    bool changed = false;
    auto move = [&](std::size_t j) {
        if (!valid_joint(skel, j)) return;
        skel.joints[j].x += delta[0];
        skel.joints[j].y += delta[1];
        skel.joints[j].z += delta[2];
        changed = true;
    };
    move(def.ankle);
    for (std::size_t j : def.sole) move(j);
    return changed;
}

}  // namespace

FloorContactStabilizer::FloorContactStabilizer(FloorContactOptions opts)
    : opts_{opts} {
    if (const char* error = floor_contact_options_error(opts_)) {
        throw std::invalid_argument(error);
    }
}

void FloorContactStabilizer::reset() {
    state_ = {};
}

FloorContactReport FloorContactStabilizer::update(infer::Skeleton3D& skel,
                                                  double dt_s) {
    FloorContactReport report;

    if (!std::isfinite(dt_s) || dt_s <= 0.0 || dt_s > opts_.reset_dt_s) {
        reset();
        dt_s = 1.0 / 30.0;
    }

    for (FootState& st : state_) {
        if (st.has_prev_ankle) st.elapsed_since_prev_s += dt_s;
    }

    for (std::size_t side = 0; side < kHalpeFeet.size(); ++side) {
        const HalpeFootJoints& def = kHalpeFeet[side];
        FootState& st = state_[side];
        FootContactReport& foot_report = report.feet[side];
        auto begin_release = [&]() {
            st.contact = false;
            st.missing_frames = 0;
            st.exit_candidate_s = 0.0;
            st.exit_candidate_frames = 0;
            st.release_active = nonzero(st.last_correction);
        };
        auto seed_prev = [&](const cv::Vec3f& raw_ankle) {
            st.prev_raw_ankle = raw_ankle;
            st.has_prev_ankle = true;
            st.elapsed_since_prev_s = 0.0;
        };
        auto decayed_release_correction = [&]() {
            if (!st.release_active) return cv::Vec3f{0.0f, 0.0f, 0.0f};
            const float decay = static_cast<float>(std::exp(
                -dt_s / std::max(opts_.release_tau_s, 1.0e-6)));
            st.last_correction *= decay;
            if (cv::norm(st.last_correction) < 1.0e-4) {
                st.last_correction = {0.0f, 0.0f, 0.0f};
                st.release_active = false;
            }
            return st.last_correction;
        };
        auto apply_report = [&](const cv::Vec3f& correction) {
            if (apply_translation(skel, def, correction)) {
                foot_report.corrected = true;
                foot_report.correction_m = correction;
            } else {
                // Report what was actually applied, not the state-machine
                // request (important during a full dropout grace frame).
                foot_report.corrected = false;
                foot_report.correction_m = {0.0f, 0.0f, 0.0f};
            }
        };

        const bool ankle_valid = valid_joint(skel, def.ankle);
        struct SoleSample {
            float z;
            std::size_t joint;
        };
        std::array<SoleSample, 3> soles{};
        std::size_t valid_soles = 0;
        for (std::size_t j : def.sole) {
            if (!valid_joint(skel, j)) continue;
            soles[valid_soles++] = {skel.joints[j].z, j};
        }
        std::sort(soles.begin(), soles.begin()
                                  + static_cast<std::ptrdiff_t>(valid_soles),
                  [](const SoleSample& a, const SoleSample& b) {
                      return a.z < b.z;
                  });

        const float floor_z = static_cast<float>(opts_.floor_z_m);
        // Reject only an isolated point that is materially below the known
        // floor. A legitimate heel contact at floor level with raised toes is
        // retained; a triangulated heel centimetres under the floor is not.
        if (valid_soles >= 2
            && soles[0].z < floor_z - kSoleOutlierPenetrationM
            && soles[1].z - soles[0].z
                > static_cast<float>(opts_.enter_height_m)) {
            skel.joints[soles[0].joint].valid = false;
            skel.joints[soles[0].joint].score = 0.0f;
            foot_report.sole_outlier_rejected = true;
            for (std::size_t i = 1; i < valid_soles; ++i) {
                soles[i - 1] = soles[i];
            }
            --valid_soles;
        }
        const bool evidence_valid = ankle_valid && valid_soles >= 2;
        foot_report.evidence_valid = evidence_valid;

        if (!evidence_valid) {
            if (st.contact && st.missing_frames < opts_.missing_grace_frames) {
                ++st.missing_frames;
                st.exit_candidate_s = 0.0;
                st.exit_candidate_frames = 0;
                foot_report.contact = true;
                foot_report.missing_grace = true;
                cv::Vec3f raw_ankle;
                if (ankle_valid) raw_ankle = point_of(skel.joints[def.ankle]);
                apply_report(st.last_correction);
                if (ankle_valid) seed_prev(raw_ankle);
                continue;
            }

            if (!ankle_valid) {
                // Once a full-foot dropout outlives grace there is no visible
                // release transition to smooth. Discard the stale correction
                // and velocity baseline; applying it when the foot reappears
                // would create a new pop rather than prevent one.
                st.contact = false;
                st.release_active = false;
                st.last_correction = {0.0f, 0.0f, 0.0f};
                st.missing_frames = 0;
                st.exit_candidate_s = 0.0;
                st.exit_candidate_frames = 0;
                st.has_prev_ankle = false;
                st.elapsed_since_prev_s = 0.0;
                continue;
            }

            const cv::Vec3f raw_ankle = point_of(skel.joints[def.ankle]);
            begin_release();
            apply_report(decayed_release_correction());
            seed_prev(raw_ankle);
            continue;
        }

        const float support_z = soles[0].z;

        const cv::Vec3f raw_ankle = point_of(skel.joints[def.ankle]);
        bool speed_known = st.has_prev_ankle && st.elapsed_since_prev_s > 0.0;
        float speed_mps = 0.0f;
        if (speed_known) {
            speed_mps = static_cast<float>(
                cv::norm(raw_ankle - st.prev_raw_ankle)
                / st.elapsed_since_prev_s);
        }

        const float support_height = support_z - floor_z;
        const float z_correction = floor_z - support_z;
        const float max_z_correction =
            static_cast<float>(opts_.max_z_correction_m);
        const float bounded_z_correction = std::clamp(
            z_correction, -max_z_correction, max_z_correction);
        const bool z_within_limit =
            std::abs(z_correction) <= max_z_correction;
        const bool deep_penetration =
            z_correction > max_z_correction;

        bool released_this_frame = false;
        bool exit_pending = false;
        if (st.contact) {
            const cv::Vec2f raw_xy{raw_ankle[0], raw_ankle[1]};
            const float anchor_error = static_cast<float>(cv::norm(st.anchor_xy - raw_xy));
            const bool exit_signal =
                support_height > static_cast<float>(opts_.exit_height_m)
                || (speed_known && speed_mps > static_cast<float>(opts_.exit_speed_mps))
                || anchor_error > static_cast<float>(opts_.max_xy_correction_m);
            if (deep_penetration) {
                begin_release();
                released_this_frame = true;
            } else if (exit_signal) {
                st.exit_candidate_s += dt_s;
                ++st.exit_candidate_frames;
                const bool exit_confirmed = opts_.exit_grace_s == 0.0
                    || (st.exit_candidate_frames >= 2
                        && st.exit_candidate_s + 1.0e-9
                            >= opts_.exit_grace_s);
                if (exit_confirmed) {
                    begin_release();
                    released_this_frame = true;
                } else {
                    exit_pending = true;
                }
            } else {
                st.exit_candidate_s = 0.0;
                st.exit_candidate_frames = 0;
            }
        }

        if (!st.contact && !st.release_active && !released_this_frame && speed_known
            && support_height <= static_cast<float>(opts_.enter_height_m)
            && speed_mps < static_cast<float>(opts_.enter_speed_mps)
            && z_within_limit) {
            st.contact = true;
            st.release_active = false;
            st.exit_candidate_s = 0.0;
            st.exit_candidate_frames = 0;
            st.anchor_xy = {raw_ankle[0], raw_ankle[1]};
            st.missing_frames = 0;
        }

        cv::Vec3f correction{0.0f, 0.0f, 0.0f};
        if (st.contact) {
            if (exit_pending) {
                // Keep the last bounded correction during a transient exit
                // signal. Recomputing from a one-frame position/height spike
                // would pin the outlier to the floor before it is confirmed.
                correction = st.last_correction;
                if (z_correction >= 0.0f) {
                    correction[2] = std::max(
                        correction[2], bounded_z_correction);
                } else {
                    // Do not carry an upward correction onto a sole that is
                    // already above the floor, or pull it through the floor.
                    correction[2] = std::clamp(
                        correction[2], bounded_z_correction, 0.0f);
                }
            } else {
                const float alpha = static_cast<float>(1.0 - std::exp(
                    -dt_s / std::max(opts_.xy_anchor_tau_s, 1.0e-6)));
                const cv::Vec2f raw_xy{raw_ankle[0], raw_ankle[1]};
                st.anchor_xy += alpha * (raw_xy - st.anchor_xy);
                const cv::Vec2f xy_correction = st.anchor_xy - raw_xy;
                correction = {
                    xy_correction[0], xy_correction[1], bounded_z_correction};
                st.last_correction = correction;
            }
            st.missing_frames = 0;
            foot_report.contact = true;
        } else {
            correction = decayed_release_correction();
            // Never fail open on a deep penetration, and do not let a negative
            // release correction pull an otherwise-above-floor sole through
            // the floor. The requested lift remains bounded by max_z.
            correction[2] = std::max(
                correction[2], bounded_z_correction);
        }

        apply_report(correction);
        seed_prev(raw_ankle);
    }

    return report;
}

}  // namespace fitra::lift
