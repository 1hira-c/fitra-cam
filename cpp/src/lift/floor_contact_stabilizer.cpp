#include "lift/floor_contact_stabilizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace fitra::lift {

namespace {

struct FootDef {
    std::size_t ankle;
    std::array<std::size_t, 3> sole;
};

// Halpe26: ankle, then big-toe / small-toe / heel for each side.
constexpr std::array<FootDef, 2> kFeet{{
    {15, {20, 22, 24}},
    {16, {21, 23, 25}},
}};

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

float vec_norm(const cv::Vec3f& v) {
    return std::sqrt(v.dot(v));
}

float vec_norm(const cv::Vec2f& v) {
    return std::sqrt(v.dot(v));
}

bool nonzero(const cv::Vec3f& v) {
    constexpr float kEps = 1.0e-7f;
    return std::abs(v[0]) > kEps || std::abs(v[1]) > kEps || std::abs(v[2]) > kEps;
}

bool apply_translation(infer::Skeleton3D& skel,
                       const FootDef& def,
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
    : opts_{opts} {}

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

    for (std::size_t side = 0; side < kFeet.size(); ++side) {
        const FootDef& def = kFeet[side];
        FootState& st = state_[side];
        FootContactReport& foot_report = report.feet[side];
        auto release_contact = [&]() {
            st.contact = false;
            st.missing_frames = 0;
            st.last_correction = {0.0f, 0.0f, 0.0f};
        };
        auto seed_prev = [&](const cv::Vec3f& raw_ankle) {
            st.prev_raw_ankle = raw_ankle;
            st.has_prev_ankle = true;
            st.elapsed_since_prev_s = 0.0;
        };

        const bool ankle_valid = valid_joint(skel, def.ankle);
        int valid_soles = 0;
        float support_z = std::numeric_limits<float>::infinity();
        for (std::size_t j : def.sole) {
            if (!valid_joint(skel, j)) continue;
            ++valid_soles;
            support_z = std::min(support_z, skel.joints[j].z);
        }
        const bool evidence_valid = ankle_valid && valid_soles >= 2;

        if (!evidence_valid) {
            if (st.contact && st.missing_frames < opts_.missing_grace_frames) {
                ++st.missing_frames;
                foot_report.contact = true;
                foot_report.missing_grace = true;
                foot_report.correction_m = st.last_correction;
                foot_report.corrected = apply_translation(
                    skel, def, st.last_correction);
                continue;
            }

            release_contact();
            if (ankle_valid) {
                seed_prev(point_of(skel.joints[def.ankle]));
            } else {
                // Once the grace window is exhausted, a full ankle dropout
                // invalidates the raw-to-raw velocity baseline. Keeping it
                // would average the next speed over an arbitrarily long gap
                // and could immediately re-latch a foot at a new location.
                st.has_prev_ankle = false;
                st.elapsed_since_prev_s = 0.0;
            }
            continue;
        }

        const cv::Vec3f raw_ankle = point_of(skel.joints[def.ankle]);
        bool speed_known = st.has_prev_ankle && st.elapsed_since_prev_s > 0.0;
        float speed_mps = 0.0f;
        if (speed_known) {
            speed_mps = vec_norm(raw_ankle - st.prev_raw_ankle)
                      / static_cast<float>(st.elapsed_since_prev_s);
        }

        const float floor_z = static_cast<float>(opts_.floor_z_m);
        const float support_height = support_z - floor_z;
        const float z_correction = floor_z - support_z;
        const bool z_within_limit =
            std::abs(z_correction) <= static_cast<float>(opts_.max_z_correction_m);

        bool released_this_frame = false;
        if (st.contact) {
            const cv::Vec2f raw_xy{raw_ankle[0], raw_ankle[1]};
            const float anchor_error = vec_norm(st.anchor_xy - raw_xy);
            const bool release =
                support_height > static_cast<float>(opts_.exit_height_m)
                || (speed_known && speed_mps > static_cast<float>(opts_.exit_speed_mps))
                || anchor_error > static_cast<float>(opts_.max_xy_correction_m)
                || !z_within_limit;
            if (release) {
                release_contact();
                released_this_frame = true;
            }
        }

        if (!st.contact && !released_this_frame && speed_known
            && support_height <= static_cast<float>(opts_.enter_height_m)
            && speed_mps < static_cast<float>(opts_.enter_speed_mps)
            && z_within_limit) {
            st.contact = true;
            st.anchor_xy = {raw_ankle[0], raw_ankle[1]};
            st.missing_frames = 0;
        }

        cv::Vec3f correction{0.0f, 0.0f, 0.0f};
        if (st.contact) {
            const float alpha = static_cast<float>(1.0 - std::exp(
                -dt_s / std::max(opts_.xy_anchor_tau_s, 1.0e-6)));
            const cv::Vec2f raw_xy{raw_ankle[0], raw_ankle[1]};
            st.anchor_xy += alpha * (raw_xy - st.anchor_xy);
            const cv::Vec2f xy_correction = st.anchor_xy - raw_xy;
            correction = {xy_correction[0], xy_correction[1], z_correction};
            st.last_correction = correction;
            st.missing_frames = 0;
            foot_report.contact = true;
        } else if (support_z < floor_z && z_within_limit) {
            // Stateless physical floor clamp while airborne or on the first
            // sample.  Keep XY free unless the contact latch is active.
            correction[2] = z_correction;
        }

        foot_report.correction_m = correction;
        foot_report.corrected = apply_translation(skel, def, correction);
        seed_prev(raw_ankle);
    }

    return report;
}

}  // namespace fitra::lift
