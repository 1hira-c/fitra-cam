#pragma once
//
// Synthetic Halpe26 head-facing endpoint.
//
// The raw nose observation is useful for telling front from back, but its
// triangulated position and reprojection error are too noisy to treat as a
// body joint. Keep only its direction: remove the component parallel to the
// filtered neck->head_top axis, normalize, and place a fixed-length endpoint
// at head_top. The endpoint reuses Halpe26 index 0 so the existing
// head_top<->nose viewer edge continues to draw a short forward ray.

#include <algorithm>
#include <cmath>

#include "infer/types.hpp"
#include "lift/skeleton_def.hpp"

namespace fitra::lift {

inline constexpr float kHeadDirectionLengthM = 0.15f;

struct HalpeHeadDirectionEvidence {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float score = 0.0f;
    bool valid = false;
};

inline HalpeHeadDirectionEvidence observe_halpe_head_direction(
    const infer::Skeleton3D& skeleton) {
    const auto& nose = skeleton.joints[kHalpeNose];
    const auto& head_top = skeleton.joints[kHalpeHeadTop];
    if (!nose.valid || !head_top.valid) return {};

    HalpeHeadDirectionEvidence out;
    out.x = nose.x - head_top.x;
    out.y = nose.y - head_top.y;
    out.z = nose.z - head_top.z;
    out.score = std::min(nose.score, head_top.score);
    out.valid = std::isfinite(out.x) && std::isfinite(out.y) &&
                std::isfinite(out.z);
    return out;
}

inline void synthesize_halpe_head_direction(
    infer::Skeleton3D& dst,
    const HalpeHeadDirectionEvidence& direction_evidence) {
    // Eyes/ears never enter the 3D skeleton. Nose is reset below and becomes
    // valid only when a finite, non-degenerate facing direction is available.
    for (std::size_t k = 0; k <= 4; ++k) dst.joints[k] = {};

    const auto& head_top = dst.joints[kHalpeHeadTop];
    const auto& neck = dst.joints[kHalpeNeck];
    if (!direction_evidence.valid || !head_top.valid || !neck.valid) {
        return;
    }

    const float ax = head_top.x - neck.x;
    const float ay = head_top.y - neck.y;
    const float az = head_top.z - neck.z;
    const float axis_sq = ax * ax + ay * ay + az * az;

    const float ox = direction_evidence.x;
    const float oy = direction_evidence.y;
    const float oz = direction_evidence.z;
    if (!std::isfinite(axis_sq) || axis_sq < 1.0e-8f ||
        !std::isfinite(ox) || !std::isfinite(oy) || !std::isfinite(oz)) {
        return;
    }

    // Keep only the component perpendicular to the head axis. This discards
    // the unreliable "how far below head_top is the nose" component while
    // retaining the observed front/back direction around the axis.
    const float along = (ox * ax + oy * ay + oz * az) / axis_sq;
    const float fx = ox - along * ax;
    const float fy = oy - along * ay;
    const float fz = oz - along * az;
    const float forward_norm = std::sqrt(fx * fx + fy * fy + fz * fz);
    if (!std::isfinite(forward_norm) || forward_norm < 1.0e-5f) return;

    const float scale = kHeadDirectionLengthM / forward_norm;
    auto& endpoint = dst.joints[kHalpeNose];
    endpoint.x = head_top.x + fx * scale;
    endpoint.y = head_top.y + fy * scale;
    endpoint.z = head_top.z + fz * scale;
    endpoint.score = std::min({direction_evidence.score, head_top.score, neck.score});
    endpoint.valid = true;
}

inline void synthesize_halpe_head_direction(
    infer::Skeleton3D& dst,
    const infer::Skeleton3D& direction_evidence) {
    // Observe before clearing dst: callers may pass the same skeleton as both
    // arguments (Triangulator does this after its per-joint loop).
    const auto observed = observe_halpe_head_direction(direction_evidence);
    synthesize_halpe_head_direction(dst, observed);
}

}  // namespace fitra::lift
