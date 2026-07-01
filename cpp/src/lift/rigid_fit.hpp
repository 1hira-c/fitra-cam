#pragma once
//
// Spatial rigid-body fit (weighted Kabsch / Procrustes) for the
// spatial-filtering track (docs/design/pose-3d-spatial-filtering.md).
//
// The rig's dominant stationary jitter is type (b): each view micro-moves
// independently, so the triangulated 3D point of a joint scatters even though
// the reprojection error stays ~0. What does NOT move is the *rigid structure*
// of a body segment (the pelvis triangle {hip_center,l_hip,r_hip}, the shoulder
// girdle {neck,l_shoulder,r_shoulder}). Forcing the three measured points back
// onto a known-shape template each frame, via a weighted rigid fit, averages the
// three joints' independent noise into a single 6-DoF pose estimate — so the
// segment (and the root it anchors) stops shimmering, with no temporal lag.
//
// This is a pure, stateless geometric op: the template shape is static (from the
// subject profile's segment distances) and the fit is solved fresh each frame.

#include <array>

#include <opencv2/core.hpp>

namespace fitra::lift {

// A rigid 3-point template: canonical positions carrying only the *shape* of a
// segment (the three pairwise distances). The absolute placement is arbitrary —
// the fit solves the 6-DoF pose — so only the shape matters.
struct RigidTemplate {
    std::array<cv::Vec3d, 3> pts{};
    bool valid = false;

    // Build from the three pairwise distances of a segment:
    //   d01 = |p0-p1|, d02 = |p0-p2|, d12 = |p1-p2|
    // p0 is the apex (hip_center / neck), p1/p2 the two side joints
    // (l/r hip or l/r shoulder). Places p0 at the origin, p1 on +x, p2 in the
    // +y half of the xy-plane. `valid` is false if any distance is non-positive,
    // the triangle inequality is violated, or the triangle is too flat
    // (perpendicular height of p0 below `min_height_m`) to constrain a 3D
    // orientation — a near-collinear segment leaves rotation about its long axis
    // unobservable, so the caller must skip the fit and fall back.
    static RigidTemplate from_distances(double d01, double d02, double d12,
                                        double min_height_m = 0.005);
};

// Weighted Kabsch (rigid: rotation + translation, NO scale). Finds R,t
// minimising Σ wᵢ · |R·templ[i] + t − measured[i]|² and writes the fitted
// (rigid, denoised) positions R·templ[i] + t to `out`. Weights are per-point
// confidences (e.g. score · g(view_count)); they need not be normalised.
//
// Returns false (leaving `out` untouched) if the template is invalid, the
// weights sum to ~0, or the measured points are so degenerate that the rotation
// is unrecoverable — the caller then keeps the measured points as-is.
bool fit_rigid_triangle(const RigidTemplate& templ,
                        const std::array<cv::Vec3d, 3>& measured,
                        const std::array<double, 3>& weights,
                        std::array<cv::Vec3d, 3>& out);

}  // namespace fitra::lift
