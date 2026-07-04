#include "lift/floor_grounding.hpp"

#include <algorithm>
#include <cmath>

namespace fitra::lift {

namespace {
// Halpe26 foot SOLE contact points: big-toe (20/21), small-toe (22/23),
// heel (24/25). The ankle (15/16) is the leg joint (~8 cm up in stance), not a
// contact point, so it is deliberately excluded. COCO17 has none of these
// (kp_count = 17), so the loop body is skipped -> no-op.
constexpr std::array<std::size_t, 6> kSoleJoints{20, 21, 22, 23, 24, 25};
}  // namespace

bool apply_floor_grounding(infer::Skeleton3D& skel,
                           FloorGroundingState& state,
                           double dt_s,
                           const FloorGroundingOptions& opts) {
    const float floor    = static_cast<float>(opts.floor_z_m);
    const float band     = static_cast<float>(opts.snap_band_m);
    const float vel_th   = static_cast<float>(opts.stance_vel_mps);
    const float dt       = std::max(1.0e-3f, static_cast<float>(dt_s));
    bool modified = false;

    for (std::size_t j : kSoleJoints) {
        if (j >= skel.kp_count) continue;         // COCO17 / short topology: no-op
        infer::Joint3D& p = skel.joints[j];
        if (!p.valid) { state.has_prev[j] = false; continue; }

        // Raw (pre-grounding) position for the stance-speed estimate + next-frame
        // anchor, captured before any modification.
        const cv::Vec3f raw{p.x, p.y, p.z};

        // (1) below-floor clamp — always, stateless.
        if (p.z < floor) { p.z = floor; modified = true; }

        // (2) stance snap — needs a previous raw sample for the speed estimate.
        if (state.has_prev[j]) {
            const cv::Vec3f d = raw - state.prev_pos[j];
            const float speed = std::sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]) / dt;
            if (p.z >= floor && p.z < floor + band && speed < vel_th) {
                if (p.z != floor) { p.z = floor; modified = true; }
            }
        }

        state.prev_pos[j] = raw;   // raw-to-raw speed next frame (grounding not fed back)
        state.has_prev[j] = true;
    }
    return modified;
}

}  // namespace fitra::lift
