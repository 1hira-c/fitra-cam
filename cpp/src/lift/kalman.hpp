#pragma once
//
// Kinematic-tree 3D skeleton Kalman.
//
// Per pose-3d/locomotion-stability: the original per-joint independent
// filters caused "extended-leg locomotion" foot freeze (hip moves but the
// ankle's Kalman has no knowledge of that velocity, so prediction stays
// put). The state is now structured around the parent tree:
//
//   * root joint (the one with parent == -1 in the active SkeletonDef
//     — hip_center under Halpe26, l_hip under COCO17) keeps the original
//     6D world position + velocity state.
//   * every other joint keeps a 6D **parent-relative offset** + velocity
//     state. Its emitted world position is parent_world + offset, so a
//     root translation is automatically carried down the chain without
//     touching individual child Kalman states.
//
// Process noise is parameterized separately for root vs offset (the
// child offset typically has near-zero "drift velocity" — small Q keeps
// the chain rigid in the absence of a measurement). The measurement
// model and update step are unchanged in shape — only the residual
// changes: for a child, residual = z_world − parent_world − x_offset.
//
// API parity: SkeletonKalman::update(measurement, dt_s) still returns
// a Skeleton3D in world coordinates, so multi_pipeline.cpp needs no
// changes.

#include <array>

#include <opencv2/core.hpp>

#include "infer/types.hpp"
#include "lift/keypoint_format.hpp"

namespace fitra::lift {

class SkeletonKalman {
public:
    struct Options {
        double q_pos = 1.0e-4;          // root world process noise (position)
        double q_vel = 1.0e-2;          // root world process noise (velocity)
        // Child offset process noise. Default matches the root values so
        // existing tuning still applies; can be lowered to keep the chain
        // more rigid when joints are well-anchored to the parent.
        double q_pos_offset = 1.0e-4;
        double q_vel_offset = 1.0e-2;
        double r_meas = 2.5e-3;  // m^2
        int reset_after_missing = 30;
    };

    SkeletonKalman();
    explicit SkeletonKalman(Options opts);

    infer::Skeleton3D update(const infer::Skeleton3D& measurement, double dt_s);

private:
    struct JointState {
        bool initialized = false;
        int missing = 0;
        // For the root joint: world position+velocity. For every other
        // joint: parent-relative offset+velocity.
        cv::Vec<double, 6> x{0, 0, 0, 0, 0, 0};
        cv::Matx<double, 6, 6> P = cv::Matx<double, 6, 6>::eye();
        // Cached world position from this update's emit. Children read
        // it during their own correct step (to convert z_world →
        // z_offset) and at output time (to FK their offset into world).
        cv::Vec3d world_pos{0, 0, 0};
    };

    void predict(JointState& s, double dt_s, double q_pos, double q_vel) const;
    void correct(JointState& s, const cv::Vec3d& z) const;
    void ensure_topology();

    Options opts_;
    // Sized for the largest supported topology. ensure_topology() populates
    // topo_order_ with the BFS-from-root traversal of the active SkeletonDef
    // so process order respects parent-before-child even when raw parent[]
    // is not topologically sorted in index order.
    std::array<JointState, infer::kMaxKeypoints> states_{};
    int                                          root_idx_   = -1;
    std::size_t                                  topo_count_ = 0;
    std::array<int, infer::kMaxKeypoints>        topo_order_{};
    // Sentinel: 0xFF means "not built yet". rebuild on first update and
    // whenever active_keypoint_format() changes (e.g., between tests).
    unsigned char                                topo_format_tag_ = 0xFF;
};

}  // namespace fitra::lift
