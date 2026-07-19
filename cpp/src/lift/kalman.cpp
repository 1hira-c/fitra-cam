#include "lift/kalman.hpp"

#include <algorithm>
#include <stdexcept>

namespace fitra::lift {

SkeletonKalman::SkeletonKalman() : SkeletonKalman(Options{}) {}

SkeletonKalman::SkeletonKalman(Options opts) : opts_{opts} {}

void SkeletonKalman::predict(JointState& s, double dt_s,
                             double q_pos, double q_vel) const {
    dt_s = std::clamp(dt_s, 1.0e-4, 0.2);
    cv::Matx<double, 6, 6> F = cv::Matx<double, 6, 6>::eye();
    F(0, 3) = dt_s;
    F(1, 4) = dt_s;
    F(2, 5) = dt_s;

    cv::Matx<double, 6, 6> Q = cv::Matx<double, 6, 6>::zeros();
    Q(0, 0) = q_pos;
    Q(1, 1) = q_pos;
    Q(2, 2) = q_pos;
    Q(3, 3) = q_vel;
    Q(4, 4) = q_vel;
    Q(5, 5) = q_vel;

    s.x = F * s.x;
    s.P = F * s.P * F.t() + Q;
}

void SkeletonKalman::correct(JointState& s, const cv::Vec3d& z) const {
    cv::Matx<double, 3, 6> H = cv::Matx<double, 3, 6>::zeros();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0;
    cv::Matx<double, 3, 3> R = cv::Matx<double, 3, 3>::eye() * opts_.r_meas;
    cv::Vec<double, 3> y = z - H * s.x;
    cv::Matx<double, 3, 3> S = H * s.P * H.t() + R;
    cv::Matx<double, 6, 3> K = s.P * H.t() * S.inv(cv::DECOMP_SVD);
    s.x = s.x + K * y;
    cv::Matx<double, 6, 6> I = cv::Matx<double, 6, 6>::eye();
    s.P = (I - K * H) * s.P;
}

void SkeletonKalman::ensure_topology() {
    const auto tag = static_cast<unsigned char>(active_keypoint_format());
    if (root_idx_ >= 0 && topo_format_tag_ == tag) return;

    const auto& def     = active_skeleton_def();
    const auto& parents = def.parents;

    // The BFS scratch arrays below are stack-allocated at kMaxKeypoints. A
    // future custom skeleton with more joints would overflow them silently;
    // fail loudly instead.
    if (parents.size() > infer::kMaxKeypoints) {
        throw std::runtime_error(
            "Active skeleton keypoint count exceeds kMaxKeypoints");
    }

    // Find the root (the joint whose parent == -1). There must be exactly
    // one — both Coco17 and Halpe26 satisfy this — but we just take the
    // first to keep the loop simple.
    root_idx_ = -1;
    for (std::size_t i = 0; i < parents.size(); ++i) {
        if (parents[i] == -1) {
            root_idx_ = static_cast<int>(i);
            break;
        }
    }

    topo_count_ = 0;
    if (root_idx_ < 0) {
        topo_format_tag_ = tag;
        return;
    }

    // BFS from root using a fixed-size queue (kMaxKeypoints is small).
    // Order matters: by the time we process a child, the parent must
    // already have been processed in the same update() pass so its
    // world_pos is fresh.
    std::array<bool, infer::kMaxKeypoints> visited{};
    std::array<int,  infer::kMaxKeypoints> queue{};
    std::size_t qhead = 0, qtail = 0;
    queue[qtail++] = root_idx_;
    visited[static_cast<std::size_t>(root_idx_)] = true;

    while (qhead < qtail) {
        const int node = queue[qhead++];
        topo_order_[topo_count_++] = node;
        for (std::size_t i = 0; i < parents.size(); ++i) {
            if (parents[i] == node && !visited[i]) {
                visited[i] = true;
                queue[qtail++] = static_cast<int>(i);
            }
        }
    }

    topo_format_tag_ = tag;
}

void SkeletonKalman::reset() {
    // Re-default every joint (initialized=false, missing=0, P=eye). The
    // topology cache (topo_order_/root_idx_/topo_format_tag_) is left intact —
    // it is rebuilt lazily only when the keypoint format changes.
    states_.fill(JointState{});
}

infer::Skeleton3D SkeletonKalman::update(const infer::Skeleton3D& measurement,
                                         double dt_s) {
    ensure_topology();

    infer::Skeleton3D out;
    const std::size_t kp_count = active_kp_count();
    out.kp_count = static_cast<std::uint8_t>(kp_count);

    if (root_idx_ < 0 || topo_count_ == 0) return out;

    const auto& def     = active_skeleton_def();
    const auto& parents = def.parents;

    // First, default the output to "invalid" so joints we don't visit
    // (e.g. unreachable from root, or skipped because the parent is
    // uninitialized) emit valid=false.
    for (std::size_t i = 0; i < kp_count; ++i) {
        out.joints[i].valid = false;
        out.joints[i].score = 0.0f;
    }

    for (std::size_t order_idx = 0; order_idx < topo_count_; ++order_idx) {
        const int   i        = topo_order_[order_idx];
        // Guard before any indexing: `i` drives states_/measurement.joints
        // below. By construction topo_order_ only holds valid indices, but
        // keep the bounds check at the top so a malformed topology can never
        // produce an out-of-bounds access.
        if (i < 0 || static_cast<std::size_t>(i) >= kp_count) continue;

        // Halpe26 facial landmarks are intentionally outside the 3D body-lift
        // contract. The output was initialized invalid above; skip all state
        // prediction/correction work for these joints.
        if (!participates_in_3d_lift(def.format,
                                     static_cast<std::size_t>(i))) continue;

        const bool  is_root  = (i == root_idx_);
        const int   parent   = is_root ? -1 : parents[static_cast<std::size_t>(i)];
        auto&       s        = states_[static_cast<std::size_t>(i)];
        const auto& z        = measurement.joints[static_cast<std::size_t>(i)];

        // A child cannot be initialized or even predicted in offset space
        // until its parent has a world position. Skip and leave the
        // measurement queued for a later frame — but still age an
        // already-initialized child's missing counter so a long parent
        // dropout drops the stale offset instead of resurrecting it once
        // the parent reappears.
        if (!is_root && (parent < 0 || static_cast<std::size_t>(parent) >= kp_count ||
                         !states_[static_cast<std::size_t>(parent)].initialized)) {
            if (s.initialized) {
                s.missing += 1;
                if (s.missing > opts_.reset_after_missing) {
                    s = JointState{};
                }
            }
            continue;
        }

        if (!s.initialized) {
            if (!z.valid) continue;
            const cv::Vec3d zv(z.x, z.y, z.z);
            if (is_root) {
                s.x = cv::Vec<double, 6>(zv[0], zv[1], zv[2], 0.0, 0.0, 0.0);
                s.world_pos = zv;
            } else {
                const cv::Vec3d parent_world =
                    states_[static_cast<std::size_t>(parent)].world_pos;
                const cv::Vec3d offset = zv - parent_world;
                s.x = cv::Vec<double, 6>(offset[0], offset[1], offset[2], 0, 0, 0);
                s.world_pos = zv;
            }
            s.initialized = true;
            s.missing = 0;
            s.P = cv::Matx<double, 6, 6>::eye() * 0.01;
        } else {
            const double q_pos = is_root ? opts_.q_pos : opts_.q_pos_offset;
            const double q_vel = is_root ? opts_.q_vel : opts_.q_vel_offset;
            predict(s, dt_s, q_pos, q_vel);

            if (z.valid) {
                cv::Vec3d zv(z.x, z.y, z.z);
                if (is_root) {
                    correct(s, zv);
                } else {
                    const cv::Vec3d parent_world =
                        states_[static_cast<std::size_t>(parent)].world_pos;
                    const cv::Vec3d z_offset = zv - parent_world;
                    correct(s, z_offset);
                }
                s.missing = 0;
            } else {
                s.missing += 1;
                if (s.missing > opts_.reset_after_missing) {
                    s = JointState{};
                    continue;
                }
            }

            if (is_root) {
                s.world_pos = cv::Vec3d(s.x(0), s.x(1), s.x(2));
            } else {
                const cv::Vec3d parent_world =
                    states_[static_cast<std::size_t>(parent)].world_pos;
                s.world_pos = parent_world +
                              cv::Vec3d(s.x(0), s.x(1), s.x(2));
            }
        }

        auto& out_j = out.joints[static_cast<std::size_t>(i)];
        out_j.x = static_cast<float>(s.world_pos[0]);
        out_j.y = static_cast<float>(s.world_pos[1]);
        out_j.z = static_cast<float>(s.world_pos[2]);
        out_j.score = z.valid ? z.score : 0.05f;
        out_j.valid = s.initialized;
    }

    return out;
}

}  // namespace fitra::lift
