#include "lift/kalman.hpp"

#include <algorithm>

#include "lift/keypoint_format.hpp"

namespace fitra::lift {

SkeletonKalman::SkeletonKalman() : SkeletonKalman(Options{}) {}

SkeletonKalman::SkeletonKalman(Options opts) : opts_{opts} {}

void SkeletonKalman::predict(JointState& s, double dt_s) const {
    dt_s = std::clamp(dt_s, 1.0e-4, 0.2);
    cv::Matx<double, 6, 6> F = cv::Matx<double, 6, 6>::eye();
    F(0, 3) = dt_s;
    F(1, 4) = dt_s;
    F(2, 5) = dt_s;

    cv::Matx<double, 6, 6> Q = cv::Matx<double, 6, 6>::zeros();
    Q(0, 0) = opts_.q_pos;
    Q(1, 1) = opts_.q_pos;
    Q(2, 2) = opts_.q_pos;
    Q(3, 3) = opts_.q_vel;
    Q(4, 4) = opts_.q_vel;
    Q(5, 5) = opts_.q_vel;

    s.x = F * s.x;
    s.P = F * s.P * F.t() + Q;
}

void SkeletonKalman::correct(JointState& s, const infer::Joint3D& z) const {
    cv::Matx<double, 3, 6> H = cv::Matx<double, 3, 6>::zeros();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0;
    cv::Matx<double, 3, 3> R = cv::Matx<double, 3, 3>::eye() * opts_.r_meas;
    cv::Vec<double, 3> zv(z.x, z.y, z.z);
    cv::Vec<double, 3> y = zv - H * s.x;
    cv::Matx<double, 3, 3> S = H * s.P * H.t() + R;
    cv::Matx<double, 6, 3> K = s.P * H.t() * S.inv(cv::DECOMP_SVD);
    s.x = s.x + K * y;
    cv::Matx<double, 6, 6> I = cv::Matx<double, 6, 6>::eye();
    s.P = (I - K * H) * s.P;
}

infer::Skeleton3D SkeletonKalman::update(const infer::Skeleton3D& measurement, double dt_s) {
    infer::Skeleton3D out;
    const std::size_t kp_count = active_kp_count();
    out.kp_count = static_cast<std::uint8_t>(kp_count);
    for (std::size_t i = 0; i < kp_count; ++i) {
        auto& s = states_[i];
        const auto& z = measurement.joints[i];
        if (!s.initialized) {
            if (!z.valid) continue;
            s.initialized = true;
            s.missing = 0;
            s.x = cv::Vec<double, 6>(z.x, z.y, z.z, 0.0, 0.0, 0.0);
            s.P = cv::Matx<double, 6, 6>::eye() * 0.01;
        } else {
            predict(s, dt_s);
            if (z.valid) {
                correct(s, z);
                s.missing = 0;
            } else {
                s.missing += 1;
                if (s.missing > opts_.reset_after_missing) {
                    s = JointState{};
                    continue;
                }
            }
        }

        out.joints[i].x = static_cast<float>(s.x(0));
        out.joints[i].y = static_cast<float>(s.x(1));
        out.joints[i].z = static_cast<float>(s.x(2));
        out.joints[i].score = z.valid ? z.score : 0.05f;
        out.joints[i].valid = s.initialized;
    }
    return out;
}

}  // namespace fitra::lift
