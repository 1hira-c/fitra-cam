#pragma once

#include <array>

#include <opencv2/core.hpp>

#include "infer/types.hpp"

namespace fitra::lift {

class SkeletonKalman {
public:
    struct Options {
        double q_pos = 1.0e-4;
        double q_vel = 1.0e-2;
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
        cv::Vec<double, 6> x{0, 0, 0, 0, 0, 0};
        cv::Matx<double, 6, 6> P = cv::Matx<double, 6, 6>::eye();
    };

    void predict(JointState& s, double dt_s) const;
    void correct(JointState& s, const infer::Joint3D& z) const;

    Options opts_;
    std::array<JointState, infer::kNumKeypoints> states_{};
};

}  // namespace fitra::lift
