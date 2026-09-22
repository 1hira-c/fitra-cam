#pragma once
// Offline-only distance-adaptive residual filter. Never wired to live output.
#include <array>
#include <cmath>
#include <stdexcept>
#include "tracking/tracker_extract.hpp"

namespace fitra::temporal_experiment {

inline cv::Vec3f distance_step(const cv::Vec3f& held, const cv::Vec3f& target, double cap_m) {
    if (!std::isfinite(cap_m) || cap_m < 0)
        throw std::runtime_error("invalid distance filter cap");
    if (cap_m == 0) return target;
    const cv::Vec3d delta = cv::Vec3d(target)-cv::Vec3d(held);
    const double squared = delta.dot(delta), radius = 2*cap_m;
    if (squared == 0) return target;
    const double alpha = squared/(squared+radius*radius);
    // Distance to accepted target is d*r^2/(d^2+r^2) <= r/2 = cap.
    // This is NOT a bound on truth error or temporal latency.
    return cv::Vec3f(cv::Vec3d(held)+alpha*delta);
}

class DistanceFilter {
    std::array<cv::Vec3f, tracking::kTrackerCount> held_{};
    std::array<bool, tracking::kTrackerCount> seen_{};
public:
    void reset() { held_ = {}; seen_ = {}; }

    void update(std::array<tracking::TrackerPose, tracking::kTrackerCount>& trackers, double cap_m) {
        for (std::size_t i = 0; i < trackers.size(); ++i) {
            auto& t = trackers[i];
            if (!t.valid) {
                seen_[i] = false;
                held_[i] = {};
                continue;
            }
            held_[i] = seen_[i] ? distance_step(held_[i], t.pos, cap_m) : t.pos;
            seen_[i] = true;
            t.pos = held_[i]; // No orientation, confidence, validity or upstream feedback.
        }
    }
};
} // namespace fitra::temporal_experiment
