#pragma once
// Causal, experimental raw-3D admission. Never feeds synthesized values back
// into observation history; the separate diagnostics retain their provenance.
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <opencv2/core.hpp>
#include "infer/types.hpp"

namespace fitra::temporal_experiment {
enum class JointAdmission { Accepted, Predicted, Unavailable };
struct JointGateDiagnostic {
    JointAdmission state = JointAdmission::Unavailable;
    bool rejected = false, reacquired = false;
    double age_s = -1; // No accepted observation yet.
};
inline const char* admission_name(JointAdmission s) {
    return s == JointAdmission::Accepted ? "accepted" :
           s == JointAdmission::Predicted ? "predicted" : "unavailable";
}
class JointOutlierGate {
    struct History {
        bool seeded = false, pending = false, consecutive_accepted = false;
        cv::Vec3d accepted{}, velocity{}, pending_position{};
        double accepted_time = 0, pending_time = 0;
        int pending_count = 0;
    };
    std::array<History, infer::kMaxKeypoints> history_{};
    double now_s_ = 0;
    static cv::Vec3d position(const infer::Joint3D& j) { return {j.x,j.y,j.z}; }
    static bool finite(const infer::Joint3D& j) {
        return std::isfinite(j.x) && std::isfinite(j.y) && std::isfinite(j.z)
            && std::isfinite(j.score) && j.score >= 0;
    }
    static cv::Vec3d cap_velocity(cv::Vec3d v) {
        const double speed = cv::norm(v);
        if (speed > max_prediction_speed_mps) v *= max_prediction_speed_mps/speed;
        return v;
    }
public:
    inline static constexpr double residual_radius_m = .08;
    inline static constexpr double acceleration_mps2 = 30;
    inline static constexpr double max_prediction_speed_mps = 3;
    inline static constexpr double max_compensation_s = .05;
    inline static constexpr int reacquire_reports = 3;
    std::array<JointGateDiagnostic, infer::kMaxKeypoints> diagnostics{};
    void reset() { history_ = {}; now_s_ = 0; diagnostics = {}; }
    infer::Skeleton3D update(const infer::Skeleton3D& raw, double dt_s) {
        if (raw.kp_count != 26 || !std::isfinite(dt_s) || dt_s <= 0 || dt_s > .5)
            throw std::runtime_error("outlier gate requires Halpe26 and a positive bounded dt");
        now_s_ += dt_s;
        infer::Skeleton3D result = raw;
        for (std::size_t k=0; k<26; ++k) {
            auto& h = history_[k];
            auto& d = diagnostics[k];
            d = {};
            const auto& observation = raw.joints[k];
            const bool usable = observation.valid && finite(observation);
            d.rejected = observation.valid && !usable;
            const double age = h.seeded ? now_s_-h.accepted_time : 0;
            const double horizon = std::min(age,max_compensation_s);
            const cv::Vec3d predicted = h.accepted+h.velocity*horizon;
            const double radius = residual_radius_m+.5*acceleration_mps2*horizon*horizon;
            // A noisy last velocity must not quarantine a small real reversal.
            // Admit near either the accepted position or its bounded forecast.
            bool accept = usable && (!h.seeded || cv::norm(position(observation)-predicted) <= radius
                || cv::norm(position(observation)-h.accepted) <= radius);
            if (usable && !accept) {
                d.rejected = true;
                // Independently test continuity of rejected *raw* observations.
                // A sustained coherent shift can reseed; it is not proof of truth.
                const double pending_dt = now_s_-h.pending_time;
                const double allowance = residual_radius_m+max_prediction_speed_mps*pending_dt
                    +.5*acceleration_mps2*pending_dt*pending_dt;
                h.pending_count = h.pending && pending_dt <= max_compensation_s+1e-9
                    && cv::norm(position(observation)-h.pending_position) <= allowance
                    ? h.pending_count+1 : 1;
                h.pending = true;
                h.pending_position = position(observation);
                h.pending_time = now_s_;
                if (h.pending_count >= reacquire_reports) {
                    accept = true;
                    d.reacquired = true;
                }
            } else if (!usable) {
                h.pending = false;
                h.pending_count = 0;
            }
            if (accept) {
                const auto q = position(observation);
                // A return across a rejected/missing interval does not establish
                // trustworthy velocity. Seed it from consecutive raw reports.
                h.velocity = h.seeded && h.consecutive_accepted && !d.reacquired
                    ? cap_velocity((q-h.accepted)/age) : cv::Vec3d{};
                h.accepted = q;
                h.accepted_time = now_s_;
                h.seeded = true;
                h.consecutive_accepted = true;
                h.pending = false;
                h.pending_count = 0;
                d.state = JointAdmission::Accepted;
                d.age_s = 0;
                // A reseed is now accepted, though it first failed the old path.
                result.joints[k] = observation;
            } else {
                h.consecutive_accepted = false;
                d.age_s = h.seeded ? age : -1;
                result.joints[k] = {};
                if (h.seeded && age <= max_compensation_s+1e-9) {
                    result.joints[k] = {float(predicted[0]),float(predicted[1]),float(predicted[2]),0,true};
                    d.state = JointAdmission::Predicted;
                }
            }
        }
        return result;
    }
};
} // namespace fitra::temporal_experiment
