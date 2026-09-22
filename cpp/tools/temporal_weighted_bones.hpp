#pragma once
// Experimental, stateless spatial solver for OFFLINE replay only. No live API.
#include <algorithm>
#include <array>
#include <cmath>
#include <ostream>
#include <stdexcept>
#include <vector>
#include <opencv2/core.hpp>
#include "lift/ik.hpp"
#include "lift/head_direction.hpp"
#include "lift/skeleton_def.hpp"

namespace fitra::temporal_experiment {
struct Bone { int parent, child; double length; };

// Mirror the EXISTING Halpe26 prior, including its asymmetric index-12 lock.
// This experiment changes projection, not anatomy/profile calibration. Targets
// are serialized and parity-tested against IkSolver on isolated bone inputs.
inline std::vector<Bone> bone_targets(const lift::IkSolver::Options& options) {
    std::array<double, 26> lengths{};
    double shoulders;
    if (options.has_subject_profile) {
        lift::validate_subject_profile(options.subject_profile);
        std::copy_n(options.subject_profile.bone_lengths_m.begin(), 26, lengths.begin());
        if (lengths[12] <= 1e-6 && options.subject_profile.hip_width_m > 1e-6)
            lengths[12] = options.subject_profile.hip_width_m;
        shoulders = options.subject_profile.shoulder_width_m;
    } else {
        const double h = options.subject_height_m;
        lengths[5] = lengths[6] = .129*h;
        lengths[7] = lengths[8] = .189*h;
        lengths[9] = lengths[10] = .146*h;
        lengths[12] = .0995*h;
        lengths[13] = lengths[14] = .226*h;
        lengths[15] = lengths[16] = .224*h;
        shoulders = .258*h;
    }
    std::vector<Bone> result;
    for (int child = 5; child < 26; ++child) {
        const int parent = lift::kHalpeParent[child];
        if (parent >= 5 && lengths[child] > 1e-6) result.push_back({parent, child, lengths[child]});
    }
    if (shoulders > 1e-6) result.push_back({5, 6, shoulders});
    for (const auto& b : result)
        if (!std::isfinite(b.length)) throw std::runtime_error("nonfinite experimental bone prior");
    return result;
}

inline cv::Vec3d point(const infer::Joint3D& joint) { return {joint.x, joint.y, joint.z}; }

inline infer::Skeleton3D soft_bones(const infer::Skeleton3D& input,
                                  const std::vector<Bone>& bones, double hip_weight) {
    // Minimize sum_i w_i |x_i-y_i|^2 + 10 sum_edges (|x_c-x_p|-L)^2.
    // Alternating unit edge directions and an SPD linear solve decrease that
    // objective. Fixed 12 iterations; no history, future samples or dt.
    constexpr int count = 26, iterations = 12;
    constexpr double stiffness = 10.;
    cv::Mat matrix = cv::Mat::zeros(count, count, CV_64F);
    cv::Mat tether = cv::Mat::zeros(count, 3, CV_64F);
    cv::Mat current = cv::Mat::zeros(count, 3, CV_64F);
    for (int k = 0; k < count; ++k) {
        const auto& j = input.joints[k];
        double w = 1.;
        if (j.valid && k >= 5) {
            w = std::pow(std::clamp(static_cast<double>(j.score), .05, 1.), 2);
            if (k == 11 || k == 12 || k == 19) w *= hip_weight;
        }
        matrix.at<double>(k, k) = w;
        if (j.valid) for (int axis = 0; axis < 3; ++axis) {
            current.at<double>(k, axis) = point(j)[axis];
            tether.at<double>(k, axis) = w*point(j)[axis];
        }
    }
    std::vector<Bone> active;
    for (const auto& b : bones) {
        if (!input.joints[b.parent].valid || !input.joints[b.child].valid) continue;
        // Coincident observations contain no bone direction. Do not fabricate it.
        if (cv::norm(point(input.joints[b.child])-point(input.joints[b.parent])) < 1e-8) continue;
        active.push_back(b);
        matrix.at<double>(b.parent, b.parent) += stiffness;
        matrix.at<double>(b.child, b.child) += stiffness;
        matrix.at<double>(b.parent, b.child) -= stiffness;
        matrix.at<double>(b.child, b.parent) -= stiffness;
    }
    for (int iteration = 0; iteration < iterations; ++iteration) {
        cv::Mat rhs = tether.clone();
        for (const auto& b : active) {
            cv::Vec3d direction;
            for (int axis = 0; axis < 3; ++axis)
                direction[axis] = current.at<double>(b.child, axis)-current.at<double>(b.parent, axis);
            double length = cv::norm(direction);
            if (length < 1e-8) {
                direction = point(input.joints[b.child])-point(input.joints[b.parent]);
                length = cv::norm(direction);
            }
            direction *= stiffness*b.length/length;
            for (int axis = 0; axis < 3; ++axis) {
                rhs.at<double>(b.child, axis) += direction[axis];
                rhs.at<double>(b.parent, axis) -= direction[axis];
            }
        }
        if (!cv::solve(matrix, rhs, current, cv::DECOMP_CHOLESKY) || !cv::checkRange(current))
            throw std::runtime_error("experimental bone solve failed");
    }
    auto output = input;
    for (int k = 5; k < count; ++k) if (output.joints[k].valid) {
        output.joints[k].x = static_cast<float>(current.at<double>(k, 0));
        output.joints[k].y = static_cast<float>(current.at<double>(k, 1));
        output.joints[k].z = static_cast<float>(current.at<double>(k, 2));
    }
    lift::synthesize_halpe_head_direction(output, input);
    return output;
}

inline void write_constraints(std::ostream& out, const infer::Skeleton3D& skeleton,
                              const std::vector<Bone>& bones) {
    double squares = 0, maximum = 0;
    int supported = 0, hinges = 0, violations = 0;
    for (const auto& b : bones) {
        if (!skeleton.joints[b.parent].valid || !skeleton.joints[b.child].valid) continue;
        const double error = std::abs(cv::norm(point(skeleton.joints[b.child])-point(skeleton.joints[b.parent]))-b.length);
        squares += error*error; maximum = std::max(maximum, error); ++supported;
    }
    for (int k : lift::kHingeJoints) {
        const int parent = lift::kHalpeParent[k], child = lift::hinge_child(k);
        if (!skeleton.joints[parent].valid || !skeleton.joints[k].valid || !skeleton.joints[child].valid) continue;
        const auto a = point(skeleton.joints[parent])-point(skeleton.joints[k]);
        const auto b = point(skeleton.joints[child])-point(skeleton.joints[k]);
        const double denominator = cv::norm(a)*cv::norm(b);
        if (denominator < 1e-12) continue;
        const double angle = std::acos(std::clamp(a.dot(b)/denominator, -1., 1.))*180/CV_PI;
        ++hinges; if (angle < 5.-1e-4 || angle > 175.+1e-4) ++violations;
    }
    out << "{\"bone_pairs\":" << supported << ",\"bone_rms_mm\":";
    if (supported) out << std::sqrt(squares/supported)*1000; else out << "null";
    out << ",\"bone_max_mm\":";
    if (supported) out << maximum*1000; else out << "null";
    out << ",\"hinge_pairs\":" << hinges << ",\"hinge_outside_5_175\":" << violations << '}';
}
} // namespace fitra::temporal_experiment
