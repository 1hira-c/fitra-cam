#pragma once

#include <array>
#include <vector>

#include "infer/types.hpp"

namespace fitra::lift {

class IkSolver {
public:
    struct Options {
        int bone_calib_frames = 150;
        int iterations = 5;
        // Keep hinge joints slightly away from singular 0/180 degree extremes.
        double min_hinge_deg = 5.0;
        double max_hinge_deg = 175.0;
        double subject_height_m = 0.0;
    };

    IkSolver();
    explicit IkSolver(Options opts);

    infer::Skeleton3D update(const infer::Skeleton3D& input);
    bool locked() const { return locked_; }
    double bone_drift_pct(const infer::Skeleton3D& skel) const;
    double subject_height_m() const { return opts_.subject_height_m; }

private:
    void apply_subject_height_model();
    void observe_lengths(const infer::Skeleton3D& skel);
    void lock_lengths();
    void enforce_lengths(infer::Skeleton3D& skel) const;
    void enforce_pair_lengths(infer::Skeleton3D& skel) const;
    void enforce_hinges(infer::Skeleton3D& skel) const;

    Options opts_;
    bool locked_ = false;
    int observed_frames_ = 0;
    std::array<double, infer::kNumKeypoints> locked_parent_len_{};
    double locked_shoulder_width_ = 0.0;
    std::array<std::vector<double>, infer::kNumKeypoints> samples_;
};

}  // namespace fitra::lift
