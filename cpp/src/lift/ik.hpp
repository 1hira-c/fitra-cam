#pragma once

#include <array>
#include <mutex>
#include <vector>

#include "infer/types.hpp"
#include "lift/subject_profile.hpp"

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
        bool has_subject_profile = false;
        SubjectProfile subject_profile;
    };

    IkSolver();
    explicit IkSolver(Options opts);

    infer::Skeleton3D update(const infer::Skeleton3D& input);

    // Atomically swap to a new subject profile. Safe to call while update()
    // is running on another thread: the next update() will use the new
    // bone lengths. Clears any in-flight observation samples and sets
    // profile_loaded_ = locked_ = true.
    void reload_from_profile(const SubjectProfile& profile);

    // Re-prime IK from a subject height (AIST/HQL anthropometry ratios) at
    // runtime. Used by the Phase 8 calibration wizard during preflight so the
    // 3D angle recognizer sees a sensible bone-length lock from frame 1.
    // Clears the observation sample buffer, clears profile_loaded_, sets
    // locked_ = true. No-op if m <= 0.
    void apply_subject_height(double m);

    bool locked() const;
    bool profile_loaded() const;
    double bone_drift_pct(const infer::Skeleton3D& skel) const;
    double subject_height_m() const;
    std::string subject_id() const;
    std::string profile_quality_status() const;

private:
    void apply_subject_height_model_locked();
    void apply_subject_profile_locked(const SubjectProfile& profile);
    void observe_lengths_locked(const infer::Skeleton3D& skel);
    void lock_lengths_locked();
    void enforce_lengths(infer::Skeleton3D& skel) const;
    void enforce_pair_lengths(infer::Skeleton3D& skel) const;
    void enforce_hinges(infer::Skeleton3D& skel) const;
    double bone_drift_pct_locked(const infer::Skeleton3D& skel) const;

    mutable std::mutex mu_;
    Options opts_;
    bool locked_ = false;
    bool profile_loaded_ = false;
    std::string subject_id_;
    std::string profile_quality_status_;
    int observed_frames_ = 0;
    // Sized for the largest supported topology (Halpe26 = 26). COCO17 runs
    // leave the trailing slots at zero; the iteration upper bound comes from
    // the active SkeletonDef so those slots are never touched.
    std::array<double, infer::kMaxKeypoints> locked_parent_len_{};
    double locked_shoulder_width_ = 0.0;
    std::array<std::vector<double>, infer::kMaxKeypoints> samples_;
};

}  // namespace fitra::lift
