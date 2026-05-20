#pragma once
//
// Per-subject IK profile produced by the Phase 8 pose-sequence workflow.

#include <array>
#include <string>

#include "infer/types.hpp"

namespace fitra::lift {

struct SubjectProfile {
    bool loaded = false;
    std::string schema = "fitra_subject_profile_v1";
    std::string subject_id;
    std::string created_at;
    std::string source_session;
    std::string quality_status;
    double subject_height_m = 0.0;
    std::array<double, infer::kNumKeypoints> bone_lengths_m{};
    double shoulder_width_m = 0.0;
    double hip_width_m = 0.0;
};

SubjectProfile load_subject_profile(const std::string& path);
void write_subject_profile(const std::string& path, const SubjectProfile& profile);
void validate_subject_profile(const SubjectProfile& profile);
std::string default_subject_profile_path(const std::string& subjects_dir,
                                         const std::string& subject_id);

}  // namespace fitra::lift
