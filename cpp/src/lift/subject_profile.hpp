#pragma once
//
// Per-subject IK profile produced by the pose-sequence calibration workflow.
//
// `schema` distinguishes COCO17 (`fitra_subject_profile_v1`) from Halpe26
// (`fitra_subject_profile_v2`). Loading a v1 profile while running in Halpe26
// mode (or vice versa) raises -- profiles are not migrated.

#include <array>
#include <string>

#include "infer/types.hpp"
#include "lift/keypoint_format.hpp"

namespace fitra::lift {

struct SubjectProfile {
    bool loaded = false;
    // Default to the COCO17 schema for backwards compatibility with existing
    // calibrations on disk; the wizard writes v2 when running under
    // --keypoint-format=halpe26 (see make_default_subject_profile()).
    std::string schema = "fitra_subject_profile_v1";
    std::string subject_id;
    std::string created_at;
    std::string source_session;
    std::string quality_status;
    double subject_height_m = 0.0;
    // Sized for the largest supported topology. COCO17 profiles leave the
    // trailing slots zeroed; readers must iterate over the active topology's
    // parent table, not bone_lengths_m.size().
    std::array<double, infer::kMaxKeypoints> bone_lengths_m{};
    double shoulder_width_m = 0.0;
    double hip_width_m = 0.0;
};

// Helper to initialize a new profile with the schema string that matches the
// active keypoint format (used by the calibration analysis pipeline).
SubjectProfile make_default_subject_profile();

SubjectProfile load_subject_profile(const std::string& path);
void write_subject_profile(const std::string& path, const SubjectProfile& profile);
void validate_subject_profile(const SubjectProfile& profile);
std::string default_subject_profile_path(const std::string& subjects_dir,
                                         const std::string& subject_id);

}  // namespace fitra::lift
