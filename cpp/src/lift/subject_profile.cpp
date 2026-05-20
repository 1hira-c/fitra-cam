#include "lift/subject_profile.hpp"

#include <filesystem>
#include <stdexcept>

#include <opencv2/core.hpp>

#include "lift/keypoint_format.hpp"
#include "lift/skeleton_def.hpp"

namespace fitra::lift {

namespace {

std::string node_string(const cv::FileNode& node) {
    return node.empty() ? std::string{} : static_cast<std::string>(node);
}

double node_real(const cv::FileNode& node, double fallback = 0.0) {
    return node.empty() ? fallback : static_cast<double>(node);
}

std::array<double, infer::kMaxKeypoints> read_lengths(const cv::FileNode& node) {
    std::array<double, infer::kMaxKeypoints> out{};
    if (node.empty()) return out;
    if (node.isSeq()) {
        std::size_t i = 0;
        for (auto it = node.begin(); it != node.end() && i < out.size(); ++it, ++i) {
            out[i] = static_cast<double>(*it);
        }
        return out;
    }
    cv::Mat mat;
    node >> mat;
    if (mat.empty()) return out;
    if (mat.depth() != CV_64F) mat.convertTo(mat, CV_64F);
    mat = mat.reshape(1, 1);
    for (int c = 0; c < mat.cols && c < static_cast<int>(out.size()); ++c) {
        out[static_cast<std::size_t>(c)] = mat.at<double>(0, c);
    }
    return out;
}

cv::Mat lengths_mat(const std::array<double, infer::kMaxKeypoints>& vals,
                    std::size_t logical_count) {
    // Only emit the leading `logical_count` entries so a Halpe26 profile
    // (26) and a COCO17 profile (17) round-trip with the right schema-implied
    // length. Readers that overflow into the trailing zero slots would still
    // work, but emitting the full 26 columns from a v1 profile would silently
    // change its on-disk shape.
    cv::Mat mat(1, static_cast<int>(logical_count), CV_64F);
    for (int c = 0; c < mat.cols; ++c) {
        mat.at<double>(0, c) = vals[static_cast<std::size_t>(c)];
    }
    return mat;
}

int usable_major_bones(const SubjectProfile& profile) {
    const auto& def = active_skeleton_def();
    int n = 0;
    for (int child : def.major_bone_children) {
        if (profile.bone_lengths_m[static_cast<std::size_t>(child)] > 1.0e-6) {
            ++n;
        }
    }
    if (profile.shoulder_width_m > 1.0e-6) ++n;
    return n;
}

}  // namespace

SubjectProfile make_default_subject_profile() {
    SubjectProfile p;
    p.schema = subject_profile_schema(active_keypoint_format());
    return p;
}

SubjectProfile load_subject_profile(const std::string& path) {
    cv::FileStorage fs{path, cv::FileStorage::READ};
    if (!fs.isOpened()) {
        throw std::runtime_error("failed to open subject profile: " + path);
    }

    SubjectProfile profile;
    profile.loaded = true;
    profile.schema = node_string(fs["schema"]);
    profile.subject_id = node_string(fs["subject_id"]);
    profile.created_at = node_string(fs["created_at"]);
    profile.source_session = node_string(fs["source_session"]);
    profile.quality_status = node_string(fs["quality_status"]);
    profile.subject_height_m = node_real(fs["subject_height_m"]);
    profile.bone_lengths_m = read_lengths(fs["bone_lengths_m"]);
    profile.shoulder_width_m = node_real(fs["shoulder_width_m"]);
    profile.hip_width_m = node_real(fs["hip_width_m"]);
    if (profile.hip_width_m > 1.0e-6 &&
        profile.bone_lengths_m[12] <= 1.0e-6) {
        profile.bone_lengths_m[12] = profile.hip_width_m;
    }
    validate_subject_profile(profile);
    if (profile.quality_status == "fail") {
        throw std::runtime_error("refusing to load failed subject profile");
    }
    return profile;
}

void write_subject_profile(const std::string& path, const SubjectProfile& profile) {
    const char* expected_schema = subject_profile_schema(active_keypoint_format());
    if (profile.schema != expected_schema || profile.subject_id.empty()) {
        throw std::runtime_error("invalid subject profile metadata for write");
    }
    std::filesystem::path out{path};
    if (out.has_parent_path()) {
        std::filesystem::create_directories(out.parent_path());
    }
    cv::FileStorage fs{path, cv::FileStorage::WRITE};
    if (!fs.isOpened()) {
        throw std::runtime_error("failed to write subject profile: " + path);
    }
    fs << "schema" << profile.schema;
    fs << "subject_id" << profile.subject_id;
    fs << "created_at" << profile.created_at;
    fs << "source_session" << profile.source_session;
    fs << "quality_status" << profile.quality_status;
    fs << "subject_height_m" << profile.subject_height_m;
    fs << "bone_lengths_m" << lengths_mat(profile.bone_lengths_m, active_kp_count());
    fs << "shoulder_width_m" << profile.shoulder_width_m;
    fs << "hip_width_m" << profile.hip_width_m;
}

void validate_subject_profile(const SubjectProfile& profile) {
    const char* expected_schema = subject_profile_schema(active_keypoint_format());
    if (profile.schema != expected_schema) {
        // A profile recorded under a different topology cannot be migrated
        // automatically (bone-length indices and major-bone subsets differ).
        // The Phase 8 wizard rewrites the profile when the operator runs a
        // fresh session.
        throw std::runtime_error(
            "subject profile schema " + profile.schema
            + " does not match active --keypoint-format ("
            + expected_schema + "); re-run the calibration wizard");
    }
    if (profile.subject_id.empty()) {
        throw std::runtime_error("subject profile has empty subject_id");
    }
    if (profile.subject_height_m < 0.0 || profile.subject_height_m > 2.5) {
        throw std::runtime_error("subject profile has implausible subject_height_m");
    }
    if (usable_major_bones(profile) < 6) {
        throw std::runtime_error("subject profile has too few usable major bone lengths");
    }
}

std::string default_subject_profile_path(const std::string& subjects_dir,
                                         const std::string& subject_id) {
    if (subject_id.empty()) {
        throw std::runtime_error("subject_id is empty");
    }
    std::filesystem::path path{subjects_dir.empty() ? "calibrations/subjects" : subjects_dir};
    path /= subject_id;
    path /= "latest_profile.yaml";
    return path.string();
}

}  // namespace fitra::lift
