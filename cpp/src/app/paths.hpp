#pragma once
//
// Repo-relative path guessing shared by the mode runners. The binary lives at
// <repo>/cpp/build/main; web assets and sibling tools are resolved from there
// unless overridden by CLI flags.

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace fitra::app {

std::filesystem::path guess_static_dir();                 // <repo>/web/dual_rtmpose
std::filesystem::path guess_subject_calib_static_dir();   // <repo>/web/subject_calibration
std::filesystem::path guess_extrinsic_calib_static_dir(); // <repo>/web/extrinsic_calibration
std::filesystem::path guess_dump_tool_path();             // build/tools/dump_keypoints_3d

// "cam0".."camN-1" — the camera id contract between FrameSource order and
// calibration YAML entries.
std::vector<std::string> expected_camera_ids(std::size_t count);

}  // namespace fitra::app
