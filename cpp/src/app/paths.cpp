#include "app/paths.hpp"

namespace fitra::app {

namespace {

std::filesystem::path repo_root() {
    auto exe = std::filesystem::canonical("/proc/self/exe");
    // build/main lives at <repo>/cpp/build/main.
    return exe.parent_path().parent_path().parent_path();
}

}  // namespace

std::filesystem::path guess_static_dir() {
    return repo_root() / "web-ui" / "dist";
}

std::filesystem::path guess_subject_calib_static_dir() {
    return repo_root() / "web-ui" / "dist";
}

std::filesystem::path guess_extrinsic_calib_static_dir() {
    return repo_root() / "web" / "extrinsic_calibration";
}

std::filesystem::path guess_dump_tool_path() {
    auto exe = std::filesystem::canonical("/proc/self/exe");
    return exe.parent_path() / "tools" / "dump_keypoints_3d";
}

std::vector<std::string> expected_camera_ids(std::size_t count) {
    std::vector<std::string> ids;
    ids.reserve(count);
    for (std::size_t i = 0; i < count; ++i) ids.push_back("cam" + std::to_string(i));
    return ids;
}

}  // namespace fitra::app
