#pragma once
//
// COCO17 vs Halpe26 topology selector.
//
// One canonical `KeypointFormat` enum drives every part of the pipeline that
// needs to know the runtime keypoint count (`kp_count`), the parent tree, the
// drawing edges, and the hinge/major-bone subsets. The process-wide active
// format is fixed once at startup by `set_active_keypoint_format()` so that
// fast paths (snapshot publisher, IK update, Kalman, drawers) can read it
// without a parameter chain.
//
// Halpe26 = COCO17 ∪ {17 head_top, 18 neck, 19 hip_center,
//                     20 l_big_toe, 21 r_big_toe, 22 l_small_toe,
//                     23 r_small_toe, 24 l_heel, 25 r_heel}. Indices 0–16
// match COCO17 exactly, so hard-coded references to landmarks 5/6/7/8/11/12/
// 13/14/15/16 (shoulders, elbows, hips, knees, ankles) still mean the same
// joints under either format.

#include <array>
#include <cstddef>
#include <span>
#include <string>
#include <utility>

namespace fitra::lift {

enum class KeypointFormat {
    Coco17 = 0,
    Halpe26 = 1,
};

struct SkeletonDef {
    KeypointFormat                                format;
    std::size_t                                   kp_count;
    std::span<const std::pair<int, int>>          edges;
    std::span<const int>                          parents;              // -1 = root
    std::span<const int>                          hinge_joints;
    std::span<const int>                          major_bone_children;
    int (*hinge_child)(int);
};

// Lookup the static SkeletonDef for a given format. Always returns the same
// reference; safe to capture as `const SkeletonDef&`.
const SkeletonDef& skeleton_def(KeypointFormat fmt);

// Set/read the process-wide active format. Call set_active_keypoint_format()
// exactly once at startup (before any pipeline thread starts), then treat the
// value as immutable. active_keypoint_format() returns Coco17 if never set.
void set_active_keypoint_format(KeypointFormat fmt);
KeypointFormat active_keypoint_format();
inline const SkeletonDef& active_skeleton_def() {
    return skeleton_def(active_keypoint_format());
}
inline std::size_t active_kp_count() {
    return active_skeleton_def().kp_count;
}

// String <-> enum helpers for CLI parsing. parse_keypoint_format() returns
// false (and leaves `out` untouched) for unknown names.
const char* keypoint_format_name(KeypointFormat fmt);
bool parse_keypoint_format(const std::string& name, KeypointFormat& out);

// Subject profile schema string for the given format (`fitra_subject_profile_v1`
// for COCO17, `fitra_subject_profile_v2` for Halpe26). Schema must match the
// active format on load or load_subject_profile() raises.
const char* subject_profile_schema(KeypointFormat fmt);

}  // namespace fitra::lift
