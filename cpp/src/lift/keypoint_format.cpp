#include "lift/keypoint_format.hpp"

#include <atomic>

#include "lift/skeleton_def.hpp"

namespace fitra::lift {

namespace {

// Active format is read on every pipeline iteration but written exactly once
// at startup. Using std::atomic with relaxed ordering avoids torn reads
// without locking the hot path.
std::atomic<KeypointFormat> g_active{KeypointFormat::Coco17};

const SkeletonDef& coco17_def() {
    static const SkeletonDef d{
        KeypointFormat::Coco17,
        /*kp_count*/ 17,
        std::span<const std::pair<int, int>>{kCocoEdges.data(), kCocoEdges.size()},
        std::span<const int>{kCocoParent.data(), kCocoParent.size()},
        std::span<const int>{kHingeJoints.data(), kHingeJoints.size()},
        std::span<const int>{kMajorBoneChildren.data(), kMajorBoneChildren.size()},
        &hinge_child,
    };
    return d;
}

const SkeletonDef& halpe26_def() {
    static const SkeletonDef d{
        KeypointFormat::Halpe26,
        /*kp_count*/ 26,
        std::span<const std::pair<int, int>>{kHalpeEdges.data(), kHalpeEdges.size()},
        std::span<const int>{kHalpeParent.data(), kHalpeParent.size()},
        std::span<const int>{kHingeJoints.data(), kHingeJoints.size()},
        std::span<const int>{kHalpeMajorBoneChildren.data(), kHalpeMajorBoneChildren.size()},
        &hinge_child,  // elbows/knees use COCO indices, same under Halpe26
    };
    return d;
}

}  // namespace

const SkeletonDef& skeleton_def(KeypointFormat fmt) {
    return fmt == KeypointFormat::Halpe26 ? halpe26_def() : coco17_def();
}

void set_active_keypoint_format(KeypointFormat fmt) {
    g_active.store(fmt, std::memory_order_relaxed);
}

KeypointFormat active_keypoint_format() {
    return g_active.load(std::memory_order_relaxed);
}

const char* keypoint_format_name(KeypointFormat fmt) {
    switch (fmt) {
        case KeypointFormat::Coco17:  return "coco17";
        case KeypointFormat::Halpe26: return "halpe26";
    }
    return "coco17";
}

bool parse_keypoint_format(const std::string& name, KeypointFormat& out) {
    if (name == "coco17")  { out = KeypointFormat::Coco17;  return true; }
    if (name == "halpe26") { out = KeypointFormat::Halpe26; return true; }
    return false;
}

const char* subject_profile_schema(KeypointFormat fmt) {
    return fmt == KeypointFormat::Halpe26
        ? "fitra_subject_profile_v2"
        : "fitra_subject_profile_v1";
}

}  // namespace fitra::lift
