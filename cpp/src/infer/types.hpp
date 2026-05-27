#pragma once
//
// Shared types for YOLOX bboxes and RTMPose persons.
//
// `Person` and `Skeleton3D` carry up to `kMaxKeypoints` (26) slots — enough
// for Halpe26. The number of *logical* keypoints depends on the active
// format and is mirrored in `kp_count` on each instance. COCO17 runs leave
// kpts[17..25] / joints[17..25] zero-initialized; consumers must loop up to
// `kp_count`, not `kpts.size()`, so the trailing zero slots are not emitted
// to JSON / drawn / processed by IK.

#include <array>
#include <cstddef>
#include <cstdint>

namespace fitra::infer {

struct Bbox {
    float x1{0.0f};
    float y1{0.0f};
    float x2{0.0f};
    float y2{0.0f};
    float score{0.0f};
};

struct Keypoint {
    float x{0.0f};
    float y{0.0f};
    float score{0.0f};
};

// Maximum keypoint count supported across all formats: COCO17 (17), Halpe26
// (26). Pick the larger so `std::array` storage is uniform.
constexpr std::size_t kMaxKeypoints = 26;

struct Person {
    Bbox                                bbox{};
    std::array<Keypoint, kMaxKeypoints> kpts{};
    // Number of valid leading entries in `kpts`. Default mirrors the
    // pre-Phase-9 behavior (COCO17) so any consumer that hasn't been
    // updated still sees a sensible value.
    std::uint8_t                        kp_count{17};
};

struct Joint3D {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float score{0.0f};
    bool  valid{false};
};

struct Skeleton3D {
    std::array<Joint3D, kMaxKeypoints> joints{};
    std::uint8_t                       kp_count{17};
};

}  // namespace fitra::infer
