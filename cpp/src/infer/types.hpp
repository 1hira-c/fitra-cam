#pragma once
//
// Shared types for YOLOX bboxes and RTMPose persons.

#include <array>
#include <cstddef>

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

constexpr std::size_t kNumKeypoints = 17;

struct Person {
    Bbox                                bbox{};
    std::array<Keypoint, kNumKeypoints> kpts{};
};

struct Joint3D {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float score{0.0f};
    bool  valid{false};
};

struct Skeleton3D {
    std::array<Joint3D, kNumKeypoints> joints{};
};

}  // namespace fitra::infer
