#pragma once
//
// COCO-17 topology used by RTMPose and the Phase 7 3D lifting tools.

#include <array>
#include <utility>

namespace fitra::lift {

constexpr std::array<std::pair<int, int>, 16> kCocoEdges{{
    {0, 1}, {0, 2}, {1, 3}, {2, 4},
    {5, 7}, {7, 9}, {6, 8}, {8, 10},
    {5, 6}, {5, 11}, {6, 12}, {11, 12},
    {11, 13}, {13, 15}, {12, 14}, {14, 16},
}};

// A tree view for length projection. COCO has no pelvis/spine joint, so left
// hip is the root and torso links keep the graph connected.
constexpr std::array<int, 17> kCocoParent{{
    5, 0, 0, 1, 2,
    11, 12, 5, 6, 7, 8,
    -1, 11, 11, 12, 13, 14,
}};

constexpr std::array<int, 4> kHingeJoints{{7, 8, 13, 14}};  // elbows, knees

constexpr std::array<int, 11> kMajorBoneChildren{{
    5, 6, 7, 8, 9, 10, 12, 13, 14, 15, 16,
}};

inline int hinge_child(int joint) {
    switch (joint) {
        case 7: return 9;
        case 8: return 10;
        case 13: return 15;
        case 14: return 16;
        default: return -1;
    }
}

}  // namespace fitra::lift
