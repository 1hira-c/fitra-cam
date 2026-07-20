#pragma once
//
// Static topology tables for COCO17 and Halpe26.
//
// Halpe26 reuses the first 17 indices from COCO17 (so hard-coded landmark
// indices like 5 = left_shoulder still mean the same joint), and appends:
//   17 head_top, 18 neck, 19 hip_center,
//   20 l_big_toe, 21 r_big_toe, 22 l_small_toe, 23 r_small_toe,
//   24 l_heel,    25 r_heel.

#include <array>
#include <cstddef>
#include <utility>

namespace fitra::lift {

// Named Halpe26 landmarks shared by the stabilizer, offline analyzer and VR
// extractor. Keeping these in the topology header prevents each consumer from
// silently inventing its own numeric foot layout.
inline constexpr std::size_t kHalpeNose          = 0;
inline constexpr std::size_t kHalpeLeftShoulder  = 5;
inline constexpr std::size_t kHalpeRightShoulder = 6;
inline constexpr std::size_t kHalpeLeftElbow     = 7;
inline constexpr std::size_t kHalpeRightElbow    = 8;
inline constexpr std::size_t kHalpeLeftWrist     = 9;
inline constexpr std::size_t kHalpeRightWrist    = 10;
inline constexpr std::size_t kHalpeLeftHip       = 11;
inline constexpr std::size_t kHalpeRightHip      = 12;
inline constexpr std::size_t kHalpeLeftKnee      = 13;
inline constexpr std::size_t kHalpeRightKnee     = 14;
inline constexpr std::size_t kHalpeLeftAnkle     = 15;
inline constexpr std::size_t kHalpeRightAnkle    = 16;
inline constexpr std::size_t kHalpeHeadTop       = 17;
inline constexpr std::size_t kHalpeNeck          = 18;
inline constexpr std::size_t kHalpeHipCenter     = 19;
inline constexpr std::size_t kHalpeLeftBigToe    = 20;
inline constexpr std::size_t kHalpeRightBigToe   = 21;
inline constexpr std::size_t kHalpeLeftSmallToe  = 22;
inline constexpr std::size_t kHalpeRightSmallToe = 23;
inline constexpr std::size_t kHalpeLeftHeel      = 24;
inline constexpr std::size_t kHalpeRightHeel     = 25;

struct HalpeFootJoints {
    std::size_t knee;
    std::size_t ankle;
    std::array<std::size_t, 3> sole;
};

// Index 0 = left, 1 = right. `sole[0]` is the big toe used by the VR foot
// direction; all three sole points participate in floor evidence.
inline constexpr std::array<HalpeFootJoints, 2> kHalpeFeet{{
    {kHalpeLeftKnee, kHalpeLeftAnkle,
     {kHalpeLeftBigToe, kHalpeLeftSmallToe, kHalpeLeftHeel}},
    {kHalpeRightKnee, kHalpeRightAnkle,
     {kHalpeRightBigToe, kHalpeRightSmallToe, kHalpeRightHeel}},
}};

inline constexpr std::array<std::size_t, 6> kHalpeSoleJoints{{
    kHalpeLeftBigToe, kHalpeRightBigToe,
    kHalpeLeftSmallToe, kHalpeRightSmallToe,
    kHalpeLeftHeel, kHalpeRightHeel,
}};

// ---------------- COCO17 ----------------

constexpr std::array<std::pair<int, int>, 16> kCocoEdges{{
    {0, 1}, {0, 2}, {1, 3}, {2, 4},
    {5, 7}, {7, 9}, {6, 8}, {8, 10},
    {5, 6}, {5, 11}, {6, 12}, {11, 12},
    {11, 13}, {13, 15}, {12, 14}, {14, 16},
}};

// COCO has no pelvis/spine joint, so left hip is the root and torso links
// keep the graph connected.
constexpr std::array<int, 17> kCocoParent{{
    5, 0, 0, 1, 2,
    11, 12, 5, 6, 7, 8,
    -1, 11, 11, 12, 13, 14,
}};

// Elbows (7, 8) and knees (13, 14). Indices match under Halpe26.
constexpr std::array<int, 4> kHingeJoints{{7, 8, 13, 14}};

constexpr std::array<int, 11> kMajorBoneChildren{{
    5, 6, 7, 8, 9, 10, 12, 13, 14, 15, 16,
}};

// Wrist/ankle indices are identical in COCO17 and Halpe26, so the mapping is
// shared.
inline int hinge_child(int joint) {
    switch (joint) {
        case 7:  return 9;
        case 8:  return 10;
        case 13: return 15;
        case 14: return 16;
        default: return -1;
    }
}

// ---------------- Halpe26 ----------------
//
// Edges grouped logically: head, torso/neck, arms, legs, feet.
constexpr std::array<std::pair<int, int>, 23> kHalpeEdges{{
    // Head
    {17, 18}, {0, 17}, {0, 1}, {0, 2}, {1, 3}, {2, 4},
    // Torso (neck/hip-center join shoulders & hips)
    {18, 5}, {18, 6}, {18, 19}, {11, 19}, {12, 19}, {5, 6}, {11, 12},
    // Arms
    {5, 7}, {7, 9}, {6, 8}, {8, 10},
    // Legs
    {11, 13}, {13, 15}, {12, 14}, {14, 16},
    // Feet share a child (ankle parent) so we limit the visual to a single
    // representative line per side; the full toe/heel triplet is rendered by
    // the per-keypoint dots.
    {15, 24}, {16, 25},
}};

// Parent tree rooted at hip-center (19).
constexpr std::array<int, 26> kHalpeParent{{
    18, 0, 0, 1, 2,             //  0–4 nose,eyes,ears
    18, 18, 5, 6, 7, 8,         //  5–10 shoulders, elbows, wrists
    19, 19, 11, 12, 13, 14,     // 11–16 hips, knees, ankles
    18, 19, -1,                 // 17 head_top, 18 neck, 19 hip-center (root)
    15, 16, 15, 16, 15, 16,     // 20–25 toes/heels (children of ankles)
}};

// Major bones used by SubjectProfile quality scoring. Halpe26 adds neck (18),
// hip-center (19) and the head_top<-neck link as anchors that exist in every
// usable pose.
constexpr std::array<int, 14> kHalpeMajorBoneChildren{{
    5, 6, 7, 8, 9, 10, 12, 13, 14, 15, 16, 17, 18, 19,
}};

}  // namespace fitra::lift
