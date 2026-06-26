#pragma once
//
// Solve VmtAlignment from HMD + chest tracker observations.
//
// Both inputs MUST be in the same VMT Driver frame (Y-up RH, X-right,
// Z-back, metres):
//   * HmdPose is published in SteamVR's Standing universe, which by
//     construction equals the VMT Driver frame consumed by VmtPublisher.
//   * Chest tracker pose lives in fitra-cam's world frame (Z-up RH); the
//     caller is responsible for piping it through `world_pos_to_vmt` /
//     `world_quat_to_vmt` (see vmt_protocol.hpp) before invoking solve_*.
//
// The resulting VmtAlignment is what you would plug into
// `apply_vmt_alignment(chest_vmt, ..., alignment)` so the chest aligns with
// the HMD. The published Y is left at 0 — the head/chest height offset is
// large and individual, see docs/phase15-vmt-hmd-auto-align.md.
//

#include <string>
#include <vector>

#include "vmt/hmd_pose_receiver.hpp"   // HmdPose
#include "vmt/vmt_protocol.hpp"        // VmtAlignment, VmtPos, VmtQuat

namespace fitra::vmt {

enum class AutoAlignmentStatus {
    Ok,
    NoHmd,             // HmdPoseBus has never seen a packet
    StaleHmd,          // HmdPoseBus is marked stale
    NotEnoughSamples,  // motion mode below minimum N
    Degenerate,        // motion mode: HMD or chest path is collinear
};

const char* status_name(AutoAlignmentStatus s);

struct AutoAlignmentResult {
    VmtAlignment        alignment{};   // valid iff status == Ok
    float               residual_m = 0.0f;  // mean 2D residual (motion); 0 for tpose
    int                 n_samples  = 0;
    AutoAlignmentStatus status     = AutoAlignmentStatus::NoHmd;
    std::string         err;          // human-readable detail
};

// Project the HMD origin back onto the body's head/neck vertical axis in the
// horizontal (xz) plane. The HMD device sits ~forward_offset_m ahead of that
// axis along the gaze direction (it rides on the face), so matching the raw
// HMD xz against a head/chest body landmark carries a head-forward lever arm
// that rotates with the head. This removes it using the HMD's OWN orientation
// (steamvr frame), so the correction is independent of the alignment being
// solved. Pitch is handled naturally: the horizontal component of the gaze
// shrinks as the user looks up/down, so a near-vertical gaze applies ~0
// horizontal correction. Pass forward_offset_m = 0 for a no-op (raw xz).
struct HmdAxisXZ { float x, z; };
HmdAxisXZ hmd_head_axis_xz(const HmdPose& hmd, float forward_offset_m);

//
// T-pose single-shot. `hmd` and `chest_vmt_*` are simultaneous samples; the
// alignment is the (yaw, x, z) such that R_y(yaw) · chest_vmt_xz + (x, z)
// matches the HMD head-axis xz. Y is left at 0 (caller may overlay a manual
// height offset).
//
// `chest_quat_xyzw` is the chest orientation in VMT frame (xyzw on the
// wire); the function extracts yaw via the Y-up formula. `forward_offset_m`
// feeds hmd_head_axis_xz (default 0 = raw HMD xz, preserves prior behavior).
//
AutoAlignmentResult solve_tpose(const HmdPose&  hmd,
                                const VmtPos&   chest_vmt_pos,
                                const VmtQuat&  chest_vmt_quat_xyzw,
                                float           forward_offset_m = 0.0f);

//
// Motion mode. xz samples only (Y dropped). `min_samples` defaults to 4;
// caller may pass higher (e.g. 30 Hz × 3 s = 90 samples). Both vectors must
// be the same length.
//
struct MotionSample {
    float hmd_x, hmd_z;
    float chest_x, chest_z;
};

AutoAlignmentResult solve_motion(const std::vector<MotionSample>& samples,
                                 int min_samples = 4);

// Extract VMT-frame yaw (radians) from a VMT-frame quaternion (xyzw).
// y-up convention: yaw = atan2(2(qw·qy + qx·qz), 1 − 2(qy² + qz²)).
float yaw_from_vmt_quat(const VmtQuat& q);

}  // namespace fitra::vmt
