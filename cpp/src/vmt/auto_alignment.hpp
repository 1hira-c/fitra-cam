#pragma once
//
// Phase 15 M3: Solve VmtAlignment from HMD + chest tracker observations.
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

//
// T-pose single-shot. `hmd` and `chest_vmt_*` are simultaneous samples; the
// alignment is the (yaw, x, z) such that R_y(yaw) · chest_vmt_xz + (x, z)
// matches hmd.xz. Y is left at 0 (caller may overlay a manual height offset).
//
// `chest_quat_xyzw` is the chest orientation in VMT frame (xyzw on the
// wire); the function extracts yaw via the Y-up formula.
//
AutoAlignmentResult solve_tpose(const HmdPose&  hmd,
                                const VmtPos&   chest_vmt_pos,
                                const VmtQuat&  chest_vmt_quat_xyzw);

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
