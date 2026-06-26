#pragma once
//
// Virtual Motion Tracker (VMT) wire protocol helpers.
//
// VMT is a SteamVR Driver that listens on UDP for OSC 1.0 packets and
// surfaces each tracker as a SteamVR virtual device. We send 10 trackers
// (one per TrackerRole) on `/VMT/Room/Driver` at 60 Hz so VRChat FBT can
// consume them directly, bypassing SlimeVR Server entirely.
//
// Protocol reference (VMT v0.15, https://gpsnmeajp.github.io/VirtualMotionTrackerDocument/api/):
//   /VMT/Room/Driver i:index i:enable f:timeoffset
//                    f:x f:y f:z f:qx f:qy f:qz f:qw
//   - Driver room space = SteamVR Driver convention: Y-up RH, X-right,
//     Z-back, units = meters / quaternion in xyzw order.

#include <array>
#include <cstdint>
#include <string>

#include "slimevr/tracker_extract.hpp"  // TrackerRole / kTrackerCount
#include "vmt/osc_writer.hpp"

namespace fitra::vmt {

// Which TrackerRoles are published to VMT. VRChat FBT consumes at most 8
// trackers (hip, chest, 2 feet, 2 knees, 2 elbows); the shin (LowerLeg) roles
// have no VRChat role and are only included by `Full` (SlimeVR-compatible).
// VRChat's own docs note fewer trackers can give a more stable IK solve, so the
// count is selectable. Index assignment stays role-tied (vmt_index_for), so a
// disabled role just leaves an index gap — the SteamVR "Manage Trackers" role
// binding (VMT_10 = Left Elbow, …) is stable across presets.
//   P3   : Waist, LeftFoot, RightFoot
//   P6   : P3 + Chest, LeftUpperLeg, RightUpperLeg
//   P8   : P6 + LeftUpperArm, RightUpperArm  (VRChat standard FBT; default)
//   Full : all 10 roles (adds LeftLowerLeg, RightLowerLeg; legacy / SlimeVR)
enum class VmtTrackerPreset : std::uint8_t { P3, P6, P8, Full };

const char* vmt_preset_name(VmtTrackerPreset p);
bool        parse_vmt_preset(const std::string& s, VmtTrackerPreset& out);

// Per-role enable mask for a preset, indexed by static_cast<int>(TrackerRole).
std::array<bool, slimevr::kTrackerCount> role_mask_for(VmtTrackerPreset p);

struct VmtPos  { float x, y, z; };
struct VmtQuat { float x, y, z, w; };  // wire order = xyzw

// Temporary manual alignment offset for matching VMT trackers to the SteamVR
// HMD playspace. Values are already in VMT Driver frame:
//   X = right, Y = up, Z = back, meters; yaw_deg rotates around +Y.
struct VmtAlignment {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float yaw_deg = 0.0f;
};

// World frame (Z-up RH, X-right, Y-forward) → VMT Driver frame (Y-up RH,
// X-right, Z-back). The Driver frame matches SteamVR's "x-right, y-up,
// z-back, right-handed" convention.
//
// test_vmt_protocol locks the cardinal-axis behaviour.
inline VmtPos world_pos_to_vmt(float x, float y, float z) {
    return {x, z, -y};
}

// Rx(-90°) basis change in quaternion form:
//   q_vmt_wxyz = P * q_world * P^{-1} with P = (cos(-45°), sin(-45°), 0, 0).
//   The closed form, expanded, is (qw, qx, qz, -qy) (still in wxyz).
// We return xyzw for direct feed into add_float() in OSC order.
inline VmtQuat world_quat_to_vmt(float qw, float qx, float qy, float qz) {
    return {qx, qz, -qy, qw};
}

// VMT tracker index mapping. The role order stays stable, but callers may add
// an index base to avoid SteamVR/VMT setups that reserve VMT_0..VMT_2 for HMD
// or controller tracking overrides.
//
// Index | TrackerRole       | suggested SteamVR Manage Trackers role
//   B+0 | LeftUpperArm      | LeftElbow / LeftShoulder (VRChat extension)
//   B+1 | RightUpperArm     | RightElbow / RightShoulder
//   B+2 | Chest             | Chest
//   B+3 | Waist (HIP)       | Waist
//   B+4 | LeftUpperLeg      | LeftKnee
//   B+5 | RightUpperLeg     | RightKnee
//   B+6 | LeftLowerLeg      | (Full preset only; no VRChat role — leave unmapped)
//   B+7 | RightLowerLeg     | (ditto)
//   B+8 | LeftFoot          | LeftFoot
//   B+9 | RightFoot         | RightFoot
inline int vmt_index_for(slimevr::TrackerRole role, int index_base = 0) {
    return index_base + static_cast<int>(role);
}

// Apply manual alignment in VMT Driver frame.
// Order: rotate around +Y by yaw_deg, then add xyz translation.
// Quaternion order mirrors position: q_out = q_yaw * q_in.
void apply_vmt_alignment(VmtPos& pos, VmtQuat& quat, const VmtAlignment& alignment);

// Inverse of (apply_vmt_alignment ∘ world_*_to_vmt): take a pose expressed in
// the VMT Driver frame (Y-up, post-alignment — the frame SteamVR reports HMD /
// controller poses in) and recover the fitra world pose (Z-up). Undoes the
// alignment (translate back, rotate by -yaw) then the basis change. Lets the 3D
// viewer draw the HMD in the same space as the triangulated skeleton.
//   out_pos_xyz  : world (x, y, z)
//   out_quat_wxyz: world quaternion in (w, x, y, z) order
void vmt_pose_to_world(const VmtPos& pos, const VmtQuat& quat_xyzw,
                       const VmtAlignment& alignment,
                       float out_pos_xyz[3], float out_quat_wxyz[4]);

// Append one `/VMT/Room/Driver` message to the writer. Caller is responsible
// for begin_bundle / end_bundle.
//   index:      0..57
//   enable:     0=disabled, 1=tracker, (2=left controller etc. - unused here)
//   timeoffset: seconds (0 = "now")
//   pos / quat: already passed through world_pos_to_vmt / world_quat_to_vmt
void encode_vmt_room_driver(OscWriter& w,
                            int index,
                            int enable,
                            float timeoffset,
                            const VmtPos&  pos,
                            const VmtQuat& quat);

}  // namespace fitra::vmt
