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

#include <cstdint>

#include "slimevr/tracker_extract.hpp"  // TrackerRole / kTrackerCount
#include "vmt/osc_writer.hpp"

namespace fitra::vmt {

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

// VMT tracker index mapping. We use TrackerRole's integer value verbatim so
// vmt_0..vmt_9 follow the same body-part order as the rest of the pipeline.
//
// Index | TrackerRole       | suggested SteamVR Manage Trackers role
//   0   | LeftUpperArm      | LeftElbow / LeftShoulder (VRChat extension)
//   1   | RightUpperArm     | RightElbow / RightShoulder
//   2   | Chest             | Chest
//   3   | Waist (HIP)       | Waist
//   4   | LeftUpperLeg      | LeftKnee
//   5   | RightUpperLeg     | RightKnee
//   6   | LeftLowerLeg      | (function-overlap with Knee; leave unmapped)
//   7   | RightLowerLeg     | (ditto)
//   8   | LeftFoot          | LeftFoot
//   9   | RightFoot         | RightFoot
inline int vmt_index_for(slimevr::TrackerRole role) {
    return static_cast<int>(role);
}

// Apply manual alignment in VMT Driver frame.
// Order: rotate around +Y by yaw_deg, then add xyz translation.
// Quaternion order mirrors position: q_out = q_yaw * q_in.
void apply_vmt_alignment(VmtPos& pos, VmtQuat& quat, const VmtAlignment& alignment);

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
