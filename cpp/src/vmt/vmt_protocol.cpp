#include "vmt/vmt_protocol.hpp"

#include <cmath>

namespace fitra::vmt {

const char* vmt_preset_name(VmtTrackerPreset p) {
    switch (p) {
        case VmtTrackerPreset::P3:   return "p3";
        case VmtTrackerPreset::P6:   return "p6";
        case VmtTrackerPreset::P8:   return "p8";
        case VmtTrackerPreset::Full: return "full";
    }
    return "p8";
}

bool parse_vmt_preset(const std::string& s, VmtTrackerPreset& out) {
    if (s == "p3")   { out = VmtTrackerPreset::P3;   return true; }
    if (s == "p6")   { out = VmtTrackerPreset::P6;   return true; }
    if (s == "p8")   { out = VmtTrackerPreset::P8;   return true; }
    if (s == "full") { out = VmtTrackerPreset::Full; return true; }
    return false;
}

std::array<bool, slimevr::kTrackerCount> role_mask_for(VmtTrackerPreset p) {
    using R = slimevr::TrackerRole;
    std::array<bool, slimevr::kTrackerCount> m{};   // all false
    auto on = [&](R r) { m[static_cast<std::size_t>(r)] = true; };
    // Nested supersets (Full only adds the shins).
    on(R::Waist); on(R::LeftFoot); on(R::RightFoot);            // P3: hip + feet
    if (p == VmtTrackerPreset::P3) return m;
    on(R::Chest); on(R::LeftUpperLeg); on(R::RightUpperLeg);    // P6: + chest + knees
    if (p == VmtTrackerPreset::P6) return m;
    on(R::LeftUpperArm); on(R::RightUpperArm);                  // P8: + elbows
    if (p == VmtTrackerPreset::P8) return m;
    on(R::LeftLowerLeg); on(R::RightLowerLeg);                  // Full: + shins
    return m;
}

namespace {

constexpr float kPi = 3.14159265358979323846f;

VmtQuat normalize_quat(VmtQuat q) {
    const float n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (n2 <= 0.0f || !std::isfinite(n2)) {
        return {0.0f, 0.0f, 0.0f, 1.0f};
    }
    const float inv = 1.0f / std::sqrt(n2);
    return {q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}

VmtQuat mul_xyzw(const VmtQuat& a, const VmtQuat& b) {
    return {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

}  // namespace

void apply_vmt_alignment(VmtPos& pos, VmtQuat& quat, const VmtAlignment& alignment) {
    const float yaw_rad = alignment.yaw_deg * (kPi / 180.0f);
    const float half = yaw_rad * 0.5f;
    const float s = std::sin(yaw_rad);
    const float c = std::cos(yaw_rad);

    const float x = pos.x;
    const float z = pos.z;
    pos.x = c * x + s * z + alignment.x;
    pos.y = pos.y + alignment.y;
    pos.z = -s * x + c * z + alignment.z;

    VmtQuat yaw_q{0.0f, std::sin(half), 0.0f, std::cos(half)};
    quat = normalize_quat(mul_xyzw(yaw_q, quat));
}

void vmt_pose_to_world(const VmtPos& pos, const VmtQuat& quat_xyzw,
                       const VmtAlignment& alignment,
                       float out_pos_xyz[3], float out_quat_wxyz[4]) {
    const float yaw_rad = alignment.yaw_deg * (kPi / 180.0f);
    const float half = yaw_rad * 0.5f;
    const float s = std::sin(yaw_rad);
    const float c = std::cos(yaw_rad);

    // 1) undo the alignment translation, then the in-plane yaw. The forward map
    //    is [[c, s], [-s, c]] over (x, z); its inverse (a rotation, so the
    //    transpose) is [[c, -s], [s, c]].
    const float px = pos.x - alignment.x;
    const float py = pos.y - alignment.y;
    const float pz = pos.z - alignment.z;
    const float vx = c * px - s * pz;
    const float vy = py;
    const float vz = s * px + c * pz;

    // 2) inverse basis change of world_pos_to_vmt: world (x, y, z) = (X, -Z, Y).
    out_pos_xyz[0] = vx;
    out_pos_xyz[1] = -vz;
    out_pos_xyz[2] = vy;

    // 3) undo the yaw quaternion (q = yaw_q * q_pre ⇒ q_pre = conj(yaw_q) * q),
    //    then invert the basis: world wxyz = (W, X, -Z, Y) of world_quat_to_vmt.
    VmtQuat yaw_q_inv{0.0f, -std::sin(half), 0.0f, std::cos(half)};
    VmtQuat q_pre = normalize_quat(mul_xyzw(yaw_q_inv, quat_xyzw));
    out_quat_wxyz[0] = q_pre.w;
    out_quat_wxyz[1] = q_pre.x;
    out_quat_wxyz[2] = -q_pre.z;
    out_quat_wxyz[3] = q_pre.y;
}

void encode_vmt_room_driver(OscWriter& w,
                            int index,
                            int enable,
                            float timeoffset,
                            const VmtPos&  pos,
                            const VmtQuat& quat) {
    w.begin_message("/VMT/Room/Driver");
    w.add_int(index);
    w.add_int(enable);
    w.add_float(timeoffset);
    w.add_float(pos.x);
    w.add_float(pos.y);
    w.add_float(pos.z);
    w.add_float(quat.x);
    w.add_float(quat.y);
    w.add_float(quat.z);
    w.add_float(quat.w);
    w.end_message();
}

}  // namespace fitra::vmt
