#include "vmt/vmt_protocol.hpp"

#include <cmath>

namespace fitra::vmt {

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
