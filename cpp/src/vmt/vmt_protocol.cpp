#include "vmt/vmt_protocol.hpp"

namespace fitra::vmt {

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
