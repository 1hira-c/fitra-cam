#pragma once
//
// Phase 15 M1: thin oscpack wrapper that serialises one /fitra/hmd_pose
// message (no bundle) and sends it via UDP to the Jetson.
//
// Wire format (matches `cpp/src/vmt/hmd_pose_receiver.cpp` parser):
//
//   address  = "/fitra/hmd_pose"
//   typetag  = ",iffffffff"   // i=valid, then 8 floats
//   args     = valid(i32)
//              timestamp_s(f32)
//              x(f32) y(f32) z(f32)
//              qx(f32) qy(f32) qz(f32) qw(f32)
//
// All values are in the SteamVR Standing universe (Y-up RH, X-right, Z-back,
// metres). The Jetson side transforms its world frame (Z-up RH) into this
// same frame via world_pos_to_vmt / world_quat_to_vmt before comparing.

#include <cstdint>
#include <string>

namespace fitra::hmd_sender {

struct HmdSample {
    bool  valid;          // tracking ok (false → host should freeze)
    float timestamp_s;    // monotonic seconds since app start (latency aid)
    float x, y, z;        // metres
    float qx, qy, qz, qw; // xyzw
};

class OscSender {
public:
    OscSender();
    ~OscSender();

    OscSender(const OscSender&) = delete;
    OscSender& operator=(const OscSender&) = delete;

    // Resolve & open UDP socket. Returns false on failure (error in
    // last_error()). Idempotent.
    bool open(const std::string& host, std::uint16_t port);

    // Serialise + send one message. Returns false on socket error.
    bool send(const HmdSample& s);

    const std::string& last_error() const { return last_error_; }

private:
    struct Impl;
    Impl* impl_;
    std::string last_error_;
};

}  // namespace fitra::hmd_sender
