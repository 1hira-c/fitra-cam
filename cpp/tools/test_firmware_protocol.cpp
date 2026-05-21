// test_firmware_protocol — byte-level golden tests for the SlimeVR Firmware
// UDP codec. Verifies the wire format against the known SlimeVR-Tracker-ESP
// layout (big-endian, sequence header, packet tag dispatch). No socket I/O.

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include "slimevr/firmware_protocol.hpp"

namespace {

using fitra::slimevr::MacBytes;
using fitra::slimevr::PacketTag;
using fitra::slimevr::QuatXyzw;
using fitra::slimevr::RotationDataType;
using fitra::slimevr::TrackerPosition;

void check(bool cond, const std::string& msg) {
    if (!cond) throw std::runtime_error(msg);
}

std::string hex(const std::vector<std::uint8_t>& v) {
    std::string s;
    s.reserve(v.size() * 3);
    char tmp[4];
    for (auto b : v) {
        std::snprintf(tmp, sizeof(tmp), "%02X ", b);
        s += tmp;
    }
    if (!s.empty()) s.pop_back();
    return s;
}

void expect_bytes(const std::vector<std::uint8_t>& got,
                  const std::vector<std::uint8_t>& want,
                  const std::string& label) {
    if (got != want) {
        std::string msg = label + ":\n  got:  " + hex(got) + "\n  want: " + hex(want);
        throw std::runtime_error(msg);
    }
}

std::uint32_t read_be32(const std::uint8_t* p) {
    return  (static_cast<std::uint32_t>(p[0]) << 24)
          | (static_cast<std::uint32_t>(p[1]) << 16)
          | (static_cast<std::uint32_t>(p[2]) <<  8)
          |  static_cast<std::uint32_t>(p[3]);
}
std::uint64_t read_be64(const std::uint8_t* p) {
    std::uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v = (v << 8) | p[i];
    return v;
}

// ---- Tests ---------------------------------------------------------------

void test_packet_header() {
    auto pkt = fitra::slimevr::encode_heartbeat(0x123456789ABCDEFull);
    check(pkt.size() == 12, "heartbeat size");
    check(read_be32(pkt.data())     == 0,
          "heartbeat tag should be 0");
    check(read_be64(pkt.data() + 4) == 0x123456789ABCDEFull,
          "heartbeat sequence should round-trip BE u64");
}

void test_handshake_layout() {
    // Known MAC, known firmware string. Expected payload bytes:
    //   tag=3 (BE u32), seq=0 (BE u64), board=4, imu=0, mcu=0, imu_info×3=0,
    //   protocol=18, fw_len, fw_bytes, mac_bytes.
    MacBytes mac{0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    auto pkt = fitra::slimevr::encode_handshake(0, mac, "abc");

    std::vector<std::uint8_t> want{
        // tag = 3
        0x00, 0x00, 0x00, 0x03,
        // sequence = 0
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        // board = 4 (CUSTOM)
        0x00, 0x00, 0x00, 0x04,
        // imuType = 0
        0x00, 0x00, 0x00, 0x00,
        // mcuType = 0
        0x00, 0x00, 0x00, 0x00,
        // imu_info[3] = 0,0,0
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        // protocolVersion = 18
        0x00, 0x00, 0x00, 0x12,
        // fw len = 3, "abc"
        0x03, 0x61, 0x62, 0x63,
        // mac
        0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
    };
    expect_bytes(pkt, want, "handshake byte layout");
}

void test_sensor_info_layout() {
    // sensor 0 = LeftUpperArm (15). seq = 42.
    auto pkt = fitra::slimevr::encode_sensor_info(
        42, /*sensor_id=*/0, TrackerPosition::LeftUpperArm);

    std::vector<std::uint8_t> want{
        // tag = 15
        0x00, 0x00, 0x00, 0x0F,
        // sequence = 42
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A,
        // sensor_id = 0
        0x00,
        // sensorStatus = OK (1)
        0x01,
        // sensorType = 0 (UNKNOWN)
        0x00,
        // sensorConfig = 0 (BE u16)
        0x00, 0x00,
        // hasCompletedRestCalibration = 0
        0x00,
        // trackerPosition = 15 (LEFT_UPPER_ARM)
        0x0F,
        // trackerDataType = 0 (ROTATION)
        0x00,
    };
    expect_bytes(pkt, want, "sensor_info LeftUpperArm sensor_id=0 seq=42");
}

void test_rotation_data_layout() {
    // Identity quat (1, 0, 0, 0) ↦ wire order (qx, qy, qz, qw) = (0, 0, 0, 1).
    // Float 0.0f is 0x00000000, 1.0f is 0x3F800000 — both well-known IEEE-754.
    QuatXyzw q{0.0f, 0.0f, 0.0f, 1.0f};
    auto pkt = fitra::slimevr::encode_rotation_data(
        /*seq=*/0xDEADBEEFull, /*sensor_id=*/5, q,
        /*accuracy=*/0, RotationDataType::Normal);

    std::vector<std::uint8_t> want{
        // tag = 17
        0x00, 0x00, 0x00, 0x11,
        // sequence = 0xDEADBEEF
        0x00, 0x00, 0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF,
        // sensor_id = 5
        0x05,
        // dataType = Normal (1)
        0x01,
        // qx = 0.0f
        0x00, 0x00, 0x00, 0x00,
        // qy = 0.0f
        0x00, 0x00, 0x00, 0x00,
        // qz = 0.0f
        0x00, 0x00, 0x00, 0x00,
        // qw = 1.0f
        0x3F, 0x80, 0x00, 0x00,
        // accuracy_info = 0
        0x00,
    };
    expect_bytes(pkt, want, "rotation_data identity quat sensor_id=5");
}

void test_ping_reply_layout() {
    // Server pings with id=0xCAFEBABE; we mirror it back with seq=0.
    auto pkt = fitra::slimevr::encode_ping_reply(0xCAFEBABEu);
    std::vector<std::uint8_t> want{
        // tag = 10
        0x00, 0x00, 0x00, 0x0A,
        // sequence = 0 (ping uses 0 outbound)
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        // ping id (mirrored)
        0xCA, 0xFE, 0xBA, 0xBE,
    };
    expect_bytes(pkt, want, "ping reply mirrors id 0xCAFEBABE");
}

void test_ping_decode_roundtrip() {
    // Build a fake server ping and decode it back.
    std::vector<std::uint8_t> srv_ping{
        0x00, 0x00, 0x00, 0x0A,                          // tag = 10
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // seq = 0
        0x12, 0x34, 0x56, 0x78,                          // ping id
    };
    std::uint32_t id = 0;
    check(fitra::slimevr::decode_ping(srv_ping.data(), srv_ping.size(), id),
          "decode_ping should accept a valid ping");
    check(id == 0x12345678u, "decoded ping id should round-trip");

    // Wrong tag → reject.
    std::vector<std::uint8_t> not_ping{
        0x00, 0x00, 0x00, 0x11,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    };
    id = 0xABCDEFu;
    check(!fitra::slimevr::decode_ping(not_ping.data(), not_ping.size(), id),
          "decode_ping should reject non-ping tag");

    // Too short → reject.
    check(!fitra::slimevr::decode_ping(srv_ping.data(), 8, id),
          "decode_ping should reject truncated buffer");
}

void test_mac_derivation_deterministic() {
    // Same input → same MAC.
    auto a1 = fitra::slimevr::mac_from_string("fitra-jetson");
    auto a2 = fitra::slimevr::mac_from_string("fitra-jetson");
    check(a1 == a2, "mac_from_string must be deterministic");

    // Different input → different MAC.
    auto b = fitra::slimevr::mac_from_string("other-host");
    check(a1 != b, "different inputs should yield different MACs");

    // Unicast + locally-administered bits.
    check((a1[0] & 0x01) == 0, "MAC must be unicast (bit 0 = 0)");
    check((a1[0] & 0x02) != 0, "MAC must be locally-administered (bit 1 = 1)");
}

void test_world_quat_to_slime() {
    constexpr float kEps = 1.0e-6f;
    auto close = [](float a, float b) { return std::fabs(a - b) < kEps; };
    const float k = 0.70710678f;   // 1/√2

    // World identity (wxyz = 1,0,0,0):
    //   wire wxyz   = ((1+0)/√2, (1-0)/√2, (0-0)/√2, (0+0)/√2) = (k, k, 0, 0)
    //   wire xyzw   = (k, 0, 0, k) = Rx(+90°). The server's AXES_OFFSET =
    //   Rx(-90°) cancels it leaving identity → ✓ subject at rest = avatar
    //   identity orientation.
    auto q = fitra::slimevr::world_quat_to_slime(1.0f, 0.0f, 0.0f, 0.0f);
    check(close(q.x, k),    "world identity qx = k (wire is Rx(+90°))");
    check(close(q.y, 0.0f), "world identity qy");
    check(close(q.z, 0.0f), "world identity qz");
    check(close(q.w, k),    "world identity qw = k");

    // World +Z rotation by 90° (subject yaws CCW looking from +Z, i.e.,
    // turning toward subject's right under our convention since the subject
    // faces +Y and +Y rotates to -X under +Z, but +X (subject's left) rotates
    // to +Y, so the subject's front face ends up pointing toward what was
    // their right side):
    //   q_world wxyz = (cos45°, 0, 0, sin45°) = (k, 0, 0, k)
    //   wire wxyz   = ((k+0)/√2, (k-0)/√2, (k-0)/√2, (0+k)/√2) = (0.5, 0.5, 0.5, 0.5)
    //   wire xyzw   = (0.5, 0.5, 0.5, 0.5)
    auto q2 = fitra::slimevr::world_quat_to_slime(k, 0.0f, 0.0f, k);
    check(close(q2.x, 0.5f),  "world +Z90 qx = 0.5");
    check(close(q2.y, 0.5f),  "world +Z90 qy = 0.5 (sign matches Y from X-flip)");
    check(close(q2.z, 0.5f),  "world +Z90 qz = 0.5");
    check(close(q2.w, 0.5f),  "world +Z90 qw = 0.5");

    // World +X rotation by 90° (subject pitches in our world frame):
    //   q_world wxyz = (k, k, 0, 0)
    //   wire wxyz   = ((k+k)/√2, (k-k)/√2, 0, 0) = (1, 0, 0, 0)
    //   wire xyzw   = (0, 0, 0, 1) = identity on the wire. After AXES_OFFSET
    //   (Rx(-90°)) this becomes Rx(-90°) in unity = pitch on unity X. ✓
    auto q3 = fitra::slimevr::world_quat_to_slime(k, k, 0.0f, 0.0f);
    check(close(q3.x, 0.0f), "world +X90 qx = 0 (wire is identity)");
    check(close(q3.y, 0.0f), "world +X90 qy = 0");
    check(close(q3.z, 0.0f), "world +X90 qz = 0");
    check(close(q3.w, 1.0f), "world +X90 qw = 1");

    // World +Y rotation by 90° (roll in our world frame):
    //   q_world wxyz = (k, 0, k, 0)
    //   wire wxyz   = ((k+0)/√2, (k-0)/√2, (0-k)/√2, (k+0)/√2)
    //               = (0.5, 0.5, -0.5, 0.5)
    //   wire xyzw   = (0.5, -0.5, 0.5, 0.5)
    auto q4 = fitra::slimevr::world_quat_to_slime(k, 0.0f, k, 0.0f);
    check(close(q4.x,  0.5f),  "world +Y90 qx = 0.5");
    check(close(q4.y, -0.5f),  "world +Y90 qy = -0.5");
    check(close(q4.z,  0.5f),  "world +Y90 qz = 0.5");
    check(close(q4.w,  0.5f),  "world +Y90 qw = 0.5");

    // Sanity: the X-flip makes (qw + qx) → (qw - qx) on the X component of
    // wire (and same magnitude on W). A roll around +Y should leave the X
    // axis fixed; check that the wire's net X-axis rotation is well-defined.
    // After AXES_OFFSET, q_used = Rx(-90°) * q_wire for +Y90 case should
    // yield a rotation around the unity Z axis (= roll), not Y or X.
    //   q_used = Rx(-90°) * (0.5,0.5,-0.5,0.5)
    //          = (k, -k, 0, 0) * (0.5, 0.5, -0.5, 0.5)
    //          = (k, 0, 0, k)  ← +90° around unity +Z = roll ✓
    // (This is a derivation check, not a separate assertion.)
}

void test_sequence_independence() {
    // The encoder is a pure function: same arguments must produce identical
    // bytes regardless of call order. Sequence handling is the publisher's
    // job; verify that here.
    QuatXyzw q{0.1f, 0.2f, 0.3f, 0.92f};
    auto a = fitra::slimevr::encode_rotation_data(100, 3, q);
    auto b = fitra::slimevr::encode_rotation_data(100, 3, q);
    auto c = fitra::slimevr::encode_rotation_data(101, 3, q);
    check(a == b, "encoder must be deterministic");
    check(a != c, "different sequence must produce different bytes");
}

}  // namespace

int main() {
    try {
        test_packet_header();             std::printf("[ok] packet header round-trip\n");
        test_handshake_layout();          std::printf("[ok] handshake byte layout\n");
        test_sensor_info_layout();        std::printf("[ok] sensor_info byte layout\n");
        test_rotation_data_layout();      std::printf("[ok] rotation_data byte layout\n");
        test_ping_reply_layout();         std::printf("[ok] ping reply byte layout\n");
        test_ping_decode_roundtrip();     std::printf("[ok] ping decode roundtrip\n");
        test_mac_derivation_deterministic(); std::printf("[ok] mac derivation\n");
        test_world_quat_to_slime();       std::printf("[ok] world→slime quat transform\n");
        test_sequence_independence();     std::printf("[ok] encoder determinism\n");
        std::printf("test_firmware_protocol: all good\n");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what());
        return 1;
    }
}
