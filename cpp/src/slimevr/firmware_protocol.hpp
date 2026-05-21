#pragma once
//
// Phase 11: SlimeVR Firmware UDP protocol — wire-format codec.
//
// Pure serialization / parsing functions; no socket I/O. The publisher
// (slimevr::NativePublisher) owns the socket and the sequence counter and
// calls these encoders to build the UDP payloads.
//
// References (verified at this revision):
//   ~/Documents/refs/slimevr/SlimeVR-Tracker-ESP/src/network/packets.h
//   ~/Documents/refs/slimevr/SlimeVR-Tracker-ESP/src/network/connection.cpp
//   ~/Documents/refs/slimevr/SlimeVR-Server/server/core/src/main/java/dev/slimevr/
//     tracking/trackers/udp/{UDPProtocolParser,UDPPacket,FirmwareConstants}.kt
//
// Common packet framing (every send):
//   [4 bytes BE u32]  packet tag
//   [8 bytes BE u64]  sequence number (auto-increment on the publisher side)
//   [N bytes]         payload (per-tag, defined below)
//
// Coordinate system: SlimeVR Server interprets RotationData quaternions in
// Y-up Unity-style left-handed space (the server pre-multiplies by AXES_OFFSET
// internally). Our world frame is Z-up right-handed; the publisher (not this
// file) applies `world_quat_to_slime()` before encoding.

#include <array>
#include <cstdint>
#include <string_view>
#include <vector>

namespace fitra::slimevr {

// ---- Constants -------------------------------------------------------------

inline constexpr std::uint16_t kDefaultPort      = 6969;   // SlimeVR firmware UDP port
inline constexpr std::int32_t  kProtocolVersion  = 18;     // current server-known protocol

// SlimeVR-Server BoardType.CUSTOM (FirmwareConstants.kt id=4).
inline constexpr std::int32_t  kBoardTypeCustom  = 4;
// IMUType.UNKNOWN (we are not an IMU; cameras are off-protocol).
inline constexpr std::int32_t  kImuTypeUnknown   = 0;
// MCUType.UNKNOWN.
inline constexpr std::int32_t  kMcuTypeUnknown   = 0;

// Send-side packet tags (subset we emit).
enum class PacketTag : std::uint32_t {
    Heartbeat      = 0,
    Handshake      = 3,
    Accel          = 4,
    PingPong       = 10,
    SensorInfo     = 15,
    RotationData   = 17,
    UserAction     = 21,
    Position       = 27,   // accepted by server but hasPosition=false for UDP devices; reserved
};

// SensorInfo.sensorStatus (sensorState in firmware enum).
enum class SensorStatus : std::uint8_t {
    Disconnected = 0,
    Ok           = 1,
    Error        = 2,
};

// SensorInfo.trackerDataType (TrackerDataType in server enum).
enum class TrackerDataType : std::uint8_t {
    Rotation       = 0,
    FlexResistance = 1,
    FlexAngle      = 2,
};

// RotationData.dataType: NORMAL is the steady-state value (firmware uses
// CORRECTION while sensor reset is being applied; we always send NORMAL).
enum class RotationDataType : std::uint8_t {
    Normal     = 1,
    Correction = 2,
};

// SensorPosition values that map to TrackerPosition on the server side.
// Source: SlimeVR-Server `TrackerPosition.kt`. Only the ten roles we need.
enum class TrackerPosition : std::uint8_t {
    None          = 0,
    Chest         = 4,
    Waist         = 5,
    LeftUpperLeg  = 7,
    RightUpperLeg = 8,
    LeftLowerLeg  = 9,
    RightLowerLeg = 10,
    LeftFoot      = 11,
    RightFoot     = 12,
    LeftUpperArm  = 15,
    RightUpperArm = 16,
};

// ---- Types -----------------------------------------------------------------

using MacBytes = std::array<std::uint8_t, 6>;

struct QuatXyzw {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;
};

// ---- Encoders --------------------------------------------------------------
//
// Each `encode_*` returns the complete UDP payload (header + body) ready to
// `sendto()`. Sequence numbers are caller-supplied so the publisher owns the
// counter and can keep handshake / sensor_info / rotation streams synchronized.
//
// Handshake (tag 3). Server reads optionally up to MAC bytes; we always
// send through MAC. `firmware_version` is a short ASCII string (≤ 255 chars).
std::vector<std::uint8_t>
encode_handshake(std::uint64_t sequence,
                 const MacBytes& mac,
                 std::string_view firmware_version);

// SensorInfo (tag 15). Sent once per sensor after handshake to declare its
// body-part role. The server uses `position` (TrackerPosition enum value) to
// display the tracker by name (e.g., "Left Foot") instead of a sequential
// number. We always send trackerDataType=Rotation.
std::vector<std::uint8_t>
encode_sensor_info(std::uint64_t sequence,
                   std::uint8_t sensor_id,
                   TrackerPosition position);

// RotationData (tag 17). Sent every cycle (default 60 Hz) per sensor.
// Quaternion components are wire-order (qx, qy, qz, qw), each big-endian
// float32. `accuracy_info` is a free byte; 0 is fine for our use case.
std::vector<std::uint8_t>
encode_rotation_data(std::uint64_t sequence,
                     std::uint8_t sensor_id,
                     const QuatXyzw& quat,
                     std::uint8_t accuracy_info = 0,
                     RotationDataType data_type = RotationDataType::Normal);

// Heartbeat (tag 0). Header only — keeps the SlimeVR server's per-device
// liveness timer from marking us as disconnected during sparse output.
std::vector<std::uint8_t>
encode_heartbeat(std::uint64_t sequence);

// PingPong reply (tag 10). Server periodically sends a 16-byte ping
// containing a 4-byte BE id; we mirror it back with the same id and
// `sequence = 0` (the server's own writer uses 0 on its outbound writes,
// and the server tolerates either value on inbound).
std::vector<std::uint8_t>
encode_ping_reply(std::uint32_t ping_id);

// ---- Decoders --------------------------------------------------------------

// Recognize an incoming Ping (tag 10). Returns true and fills `out_ping_id`
// if `data` is a valid 16-byte ping packet. The publisher's recv loop calls
// this on every datagram; on success it replies with `encode_ping_reply`.
bool decode_ping(const std::uint8_t* data,
                 std::size_t len,
                 std::uint32_t& out_ping_id);

// ---- Helpers ---------------------------------------------------------------

// Deterministic 6-byte MAC derived from the host's name. Same Jetson → same
// MAC across reboots, so SlimeVR remembers the trackerPosition mapping the
// user previously approved. Fallback to a zero MAC if the lookup fails.
MacBytes mac_from_hostname();

// Test seam: derive a MAC from an arbitrary string. SHA-1 first 6 bytes,
// then unicast bit cleared (LSB of first octet = 0) and locally-administered
// bit set (second-LSB of first octet = 1).
MacBytes mac_from_string(std::string_view name);

// Convert our world-frame quaternion (right-handed Z-up, wxyz storage) into
// the SlimeVR Firmware UDP wire frame. Returns xyzw on the wire.
//
// Two transforms are composed:
//   1. World (Z-up RH, X-right, Y-forward) → Unity (Y-up, Z-forward) basis
//      change with X axis FLIPPED so subject's left (+X world) maps to the
//      avatar's left (-X unity). Without the X-flip, lateral limb motion
//      shows up on the opposite side of the avatar — observed in the field
//      as "横方向の動きが左右逆になる".
//      The basis-change quaternion (Hamilton form):
//        P_wxyz   = (0, 0, 1/√2, 1/√2)        // 180° around (0,1,1)/√2
//        q_unity  = P * q_world * conj(P) = (qw, -qx, qz, qy)
//   2. Pre-cancel the server-side AXES_OFFSET = Rx(-90°):
//        q_wire   = Rx(+90°) * q_unity
//      `TrackersUDPServer.kt:415` left-multiplies every incoming rotation by
//      AXES_OFFSET before passing it to the IK; without this pre-cancel,
//      pure subject yaw lands on the unity Z axis (= leg roll / abduction).
//
// Composing the two collapses to:
//   wire wxyz = ( (qw+qx)/√2, (qw-qx)/√2, (qz-qy)/√2, (qy+qz)/√2 )
//
// Verified by hand for all four cardinal cases:
//   - identity      → wire = Rx(+90°)            (AXES_OFFSET cancels to identity)
//   - world +X 90°  → wire = identity             (pitch on unity X after offset)
//   - world +Y 90°  → wire = (0.5, -0.5, 0.5, 0.5)_xyzw   (roll on unity Z)
//   - world +Z 90°  → wire = (0.5, 0.5, 0.5, 0.5)_xyzw    (yaw on unity Y)
//
// And against the left-leg abduction synthesised pose: delta from rest
// becomes -45° around unity +Z (= avatar's left leg goes to avatar's left),
// not +45° (which would be mirrored to the right side).
QuatXyzw world_quat_to_slime(float qw, float qx, float qy, float qz);

}  // namespace fitra::slimevr
