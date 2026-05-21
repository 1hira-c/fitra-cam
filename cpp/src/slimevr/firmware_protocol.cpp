#include "slimevr/firmware_protocol.hpp"

#include <algorithm>
#include <cstring>
#include <unistd.h>

namespace fitra::slimevr {

namespace {

inline void append_u8(std::vector<std::uint8_t>& out, std::uint8_t v) {
    out.push_back(v);
}

inline void append_u16_be(std::vector<std::uint8_t>& out, std::uint16_t v) {
    out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>(v        & 0xFF));
}

inline void append_u32_be(std::vector<std::uint8_t>& out, std::uint32_t v) {
    out.push_back(static_cast<std::uint8_t>((v >> 24) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((v >> 16) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((v >>  8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>( v        & 0xFF));
}

inline void append_i32_be(std::vector<std::uint8_t>& out, std::int32_t v) {
    append_u32_be(out, static_cast<std::uint32_t>(v));
}

inline void append_u64_be(std::vector<std::uint8_t>& out, std::uint64_t v) {
    for (int shift = 56; shift >= 0; shift -= 8) {
        out.push_back(static_cast<std::uint8_t>((v >> shift) & 0xFF));
    }
}

inline void append_float_be(std::vector<std::uint8_t>& out, float v) {
    static_assert(sizeof(float) == 4, "float must be 32 bits");
    std::uint32_t bits;
    std::memcpy(&bits, &v, 4);
    append_u32_be(out, bits);
}

inline void append_bytes(std::vector<std::uint8_t>& out,
                          const std::uint8_t* data, std::size_t n) {
    out.insert(out.end(), data, data + n);
}

inline void append_header(std::vector<std::uint8_t>& out,
                           PacketTag tag, std::uint64_t seq) {
    append_u32_be(out, static_cast<std::uint32_t>(tag));
    append_u64_be(out, seq);
}

// Pull the first 4 bytes (BE u32) without bounds-checking; caller ensures size.
inline std::uint32_t read_u32_be(const std::uint8_t* p) {
    return  (static_cast<std::uint32_t>(p[0]) << 24)
          | (static_cast<std::uint32_t>(p[1]) << 16)
          | (static_cast<std::uint32_t>(p[2]) <<  8)
          |  static_cast<std::uint32_t>(p[3]);
}

// SHA-1 minimal implementation. Used only for MAC derivation (constant-size
// input, no security relevance). Avoids dragging OpenSSL into fitra_slimevr.
struct Sha1 {
    std::uint32_t h[5];
    std::uint64_t length_bits;
    std::uint8_t  buffer[64];
    std::size_t   buffer_len;

    Sha1() { reset(); }

    void reset() {
        h[0] = 0x67452301; h[1] = 0xEFCDAB89; h[2] = 0x98BADCFE;
        h[3] = 0x10325476; h[4] = 0xC3D2E1F0;
        length_bits = 0;
        buffer_len = 0;
    }

    static std::uint32_t rol(std::uint32_t v, int n) {
        return (v << n) | (v >> (32 - n));
    }

    void process_block(const std::uint8_t* block) {
        std::uint32_t w[80];
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<std::uint32_t>(block[i*4    ]) << 24)
                 | (static_cast<std::uint32_t>(block[i*4 + 1]) << 16)
                 | (static_cast<std::uint32_t>(block[i*4 + 2]) <<  8)
                 |  static_cast<std::uint32_t>(block[i*4 + 3]);
        }
        for (int i = 16; i < 80; ++i) {
            w[i] = rol(w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16], 1);
        }
        std::uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
        for (int i = 0; i < 80; ++i) {
            std::uint32_t f, k;
            if      (i < 20) { f = (b & c) | ((~b) & d);     k = 0x5A827999; }
            else if (i < 40) { f = b ^ c ^ d;                k = 0x6ED9EBA1; }
            else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
            else             { f = b ^ c ^ d;                k = 0xCA62C1D6; }
            std::uint32_t t = rol(a, 5) + f + e + k + w[i];
            e = d; d = c; c = rol(b, 30); b = a; a = t;
        }
        h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
    }

    void update(const std::uint8_t* data, std::size_t n) {
        length_bits += n * 8;
        while (n > 0) {
            std::size_t take = std::min<std::size_t>(64 - buffer_len, n);
            std::memcpy(buffer + buffer_len, data, take);
            buffer_len += take;
            data       += take;
            n          -= take;
            if (buffer_len == 64) {
                process_block(buffer);
                buffer_len = 0;
            }
        }
    }

    void finalize(std::uint8_t digest[20]) {
        // Append 0x80, then pad with zeros, then 64-bit length BE.
        std::uint64_t total_bits = length_bits;
        buffer[buffer_len++] = 0x80;
        if (buffer_len > 56) {
            std::memset(buffer + buffer_len, 0, 64 - buffer_len);
            process_block(buffer);
            buffer_len = 0;
        }
        std::memset(buffer + buffer_len, 0, 56 - buffer_len);
        for (int i = 0; i < 8; ++i) {
            buffer[56 + i] = static_cast<std::uint8_t>(
                (total_bits >> (56 - 8 * i)) & 0xFF);
        }
        process_block(buffer);
        for (int i = 0; i < 5; ++i) {
            digest[i*4    ] = static_cast<std::uint8_t>((h[i] >> 24) & 0xFF);
            digest[i*4 + 1] = static_cast<std::uint8_t>((h[i] >> 16) & 0xFF);
            digest[i*4 + 2] = static_cast<std::uint8_t>((h[i] >>  8) & 0xFF);
            digest[i*4 + 3] = static_cast<std::uint8_t>( h[i]        & 0xFF);
        }
    }
};

}  // namespace

// ---------------------------------------------------------------------------

std::vector<std::uint8_t>
encode_handshake(std::uint64_t sequence,
                 const MacBytes& mac,
                 std::string_view firmware_version) {
    std::vector<std::uint8_t> out;
    out.reserve(64);
    append_header(out, PacketTag::Handshake, sequence);
    append_i32_be(out, kBoardTypeCustom);          // boardType
    append_i32_be(out, kImuTypeUnknown);           // imuType (primary)
    append_i32_be(out, kMcuTypeUnknown);           // mcuType
    append_i32_be(out, 0);                         // imu_info[0]
    append_i32_be(out, 0);                         // imu_info[1]
    append_i32_be(out, 0);                         // imu_info[2]
    append_i32_be(out, kProtocolVersion);          // protocolVersion
    // Firmware string: u8 length + ASCII bytes. Server reads at most 255.
    const std::size_t fw_len = std::min<std::size_t>(firmware_version.size(), 255);
    append_u8(out, static_cast<std::uint8_t>(fw_len));
    append_bytes(out,
                 reinterpret_cast<const std::uint8_t*>(firmware_version.data()),
                 fw_len);
    append_bytes(out, mac.data(), mac.size());     // 6-byte MAC
    return out;
}

std::vector<std::uint8_t>
encode_sensor_info(std::uint64_t sequence,
                   std::uint8_t sensor_id,
                   TrackerPosition position) {
    std::vector<std::uint8_t> out;
    out.reserve(20);
    append_header(out, PacketTag::SensorInfo, sequence);
    append_u8 (out, sensor_id);
    append_u8 (out, static_cast<std::uint8_t>(SensorStatus::Ok));   // sensorStatus
    append_u8 (out, 0);                                              // sensorType=UNKNOWN
    append_u16_be(out, 0);                                           // sensorConfig (no mag, etc.)
    append_u8 (out, 0);                                              // hasCompletedRestCalibration
    append_u8 (out, static_cast<std::uint8_t>(position));            // trackerPosition
    append_u8 (out, static_cast<std::uint8_t>(TrackerDataType::Rotation));
    return out;
}

std::vector<std::uint8_t>
encode_rotation_data(std::uint64_t sequence,
                     std::uint8_t sensor_id,
                     const QuatXyzw& quat,
                     std::uint8_t accuracy_info,
                     RotationDataType data_type) {
    std::vector<std::uint8_t> out;
    out.reserve(32);
    append_header(out, PacketTag::RotationData, sequence);
    append_u8 (out, sensor_id);
    append_u8 (out, static_cast<std::uint8_t>(data_type));
    append_float_be(out, quat.x);
    append_float_be(out, quat.y);
    append_float_be(out, quat.z);
    append_float_be(out, quat.w);
    append_u8 (out, accuracy_info);
    return out;
}

std::vector<std::uint8_t>
encode_heartbeat(std::uint64_t sequence) {
    std::vector<std::uint8_t> out;
    out.reserve(12);
    append_header(out, PacketTag::Heartbeat, sequence);
    return out;
}

std::vector<std::uint8_t>
encode_ping_reply(std::uint32_t ping_id) {
    std::vector<std::uint8_t> out;
    out.reserve(16);
    // Server's own ping writer uses sequence=0 and parser is lenient about
    // sequence ordering for ping; mirror that here.
    append_header(out, PacketTag::PingPong, 0);
    append_u32_be(out, ping_id);
    return out;
}

bool decode_ping(const std::uint8_t* data, std::size_t len, std::uint32_t& out_ping_id) {
    // Header (12) + ping id (4) = 16 bytes total.
    if (len < 16) return false;
    if (read_u32_be(data) != static_cast<std::uint32_t>(PacketTag::PingPong)) return false;
    out_ping_id = read_u32_be(data + 12);
    return true;
}

// ---------------------------------------------------------------------------

MacBytes mac_from_string(std::string_view name) {
    Sha1 sha;
    sha.update(reinterpret_cast<const std::uint8_t*>(name.data()), name.size());
    std::uint8_t digest[20];
    sha.finalize(digest);
    MacBytes mac{};
    for (int i = 0; i < 6; ++i) mac[i] = digest[i];
    // Force locally-administered + unicast bits per IEEE 802.
    mac[0] &= 0xFE;   // unicast (LSB = 0)
    mac[0] |= 0x02;   // locally-administered (bit 1 = 1)
    return mac;
}

MacBytes mac_from_hostname() {
    char buf[256];
    if (gethostname(buf, sizeof(buf)) != 0) {
        return MacBytes{};   // zero MAC fallback
    }
    buf[sizeof(buf) - 1] = '\0';
    return mac_from_string(buf);
}

QuatXyzw world_quat_to_slime(float qw, float qx, float qy, float qz) {
    // World frame (Z-up RH, X-right, Y-forward) → SlimeVR Firmware UDP wire.
    //
    // Composition of:
    //   q_unity_wxyz = (qw, -qx, -qz, -qy)              // Y↔Z swap, RH→LH
    //   q_wire       = Rx(+90°) * q_unity               // pre-cancel AXES_OFFSET
    //
    // Algebraic expansion:
    //   wire wxyz = ( (qw+qx)/√2, (qw-qx)/√2, (qy-qz)/√2, -(qy+qz)/√2 )
    //
    // After the server applies AXES_OFFSET = Rx(-90°), pure subject yaw lands
    // on unity Y (correct yaw), pitch on unity X (correct pitch), and roll on
    // unity Z (correct roll). The legacy VMC formula `(qx, qz, -qy, -qw)`
    // mapped yaw onto the unity Z axis (= leg roll), producing the abduction
    // symptom observed in the first Phase 11 deployment.
    static constexpr float kInvSqrt2 = 0.70710678f;
    QuatXyzw q;
    q.w =  kInvSqrt2 * (qw + qx);
    q.x =  kInvSqrt2 * (qw - qx);
    q.y =  kInvSqrt2 * (qy - qz);
    q.z = -kInvSqrt2 * (qy + qz);
    return q;
}

}  // namespace fitra::slimevr
