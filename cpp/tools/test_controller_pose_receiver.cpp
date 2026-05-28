// Golden-bytes unit tests for parse_controller_pose_packet, the running_ok
// gate, and ControllerPoseBus stale judgement. Pure unit tests, no socket I/O.

#include "vmt/controller_pose_receiver.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

namespace {

using fitra::vmt::ControllerPose;
using fitra::vmt::ControllerPoseBus;
using fitra::vmt::kTrackingResultRunningOk;
using fitra::vmt::parse_controller_pose_packet;

int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++g_fail; } \
} while (0)

#define CHECK_NEAR(a, b, eps) do { \
    if (std::fabs((a) - (b)) > (eps)) { \
        std::fprintf(stderr, "FAIL %s:%d %s != %s (%g vs %g)\n", \
            __FILE__, __LINE__, #a, #b, double(a), double(b)); \
        ++g_fail; \
    } \
} while (0)

inline void be32(std::vector<std::uint8_t>& out, std::uint32_t v) {
    out.push_back(static_cast<std::uint8_t>((v >> 24) & 0xff));
    out.push_back(static_cast<std::uint8_t>((v >> 16) & 0xff));
    out.push_back(static_cast<std::uint8_t>((v >>  8) & 0xff));
    out.push_back(static_cast<std::uint8_t>( v        & 0xff));
}

inline void be_float(std::vector<std::uint8_t>& out, float v) {
    std::uint32_t u;
    std::memcpy(&u, &v, sizeof(u));
    be32(out, u);
}

inline void osc_string(std::vector<std::uint8_t>& out, const char* s) {
    std::size_t n = std::strlen(s);
    out.insert(out.end(), s, s + n);
    out.push_back('\0');
    std::size_t total = n + 1;
    std::size_t pad   = (4 - (total % 4)) % 4;
    for (std::size_t i = 0; i < pad; ++i) out.push_back('\0');
}

std::vector<std::uint8_t> make_golden(const ControllerPose& p) {
    std::vector<std::uint8_t> b;
    osc_string(b, "/fitra/controller_pose");
    osc_string(b, ",iiffffffff");
    be32     (b, p.valid ? 1u : 0u);
    be32     (b, static_cast<std::uint32_t>(p.tracking_result));
    be_float (b, p.timestamp_s);
    be_float (b, p.x);
    be_float (b, p.y);
    be_float (b, p.z);
    be_float (b, p.qx);
    be_float (b, p.qy);
    be_float (b, p.qz);
    be_float (b, p.qw);
    return b;
}

// 1) Round-trip parse of a well-formed packet + running_ok gate.
void test_roundtrip() {
    ControllerPose in;
    in.valid           = true;
    in.tracking_result = kTrackingResultRunningOk;
    in.timestamp_s     = 12.5f;
    in.x = 0.25f; in.y = 1.75f; in.z = -0.5f;
    in.qx = 0.0f; in.qy = 0.382683f; in.qz = 0.0f; in.qw = 0.923880f;

    const auto bytes = make_golden(in);
    // address "/fitra/controller_pose" = 22+1=23 -> pad to 24.
    // typetag ",iiffffffff" = 11+1=12 (already aligned). args = 10*4 = 40.
    CHECK(bytes.size() == 24 + 12 + 40);

    ControllerPose out;
    bool ok = parse_controller_pose_packet(bytes.data(), bytes.size(), out);
    CHECK(ok);
    CHECK(out.valid == in.valid);
    CHECK(out.tracking_result == in.tracking_result);
    CHECK(out.running_ok());
    CHECK_NEAR(out.timestamp_s, in.timestamp_s, 1e-6);
    CHECK_NEAR(out.x,  in.x,  1e-6);
    CHECK_NEAR(out.y,  in.y,  1e-6);
    CHECK_NEAR(out.z,  in.z,  1e-6);
    CHECK_NEAR(out.qx, in.qx, 1e-6);
    CHECK_NEAR(out.qy, in.qy, 1e-6);
    CHECK_NEAR(out.qz, in.qz, 1e-6);
    CHECK_NEAR(out.qw, in.qw, 1e-6);
}

// 2) valid but degraded (Running_OutOfRange = 201) must fail running_ok.
void test_degraded_gate() {
    ControllerPose in;
    in.valid           = true;
    in.tracking_result = 201;  // Running_OutOfRange
    const auto bytes = make_golden(in);
    ControllerPose out;
    CHECK(parse_controller_pose_packet(bytes.data(), bytes.size(), out));
    CHECK(out.valid == true);
    CHECK(out.tracking_result == 201);
    CHECK(!out.running_ok());

    // valid=0 + OK tracking_result still fails (both required).
    ControllerPose lost;
    lost.valid = false;
    lost.tracking_result = kTrackingResultRunningOk;
    const auto bytes2 = make_golden(lost);
    ControllerPose out2;
    CHECK(parse_controller_pose_packet(bytes2.data(), bytes2.size(), out2));
    CHECK(!out2.running_ok());
}

// 3) Address mismatch must reject.
void test_address_mismatch() {
    std::vector<std::uint8_t> b;
    osc_string(b, "/fitra/hmd_pose");  // the HMD address, not ours
    osc_string(b, ",iiffffffff");
    for (int i = 0; i < 10; ++i) be32(b, 0);
    ControllerPose out;
    CHECK(!parse_controller_pose_packet(b.data(), b.size(), out));
}

// 4) Typetag mismatch must reject (HMD's one-int typetag).
void test_typetag_mismatch() {
    std::vector<std::uint8_t> b;
    osc_string(b, "/fitra/controller_pose");
    osc_string(b, ",iffffffff");  // missing the second int
    for (int i = 0; i < 9; ++i) be32(b, 0);
    ControllerPose out;
    CHECK(!parse_controller_pose_packet(b.data(), b.size(), out));
}

// 5) Truncated packet must reject. Also test the bus stale judgement.
void test_truncated_and_stale() {
    ControllerPose in;
    in.valid = true;
    in.tracking_result = kTrackingResultRunningOk;
    const auto bytes = make_golden(in);
    ControllerPose out;
    CHECK(!parse_controller_pose_packet(bytes.data(), 50, out));  // header but no full args
    CHECK(!parse_controller_pose_packet(bytes.data(), 4,  out));

    ControllerPoseBus bus;
    auto s0 = bus.snapshot(/*stale_ms*/ 100.0);
    CHECK(s0.have_any == false);
    CHECK(s0.stale    == true);

    ControllerPose p;
    p.valid = true;
    p.tracking_result = kTrackingResultRunningOk;
    p.x = 1.0f;
    bus.publish(p);
    auto s1 = bus.snapshot(100.0);
    CHECK(s1.have_any == true);
    CHECK(s1.stale    == false);
    CHECK(s1.pose.running_ok());
    CHECK_NEAR(s1.pose.x, 1.0, 1e-6);

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    auto s2 = bus.snapshot(100.0);
    CHECK(s2.have_any == true);
    CHECK(s2.stale    == true);
}

}  // namespace

int main() {
    test_roundtrip();
    test_degraded_gate();
    test_address_mismatch();
    test_typetag_mismatch();
    test_truncated_and_stale();
    if (g_fail) {
        std::fprintf(stderr, "test_controller_pose_receiver: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_controller_pose_receiver: OK\n");
    return 0;
}
