// Golden-bytes unit tests for /fitra/tracked_pose parser and role dispatch.

#include "vmt/tracked_pose_receiver.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

using fitra::vmt::ControllerPose;
using fitra::vmt::ControllerPoseBus;
using fitra::vmt::HmdPose;
using fitra::vmt::HmdPoseBus;
using fitra::vmt::TrackedPose;
using fitra::vmt::TrackedPoseRole;
using fitra::vmt::kTrackingResultRunningOk;
using fitra::vmt::parse_tracked_pose_packet;
using fitra::vmt::parse_tracked_pose_role;

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
    std::size_t pad = (4 - (total % 4)) % 4;
    for (std::size_t i = 0; i < pad; ++i) out.push_back('\0');
}

std::vector<std::uint8_t> make_tracked(const TrackedPose& p) {
    std::vector<std::uint8_t> b;
    osc_string(b, "/fitra/tracked_pose");
    osc_string(b, ",iiiiffffffff");
    be32(b, static_cast<std::uint32_t>(static_cast<std::int32_t>(p.role)));
    be32(b, static_cast<std::uint32_t>(p.device_index));
    be32(b, p.valid ? 1u : 0u);
    be32(b, static_cast<std::uint32_t>(p.tracking_result));
    be_float(b, p.timestamp_s);
    be_float(b, p.x);
    be_float(b, p.y);
    be_float(b, p.z);
    be_float(b, p.qx);
    be_float(b, p.qy);
    be_float(b, p.qz);
    be_float(b, p.qw);
    return b;
}

std::vector<std::uint8_t> make_hmd_legacy(const HmdPose& p) {
    std::vector<std::uint8_t> b;
    osc_string(b, "/fitra/hmd_pose");
    osc_string(b, ",iffffffff");
    be32(b, p.valid ? 1u : 0u);
    be_float(b, p.timestamp_s);
    be_float(b, p.x);
    be_float(b, p.y);
    be_float(b, p.z);
    be_float(b, p.qx);
    be_float(b, p.qy);
    be_float(b, p.qz);
    be_float(b, p.qw);
    return b;
}

std::vector<std::uint8_t> make_controller_legacy(const ControllerPose& p) {
    std::vector<std::uint8_t> b;
    osc_string(b, "/fitra/controller_pose");
    osc_string(b, ",iiffffffff");
    be32(b, p.valid ? 1u : 0u);
    be32(b, static_cast<std::uint32_t>(p.tracking_result));
    be_float(b, p.timestamp_s);
    be_float(b, p.x);
    be_float(b, p.y);
    be_float(b, p.z);
    be_float(b, p.qx);
    be_float(b, p.qy);
    be_float(b, p.qz);
    be_float(b, p.qw);
    return b;
}

std::vector<std::uint8_t> make_bundle(const std::vector<std::vector<std::uint8_t>>& elems) {
    std::vector<std::uint8_t> b;
    b.insert(b.end(), {'#','b','u','n','d','l','e','\0'});
    for (int i = 0; i < 8; ++i) b.push_back(0);  // timetag
    for (const auto& e : elems) {
        be32(b, static_cast<std::uint32_t>(e.size()));
        b.insert(b.end(), e.begin(), e.end());
    }
    return b;
}

void test_tracked_roundtrip() {
    TrackedPose in;
    in.role = TrackedPoseRole::RightController;
    in.device_index = 7;
    in.valid = true;
    in.tracking_result = kTrackingResultRunningOk;
    in.timestamp_s = 3.25f;
    in.x = 0.1f; in.y = 1.2f; in.z = -0.3f;
    in.qx = 0.0f; in.qy = 0.5f; in.qz = 0.0f; in.qw = 0.866025f;

    const auto bytes = make_tracked(in);
    TrackedPose out;
    CHECK(parse_tracked_pose_packet(bytes.data(), bytes.size(), out));
    CHECK(out.role == in.role);
    CHECK(out.device_index == in.device_index);
    CHECK(out.running_ok());
    CHECK_NEAR(out.timestamp_s, in.timestamp_s, 1e-6);
    CHECK_NEAR(out.x, in.x, 1e-6);
    CHECK_NEAR(out.qw, in.qw, 1e-6);
}

void test_role_parser() {
    TrackedPoseRole role{};
    CHECK(parse_tracked_pose_role("left", role));
    CHECK(role == TrackedPoseRole::LeftController);
    CHECK(parse_tracked_pose_role("right_controller", role));
    CHECK(role == TrackedPoseRole::RightController);
    CHECK(!parse_tracked_pose_role("controller3", role));
}

void test_bundle_dispatch() {
    HmdPoseBus hmd_bus;
    ControllerPoseBus controller_bus;
    fitra::vmt::TrackedPoseReceiverOptions opts;
    opts.controller_role = TrackedPoseRole::RightController;
    fitra::vmt::TrackedPoseReceiver rx(hmd_bus, controller_bus, opts);

    TrackedPose hmd;
    hmd.role = TrackedPoseRole::Hmd;
    hmd.valid = true;
    hmd.tracking_result = kTrackingResultRunningOk;
    hmd.timestamp_s = 1.0f;
    hmd.x = 0.4f;

    TrackedPose left;
    left.role = TrackedPoseRole::LeftController;
    left.valid = true;
    left.tracking_result = kTrackingResultRunningOk;
    left.x = 9.0f;

    TrackedPose right;
    right.role = TrackedPoseRole::RightController;
    right.valid = true;
    right.tracking_result = kTrackingResultRunningOk;
    right.x = 2.0f;

    const auto bundle = make_bundle({make_tracked(hmd), make_tracked(left), make_tracked(right)});
    CHECK(rx.ingest_packet(bundle.data(), bundle.size()));

    auto hs = hmd_bus.snapshot(100.0);
    CHECK(hs.have_any);
    CHECK(hs.pose.valid);
    CHECK_NEAR(hs.pose.x, 0.4f, 1e-6);

    auto cs = controller_bus.snapshot(100.0);
    CHECK(cs.have_any);
    CHECK(cs.pose.running_ok());
    CHECK_NEAR(cs.pose.x, 2.0f, 1e-6);

    auto stats = rx.stats();
    CHECK(stats.packets_total == 1);
    CHECK(stats.tracked_pose_messages == 3);
}

void test_legacy_parse_still_works() {
    HmdPose h;
    h.valid = true;
    h.timestamp_s = 2.0f;
    h.x = 1.0f;
    const auto hb = make_hmd_legacy(h);
    HmdPose h2;
    CHECK(fitra::vmt::parse_hmd_pose_packet(hb.data(), hb.size(), h2));
    CHECK(h2.valid);
    CHECK_NEAR(h2.x, 1.0f, 1e-6);

    ControllerPose c;
    c.valid = true;
    c.tracking_result = kTrackingResultRunningOk;
    c.timestamp_s = 2.0f;
    c.x = 3.0f;
    const auto cb = make_controller_legacy(c);
    ControllerPose c2;
    CHECK(fitra::vmt::parse_controller_pose_packet(cb.data(), cb.size(), c2));
    CHECK(c2.running_ok());
    CHECK_NEAR(c2.x, 3.0f, 1e-6);
}

}  // namespace

int main() {
    test_tracked_roundtrip();
    test_role_parser();
    test_bundle_dispatch();
    test_legacy_parse_still_works();
    if (g_fail) {
        std::fprintf(stderr, "test_tracked_pose_receiver: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_tracked_pose_receiver: OK\n");
    return 0;
}
