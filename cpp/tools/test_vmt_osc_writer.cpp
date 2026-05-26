// test_vmt_osc_writer — exercise fitra::vmt::OscWriter against hand-crafted
// byte tables. The fifth case is the `/VMT/Room/Driver` packet shape.
//
// Each test case builds an OSC packet with the writer, then compares byte-by-
// byte against a literal expected buffer constructed from the OSC 1.0 spec.
// This is the only way to catch padding / endianness / typetag-comma bugs
// short of actually running SteamVR + VMT Driver.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include "vmt/osc_writer.hpp"

namespace {

std::string hex_dump(const std::uint8_t* p, std::size_t n) {
    std::string out;
    char tmp[8];
    for (std::size_t i = 0; i < n; ++i) {
        std::snprintf(tmp, sizeof(tmp), "%02x ", p[i]);
        out += tmp;
        if ((i + 1) % 16 == 0) out += "\n";
    }
    return out;
}

void check_bytes(const std::uint8_t* got, std::size_t got_n,
                 const std::uint8_t* want, std::size_t want_n,
                 const std::string& label) {
    if (got_n != want_n || std::memcmp(got, want, got_n) != 0) {
        std::string msg = label + " mismatch\n  want (" + std::to_string(want_n) + " bytes):\n"
                        + hex_dump(want, want_n)
                        + "\n  got  (" + std::to_string(got_n) + " bytes):\n"
                        + hex_dump(got, got_n);
        throw std::runtime_error(msg);
    }
}

void want_str(std::vector<std::uint8_t>& v, const char* s) {
    while (*s) v.push_back(static_cast<std::uint8_t>(*s++));
    v.push_back('\0');
    while (v.size() % 4 != 0) v.push_back('\0');
}
void want_be32(std::vector<std::uint8_t>& v, std::uint32_t x) {
    v.push_back(static_cast<std::uint8_t>((x >> 24) & 0xff));
    v.push_back(static_cast<std::uint8_t>((x >> 16) & 0xff));
    v.push_back(static_cast<std::uint8_t>((x >>  8) & 0xff));
    v.push_back(static_cast<std::uint8_t>( x        & 0xff));
}
void want_be64(std::vector<std::uint8_t>& v, std::uint64_t x) {
    want_be32(v, static_cast<std::uint32_t>(x >> 32));
    want_be32(v, static_cast<std::uint32_t>(x & 0xffffffffULL));
}
void want_float(std::vector<std::uint8_t>& v, float f) {
    std::uint32_t u;
    std::memcpy(&u, &f, sizeof(u));
    want_be32(v, u);
}
void want_int(std::vector<std::uint8_t>& v, std::int32_t i) {
    want_be32(v, static_cast<std::uint32_t>(i));
}

void test_message_with_ints() {
    fitra::vmt::OscWriter w;
    w.clear();
    w.begin_message("/foo");
    w.add_int(1000);
    w.add_int(-1);
    w.add_int(0);
    w.add_int(0x7FFFFFFF);
    w.end_message();

    std::vector<std::uint8_t> want;
    want_str(want, "/foo");
    want_str(want, ",iiii");
    want_be32(want, 1000);
    want_be32(want, static_cast<std::uint32_t>(-1));
    want_be32(want, 0);
    want_be32(want, 0x7FFFFFFF);
    auto got = w.data();
    check_bytes(got.data(), got.size(), want.data(), want.size(), "message_with_ints");
}

void test_message_string_and_float() {
    fitra::vmt::OscWriter w;
    w.clear();
    w.begin_message("/test");
    w.add_string("hello");
    w.add_float(3.14f);
    w.end_message();

    std::vector<std::uint8_t> want;
    want_str(want, "/test");
    want_str(want, ",sf");
    want_str(want, "hello");
    want_float(want, 3.14f);
    auto got = w.data();
    check_bytes(got.data(), got.size(), want.data(), want.size(), "message_string_and_float");
}

void test_bundle_two_messages() {
    fitra::vmt::OscWriter w;
    w.clear();
    w.begin_bundle(0x123456789ABCDEF0ULL);
    w.begin_message("/a");
    w.add_int(7);
    w.end_message();
    w.begin_message("/bb");
    w.add_float(1.5f);
    w.end_message();
    w.end_bundle();

    std::vector<std::uint8_t> want;
    want_str(want, "#bundle");
    want_be64(want, 0x123456789ABCDEF0ULL);

    std::vector<std::uint8_t> m1;
    want_str(m1, "/a");
    want_str(m1, ",i");
    want_be32(m1, 7);
    want_be32(want, static_cast<std::uint32_t>(m1.size()));
    want.insert(want.end(), m1.begin(), m1.end());

    std::vector<std::uint8_t> m2;
    want_str(m2, "/bb");
    want_str(m2, ",f");
    want_float(m2, 1.5f);
    want_be32(want, static_cast<std::uint32_t>(m2.size()));
    want.insert(want.end(), m2.begin(), m2.end());

    auto got = w.data();
    check_bytes(got.data(), got.size(), want.data(), want.size(), "bundle_two_messages");
}

void test_empty_bundle() {
    fitra::vmt::OscWriter w;
    w.clear();
    w.begin_bundle(0);
    w.end_bundle();

    std::vector<std::uint8_t> want;
    want_str(want, "#bundle");
    want_be64(want, 0);
    auto got = w.data();
    check_bytes(got.data(), got.size(), want.data(), want.size(), "empty_bundle");
}

void test_vmt_room_driver_packet_shape() {
    // The actual payload shape we'll send to SteamVR + VMT. This is the
    // highest-value regression target -- if this drifts, VMT silently drops
    // the packet or applies bad coordinates.
    //
    // /VMT/Room/Driver ,iifffffffff index=3 enable=1 timeoffset=0
    //                              x=0.0 y=0.94 z=-0.10
    //                              qx=0 qy=0 qz=0 qw=1
    fitra::vmt::OscWriter w;
    w.clear();
    w.begin_bundle(0);
    w.begin_message("/VMT/Room/Driver");
    w.add_int(3);          // index = Hip
    w.add_int(1);          // enable
    w.add_float(0.0f);     // timeoffset
    w.add_float(0.0f);  w.add_float(0.94f); w.add_float(-0.10f);  // pos
    w.add_float(0.0f);  w.add_float(0.0f);  w.add_float(0.0f);  w.add_float(1.0f);  // quat xyzw
    w.end_message();
    w.end_bundle();

    std::vector<std::uint8_t> want;
    want_str(want, "#bundle");
    want_be64(want, 0);
    std::vector<std::uint8_t> m;
    want_str(m, "/VMT/Room/Driver");
    want_str(m, ",iiffffffff");  // 2 ints + 8 floats
    want_int  (m, 3);
    want_int  (m, 1);
    want_float(m, 0.0f);
    want_float(m, 0.0f);  want_float(m, 0.94f); want_float(m, -0.10f);
    want_float(m, 0.0f);  want_float(m, 0.0f);  want_float(m, 0.0f);  want_float(m, 1.0f);
    want_be32(want, static_cast<std::uint32_t>(m.size()));
    want.insert(want.end(), m.begin(), m.end());

    auto got = w.data();
    check_bytes(got.data(), got.size(), want.data(), want.size(), "vmt_room_driver_packet_shape");
}

}  // namespace

int main() {
    try {
        test_message_with_ints();
        test_message_string_and_float();
        test_bundle_two_messages();
        test_empty_bundle();
        test_vmt_room_driver_packet_shape();
        std::puts("test_vmt_osc_writer ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_vmt_osc_writer failed: %s\n", e.what());
        return 1;
    }
}
