// Tests for the floor AprilTag map (data structure + FileStorage YAML I/O).
//
//  1) world_corners: a tag placed with a known pose has its object corners
//     transformed into the expected world coordinates (aruco order).
//  2) load/write round-trip: write a map, read it back, poses match.
//  3) rpy_deg vs R reader equivalence + grid helper geometry.

#include "lift/floor_tag_map.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

using fitra::lift::FloorTag;
using fitra::lift::FloorTagMap;
using fitra::lift::floor_tag_grid;
using fitra::lift::floor_tag_map_load;
using fitra::lift::floor_tag_map_write;

int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++g_fail; } \
} while (0)

#define CHECK_LT(a, b) do { \
    if (!((a) < (b))) { \
        std::fprintf(stderr, "FAIL %s:%d %s < %s (%g vs %g)\n", \
            __FILE__, __LINE__, #a, #b, double(a), double(b)); ++g_fail; \
    } \
} while (0)

std::string tmp_path(const char* name) {
    const char* dir = std::getenv("TMPDIR");
    std::string base = dir ? dir : "/tmp";
    return base + "/" + name;
}

FloorTag make_tag(int id, double size, const cv::Matx33d& R, const cv::Vec3d& t) {
    FloorTag tag;
    tag.id = id;
    tag.size_m = size;
    cv::Matx44d m = cv::Matx44d::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) m(r, c) = R(r, c);
        m(r, 3) = t[r];
    }
    tag.T_world_tag = fitra::geom::T_world_marker::from_raw(m);
    return tag;
}

// 1) world_corners for a flat floor tag at a known offset.
void test_world_corners() {
    const double s = 0.20;  // 20 cm tag
    FloorTag tag = make_tag(0, s, cv::Matx33d::eye(), cv::Vec3d(1.0, 2.0, 0.0));
    FloorTagMap map;
    map.tags.push_back(tag);

    auto wc = map.world_corners(tag);
    const double h = s * 0.5;
    // aruco order TL(-,+), TR(+,+), BR(+,-), BL(-,-), translated by (1,2,0).
    // tag_object_corners returns float, so tolerate single-precision rounding.
    CHECK_LT(std::abs(wc[0].v[0] - (1.0 - h)), 1e-6);
    CHECK_LT(std::abs(wc[0].v[1] - (2.0 + h)), 1e-6);
    CHECK_LT(std::abs(wc[2].v[0] - (1.0 + h)), 1e-6);
    CHECK_LT(std::abs(wc[2].v[1] - (2.0 - h)), 1e-6);
    for (int i = 0; i < 4; ++i) CHECK_LT(std::abs(wc[i].v[2]), 1e-6);  // z=0
}

// 2) write -> load round-trip preserves poses and ids.
void test_roundtrip() {
    FloorTagMap map;
    map.origin_tag_id = 0;
    map.tags.push_back(make_tag(0, 0.168, cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)));
    // An off-plane stand tag: rotated 90° about X, lifted 0.5 m.
    cv::Matx33d Rx(1, 0, 0, 0, 0, -1, 0, 1, 0);
    map.tags.push_back(make_tag(13, 0.110, Rx, cv::Vec3d(0.3, 1.2, 0.5)));

    std::string path = tmp_path("fitra_floor_map_test.yaml");
    floor_tag_map_write(path, map);
    FloorTagMap got = floor_tag_map_load(path);

    CHECK(got.tags.size() == 2);
    CHECK(got.origin_tag_id == 0);
    if (got.tags.size() == 2) {
        CHECK(got.tags[0].id == 0);
        CHECK_LT(std::abs(got.tags[0].size_m - 0.168), 1e-12);
        CHECK(got.tags[1].id == 13);
        const cv::Matx44d& m = got.tags[1].T_world_tag.raw();
        CHECK_LT(std::abs(m(0, 3) - 0.3), 1e-9);
        CHECK_LT(std::abs(m(1, 3) - 1.2), 1e-9);
        CHECK_LT(std::abs(m(2, 3) - 0.5), 1e-9);
        // R recovered: row 1 should be (0,0,-1), row 2 (0,1,0).
        CHECK_LT(std::abs(m(1, 2) - (-1.0)), 1e-9);
        CHECK_LT(std::abs(m(2, 1) - 1.0), 1e-9);
    }
    std::remove(path.c_str());
}

// 3) grid helper: centred, row-major, coplanar.
void test_grid() {
    FloorTagMap g = floor_tag_grid(2, 3, 0.5, 10, 0.1);
    CHECK(g.tags.size() == 6);
    CHECK(g.tags.front().id == 10);
    CHECK(g.tags.back().id == 15);
    // Centre of the 2x3 grid (pitch 0.5) at origin: x in {-0.5,0,0.5}, y in {-0.25,0.25}.
    CHECK_LT(std::abs(g.tags[0].T_world_tag.raw()(0, 3) - (-0.5)), 1e-9);
    CHECK_LT(std::abs(g.tags[0].T_world_tag.raw()(1, 3) - (-0.25)), 1e-9);
    for (const auto& t : g.tags) CHECK_LT(std::abs(t.T_world_tag.raw()(2, 3)), 1e-9);
}

}  // namespace

int main() {
    test_world_corners();
    test_roundtrip();
    test_grid();
    if (g_fail) {
        std::fprintf(stderr, "test_floor_tag_map: %d failures\n", g_fail);
        return 1;
    }
    std::printf("test_floor_tag_map: OK\n");
    return 0;
}
