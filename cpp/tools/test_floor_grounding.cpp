// test_floor_grounding — floor-contact grounding (spatial-filtering M-D).
//
// Pins apply_floor_grounding (docs/design/pose-3d-floor-grounding.md):
//   1. below-floor clamp — a sole point at z<floor snaps to floor (stateless)
//   2. above-floor point is untouched
//   3. stance snap — a near-floor, low-speed point plants on the floor (Z only)
//   4. swing — a near-floor but FAST point is NOT snapped
//   5. COCO17 (kp_count=17, no toe/heel slots) is a no-op
//   6. floor_z offset respected
//   7. invalid sole joint skipped (+ prev anchor dropped)
//   8. the ANKLE (leg joint) is NOT grounded — only sole points
//   9. idempotent

#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

#include "infer/types.hpp"
#include "lift/floor_grounding.hpp"

namespace {

constexpr float kEps = 1.0e-5f;
using fitra::infer::Skeleton3D;
using fitra::lift::FloorGroundingOptions;
using fitra::lift::FloorGroundingState;
using fitra::lift::apply_floor_grounding;

// Halpe26 indices.
constexpr std::size_t kLAnkle = 15, kLBigToe = 20, kLHeel = 24;
constexpr double kDt = 1.0 / 60.0;

void check(bool cond, const std::string& msg) {
    if (!cond) throw std::runtime_error(msg);
}
void check_close(float got, float want, const std::string& label, float eps = kEps) {
    if (std::abs(got - want) > eps) {
        char b[200];
        std::snprintf(b, sizeof(b), "%s: got=%.6f want=%.6f", label.c_str(), got, want);
        throw std::runtime_error(b);
    }
}

Skeleton3D halpe_skel() {
    Skeleton3D s;
    s.kp_count = 26;
    return s;
}
void set_joint(Skeleton3D& s, std::size_t j, float x, float y, float z, bool valid = true) {
    s.joints[j].x = x; s.joints[j].y = y; s.joints[j].z = z; s.joints[j].valid = valid;
}

void test_below_floor_clamp() {
    Skeleton3D s = halpe_skel();
    set_joint(s, kLBigToe, 0.1f, 0.2f, -0.03f);  // 30mm below floor
    FloorGroundingState st;
    bool mod = apply_floor_grounding(s, st, kDt, FloorGroundingOptions{});
    check(mod, "clamp should report modified");
    check_close(s.joints[kLBigToe].z, 0.0f, "below-floor clamp to 0");
    check_close(s.joints[kLBigToe].x, 0.1f, "clamp leaves X");
    check_close(s.joints[kLBigToe].y, 0.2f, "clamp leaves Y");
}

void test_above_floor_untouched() {
    Skeleton3D s = halpe_skel();
    set_joint(s, kLBigToe, 0.0f, 0.0f, 0.20f);  // 20cm up (mid-swing / lifted)
    FloorGroundingState st;
    apply_floor_grounding(s, st, kDt, FloorGroundingOptions{});
    check_close(s.joints[kLBigToe].z, 0.20f, "above-floor untouched");
}

void test_stance_snap() {
    FloorGroundingOptions o;  // band 0.03, stance_vel 0.15
    FloorGroundingState st;
    // Frame 1: near-floor (15mm), establishes prev. z>=0 so no clamp; not yet
    // snapped (has_prev=false → speed unknown).
    Skeleton3D s1 = halpe_skel();
    set_joint(s1, kLBigToe, 0.0f, 0.0f, 0.015f);
    apply_floor_grounding(s1, st, kDt, o);
    check_close(s1.joints[kLBigToe].z, 0.015f, "frame1 no snap (no prev)");
    // Frame 2: same position (speed ≈ 0 < stance_vel) & in band → snap to floor.
    Skeleton3D s2 = halpe_skel();
    set_joint(s2, kLBigToe, 0.0f, 0.0f, 0.015f);
    bool mod = apply_floor_grounding(s2, st, kDt, o);
    check(mod, "stance snap should report modified");
    check_close(s2.joints[kLBigToe].z, 0.0f, "stance snap to floor");
}

void test_swing_not_snapped() {
    FloorGroundingOptions o;
    FloorGroundingState st;
    Skeleton3D s1 = halpe_skel();
    set_joint(s1, kLBigToe, 0.0f, 0.0f, 0.015f);
    apply_floor_grounding(s1, st, kDt, o);  // prev
    // Frame 2: near-floor but moved 20mm in one tick → ~1.2 m/s ≫ stance_vel.
    Skeleton3D s2 = halpe_skel();
    set_joint(s2, kLBigToe, 0.02f, 0.0f, 0.015f);
    apply_floor_grounding(s2, st, kDt, o);
    check_close(s2.joints[kLBigToe].z, 0.015f, "fast (swing) not snapped");
}

void test_coco17_noop() {
    Skeleton3D s;                     // default kp_count = 17
    set_joint(s, kLBigToe, 0.0f, 0.0f, -0.03f);  // slot exists in storage but > kp_count
    FloorGroundingState st;
    bool mod = apply_floor_grounding(s, st, kDt, FloorGroundingOptions{});
    check(!mod, "COCO17 must be a no-op");
    check_close(s.joints[kLBigToe].z, -0.03f, "COCO17 leaves slot 20 untouched");
}

void test_floor_z_offset() {
    FloorGroundingOptions o; o.floor_z_m = 0.1;
    Skeleton3D s = halpe_skel();
    set_joint(s, kLHeel, 0.0f, 0.0f, 0.05f);  // below the raised floor
    FloorGroundingState st;
    apply_floor_grounding(s, st, kDt, o);
    check_close(s.joints[kLHeel].z, 0.1f, "clamp to offset floor_z");
}

void test_invalid_skipped() {
    Skeleton3D s = halpe_skel();
    set_joint(s, kLBigToe, 0.0f, 0.0f, -0.03f, /*valid=*/false);
    FloorGroundingState st;
    bool mod = apply_floor_grounding(s, st, kDt, FloorGroundingOptions{});
    check(!mod, "invalid joint not modified");
    check_close(s.joints[kLBigToe].z, -0.03f, "invalid joint untouched");
    check(!st.has_prev[kLBigToe], "invalid joint drops prev anchor");
}

void test_ankle_not_grounded() {
    Skeleton3D s = halpe_skel();
    set_joint(s, kLAnkle, 0.0f, 0.0f, -0.03f);  // ankle below floor (contrived)
    FloorGroundingState st;
    apply_floor_grounding(s, st, kDt, FloorGroundingOptions{});
    check_close(s.joints[kLAnkle].z, -0.03f, "ankle is NOT a sole point → untouched");
}

void test_idempotent() {
    Skeleton3D s = halpe_skel();
    set_joint(s, kLBigToe, 0.0f, 0.0f, -0.05f);
    FloorGroundingState st;
    apply_floor_grounding(s, st, kDt, FloorGroundingOptions{});
    const float z1 = s.joints[kLBigToe].z;
    bool mod2 = apply_floor_grounding(s, st, kDt, FloorGroundingOptions{});
    check_close(s.joints[kLBigToe].z, z1, "idempotent clamp");
    check(!mod2, "second apply on already-grounded static point is a no-op");
}

}  // namespace

int main() {
    try {
        test_below_floor_clamp();
        test_above_floor_untouched();
        test_stance_snap();
        test_swing_not_snapped();
        test_coco17_noop();
        test_floor_z_offset();
        test_invalid_skipped();
        test_ankle_not_grounded();
        test_idempotent();
        std::puts("test_floor_grounding ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_floor_grounding failed: %s\n", e.what());
        return 1;
    }
}
