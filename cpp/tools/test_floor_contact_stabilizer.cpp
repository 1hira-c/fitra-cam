#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "infer/types.hpp"
#include "lift/floor_contact_stabilizer.hpp"

namespace {

using fitra::infer::Skeleton3D;
using fitra::lift::FloorContactOptions;
using fitra::lift::FloorContactStabilizer;

constexpr std::size_t kLKnee = 13;
constexpr std::size_t kLAnkle = 15;
constexpr std::size_t kRAnkle = 16;
constexpr std::size_t kLBigToe = 20;
constexpr std::size_t kRBigToe = 21;
constexpr std::size_t kLSmallToe = 22;
constexpr std::size_t kRSmallToe = 23;
constexpr std::size_t kLHeel = 24;
constexpr std::size_t kRHeel = 25;
constexpr double kDt = 1.0 / 60.0;
constexpr float kEps = 1.0e-5f;

void check(bool cond, const std::string& msg) {
    if (!cond) throw std::runtime_error(msg);
}

void close(float got, float want, const std::string& msg, float eps = kEps) {
    if (std::abs(got - want) > eps) {
        throw std::runtime_error(msg + ": got=" + std::to_string(got)
                                 + " want=" + std::to_string(want));
    }
}

void set_joint(Skeleton3D& s, std::size_t j,
               float x, float y, float z, bool valid = true) {
    auto& p = s.joints[j];
    p.x = x; p.y = y; p.z = z; p.score = valid ? 1.0f : 0.0f; p.valid = valid;
}

Skeleton3D skeleton(float left_x = 0.0f,
                    float left_floor_z = 0.01f,
                    float right_floor_z = 0.20f) {
    Skeleton3D s;
    s.kp_count = 26;
    set_joint(s, kLKnee, left_x, 0.0f, 0.48f);
    set_joint(s, kLAnkle, left_x, 0.0f, 0.08f + left_floor_z);
    set_joint(s, kLBigToe, left_x, 0.18f, left_floor_z);
    set_joint(s, kLSmallToe, left_x + 0.03f, 0.17f, left_floor_z + 0.002f);
    set_joint(s, kLHeel, left_x, -0.04f, left_floor_z + 0.004f);

    set_joint(s, kRAnkle, 0.30f, 0.0f, 0.08f + right_floor_z);
    set_joint(s, kRBigToe, 0.30f, 0.18f, right_floor_z);
    set_joint(s, kRSmallToe, 0.33f, 0.17f, right_floor_z + 0.002f);
    set_joint(s, kRHeel, 0.30f, -0.04f, right_floor_z + 0.004f);
    return s;
}

void enter_left_contact(FloorContactStabilizer& st) {
    auto a = skeleton();
    auto r1 = st.update(a, kDt);
    check(!r1.feet[0].contact, "first frame only seeds velocity");
    auto b = skeleton();
    auto r2 = st.update(b, kDt);
    check(r2.feet[0].contact, "second slow near-floor frame enters contact");
}

void test_contact_rigid_translation_and_side_independence() {
    FloorContactStabilizer st;
    enter_left_contact(st);

    auto s = skeleton(0.002f);
    const auto ankle_before = s.joints[kLAnkle];
    const auto toe_before = s.joints[kLBigToe];
    const auto knee_before = s.joints[kLKnee];
    const auto right_before = s.joints[kRAnkle];
    auto r = st.update(s, kDt);

    check(r.feet[0].contact && r.feet[0].corrected, "left contact corrects foot");
    check(!r.feet[1].contact && !r.feet[1].corrected, "right airborne foot untouched");
    close(s.joints[kLBigToe].z, 0.0f, "lowest sole is on floor");
    close(s.joints[kLAnkle].x - ankle_before.x,
          s.joints[kLBigToe].x - toe_before.x,
          "ankle and toe receive same X translation");
    close(s.joints[kLAnkle].z - ankle_before.z,
          s.joints[kLBigToe].z - toe_before.z,
          "ankle and toe receive same Z translation");
    close(s.joints[kLKnee].x, knee_before.x, "knee X is untouched");
    close(s.joints[kLKnee].z, knee_before.z, "knee Z is untouched");
    close(s.joints[kRAnkle].z, right_before.z, "other ankle is untouched");
}

void test_planted_xy_jitter_is_attenuated() {
    FloorContactStabilizer st;
    enter_left_contact(st);

    constexpr float kRawJitter = 0.01f;
    auto s = skeleton(kRawJitter);
    auto r = st.update(s, kDt);

    check(r.feet[0].contact, "bounded XY jitter keeps contact latched");
    check(std::abs(s.joints[kLAnkle].x) < kRawJitter * 0.2f,
          "planted ankle XY jitter is attenuated by at least 80 percent");
    check(std::hypot(r.feet[0].correction_m[0], r.feet[0].correction_m[1])
              <= 0.03f + kEps,
          "XY correction stays within the configured safety bound");
}

void test_stateless_penetration_clamp() {
    FloorContactStabilizer st;
    auto s = skeleton(0.0f, -0.02f);
    const float ankle_z = s.joints[kLAnkle].z;
    auto r = st.update(s, kDt);
    check(!r.feet[0].contact, "first penetrated frame is not latched");
    check(r.feet[0].corrected, "penetration is corrected without contact history");
    close(s.joints[kLBigToe].z, 0.0f, "penetrated sole clamps to floor");
    close(s.joints[kLAnkle].z, ankle_z + 0.02f, "whole foot lifts rigidly");
}

void test_release_conditions() {
    {
        FloorContactStabilizer st;
        enter_left_contact(st);
        auto high = skeleton(0.0f, 0.07f);
        auto r = st.update(high, kDt);
        check(!r.feet[0].contact, "exit height releases immediately");
        close(high.joints[kLBigToe].z, 0.07f, "released high foot is raw");
    }
    {
        FloorContactStabilizer st;
        enter_left_contact(st);
        auto fast = skeleton(0.05f);
        auto r = st.update(fast, kDt);
        check(!r.feet[0].contact, "exit speed/XY bound releases immediately");
        close(fast.joints[kLAnkle].x, 0.05f, "released fast foot is raw");
    }
    {
        FloorContactStabilizer st;
        enter_left_contact(st);
        auto outlier = skeleton(0.0f, -0.09f);
        auto r = st.update(outlier, kDt);
        check(!r.feet[0].contact, "oversized Z correction releases");
        close(outlier.joints[kLBigToe].z, -0.09f, "oversized Z outlier is fail-open");
    }
}

void test_missing_grace_then_release() {
    FloorContactStabilizer st;
    enter_left_contact(st);

    for (int frame = 1; frame <= 2; ++frame) {
        auto s = skeleton();
        s.joints[kLSmallToe].valid = false;
        s.joints[kLHeel].valid = false;  // only one sole point remains
        auto r = st.update(s, kDt);
        check(r.feet[0].contact && r.feet[0].missing_grace,
              "missing frame " + std::to_string(frame) + " stays latched");
        check(r.feet[0].corrected,
              "missing grace reapplies the last rigid correction");
        close(s.joints[kLBigToe].z, 0.0f,
              "missing grace keeps the remaining sole point grounded");
    }

    auto third = skeleton();
    third.joints[kLSmallToe].valid = false;
    third.joints[kLHeel].valid = false;
    auto r3 = st.update(third, kDt);
    check(!r3.feet[0].contact && !r3.feet[0].corrected,
          "third missing frame releases");
}

void test_full_dropout_clears_velocity_history() {
    FloorContactStabilizer st;
    enter_left_contact(st);

    for (int frame = 0; frame < 3; ++frame) {
        auto missing = skeleton();
        missing.joints[kLAnkle].valid = false;
        missing.joints[kLBigToe].valid = false;
        missing.joints[kLSmallToe].valid = false;
        missing.joints[kLHeel].valid = false;
        st.update(missing, kDt);
    }

    auto recovered = skeleton();
    auto first = st.update(recovered, kDt);
    check(!first.feet[0].contact,
          "first frame after full dropout only reseeds velocity");

    auto stable = skeleton();
    auto second = st.update(stable, kDt);
    check(second.feet[0].contact,
          "second stable frame after dropout may re-enter contact");
}

void test_reset_and_long_dt() {
    FloorContactStabilizer st;
    enter_left_contact(st);
    st.reset();
    auto after_reset = skeleton();
    check(!st.update(after_reset, kDt).feet[0].contact, "explicit reset drops latch");

    FloorContactStabilizer st2;
    enter_left_contact(st2);
    auto after_gap = skeleton();
    check(!st2.update(after_gap, 0.2).feet[0].contact, "long dt drops latch");
}

void test_floor_offset() {
    FloorContactOptions opts;
    opts.floor_z_m = 0.10;
    FloorContactStabilizer st{opts};
    auto s = skeleton(0.0f, 0.08f, 0.30f);
    st.update(s, kDt);
    close(s.joints[kLBigToe].z, 0.10f, "configured floor offset is respected");
}

void test_nonfinite_joint_is_ignored() {
    FloorContactStabilizer st;
    auto first = skeleton();
    first.joints[kLBigToe].z = std::nanf("");
    st.update(first, kDt);

    auto second = skeleton();
    second.joints[kLBigToe].z = std::nanf("");
    auto report = st.update(second, kDt);
    check(report.feet[0].contact,
          "two finite sole points are enough when one point is non-finite");
    check(std::isfinite(report.feet[0].correction_m[0])
              && std::isfinite(report.feet[0].correction_m[1])
              && std::isfinite(report.feet[0].correction_m[2]),
          "non-finite input does not contaminate the correction");
    check(std::isnan(second.joints[kLBigToe].z),
          "non-finite joint is not translated");
}

void test_coco17_noop() {
    Skeleton3D s;
    s.kp_count = 17;
    set_joint(s, kLAnkle, 1.0f, 2.0f, -1.0f);
    set_joint(s, kLBigToe, 1.0f, 2.0f, -1.0f);
    FloorContactStabilizer st;
    auto r = st.update(s, kDt);
    check(!r.feet[0].contact && !r.feet[0].corrected, "COCO17 is no-op");
    close(s.joints[kLAnkle].z, -1.0f, "COCO17 storage remains unchanged");
}

}  // namespace

int main() {
    try {
        test_contact_rigid_translation_and_side_independence();
        test_planted_xy_jitter_is_attenuated();
        test_stateless_penetration_clamp();
        test_release_conditions();
        test_missing_grace_then_release();
        test_full_dropout_clears_velocity_history();
        test_reset_and_long_dt();
        test_floor_offset();
        test_nonfinite_joint_is_ignored();
        test_coco17_noop();
        std::puts("test_floor_contact_stabilizer ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_floor_contact_stabilizer failed: %s\n", e.what());
        return 1;
    }
}
