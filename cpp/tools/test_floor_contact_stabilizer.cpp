#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "infer/types.hpp"
#include "lift/floor_contact_stabilizer.hpp"
#include "lift/skeleton_def.hpp"

namespace {

using fitra::infer::Skeleton3D;
using fitra::lift::FloorContactOptions;
using fitra::lift::FloorContactStabilizer;

constexpr std::size_t kLKnee = fitra::lift::kHalpeLeftKnee;
constexpr std::size_t kLAnkle = fitra::lift::kHalpeLeftAnkle;
constexpr std::size_t kRAnkle = fitra::lift::kHalpeRightAnkle;
constexpr std::size_t kLBigToe = fitra::lift::kHalpeLeftBigToe;
constexpr std::size_t kRBigToe = fitra::lift::kHalpeRightBigToe;
constexpr std::size_t kLSmallToe = fitra::lift::kHalpeLeftSmallToe;
constexpr std::size_t kRSmallToe = fitra::lift::kHalpeRightSmallToe;
constexpr std::size_t kLHeel = fitra::lift::kHalpeLeftHeel;
constexpr std::size_t kRHeel = fitra::lift::kHalpeRightHeel;
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
    check(r.feet[0].evidence_valid, "contact has current sole evidence");
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
              <= 0.04f + kEps,
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
        FloorContactOptions opts;
        opts.exit_grace_s = 0.0;
        opts.exit_height_m = 0.06;
        FloorContactStabilizer st{opts};
        enter_left_contact(st);
        auto high = skeleton(0.0f, 0.07f);
        auto r = st.update(high, kDt);
        check(!r.feet[0].contact, "exit height releases immediately");
        check(r.feet[0].corrected && r.feet[0].correction_m[2] < 0.0f,
              "released high foot retains a decaying correction");
        check(high.joints[kLBigToe].z > 0.06f
                  && high.joints[kLBigToe].z < 0.07f,
              "release removes the correction gradually, not in one frame");
    }
    {
        FloorContactOptions opts;
        opts.exit_grace_s = 0.0;
        FloorContactStabilizer st{opts};
        enter_left_contact(st);
        auto drift = skeleton(0.01f);
        auto planted = st.update(drift, kDt);
        check(planted.feet[0].contact
                  && planted.feet[0].correction_m[0] < -0.005f,
              "planted frame accumulates a visible XY correction");
        auto fast = skeleton(0.05f);
        auto r = st.update(fast, kDt);
        check(!r.feet[0].contact, "exit speed/XY bound releases immediately");
        check(r.feet[0].correction_m[0] < 0.0f,
              "release frame decays the previous XY correction");
        check(fast.joints[kLAnkle].x < 0.05f,
              "release frame avoids an XY snap to the raw ankle");
    }
    {
        FloorContactOptions opts;
        opts.exit_grace_s = 0.0;
        FloorContactStabilizer st{opts};
        enter_left_contact(st);
        auto fast_low = skeleton(0.05f, 0.005f);
        auto r = st.update(fast_low, kDt);
        check(!r.feet[0].contact, "fast low foot releases contact");
        close(fast_low.joints[kLBigToe].z, 0.0f,
              "release decay cannot pull a sole through the floor");
    }
    {
        FloorContactOptions opts;
        opts.exit_grace_s = 0.0;
        FloorContactStabilizer st{opts};
        enter_left_contact(st);
        auto outlier = skeleton(0.0f, -0.09f);
        auto r = st.update(outlier, kDt);
        check(!r.feet[0].contact, "oversized Z correction releases");
        close(r.feet[0].correction_m[2], 0.08f,
              "oversized penetration is corrected to the configured cap");
        close(outlier.joints[kLBigToe].z, -0.01f,
              "oversized penetration no longer fails open");
    }
}

void test_transient_exit_signal_keeps_contact_latched() {
    FloorContactStabilizer st;
    enter_left_contact(st);

    auto drift = skeleton(0.01f);
    auto planted = st.update(drift, kDt);
    check(planted.feet[0].contact,
          "small planted drift remains in contact");
    const cv::Vec3f prior_correction = planted.feet[0].correction_m;

    auto spike = skeleton(0.06f, -0.02f);
    auto first = st.update(spike, kDt);
    check(first.feet[0].contact,
          "one-frame speed/XY spike starts grace without unlatching");
    close(first.feet[0].correction_m[0], prior_correction[0],
          "exit grace holds the last bounded X correction");
    close(first.feet[0].correction_m[1], prior_correction[1],
          "exit grace holds the last bounded Y correction");
    close(spike.joints[kLBigToe].z, 0.0f,
          "exit grace still prevents floor penetration");

    auto recovered = skeleton(0.01f);
    auto second = st.update(recovered, kDt);
    check(second.feet[0].contact,
          "return edge of a one-frame spike also remains latched");

    auto stable = skeleton(0.01f);
    auto third = st.update(stable, kDt);
    check(third.feet[0].contact,
          "stable evidence clears the pending exit signal");
}

void test_persistent_exit_signal_releases_after_grace() {
    FloorContactStabilizer st;
    enter_left_contact(st);

    for (int frame = 1; frame <= 2; ++frame) {
        auto moving = skeleton(0.06f * static_cast<float>(frame));
        auto report = st.update(moving, kDt);
        check(report.feet[0].contact,
              "exit signal frame " + std::to_string(frame)
                  + " remains inside grace");
    }

    auto moving = skeleton(0.18f);
    auto released = st.update(moving, kDt);
    check(!released.feet[0].contact,
          "persistent exit signal releases at the configured grace time");
}

void test_low_rate_single_exit_sample_keeps_contact_latched() {
    FloorContactStabilizer st;
    enter_left_contact(st);

    auto spike = skeleton(0.0f, 0.09f);
    auto first = st.update(spike, 0.12);
    check(first.feet[0].contact,
          "one high support sample at 8 fps remains inside grace");

    auto moving = skeleton(0.0f, 0.09f);
    auto second = st.update(moving, 0.12);
    check(!second.feet[0].contact,
          "a second low-rate high support sample confirms release");
}

void test_velocity_division_keeps_double_dt_precision() {
    FloorContactStabilizer st;

    auto first = skeleton();
    st.update(first, 1.0e-50);
    auto second = skeleton();
    auto report = st.update(second, 1.0e-50);

    check(report.feet[0].contact,
          "subnormal-for-float dt still produces finite zero speed");
}

void test_contact_z_correction_respects_configured_bound() {
    FloorContactOptions opts;
    opts.exit_height_m = 0.20;
    opts.exit_speed_mps = 100.0;
    FloorContactStabilizer st{opts};
    enter_left_contact(st);

    auto high = skeleton(0.0f, 0.12f);
    auto report = st.update(high, kDt);

    check(report.feet[0].contact,
          "wide exit thresholds keep the configured contact latched");
    close(report.feet[0].correction_m[2], -0.08f,
          "contact downward Z correction is capped at max_z");
    close(high.joints[kLBigToe].z, 0.04f,
          "bounded contact correction cannot pull the sole to the floor");
}

void test_isolated_low_sole_outlier_is_rejected() {
    FloorContactStabilizer st;
    enter_left_contact(st);

    auto s = skeleton();
    s.joints[kLBigToe].z = -0.07f;
    const float ankle_before = s.joints[kLAnkle].z;
    auto r = st.update(s, kDt);

    check(r.feet[0].contact, "one isolated low sole does not break the latch");
    check(r.feet[0].sole_outlier_rejected
              && !s.joints[kLBigToe].valid,
          "isolated below-floor sole is removed from downstream geometry");
    check(std::abs(r.feet[0].correction_m[2]) < 0.02f,
          "one isolated low sole cannot lift the whole foot by centimetres");
    check(std::abs(s.joints[kLAnkle].z - ankle_before) < 0.02f,
          "ankle translation uses robust sole support");
}

void test_release_correction_converges_to_zero() {
    FloorContactStabilizer st;
    enter_left_contact(st);
    auto drift = skeleton(0.01f);
    st.update(drift, kDt);

    float previous_abs = 1.0f;
    for (int frame = 0; frame < 30; ++frame) {
        auto moving = skeleton(0.05f, 0.07f);
        auto r = st.update(moving, kDt);
        const float current_abs = std::abs(r.feet[0].correction_m[0]);
        check(current_abs <= previous_abs + kEps,
              "release XY correction decays monotonically");
        previous_abs = current_abs;
    }
    check(previous_abs < 1.0e-4f,
          "release correction converges to zero in bounded time");
}

void test_missing_grace_then_release() {
    FloorContactStabilizer st;
    enter_left_contact(st);

    for (int frame = 1; frame <= 4; ++frame) {
        auto s = skeleton();
        s.joints[kLSmallToe].valid = false;
        s.joints[kLHeel].valid = false;  // only one sole point remains
        auto r = st.update(s, kDt);
        check(r.feet[0].contact && r.feet[0].missing_grace,
              "missing frame " + std::to_string(frame) + " stays latched");
        check(!r.feet[0].evidence_valid,
              "missing grace is not counted as current contact evidence");
        check(r.feet[0].corrected,
              "missing grace reapplies the last rigid correction");
        close(s.joints[kLBigToe].z, 0.0f,
              "missing grace keeps the remaining sole point grounded");
    }

    auto after_grace = skeleton();
    after_grace.joints[kLSmallToe].valid = false;
    after_grace.joints[kLHeel].valid = false;
    auto released = st.update(after_grace, kDt);
    check(!released.feet[0].contact && released.feet[0].corrected,
          "missing evidence releases after the enlarged default grace");
}

void test_full_dropout_reports_no_applied_correction() {
    FloorContactStabilizer st;
    enter_left_contact(st);
    auto missing = skeleton();
    missing.joints[kLAnkle].valid = false;
    missing.joints[kLBigToe].valid = false;
    missing.joints[kLSmallToe].valid = false;
    missing.joints[kLHeel].valid = false;
    auto r = st.update(missing, kDt);
    check(r.feet[0].contact && r.feet[0].missing_grace,
          "full dropout remains in state-machine grace");
    check(!r.feet[0].corrected && cv::norm(r.feet[0].correction_m) == 0.0,
          "full dropout reports zero because no joint was actually corrected");
}

void test_full_dropout_clears_velocity_history() {
    FloorContactStabilizer st;
    enter_left_contact(st);

    for (int frame = 0; frame < 5; ++frame) {
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
    auto low_rate = skeleton();
    check(st2.update(low_rate, 0.12).feet[0].contact,
          "default reset gap supports measured rates below 10 fps");

    FloorContactOptions opts;
    opts.reset_dt_s = 0.10;
    FloorContactStabilizer st3{opts};
    enter_left_contact(st3);
    auto after_gap = skeleton();
    check(!st3.update(after_gap, 0.2).feet[0].contact,
          "configured long dt drops latch");
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
        test_transient_exit_signal_keeps_contact_latched();
        test_persistent_exit_signal_releases_after_grace();
        test_low_rate_single_exit_sample_keeps_contact_latched();
        test_velocity_division_keeps_double_dt_precision();
        test_contact_z_correction_respects_configured_bound();
        test_isolated_low_sole_outlier_is_rejected();
        test_release_correction_converges_to_zero();
        test_missing_grace_then_release();
        test_full_dropout_reports_no_applied_correction();
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
