// test_kalman_chain — exercise the kinematic-tree SkeletonKalman.
//
// The chain Kalman runs the root joint (hip_center under Halpe26) as a 6D
// world position+velocity filter, and every other joint as a 6D
// parent-relative offset+velocity filter. Output is world coordinates
// reconstructed via FK (parent_world + offset).
//
// What we verify:
//   1. A root translation propagates to an unobserved child via FK alone —
//      this is the structural fix for the extended-leg locomotion freeze.
//   2. When the child IS observed, the offset state corrects to the
//      measurement so non-rigid configurations (joint moves relative to
//      parent) are tracked.
//   3. After a long missing run the child resets and re-initializes on the
//      next valid measurement.
//   4. A child whose parent has never been observed is skipped (no
//      hallucinated world output, no inverted state).

#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

#include "infer/types.hpp"
#include "lift/head_direction.hpp"
#include "lift/kalman.hpp"
#include "lift/keypoint_format.hpp"

namespace {

constexpr double kEps = 5.0e-4;  // 0.5 mm

void check(bool cond, const std::string& msg) {
    if (!cond) throw std::runtime_error(msg);
}

void check_close(double got, double want, const std::string& label,
                 double eps = kEps) {
    if (std::abs(got - want) > eps) {
        char buf[200];
        std::snprintf(buf, sizeof(buf), "%s: got=%.6f want=%.6f diff=%.6f",
                      label.c_str(), got, want, std::abs(got - want));
        throw std::runtime_error(buf);
    }
}

void set_joint(fitra::infer::Skeleton3D& s, std::size_t i,
               float x, float y, float z, bool valid = true) {
    s.joints[i].x = x;
    s.joints[i].y = y;
    s.joints[i].z = z;
    s.joints[i].score = 1.0f;
    s.joints[i].valid = valid;
}

// Build a synthetic Halpe26 skeleton at rest. Only the joints used by the
// chain tests (hip_center, l_hip, l_knee, l_ankle, neck) are populated;
// the rest stay invalid which is fine — the chain Kalman silently skips
// uninitialized joints.
fitra::infer::Skeleton3D make_skel(float hip_x, float hip_y, float hip_z,
                                    bool include_ankle = true) {
    fitra::infer::Skeleton3D s;
    s.kp_count = 26;
    // hip_center (root)
    set_joint(s, 19, hip_x, hip_y, hip_z);
    // l_hip — child of hip_center, offset (0.1, 0, 0)
    set_joint(s, 11, hip_x + 0.1f, hip_y, hip_z);
    // l_knee — child of l_hip, offset (0, 0, -0.45)
    set_joint(s, 13, hip_x + 0.1f, hip_y, hip_z - 0.45f);
    // l_ankle — child of l_knee, offset (0, 0, -0.45)
    set_joint(s, 15, hip_x + 0.1f, hip_y, hip_z - 0.90f, include_ankle);
    // neck — child of hip_center, offset (0, 0, +0.55) — used in a couple
    // of cross-chain tests
    set_joint(s, 18, hip_x, hip_y, hip_z + 0.55f);
    return s;
}

// Run a few "settle" frames so all joints in the chain become initialized.
void settle(fitra::lift::SkeletonKalman& kf) {
    const double dt = 1.0 / 60.0;
    auto skel = make_skel(0.0f, 0.0f, 0.9f);
    for (int i = 0; i < 5; ++i) (void)kf.update(skel, dt);
}

// ---------- Test 1: root motion propagates to unobserved child ----------
void test_chain_propagates_root_motion_to_unobserved_child() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    fitra::lift::SkeletonKalman kf;
    settle(kf);

    // Frame N+1: hip translates by +1 m in X. The whole chain is observed
    // (so the system can stay in valid state), then one frame later we drop
    // the ankle but keep moving the hip.
    const double dt = 1.0 / 60.0;
    auto skel_move1 = make_skel(0.5f, 0.0f, 0.9f);
    (void)kf.update(skel_move1, dt);

    // Now hip moves further, ankle drops out.
    auto skel_move2 = make_skel(1.0f, 0.0f, 0.9f, /*include_ankle=*/false);
    auto out = kf.update(skel_move2, dt);

    // The ankle should still be valid (was initialized earlier) and its
    // world X should track the hip's movement via FK (parent_world +
    // offset). With root + offset chain, hip_x = 1.0 ⇒ l_hip_x ≈ 1.1 ⇒
    // l_knee_x ≈ 1.1 ⇒ l_ankle_x ≈ 1.1.
    check(out.joints[15].valid, "chain.left_ankle still valid after drop");
    // Allow some Kalman slack — root EMA hasn't fully snapped to measurement.
    check(out.joints[15].x > 0.7f,
          "chain.left_ankle world x must follow hip: got " +
              std::to_string(out.joints[15].x));
}

// ---------- Test 2: child measurement corrects offset ----------
void test_chain_corrects_offset_when_child_observed() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    fitra::lift::SkeletonKalman kf;
    settle(kf);

    const double dt = 1.0 / 60.0;
    // Keep hip at origin but move the ankle (e.g. knee bends, ankle lifts).
    // After settling at the rest pose, push ankle world Z up by 0.5 m for
    // a few frames and verify the output tracks it.
    for (int i = 0; i < 20; ++i) {
        auto skel = make_skel(0.0f, 0.0f, 0.9f);
        set_joint(skel, 15, 0.1f, 0.0f, 0.4f);  // ankle lifted (world z = 0.4)
        auto out = kf.update(skel, dt);
        (void)out;
    }
    auto skel = make_skel(0.0f, 0.0f, 0.9f);
    set_joint(skel, 15, 0.1f, 0.0f, 0.4f);
    auto out = kf.update(skel, dt);

    // Ankle world z should be near the new measurement.
    check_close(out.joints[15].z, 0.4f, "chain.ankle.world.z tracks measurement", 0.02);
}

// ---------- Test 3: recovery after long missing run ----------
void test_chain_recovers_after_long_missing() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    fitra::lift::SkeletonKalman::Options opts;
    opts.reset_after_missing = 5;
    fitra::lift::SkeletonKalman kf(opts);
    settle(kf);

    const double dt = 1.0 / 60.0;
    // Drop the ankle for 10 frames (> reset_after_missing).
    for (int i = 0; i < 10; ++i) {
        auto skel = make_skel(0.0f, 0.0f, 0.9f, /*include_ankle=*/false);
        (void)kf.update(skel, dt);
    }
    // After reset, the next valid frame must re-initialize. World pos
    // should match the new measurement (no stale state).
    auto skel = make_skel(0.0f, 0.0f, 0.9f);
    set_joint(skel, 15, 0.2f, 0.3f, 0.5f);  // ankle at a new location
    auto out = kf.update(skel, dt);
    check(out.joints[15].valid, "chain.ankle.recover.valid");
    check_close(out.joints[15].x, 0.2f, "chain.ankle.recover.x");
    check_close(out.joints[15].y, 0.3f, "chain.ankle.recover.y");
    check_close(out.joints[15].z, 0.5f, "chain.ankle.recover.z");
}

// ---------- Test 4b: child ages while parent unavailable ----------
//
// Regression for codex review: when a child is skipped because its parent
// has reset / become uninitialized, the child's missing counter still
// needs to advance so a long parent dropout drops the stale child offset.
// Without this, a child kept "frozen alive" can re-emerge with a stale
// offset the moment the parent reappears.
void test_chain_child_ages_when_parent_skipped() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    fitra::lift::SkeletonKalman::Options opts;
    opts.reset_after_missing = 3;
    fitra::lift::SkeletonKalman kf(opts);
    settle(kf);

    const double dt = 1.0 / 60.0;

    // Drop the hip_center for long enough that the root state resets.
    // While the root is missing, ALL children are skipped on the
    // "parent uninitialized" path, so their missing counters must
    // advance there too.
    for (int i = 0; i < 10; ++i) {
        fitra::infer::Skeleton3D skel;
        skel.kp_count = 26;
        for (auto& j : skel.joints) j.valid = false;
        // No joints valid → root resets after 3 frames, then children
        // age on the "parent uninitialized" path.
        (void)kf.update(skel, dt);
    }

    // Re-establish the root only, with the body translated 2 m in X.
    // If the child ankle state still held a stale offset learned at the
    // origin, the next ankle measurement would only nudge it slightly,
    // so the output ankle would NOT jump to the new measured world
    // position. The fix forces a clean re-init.
    auto skel_recover = make_skel(2.0f, 0.0f, 0.9f);
    auto out = kf.update(skel_recover, dt);

    // Ankle world x should be at the new measurement, not somewhere
    // between the old (~0.1) and new (~2.1) due to stale state.
    check(out.joints[15].valid, "chain.age.ankle valid after re-init");
    check_close(out.joints[15].x, 2.1f, "chain.age.ankle.x == fresh measurement", 0.05);
}

// ---------- Test 5: child skipped when parent uninitialized ----------
void test_chain_child_skipped_when_parent_never_seen() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    fitra::lift::SkeletonKalman kf;

    // Only the ankle is "observed", parents are missing. The chain Kalman
    // must NOT emit a world position for ankle (it has no way to know the
    // hip's location, and falling back to "treat z as world" would defeat
    // the chain invariant).
    fitra::infer::Skeleton3D skel;
    skel.kp_count = 26;
    for (auto& j : skel.joints) j.valid = false;
    set_joint(skel, 15, 0.1f, 0.0f, 0.0f);  // ankle only

    auto out = kf.update(skel, 1.0 / 60.0);
    check(!out.joints[15].valid,
          "chain.ankle.no-parent: must stay invalid without parent chain");
    check(!out.joints[19].valid,
          "chain.root.no-meas: root stays invalid when never observed");
}

// ---------- Test 6: face state is excluded, direction endpoint retained ----
void test_halpe_face_joints_are_excluded() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    fitra::lift::SkeletonKalman kf;

    auto skel = make_skel(0.0f, 0.0f, 0.9f);
    set_joint(skel, 17, 0.0f, 0.0f, 1.72f);  // head_top, retained
    set_joint(skel, 0, 0.0f, 0.10f, 1.62f);  // nose
    set_joint(skel, 1, -0.03f, 0.10f, 1.65f);
    set_joint(skel, 2, 0.03f, 0.10f, 1.65f);
    set_joint(skel, 3, -0.07f, 0.06f, 1.64f);
    set_joint(skel, 4, 0.07f, 0.06f, 1.64f);

    auto out = kf.update(skel, 1.0 / 60.0);
    check(out.joints[0].valid,
          "nose must survive only as a synthetic direction endpoint");
    const double dx = static_cast<double>(out.joints[0].x) - out.joints[17].x;
    const double dy = static_cast<double>(out.joints[0].y) - out.joints[17].y;
    const double dz = static_cast<double>(out.joints[0].z) - out.joints[17].z;
    check_close(std::sqrt(dx * dx + dy * dy + dz * dz),
                fitra::lift::kHeadDirectionLengthM,
                "head direction fixed length", 1.0e-4);
    for (std::size_t k = 1; k <= 4; ++k) {
        check(!out.joints[k].valid,
              "Halpe26 eye/ear Kalman state must stay invalid");
    }
    check(out.joints[17].valid,
          "head_top Kalman state must remain available");
    check(out.joints[18].valid,
          "neck Kalman state must remain available");

    // A single bad nose observation must not reverse the display ray. The
    // direction-only EMA is deliberately much cheaper than reviving a 6D
    // per-joint Kalman state for the nose.
    auto flipped = skel;
    set_joint(flipped, 0, 0.0f, -0.10f, 1.62f);
    auto after_flip = kf.update(flipped, 1.0 / 60.0);
    const double first_y = out.joints[0].y - out.joints[17].y;
    const double flipped_y =
        after_flip.joints[0].y - after_flip.joints[17].y;
    check(first_y * flipped_y > 0.0,
          "one-frame nose flip must not reverse smoothed head direction");
}

// ---------- Test 7: explicit lifecycle reset re-anchors immediately --------
void test_lifecycle_reset_reanchors_immediately() {
    fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
    fitra::lift::SkeletonKalman kf;
    settle(kf);

    // A new subject/coordinate lifecycle can be far from the previous one.
    // reset() must make its first measurement an exact seed, not a Kalman
    // interpolation with the old subject.
    kf.reset();
    const auto measurement = make_skel(2.0f, -1.0f, 1.2f);
    const auto out = kf.update(measurement, 1.0 / 60.0);
    check_close(out.joints[19].x, measurement.joints[19].x,
                "lifecycle.reset.root.x");
    check_close(out.joints[19].y, measurement.joints[19].y,
                "lifecycle.reset.root.y");
    check_close(out.joints[19].z, measurement.joints[19].z,
                "lifecycle.reset.root.z");
    check_close(out.joints[15].x, measurement.joints[15].x,
                "lifecycle.reset.ankle.x");
    check_close(out.joints[15].y, measurement.joints[15].y,
                "lifecycle.reset.ankle.y");
    check_close(out.joints[15].z, measurement.joints[15].z,
                "lifecycle.reset.ankle.z");
}

}  // namespace

int main() {
    try {
        test_chain_propagates_root_motion_to_unobserved_child();
        std::printf("[ok] chain Kalman propagates root motion to unobserved child\n");
        test_chain_corrects_offset_when_child_observed();
        std::printf("[ok] chain Kalman corrects offset from child measurement\n");
        test_chain_recovers_after_long_missing();
        std::printf("[ok] chain Kalman recovers after long missing run\n");
        test_chain_child_ages_when_parent_skipped();
        std::printf("[ok] chain Kalman ages child while parent is unavailable\n");
        test_chain_child_skipped_when_parent_never_seen();
        std::printf("[ok] chain Kalman skips child whose parent never observed\n");
        test_halpe_face_joints_are_excluded();
        std::printf("[ok] chain Kalman retains direction-only nose endpoint\n");
        test_lifecycle_reset_reanchors_immediately();
        std::printf("[ok] chain Kalman lifecycle reset re-anchors immediately\n");
        std::puts("test_kalman_chain ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_kalman_chain failed: %s\n", e.what());
        return 1;
    }
}
