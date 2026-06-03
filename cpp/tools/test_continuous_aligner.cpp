// Unit tests for the continuous HMD-driven alignment helpers.
//
// Pure layer only (no threads / no buses): ramp, verticality_score,
// make_sample (head/chest selection + quality), SampleReservoir
// (admit/prune/cap), blend_alignment (Y-hold + step clamp + yaw arc), and a
// reservoir -> solve_motion round-trip.

#include "vmt/continuous_aligner.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace {

using fitra::vmt::AlignSample;
using fitra::vmt::AutoAlignmentStatus;
using fitra::vmt::blend_alignment;
using fitra::vmt::ContinuousAlignerConfig;
using fitra::vmt::CorrSource;
using fitra::vmt::LockState;
using fitra::vmt::update_lock_state;
using fitra::vmt::HmdPose;
using fitra::vmt::make_sample;
using fitra::vmt::ramp;
using fitra::vmt::SampleInputs;
using fitra::vmt::SampleReservoir;
using fitra::vmt::solve_motion;
using fitra::vmt::verticality_score;
using fitra::vmt::VmtAlignment;

constexpr float kPi = 3.14159265358979323846f;
int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++g_fail; } \
} while (0)

#define CHECK_NEAR(a, b, eps) do { \
    if (std::fabs(double((a)) - double((b))) > (eps)) { \
        std::fprintf(stderr, "FAIL %s:%d %s != %s (%g vs %g)\n", \
            __FILE__, __LINE__, #a, #b, double(a), double(b)); \
        ++g_fail; \
    } \
} while (0)

HmdPose hmd_at(float x, float z) {
    HmdPose h;
    h.valid = true;
    h.x = x; h.y = 1.7f; h.z = z;
    h.qw = 1.0f;
    return h;
}

void test_ramp() {
    // Ascending bounds.
    CHECK_NEAR(ramp(0.0f, 0.0f, 1.0f), 0.0f, 1e-6);
    CHECK_NEAR(ramp(1.0f, 0.0f, 1.0f), 1.0f, 1e-6);
    CHECK_NEAR(ramp(0.5f, 0.0f, 1.0f), 0.5f, 1e-6);
    CHECK(ramp(-1.0f, 0.0f, 1.0f) == 0.0f);
    CHECK(ramp(2.0f, 0.0f, 1.0f) == 1.0f);
    // Descending bounds (full < zero): used by the verticality + velocity terms.
    CHECK_NEAR(ramp(15.0f, 40.0f, 15.0f), 1.0f, 1e-6);
    CHECK_NEAR(ramp(40.0f, 40.0f, 15.0f), 0.0f, 1e-6);
    CHECK_NEAR(ramp(27.5f, 40.0f, 15.0f), 0.5f, 1e-6);
    // Degenerate band (zero_at == full_at): step at the shared threshold,
    // honoring the "1 at full_at" contract.
    CHECK(ramp(0.5f, 1.0f, 1.0f) == 0.0f);  // below
    CHECK(ramp(1.0f, 1.0f, 1.0f) == 1.0f);  // at full_at == zero_at
    CHECK(ramp(2.0f, 1.0f, 1.0f) == 1.0f);  // above
}

void test_verticality() {
    // Straight up (+Z) -> upright -> 1.
    CHECK_NEAR(verticality_score({0.0f, 0.0f, 1.0f}, 15.0f, 40.0f), 1.0f, 1e-5);
    // Horizontal -> 0.
    CHECK_NEAR(verticality_score({1.0f, 0.0f, 0.0f}, 15.0f, 40.0f), 0.0f, 1e-5);
    // Tilt 27.5 deg from vertical -> midpoint of the ramp -> 0.5.
    const float c = std::cos(27.5f * kPi / 180.0f);
    const float s = std::sin(27.5f * kPi / 180.0f);
    CHECK_NEAR(verticality_score({s, 0.0f, c}, 15.0f, 40.0f), 0.5f, 1e-3);
    // Sign of Z must not matter (upside-down bone is still "vertical").
    CHECK_NEAR(verticality_score({0.0f, 0.0f, -1.0f}, 15.0f, 40.0f), 1.0f, 1e-5);
    // Degenerate bone -> 0.
    CHECK_NEAR(verticality_score({0.0f, 0.0f, 0.0f}, 15.0f, 40.0f), 0.0f, 1e-6);
}

void test_make_sample() {
    ContinuousAlignerConfig cfg;  // defaults

    // World frame is Z-up; world_pos_to_vmt(x,y,z) = {x, z, -y}, so the ground
    // plane the Procrustes runs on is world (x, y) -> vmt (x, z). Keep the spine
    // vertical (neck above hip in world Z) and give the joints a horizontal
    // offset so the vmt xz mapping is exercised.
    //
    // Upright, high head score -> head source, quality == head score (vel=1).
    SampleInputs in;
    in.hip_center = {0.2f, 0.6f, 0.0f}; in.hip_valid = true; in.hip_score = 0.8f;
    in.neck       = {0.2f, 0.6f, 0.5f}; in.neck_valid = true; in.neck_score = 0.8f;
    in.head_top   = {0.4f, 0.7f, 1.6f}; in.head_valid = true; in.head_score = 0.9f;
    {
        AlignSample s = make_sample(in, hmd_at(2.0f, 3.0f), 0.0f, 0.0, cfg);
        CHECK(s.source == CorrSource::Head);
        CHECK_NEAR(s.quality, 0.9f, 1e-4);
        CHECK_NEAR(s.hmd_x, 2.0f, 1e-6);
        CHECK_NEAR(s.hmd_z, 3.0f, 1e-6);
        // world_pos_to_vmt(head 0.4,0.7,1.6) = {x, z, -y} = {0.4, 1.6, -0.7}.
        CHECK_NEAR(s.body_x, 0.4f, 1e-6);
        CHECK_NEAR(s.body_z, -0.7f, 1e-6);
    }

    // Low head score -> chest fallback; body = midpoint(neck, hip) in VMT.
    in.head_score = 0.3f;
    {
        AlignSample s = make_sample(in, hmd_at(0.0f, 0.0f), 0.0f, 0.0, cfg);
        CHECK(s.source == CorrSource::Chest);
        // midpoint world = (0.2, 0.6, 0.25) -> vmt {0.2, 0.25, -0.6}.
        CHECK_NEAR(s.body_x, 0.2f, 1e-6);
        CHECK_NEAR(s.body_z, -0.6f, 1e-6);
        CHECK_NEAR(s.quality, std::min(in.neck_score, in.hip_score), 1e-4);
    }

    // Fast HMD motion drives the velocity weight (and thus quality) to ~0.
    in.head_score = 0.9f;
    {
        AlignSample s = make_sample(in, hmd_at(0.0f, 0.0f), 5.0f, 0.0, cfg);
        CHECK_NEAR(s.quality, 0.0f, 1e-5);
    }

    // Missing neck/hip -> no usable sample.
    in.neck_valid = false;
    {
        AlignSample s = make_sample(in, hmd_at(0.0f, 0.0f), 0.0f, 0.0, cfg);
        CHECK(s.source == CorrSource::None);
    }

    // Non-finite input -> rejected (would otherwise poison reservoir/solve and
    // hit float->int UB in key_of).
    in.neck_valid = true;
    {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        AlignSample s = make_sample(in, hmd_at(nan, 0.0f), 0.0f, 0.0, cfg);
        CHECK(s.source == CorrSource::None);
    }
}

AlignSample sample_at(float hx, float hz, float quality, double t) {
    AlignSample s;
    s.hmd_x = hx; s.hmd_z = hz;
    s.body_x = hx; s.body_z = hz;
    s.quality = quality;
    s.t_s = t;
    s.source = CorrSource::Head;
    return s;
}

void test_reservoir() {
    ContinuousAlignerConfig cfg;
    cfg.cell_size_m = 0.3f;
    cfg.max_cells = 2;
    cfg.sample_ttl_s = 60.0;

    SampleReservoir r{cfg};
    // Two samples in the same cell (0,0): the better one wins.
    CHECK(r.admit(sample_at(0.05f, 0.05f, 0.4f, 0.0)));
    CHECK(r.admit(sample_at(0.05f, 0.05f, 0.8f, 0.1)));  // higher -> replace
    CHECK(r.occupied_cells() == 1);
    // A clearly worse sample (< 0.9 * incumbent) does not replace.
    CHECK(!r.admit(sample_at(0.05f, 0.05f, 0.5f, 0.2)));
    CHECK(r.occupied_cells() == 1);

    // Distinct cells accumulate.
    r.admit(sample_at(1.0f, 0.0f, 0.6f, 0.0));
    r.admit(sample_at(2.0f, 0.0f, 0.7f, 0.0));
    CHECK(r.occupied_cells() == 3);

    // prune evicts down to max_cells (2), dropping the lowest quality (0.6).
    r.prune(1.0);
    CHECK(r.occupied_cells() == 2);

    // TTL expiry: everything older than ttl is dropped.
    r.prune(1000.0);
    CHECK(r.occupied_cells() == 0);

    // Negative room coords (VMT x/z routinely go negative): the four sign
    // quadrants must map to four distinct cells. Regression guard for the
    // signed-left-shift UB in key_of — a broken pack collapses these.
    ContinuousAlignerConfig ncfg;
    ncfg.cell_size_m = 0.3f;
    ncfg.max_cells = 16;
    ncfg.sample_ttl_s = 60.0;
    SampleReservoir nr{ncfg};
    CHECK(nr.admit(sample_at(-1.0f, -1.0f, 0.6f, 0.0)));
    CHECK(nr.admit(sample_at(-1.0f,  1.0f, 0.6f, 0.0)));
    CHECK(nr.admit(sample_at( 1.0f, -1.0f, 0.6f, 0.0)));
    CHECK(nr.admit(sample_at( 1.0f,  1.0f, 0.6f, 0.0)));
    CHECK(nr.occupied_cells() == 4);
}

void test_blend() {
    VmtAlignment cur;  cur.x = 0.0f; cur.y = 1.5f; cur.z = 0.0f; cur.yaw_deg = 0.0f;
    VmtAlignment tgt;  tgt.x = 1.0f; tgt.y = 9.0f; tgt.z = 1.0f; tgt.yaw_deg = 90.0f;

    VmtAlignment out = blend_alignment(cur, tgt, 0.2f, 2.0f, 0.05f);
    // Y is held at the current (manual) value.
    CHECK_NEAR(out.y, 1.5f, 1e-6);
    // Translation step clamped to 0.05 m magnitude.
    const float mag = std::sqrt(out.x * out.x + out.z * out.z);
    CHECK_NEAR(mag, 0.05f, 1e-4);
    // Yaw step clamped to 2 deg (0.2 * 90 = 18 -> 2).
    CHECK_NEAR(out.yaw_deg, 2.0f, 1e-4);

    // Repeated blending converges toward the target.
    VmtAlignment a = cur;
    for (int i = 0; i < 2000; ++i) {
        a = blend_alignment(a, tgt, 0.2f, 5.0f, 0.2f);
    }
    CHECK_NEAR(a.x, tgt.x, 1e-2);
    CHECK_NEAR(a.z, tgt.z, 1e-2);
    CHECK_NEAR(a.yaw_deg, tgt.yaw_deg, 1e-1);

    // Yaw blends on the shortest arc: -170 -> +170 goes the short way (negative).
    VmtAlignment c2; c2.yaw_deg = 170.0f;
    VmtAlignment t2; t2.yaw_deg = -170.0f;
    VmtAlignment o2 = blend_alignment(c2, t2, 0.5f, 30.0f, 1.0f);
    // Short arc 170 -> -170 is +20deg; 0.5*20 = +10 -> 180 (== -170 side).
    CHECK_NEAR(o2.yaw_deg, 180.0f, 1e-3);
}

void test_lock_state() {
    ContinuousAlignerConfig cfg;  // lock_streak=3, lock tol (0.10m, 3deg),
                                  // unlock (0.50m, 20deg), residual_max 0.15m
    VmtAlignment live;  live.x = 0.0f; live.z = 0.0f; live.yaw_deg = 0.0f;

    // Far-off solve while unlocked: no streak, stays coarse.
    VmtAlignment far;  far.x = 1.0f; far.z = 0.0f; far.yaw_deg = 45.0f;
    LockState st;
    st = update_lock_state(st, live, far, 0.05f, cfg);
    CHECK(!st.locked && st.streak == 0);

    // A converged solve (close in xz + yaw, low residual) builds the streak and
    // latches after lock_streak resolves.
    VmtAlignment near; near.x = 0.02f; near.z = 0.01f; near.yaw_deg = 1.0f;
    st = update_lock_state(st, live, near, 0.03f, cfg);  // streak 1
    CHECK(!st.locked && st.streak == 1);
    st = update_lock_state(st, live, near, 0.03f, cfg);  // streak 2
    CHECK(!st.locked && st.streak == 2);
    st = update_lock_state(st, live, near, 0.03f, cfg);  // streak 3 -> lock
    CHECK(st.locked);

    // High residual breaks the streak before locking.
    LockState st2;
    st2 = update_lock_state(st2, live, near, 0.03f, cfg);  // streak 1
    st2 = update_lock_state(st2, live, near, 0.20f, cfg);  // residual > max -> reset
    CHECK(!st2.locked && st2.streak == 0);

    // Once locked, a small solve keeps it locked; a large divergence (re-center)
    // drops back to coarse.
    LockState locked; locked.locked = true; locked.streak = 3;
    LockState keep = update_lock_state(locked, live, near, 0.03f, cfg);
    CHECK(keep.locked);
    VmtAlignment jumped; jumped.x = 0.8f; jumped.z = 0.0f; jumped.yaw_deg = 0.0f;
    LockState drop = update_lock_state(locked, live, jumped, 0.03f, cfg);
    CHECK(!drop.locked && drop.streak == 0);
    VmtAlignment yawed; yawed.x = 0.0f; yawed.z = 0.0f; yawed.yaw_deg = 30.0f;
    LockState drop2 = update_lock_state(locked, live, yawed, 0.03f, cfg);
    CHECK(!drop2.locked);
}

void test_reservoir_solve_roundtrip() {
    // Build a known forward transform hmd = R*body + t (VMT convention,
    // R = [[c, s], [-s, c]]) and confirm solve_motion on the reservoir
    // recovers it.
    const float yaw = 30.0f * kPi / 180.0f;
    const float c = std::cos(yaw), s = std::sin(yaw);
    const float tx = 0.5f, tz = -0.3f;

    ContinuousAlignerConfig cfg;
    cfg.cell_size_m = 0.3f;
    cfg.max_cells = 64;
    cfg.min_cells = 8;
    SampleReservoir r{cfg};

    int admitted = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const float bx = static_cast<float>(i);  // >> cell size -> distinct cells
            const float bz = static_cast<float>(j);
            const float hx = c * bx + s * bz + tx;
            const float hz = -s * bx + c * bz + tz;
            AlignSample sm;
            sm.hmd_x = hx; sm.hmd_z = hz;
            sm.body_x = bx; sm.body_z = bz;
            sm.quality = 0.8f; sm.t_s = 0.0; sm.source = CorrSource::Head;
            if (r.admit(sm)) ++admitted;
        }
    }
    CHECK(admitted == 9);

    auto samples = r.motion_samples();
    CHECK(static_cast<int>(samples.size()) == 9);
    auto res = solve_motion(samples, cfg.min_cells);
    CHECK(res.status == AutoAlignmentStatus::Ok);
    CHECK_NEAR(res.alignment.yaw_deg, 30.0f, 1e-2);
    CHECK_NEAR(res.alignment.x, tx, 1e-3);
    CHECK_NEAR(res.alignment.z, tz, 1e-3);
    CHECK(res.residual_m < 1e-3f);
}

}  // namespace

int main() {
    test_ramp();
    test_verticality();
    test_make_sample();
    test_reservoir();
    test_blend();
    test_lock_state();
    test_reservoir_solve_roundtrip();
    if (g_fail == 0) std::printf("test_continuous_aligner: all checks passed\n");
    return g_fail == 0 ? 0 : 1;
}
