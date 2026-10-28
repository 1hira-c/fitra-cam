// Independent synthetic projections test relative geometry, not metric/floor
// accuracy or person-present acceptance. Nothing opens cameras or uses CUDA.
#include "markerless_test_scene.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

namespace {

using fitra::lift::MarkerlessRelativeResult;
using fitra::lift::MarkerlessSolverOptions;
using fitra::lift::MarkerlessTimestampKind;
using fitra::lift::solve_markerless_relative;
using markerless_test::Scene;
using markerless_test::SceneOptions;
using markerless_test::make_scene;
using markerless_test::rotation_error_deg;
using markerless_test::translation;

int failures = 0;

#define CHECK(condition) do { \
    if (!(condition)) { \
        std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #condition); \
        ++failures; \
    } \
} while (0)

#define CHECK_LT(a, b) do { \
    const double check_a = static_cast<double>(a); \
    const double check_b = static_cast<double>(b); \
    if (!(check_a < check_b)) { \
        std::fprintf(stderr, "FAIL %s:%d %s < %s (%g vs %g)\n", \
                     __FILE__, __LINE__, #a, #b, check_a, check_b); \
        ++failures; \
    } \
} while (0)

MarkerlessRelativeResult solve(const Scene& scene, const MarkerlessSolverOptions& options = {}) {
    // Control the test's own RANSAC environment. Production code must not
    // overwrite process-global RNG state to make a replay look deterministic.
    cv::setRNGSeed(38467);
    return solve_markerless_relative(scene.intrinsics, scene.samples, options);
}

void expect_rejected(const Scene& scene, const char* label,
                     const MarkerlessSolverOptions& options = {}) {
    const auto result = solve(scene, options);
    if (result.ok) {
        std::fprintf(stderr, "unexpected success: %s\n", label);
        ++failures;
    }
    CHECK(!result.reason.empty());
    CHECK(result.cameras.empty());
    CHECK(!result.metric_scale_known);
    CHECK(!result.floor_aligned);
    CHECK(!result.runtime_ready);
}

MarkerlessRelativeResult expect_recovery(const Scene& scene, const char* label,
                                        double max_rotation_deg, double max_translation,
                                        const MarkerlessSolverOptions& options = {}) {
    const auto result = solve(scene, options);
    if (!result.ok) {
        std::fprintf(stderr, "recovery failed: %s: %s\n", label, result.reason.c_str());
        ++failures;
        return result;
    }
    CHECK(result.cameras.size() == scene.intrinsics.cameras.size());
    CHECK(result.reference_camera_id == scene.intrinsics.cameras[0].id);
    CHECK(!result.metric_scale_known);
    CHECK(!result.floor_aligned);
    CHECK(!result.runtime_ready);
    CHECK(result.training_frame_sets + result.validation_frame_sets == scene.samples.size());
    CHECK(result.validation_frame_sets == scene.samples.size() / options.holdout_stride);
    CHECK(result.inlier_points >= options.min_pair_points);
    CHECK(std::isfinite(result.training_reprojection_median_px));
    CHECK(std::isfinite(result.validation_reprojection_median_px));
    CHECK(std::isfinite(result.validation_reprojection_p95_px));
    CHECK(result.structure_ratio > options.min_structure_ratio);
    CHECK(result.median_ray_angle_deg >= options.min_ray_angle_deg);

    double baseline = 0;
    for (std::size_t i = 1; i < scene.intrinsics.cameras.size(); ++i)
        if (scene.intrinsics.cameras[i].id == result.baseline_camera_id)
            baseline = cv::norm(translation(scene.T_camera_reference_gt[i]));
    CHECK(baseline > 0.0);
    if (baseline <= 0.0) return result;

    double largest_rotation = 0, largest_translation = 0;
    for (const auto& camera : result.cameras) {
        bool found = false;
        for (std::size_t i = 0; i < scene.intrinsics.cameras.size(); ++i) {
            if (camera.id != scene.intrinsics.cameras[i].id) continue;
            found = true;
            const auto& gt = scene.T_camera_reference_gt[i];
            const double r = rotation_error_deg(camera.T_camera_reference, gt);
            const double t = cv::norm(translation(camera.T_camera_reference) - translation(gt) / baseline);
            largest_rotation = std::max(largest_rotation, r);
            largest_translation = std::max(largest_translation, t);
            CHECK_LT(r, max_rotation_deg);
            CHECK_LT(t, max_translation);
            if (i == 0) CHECK_LT(cv::norm(cv::Mat(camera.T_camera_reference - cv::Matx44d::eye())), 1e-10);
        }
        CHECK(found);
    }
    std::printf("%s: max rotation %.6g deg, translation %.6g baseline units, "
                "validation median/p95 %.6g/%.6g px\n", label,
                largest_rotation, largest_translation,
                result.validation_reprojection_median_px,
                result.validation_reprojection_p95_px);
    return result;
}

void test_exact_and_distortion() {
    SceneOptions options;
    options.camera_count = 2;
    const auto two = make_scene(options);
    const auto result = expect_recovery(two, "two-camera pinhole", .01, 1e-3);
    if (result.ok) CHECK_LT(result.validation_reprojection_p95_px, 1e-3);

    options.camera_count = 3;
    const auto three = make_scene(options);
    const auto result_three = expect_recovery(three, "three-camera pinhole", .01, 1e-3);
    if (result_three.ok) CHECK_LT(result_three.validation_reprojection_p95_px, 1e-3);

    options.fisheye = true;
    const auto fish = expect_recovery(make_scene(options), "three-camera fisheye", .01, 1e-3);
    if (fish.ok) CHECK_LT(fish.validation_reprojection_p95_px, 1e-3);

    options.fisheye = false;
    options.mixed_distortion = true;
    const auto mixed = expect_recovery(make_scene(options), "mixed pinhole/fisheye", .01, 1e-3);
    if (mixed.ok) CHECK_LT(mixed.validation_reprojection_p95_px, 1e-3);
}

void test_noise_outliers_and_missing() {
    // Vary injected image noise as well as camera count/lens model. A consensus
    // refit must recover geometry, not merely fit one favorable random sample.
    for (int cameras : {2, 3}) {
        for (unsigned seed = 1949; seed < 1955; ++seed) {
            SceneOptions options;
            options.camera_count = cameras;
            options.sample_count = 30;
            options.noise_px = .3;
            options.outliers = true;
            options.missing = true;
            options.seed = seed;
            options.fisheye = seed % 3 == 0;
            options.mixed_distortion = seed % 3 == 1;
            const auto label = "noise/outliers/missing cam" + std::to_string(cameras)
                             + " seed" + std::to_string(seed);
            const auto result = expect_recovery(make_scene(options), label.c_str(), .5, .025);
            if (result.ok) {
                CHECK_LT(result.validation_reprojection_median_px, 1.0);
                CHECK_LT(result.validation_reprojection_p95_px, 3.0);
            }
        }
    }
}

void test_faces_and_saved_extrinsics_are_not_inputs() {
    const auto original = make_scene();
    auto altered = make_scene();
    for (auto& set : altered.samples)
        for (auto& frame : set.frames)
            for (auto& p : frame.keypoints)
                if (p.joint_index < 5) p.pixel = {1200.0 - 11 * p.joint_index, 50.0 + 13 * p.joint_index};
    for (auto& camera : altered.intrinsics.cameras) {
        camera.has_extrinsics = true;
        camera.extrinsics.T_cw = cv::Mat::eye(4, 4, CV_64F);
        camera.extrinsics.T_cw.at<double>(0, 3) = 123.0;
        camera.extrinsics.T_cw.at<double>(1, 3) = -59.0;
        camera.extrinsics.T_cw.at<double>(2, 3) = 14.0;
    }
    const auto a = solve(original), b = solve(altered);
    CHECK(a.ok);
    CHECK(b.ok);
    if (!a.ok || !b.ok || a.cameras.size() != b.cameras.size()) return;
    for (std::size_t i = 0; i < a.cameras.size(); ++i) {
        CHECK(a.cameras[i].id == b.cameras[i].id);
        CHECK_LT(cv::norm(cv::Mat(a.cameras[i].T_camera_reference - b.cameras[i].T_camera_reference)), 1e-10);
    }
    // The const API must not mutate calibrated matrices or raw image points.
    for (const auto& camera : altered.intrinsics.cameras)
        CHECK(camera.extrinsics.T_cw.at<double>(0, 3) == 123.0);
    CHECK(altered.samples[0].frames[0].keypoints[0].pixel.x == 1200.0);
}

void test_validation_is_excluded_from_fit() {
    const auto original = make_scene();
    auto changed = make_scene();
    for (std::size_t i = 4; i < changed.samples.size(); i += 5)
        for (auto& p : changed.samples[i].frames[1].keypoints) p.pixel.x += .5;
    const auto a = solve(original), b = solve(changed);
    CHECK(a.ok);
    CHECK(b.ok);
    if (a.ok && b.ok && a.cameras.size() == b.cameras.size()) {
        for (std::size_t i = 0; i < a.cameras.size(); ++i)
            CHECK_LT(cv::norm(cv::Mat(a.cameras[i].T_camera_reference - b.cameras[i].T_camera_reference)), 1e-10);
        CHECK(b.validation_reprojection_median_px > a.validation_reprojection_median_px + .01);
    }
    // An otherwise perfect training sequence cannot hide inconsistent held-out
    // correspondences. Reprojection gating has to observe validation samples.
    for (std::size_t i = 4; i < changed.samples.size(); i += 5)
        for (auto& p : changed.samples[i].frames[1].keypoints) p.pixel.y += 50.0;
    expect_rejected(changed, "inconsistent held-out geometry");
}

void test_camera_identity_and_third_view_support() {
    const auto original = make_scene();
    auto reordered = make_scene();
    for (auto& sample : reordered.samples) std::reverse(sample.frames.begin(), sample.frames.end());
    const auto a = solve(original), b = solve(reordered);
    CHECK(a.ok);
    CHECK(b.ok);
    if (a.ok && b.ok && a.cameras.size() == b.cameras.size()) {
        for (std::size_t i = 0; i < a.cameras.size(); ++i) {
            CHECK(a.cameras[i].id == b.cameras[i].id);
            CHECK_LT(cv::norm(cv::Mat(a.cameras[i].T_camera_reference - b.cameras[i].T_camera_reference)), 1e-10);
        }
    }

    // Three-camera calibration cannot silently succeed as a two-camera rig.
    auto missing_third = make_scene();
    for (auto& sample : missing_third.samples) sample.frames[2].keypoints.clear();
    expect_rejected(missing_third, "third camera has no observations");
    auto inconsistent_third = make_scene();
    for (std::size_t i = 4; i < inconsistent_third.samples.size(); i += 5)
        for (auto& point : inconsistent_third.samples[i].frames[2].keypoints) point.pixel.y += 50.0;
    expect_rejected(inconsistent_third, "third camera lacks held-out support");
}

void test_degeneracy() {
    SceneOptions options;
    options.pure_rotation = true;
    expect_rejected(make_scene(options), "pure rotation");
    options.pure_rotation = false;
    options.baseline_scale = 1e-4;
    expect_rejected(make_scene(options), "negligible parallax");
    options.baseline_scale = 1;
    options.structure = markerless_test::Structure::Planar;
    expect_rejected(make_scene(options), "all correspondences coplanar");
    options.structure = markerless_test::Structure::Collinear;
    expect_rejected(make_scene(options), "all correspondences collinear");
    options.structure = markerless_test::Structure::Articulated;
    options.sample_count = 3;
    expect_rejected(make_scene(options), "insufficient sample groups");
    auto empty = make_scene();
    for (auto& sample : empty.samples)
        for (auto& frame : sample.frames) frame.keypoints.clear();
    expect_rejected(empty, "no measured correspondences");
    auto low_score = make_scene();
    for (auto& sample : low_score.samples)
        for (auto& frame : sample.frames)
            for (auto& point : frame.keypoints) point.score = .1;
    expect_rejected(low_score, "all observations below confidence gate");
}

template <typename Fn>
void invalid_case(const char* label, Fn mutate) {
    auto scene = make_scene();
    mutate(scene);
    expect_rejected(scene, label);
}

void test_input_validation() {
    invalid_case("multiple people", [](Scene& s) { s.samples[0].frames[0].person_count = 2; });
    invalid_case("no person", [](Scene& s) { s.samples[0].frames[0].person_count = 0; });
    invalid_case("unknown camera", [](Scene& s) { s.samples[0].frames[0].camera_id = "absent"; });
    invalid_case("duplicate camera", [](Scene& s) { s.samples[0].frames[1].camera_id = "cam0"; });
    invalid_case("missing camera", [](Scene& s) { s.samples[0].frames.pop_back(); });
    invalid_case("image size mismatch", [](Scene& s) { --s.samples[0].frames[0].width; });
    invalid_case("unknown timestamp", [](Scene& s) {
        s.samples[0].frames[0].timestamp_kind = MarkerlessTimestampKind::Unknown;
    });
    invalid_case("mixed timestamp semantics", [](Scene& s) {
        s.samples[0].frames[0].timestamp_kind = MarkerlessTimestampKind::KernelMonotonicEof;
    });
    invalid_case("missing capture time", [](Scene& s) { s.samples[0].frames[0].capture_mono_ns = 0; });
    invalid_case("capture time reversed", [](Scene& s) {
        s.samples[2].frames[0].capture_mono_ns = s.samples[1].frames[0].capture_mono_ns - 1;
    });
    invalid_case("cross-camera skew", [](Scene& s) { s.samples[0].frames[0].capture_mono_ns += 30000000; });
    invalid_case("same source twice", [](Scene& s) { s.samples[2].frames[0].source_seq = 2; });
    invalid_case("same sample twice", [](Scene& s) { s.samples[2].sample_id = 2; });
    invalid_case("source epoch changed", [](Scene& s) { s.samples[2].frames[0].stream_epoch = 2; });
    invalid_case("person continuity changed", [](Scene& s) { s.samples[2].continuity_id = "other-person"; });
    invalid_case("unknown person continuity", [](Scene& s) { s.samples[0].continuity_id.clear(); });
    invalid_case("duplicate joint", [](Scene& s) {
        s.samples[0].frames[0].keypoints.push_back(s.samples[0].frames[0].keypoints[5]);
    });
    invalid_case("invalid joint index", [](Scene& s) { s.samples[0].frames[0].keypoints[5].joint_index = 26; });
    invalid_case("nonfinite score", [](Scene& s) {
        s.samples[0].frames[0].keypoints[5].score = std::numeric_limits<double>::quiet_NaN();
    });
    invalid_case("out-of-range score", [](Scene& s) { s.samples[0].frames[0].keypoints[5].score = 1.1; });
    invalid_case("nonfinite pixel", [](Scene& s) {
        s.samples[0].frames[0].keypoints[5].pixel.x = std::numeric_limits<double>::infinity();
    });
    invalid_case("pixel outside image", [](Scene& s) { s.samples[0].frames[0].keypoints[5].pixel.x = -1; });
    invalid_case("duplicate intrinsic camera ID", [](Scene& s) { s.intrinsics.cameras[1].id = "cam0"; });
    invalid_case("invalid focal length", [](Scene& s) { s.intrinsics.cameras[0].intrinsics.K.at<double>(0, 0) = 0; });

    const auto valid = make_scene();
    MarkerlessSolverOptions options;
    options.holdout_stride = 0;
    expect_rejected(valid, "invalid holdout stride", options);
    options = {};
    options.max_refine_iterations = -1;
    expect_rejected(valid, "invalid iteration bound", options);
}

}  // namespace

int main() {
    test_exact_and_distortion();
    test_noise_outliers_and_missing();
    test_faces_and_saved_extrinsics_are_not_inputs();
    test_validation_is_excluded_from_fit();
    test_camera_identity_and_third_view_support();
    test_degeneracy();
    test_input_validation();
    if (failures) std::fprintf(stderr, "markerless geometry: %d failure(s)\n", failures);
    else std::puts("markerless geometry: all synthetic checks passed");
    return failures ? 1 : 0;
}
