#pragma once

// Offline markerless geometry. The gauge is camera[0]'s OpenCV frame, with
// one recovered baseline set to one. These poses are NOT metric FitraWorld
// extrinsics and must never be written using write_calibration().
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "lift/calib_io.hpp"

namespace fitra::lift {

enum class MarkerlessTimestampKind {
    Unknown,
    KernelMonotonicSoe,
    KernelMonotonicEof,
    SyntheticMonotonic,
};

struct MarkerlessKeypoint {
    int joint_index = -1;  // Raw Halpe26 index; facial landmarks 0..4 are excluded.
    cv::Point2d pixel{};
    double score = 0.0;
};

struct MarkerlessCameraFrame {
    std::string camera_id;
    std::uint64_t source_seq = 0;
    std::uint64_t stream_epoch = 0;
    std::int64_t capture_mono_ns = 0;
    MarkerlessTimestampKind timestamp_kind = MarkerlessTimestampKind::Unknown;
    int width = 0;
    int height = 0;
    int person_count = 1;
    std::vector<MarkerlessKeypoint> keypoints;
};

struct MarkerlessFrameSet {
    std::uint64_t sample_id = 0;
    // Producer-supplied session/person continuity token. Never derive identity
    // from a person's position in an inference array.
    std::string continuity_id;
    std::vector<MarkerlessCameraFrame> frames;
};

struct MarkerlessSolverOptions {
    double min_score = 0.5;
    double max_sync_skew_ms = 20.0;
    std::size_t min_frame_sets = 6;
    std::size_t min_pair_points = 24;
    double ransac_threshold_px = 3.0;
    double max_reprojection_px = 8.0;
    double min_ray_angle_deg = 1.0;
    double min_structure_ratio = 1.0e-3;
    int max_refine_iterations = 20;
    // Every fifth complete sample is reserved for validation, never for pose
    // initialization/refinement. Split by pose/time interval in real sessions.
    std::size_t holdout_stride = 5;
};

struct MarkerlessRelativeCamera {
    std::string id;
    cv::Matx44d T_camera_reference = cv::Matx44d::eye();
};

struct MarkerlessRelativeResult {
    bool ok = false;  // Relative geometry only, never permission to apply.
    std::string reason;
    std::vector<MarkerlessRelativeCamera> cameras;
    std::string reference_camera_id;
    std::string baseline_camera_id;
    std::size_t training_frame_sets = 0;
    std::size_t validation_frame_sets = 0;
    std::size_t inlier_points = 0;
    // Unknown diagnostics serialize as null, never as a measured zero error.
    double training_reprojection_median_px = std::numeric_limits<double>::quiet_NaN();
    double validation_reprojection_median_px = std::numeric_limits<double>::quiet_NaN();
    double validation_reprojection_p95_px = std::numeric_limits<double>::quiet_NaN();
    double structure_ratio = std::numeric_limits<double>::quiet_NaN();
    double median_ray_angle_deg = std::numeric_limits<double>::quiet_NaN();
    // Deliberately constants: this stage cannot establish either property.
    static constexpr bool metric_scale_known = false;
    static constexpr bool floor_aligned = false;
    static constexpr bool runtime_ready = false;
};

// Fixed intrinsics, raw single-person Halpe26 correspondence sets only.
// The function validates its direct input as well as collected/replayed input.
// Invalid/degenerate data returns ok=false with a reason, without partial poses.
MarkerlessRelativeResult solve_markerless_relative(
    const CalibrationSet& intrinsics,
    const std::vector<MarkerlessFrameSet>& frame_sets,
    const MarkerlessSolverOptions& options = {});

}  // namespace fitra::lift
