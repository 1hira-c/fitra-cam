#include "lift/markerless_extrinsic.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>
#include <utility>

#include <opencv2/calib3d.hpp>

#include "lift/markerless_collection.hpp"

namespace fitra::lift {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct Observation {
    bool present = false;
    cv::Point2d pixel;
    cv::Point2d normalized;
    double score = 0.0;
};

struct Track {
    std::vector<Observation> views;
    bool validation = false;
};

struct Pose {
    cv::Matx33d R = cv::Matx33d::eye();
    cv::Vec3d t{0, 0, 0};
    bool known = false;
};

struct Point {
    bool valid = false;
    cv::Vec3d X{0, 0, 0};
    std::vector<std::size_t> inliers;
    std::vector<double> errors;
    double angle_deg = 0.0;
};

struct Statistics {
    std::vector<double> errors;
    std::vector<double> angles;
    std::vector<cv::Vec3d> points;
    std::size_t possible_points = 0;
    std::vector<std::size_t> possible_views;
    std::vector<std::size_t> supported_views;
};

double percentile(std::vector<double> v, double q) {
    if (v.empty()) return std::numeric_limits<double>::infinity();
    std::sort(v.begin(), v.end());
    const double index = q * static_cast<double>(v.size() - 1);
    const auto lo = static_cast<std::size_t>(index);
    const auto hi = std::min(lo + 1, v.size() - 1);
    return v[lo] + (index - static_cast<double>(lo)) * (v[hi] - v[lo]);
}

bool finite(const cv::Vec3d& v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

double focal(const Intrinsics& cam) {
    return std::sqrt(cam.K.at<double>(0, 0) * cam.K.at<double>(1, 1));
}

cv::Matx34d projection(const Pose& p) {
    return {p.R(0, 0), p.R(0, 1), p.R(0, 2), p.t[0],
            p.R(1, 0), p.R(1, 1), p.R(1, 2), p.t[1],
            p.R(2, 0), p.R(2, 1), p.R(2, 2), p.t[2]};
}

cv::Matx44d matrix(const Pose& p) {
    cv::Matx44d T = cv::Matx44d::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) T(r, c) = p.R(r, c);
        T(r, 3) = p.t[r];
    }
    return T;
}

std::vector<Track> make_tracks(const CalibrationSet& cameras,
                               const std::vector<MarkerlessFrameSet>& samples,
                               const MarkerlessSolverOptions& options) {
    std::map<std::string, std::size_t> indices;
    for (std::size_t i = 0; i < cameras.cameras.size(); ++i)
        indices.emplace(cameras.cameras[i].id, i);
    std::vector<Track> tracks;
    tracks.reserve(samples.size() * 21);
    for (std::size_t sample = 0; sample < samples.size(); ++sample) {
        std::array<Track, 26> joints;
        for (auto& track : joints) {
            track.views.resize(cameras.cameras.size());
            track.validation = sample % options.holdout_stride == options.holdout_stride - 1;
        }
        for (const auto& frame : samples[sample].frames) {
            const auto index = indices.at(frame.camera_id);
            const auto& camera = cameras.cameras[index].intrinsics;
            std::vector<cv::Point2d> pixels;
            std::vector<const MarkerlessKeypoint*> keypoints;
            for (const auto& kp : frame.keypoints) {
                if (kp.joint_index < 5 || kp.score < options.min_score) continue;
                pixels.push_back(kp.pixel);
                keypoints.push_back(&kp);
            }
            if (pixels.empty()) continue;
            std::vector<cv::Point2d> normalized;
            if (camera.is_fisheye())
                cv::fisheye::undistortPoints(pixels, normalized, camera.K, camera.dist);
            else
                cv::undistortPoints(pixels, normalized, camera.K, camera.dist);
            for (std::size_t k = 0; k < pixels.size(); ++k) {
                if (!std::isfinite(normalized[k].x) || !std::isfinite(normalized[k].y))
                    throw std::runtime_error("undistortion produced non-finite points");
                auto& obs = joints[static_cast<std::size_t>(keypoints[k]->joint_index)].views[index];
                obs = {true, pixels[k], normalized[k], keypoints[k]->score};
            }
        }
        for (std::size_t joint = 5; joint < joints.size(); ++joint) {
            const auto n = std::count_if(joints[joint].views.begin(), joints[joint].views.end(),
                                         [](const Observation& o) { return o.present; });
            if (n >= 2) tracks.push_back(std::move(joints[joint]));
        }
    }
    return tracks;
}

bool triangulate(const Track& track, const std::vector<Pose>& poses,
                  const std::vector<std::size_t>& views, cv::Vec3d& point) {
    cv::Mat A(static_cast<int>(views.size() * 2), 4, CV_64F);
    for (std::size_t j = 0; j < views.size(); ++j) {
        const auto v = views[j];
        const auto P = projection(poses[v]);
        const auto& obs = track.views[v];
        const double weight = std::sqrt(obs.score);
        for (int c = 0; c < 4; ++c) {
            A.at<double>(static_cast<int>(2 * j), c) =
                weight * (obs.normalized.x * P(2, c) - P(0, c));
            A.at<double>(static_cast<int>(2 * j + 1), c) =
                weight * (obs.normalized.y * P(2, c) - P(1, c));
        }
    }
    cv::Mat singular, u, vt;
    cv::SVD::compute(A, singular, u, vt, cv::SVD::FULL_UV);
    const double w = vt.at<double>(3, 3);
    if (!std::isfinite(w) || std::abs(w) < 1.0e-12) return false;
    point = {vt.at<double>(3, 0) / w, vt.at<double>(3, 1) / w,
             vt.at<double>(3, 2) / w};
    return finite(point) && cv::norm(point) < 1.0e8;
}

double reprojection(const cv::Vec3d& X, const Pose& pose,
                     const Intrinsics& camera, const cv::Point2d& observed) {
    const cv::Vec3d camera_point = pose.R * X + pose.t;
    if (!finite(camera_point) || camera_point[2] <= 1.0e-8)
        return std::numeric_limits<double>::infinity();
    const std::vector<cv::Point3d> input{{camera_point[0], camera_point[1], camera_point[2]}};
    std::vector<cv::Point2d> output;
    if (camera.is_fisheye())
        cv::fisheye::projectPoints(input, output, cv::Vec3d{}, cv::Vec3d{}, camera.K, camera.dist);
    else
        cv::projectPoints(input, cv::Vec3d{}, cv::Vec3d{}, camera.K, camera.dist, output);
    return cv::norm(output[0] - observed);
}

double ray_angle(const Track& track, const std::vector<Pose>& poses,
                 const std::vector<std::size_t>& views) {
    double largest = 0.0;
    for (std::size_t i = 0; i < views.size(); ++i) {
        const auto a = views[i];
        const auto& n = track.views[a].normalized;
        const cv::Vec3d r = poses[a].R.t() * cv::Vec3d(n.x, n.y, 1.0);
        for (std::size_t j = i + 1; j < views.size(); ++j) {
            const auto b = views[j];
            const auto& m = track.views[b].normalized;
            const cv::Vec3d s = poses[b].R.t() * cv::Vec3d(m.x, m.y, 1.0);
            const double c = std::clamp(std::abs(r.dot(s) / (cv::norm(r) * cv::norm(s))), 0.0, 1.0);
            largest = std::max(largest, std::acos(c) * 180.0 / kPi);
        }
    }
    return largest;
}

Point evaluate(const cv::Vec3d& X, const Track& track, const std::vector<Pose>& poses,
               const CalibrationSet& cameras, const MarkerlessSolverOptions& options) {
    Point point;
    point.X = X;
    for (std::size_t c = 0; c < poses.size(); ++c) {
        if (!poses[c].known || !track.views[c].present) continue;
        const double error = reprojection(X, poses[c], cameras.cameras[c].intrinsics,
                                         track.views[c].pixel);
        if (std::isfinite(error) && error <= options.max_reprojection_px) {
            point.inliers.push_back(c);
            point.errors.push_back(error);
        }
    }
    if (point.inliers.size() < 2) return point;
    point.angle_deg = ray_angle(track, poses, point.inliers);
    point.valid = point.angle_deg >= options.min_ray_angle_deg;
    return point;
}

// At most three views: enumerate the three pairs, choose a consensus, and
// refit using only its inliers. An occluded third view cannot drag both good
// views into a bad three-view DLT solution.
Point reconstruct(const Track& track, const std::vector<Pose>& poses,
                  const CalibrationSet& cameras, const MarkerlessSolverOptions& options) {
    Point best;
    std::vector<std::size_t> views;
    for (std::size_t c = 0; c < poses.size(); ++c)
        if (poses[c].known && track.views[c].present) views.push_back(c);
    for (std::size_t a = 0; a < views.size(); ++a) {
        for (std::size_t b = a + 1; b < views.size(); ++b) {
            cv::Vec3d X;
            if (!triangulate(track, poses, {views[a], views[b]}, X)) continue;
            Point candidate = evaluate(X, track, poses, cameras, options);
            if (!candidate.valid) continue;
            if (!best.valid || candidate.inliers.size() > best.inliers.size()
                || (candidate.inliers.size() == best.inliers.size()
                    && percentile(candidate.errors, 0.5) < percentile(best.errors, 0.5)))
                best = std::move(candidate);
        }
    }
    if (best.valid && best.inliers.size() > 2) {
        cv::Vec3d X;
        if (triangulate(track, poses, best.inliers, X)) {
            Point refined = evaluate(X, track, poses, cameras, options);
            if (refined.valid && refined.inliers.size() >= best.inliers.size())
                best = std::move(refined);
        }
    }
    return best;
}

Statistics statistics(const std::vector<Track>& tracks, const std::vector<Pose>& poses,
                       const CalibrationSet& cameras, const MarkerlessSolverOptions& options,
                       bool validation) {
    Statistics stats;
    stats.possible_views.resize(poses.size());
    stats.supported_views.resize(poses.size());
    for (const auto& track : tracks) {
        if (track.validation != validation) continue;
        ++stats.possible_points;
        for (std::size_t c = 0; c < poses.size(); ++c)
            if (track.views[c].present) ++stats.possible_views[c];
        const Point point = reconstruct(track, poses, cameras, options);
        if (!point.valid) continue;
        stats.points.push_back(point.X);
        stats.angles.push_back(point.angle_deg);
        for (const auto c : point.inliers) ++stats.supported_views[c];
        // Score all present views, including rejected views. Reporting only
        // the fitted inlier errors would make the held-out test tautological.
        for (std::size_t c = 0; c < poses.size(); ++c) {
            if (!poses[c].known || !track.views[c].present) continue;
            stats.errors.push_back(reprojection(point.X, poses[c], cameras.cameras[c].intrinsics,
                                                 track.views[c].pixel));
        }
    }
    return stats;
}

double structure_ratio(const std::vector<cv::Vec3d>& points) {
    if (points.size() < 4) return 0.0;
    cv::Vec3d mean{0, 0, 0};
    for (const auto& point : points) mean += point;
    mean *= 1.0 / static_cast<double>(points.size());
    cv::Matx33d covariance = cv::Matx33d::zeros();
    for (const auto& point : points) {
        const auto d = point - mean;
        covariance += d * d.t();
    }
    cv::Vec3d values;
    cv::eigen(covariance, values);
    if (values[0] < 1.0e-12) return 0.0;
    return std::sqrt(std::max(0.0, values[2]) / values[0]);
}

bool initialize_pair(const std::vector<Track>& tracks, const CalibrationSet& cameras,
                     std::size_t second, const MarkerlessSolverOptions& options,
                     std::vector<Pose>& poses, std::size_t& support) {
    std::vector<cv::Point2d> a, b;
    for (const auto& track : tracks) {
        if (!track.validation && track.views[0].present && track.views[second].present) {
            a.push_back(track.views[0].normalized);
            b.push_back(track.views[second].normalized);
        }
    }
    if (a.size() < options.min_pair_points) return false;
    const double f = std::sqrt(focal(cameras.cameras[0].intrinsics)
                               * focal(cameras.cameras[second].intrinsics));
    const double threshold = options.ransac_threshold_px / f;
    cv::Mat mask;
    auto E = cv::findEssentialMat(a, b, cv::Matx33d::eye(), cv::RANSAC,
                                  0.999, threshold, 2000, mask);
    if (E.empty()) return false;
    // A plane or pure rotation supports a homography too well to establish a
    // dependable baseline. Do not accept an arbitrary essential decomposition.
    cv::Mat homography_mask;
    const auto H = cv::findHomography(a, b, cv::RANSAC, threshold, homography_mask,
                                      2000, 0.999);
    if (!H.empty() && cv::countNonZero(homography_mask) >= static_cast<int>(0.95 * a.size()))
        return false;
    // The minimal five-point RANSAC model can carry a rotation bias even if
    // nearly every noisy correspondence is an inlier. Refit its consensus
    // using all training correspondences, then impose the calibrated-camera
    // essential constraint (two equal nonzero singular values).
    std::vector<cv::Point2d> inlier_a, inlier_b;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (mask.at<unsigned char>(static_cast<int>(i)) == 0) continue;
        inlier_a.push_back(a[i]);
        inlier_b.push_back(b[i]);
    }
    if (inlier_a.size() >= 8) {
        const cv::Mat all_inlier = cv::findFundamentalMat(inlier_a, inlier_b, cv::FM_8POINT);
        if (!all_inlier.empty() && cv::checkRange(all_inlier)) {
            const cv::SVD svd(all_inlier, cv::SVD::FULL_UV);
            const double s = (svd.w.at<double>(0) + svd.w.at<double>(1)) * 0.5;
            const cv::Mat diagonal = (cv::Mat_<double>(3, 3) << s, 0, 0, 0, s, 0, 0, 0, 0);
            E = svd.u * diagonal * svd.vt;
        }
    }
    cv::Mat R, t;
    const int inliers = cv::recoverPose(E, a, b, cv::Matx33d::eye(), R, t, 1.0e6, mask);
    if (inliers < static_cast<int>(options.min_pair_points)) return false;
    poses.assign(cameras.cameras.size(), Pose{});
    poses[0].known = true;
    poses[second].known = true;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) poses[second].R(r, c) = R.at<double>(r, c);
    poses[second].t = {t.at<double>(0), t.at<double>(1), t.at<double>(2)};
    const double norm = cv::norm(poses[second].t);
    if (!std::isfinite(norm) || norm <= 1.0e-12) return false;
    poses[second].t *= 1.0 / norm;
    const auto stats = statistics(tracks, poses, cameras, options, false);
    support = stats.points.size();
    return support >= options.min_pair_points
        && percentile(stats.angles, 0.5) >= options.min_ray_angle_deg
        && structure_ratio(stats.points) >= options.min_structure_ratio;
}

void pose_vectors(const Pose& pose, cv::Mat& rotation, cv::Mat& translation) {
    cv::Rodrigues(pose.R, rotation);
    translation = cv::Mat(pose.t).clone();
}

Pose from_vectors(const cv::Mat& rotation, const cv::Mat& translation) {
    Pose pose;
    cv::Mat R;
    cv::Rodrigues(rotation, R);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) pose.R(r, c) = R.at<double>(r, c);
    pose.t = {translation.at<double>(0), translation.at<double>(1), translation.at<double>(2)};
    pose.known = true;
    return pose;
}

bool initialize_camera(std::size_t camera, const std::vector<Track>& tracks,
                       const CalibrationSet& cameras, const MarkerlessSolverOptions& options,
                       std::vector<Pose>& poses) {
    std::vector<cv::Point3d> object;
    std::vector<cv::Point2d> image;
    for (const auto& track : tracks) {
        if (track.validation || !track.views[camera].present) continue;
        const auto point = reconstruct(track, poses, cameras, options);
        if (!point.valid) continue;
        object.emplace_back(point.X[0], point.X[1], point.X[2]);
        image.push_back(track.views[camera].normalized);
    }
    if (object.size() < options.min_pair_points) return false;
    const double threshold = options.ransac_threshold_px / focal(cameras.cameras[camera].intrinsics);
    cv::Mat rotation, translation, inliers;
    if (!cv::solvePnPRansac(object, image, cv::Matx33d::eye(), cv::Mat(), rotation,
                            translation, false, 200, static_cast<float>(threshold),
                            0.999, inliers, cv::SOLVEPNP_EPNP)
        || inliers.total() < options.min_pair_points) return false;
    std::vector<cv::Point3d> object_inliers;
    std::vector<cv::Point2d> image_inliers;
    for (std::size_t i = 0; i < inliers.total(); ++i) {
        const int index = inliers.at<int>(static_cast<int>(i));
        object_inliers.push_back(object[static_cast<std::size_t>(index)]);
        image_inliers.push_back(image[static_cast<std::size_t>(index)]);
    }
    cv::solvePnPRefineLM(object_inliers, image_inliers, cv::Matx33d::eye(), cv::Mat(),
                         rotation, translation);
    const Pose pose = from_vectors(rotation, translation);
    if (!finite(pose.t)) return false;
    std::size_t positive = 0;
    for (const auto& point : object_inliers)
        if ((pose.R * cv::Vec3d(point.x, point.y, point.z) + pose.t)[2] > 1.0e-8) ++positive;
    if (positive * 10 < object_inliers.size() * 9) return false;
    poses[camera] = pose;
    return true;
}

void refine(const std::vector<Track>& tracks, const CalibrationSet& cameras,
            std::size_t baseline_camera, const MarkerlessSolverOptions& options,
            std::vector<Pose>& poses) {
    auto best_stats = statistics(tracks, poses, cameras, options, false);
    double best_error = percentile(best_stats.errors, 0.5);
    for (int iteration = 0; iteration < options.max_refine_iterations; ++iteration) {
        std::vector<Point> points;
        points.reserve(tracks.size());
        for (const auto& track : tracks)
            points.push_back(track.validation ? Point{} : reconstruct(track, poses, cameras, options));
        auto candidate = poses;
        bool updated = false;
        for (std::size_t c = 1; c < poses.size(); ++c) {
            std::vector<cv::Point3d> object;
            std::vector<cv::Point2d> image;
            for (std::size_t k = 0; k < tracks.size(); ++k) {
                const auto& point = points[k];
                if (!point.valid || std::find(point.inliers.begin(), point.inliers.end(), c)
                                      == point.inliers.end()) continue;
                object.emplace_back(point.X[0], point.X[1], point.X[2]);
                image.push_back(tracks[k].views[c].normalized);
            }
            if (object.size() < options.min_pair_points) continue;
            cv::Mat rotation, translation;
            pose_vectors(poses[c], rotation, translation);
            cv::solvePnPRefineLM(object, image, cv::Matx33d::eye(), cv::Mat(), rotation,
                                 translation, cv::TermCriteria(cv::TermCriteria::COUNT
                                     | cv::TermCriteria::EPS, 10, 1.0e-9));
            candidate[c] = from_vectors(rotation, translation);
            updated = updated || finite(candidate[c].t);
        }
        if (!updated) break;
        const double scale = cv::norm(candidate[baseline_camera].t);
        if (!std::isfinite(scale) || scale <= 1.0e-12) break;
        for (auto& pose : candidate) pose.t *= 1.0 / scale;
        auto candidate_stats = statistics(tracks, candidate, cameras, options, false);
        const double error = percentile(candidate_stats.errors, 0.5);
        if (!std::isfinite(error) || error > best_error
            || candidate_stats.points.size() * 100 < best_stats.points.size() * 95) break;
        poses = std::move(candidate);
        const double improvement = best_error - error;
        best_error = error;
        best_stats = std::move(candidate_stats);
        if (improvement < 1.0e-7) break;
    }
}

bool valid_options(const MarkerlessSolverOptions& options) {
    return std::isfinite(options.min_score) && options.min_score >= 0.0 && options.min_score <= 1.0
        && options.min_frame_sets >= 3 && options.min_pair_points >= 8
        && options.min_pair_points <= 100000 && options.holdout_stride >= 2
        && options.holdout_stride <= 1000
        && std::isfinite(options.ransac_threshold_px) && options.ransac_threshold_px > 0.0
        && std::isfinite(options.max_reprojection_px) && options.max_reprojection_px > 0.0
        && std::isfinite(options.min_ray_angle_deg) && options.min_ray_angle_deg > 0.0
        && options.min_ray_angle_deg < 90.0
        && std::isfinite(options.min_structure_ratio) && options.min_structure_ratio > 0.0
        && options.min_structure_ratio < 1.0
        && options.max_refine_iterations >= 0 && options.max_refine_iterations <= 100;
}

}  // namespace

MarkerlessRelativeResult solve_markerless_relative(
    const CalibrationSet& cameras, const std::vector<MarkerlessFrameSet>& samples,
    const MarkerlessSolverOptions& options) {
    MarkerlessRelativeResult result;
    try {
        if (!valid_options(options)) {
            result.reason = "invalid solver options";
            return result;
        }
        result.reason = validate_markerless_inputs(cameras, samples, options.max_sync_skew_ms);
        if (!result.reason.empty()) return result;
        if (samples.size() < options.min_frame_sets) {
            result.reason = "insufficient frame sets";
            return result;
        }
        for (std::size_t i = 0; i < samples.size(); ++i) {
            if (i % options.holdout_stride == options.holdout_stride - 1)
                ++result.validation_frame_sets;
            else ++result.training_frame_sets;
        }
        if (result.validation_frame_sets == 0) {
            result.reason = "no held-out frame sets";
            return result;
        }
        const auto tracks = make_tracks(cameras, samples, options);
        std::vector<Pose> poses;
        std::size_t baseline = 0, largest_support = 0;
        for (std::size_t c = 1; c < cameras.cameras.size(); ++c) {
            std::vector<Pose> candidate;
            std::size_t support = 0;
            if (initialize_pair(tracks, cameras, c, options, candidate, support)
                && support > largest_support) {
                poses = std::move(candidate);
                baseline = c;
                largest_support = support;
            }
        }
        if (baseline == 0) {
            result.reason = "insufficient or degenerate reference-camera geometry";
            return result;
        }
        for (std::size_t c = 1; c < cameras.cameras.size(); ++c) {
            if (poses[c].known) continue;
            if (!initialize_camera(c, tracks, cameras, options, poses)) {
                result.reason = "camera has insufficient common 3D tracks: " + cameras.cameras[c].id;
                return result;
            }
        }
        refine(tracks, cameras, baseline, options, poses);
        const auto training = statistics(tracks, poses, cameras, options, false);
        const auto validation = statistics(tracks, poses, cameras, options, true);
        const auto minimum_validation = std::max<std::size_t>(6, options.min_pair_points / 2);
        if (training.points.size() < options.min_pair_points
            || validation.points.size() < minimum_validation
            || training.points.size() * 10 < training.possible_points * 6
            || validation.points.size() * 10 < validation.possible_points * 6) {
            result.reason = "insufficient geometrically supported training or held-out points";
            return result;
        }
        for (std::size_t c = 0; c < cameras.cameras.size(); ++c) {
            if (training.supported_views[c] < options.min_pair_points
                || validation.supported_views[c] < 6
                || training.supported_views[c] * 10 < training.possible_views[c] * 6
                || validation.supported_views[c] * 10 < validation.possible_views[c] * 6) {
                result.reason = "camera lacks independent held-out support: " + cameras.cameras[c].id;
                return result;
            }
        }
        result.inlier_points = training.points.size();
        result.structure_ratio = structure_ratio(training.points);
        result.median_ray_angle_deg = percentile(training.angles, 0.5);
        result.training_reprojection_median_px = percentile(training.errors, 0.5);
        result.validation_reprojection_median_px = percentile(validation.errors, 0.5);
        result.validation_reprojection_p95_px = percentile(validation.errors, 0.95);
        if (result.structure_ratio < options.min_structure_ratio
            || result.median_ray_angle_deg < options.min_ray_angle_deg) {
            result.reason = "degenerate reconstructed structure or parallax";
            return result;
        }
        if (result.validation_reprojection_median_px > options.max_reprojection_px
            || result.validation_reprojection_p95_px > options.max_reprojection_px) {
            result.reason = "held-out reprojection validation failed";
            return result;
        }
        for (std::size_t c = 0; c < cameras.cameras.size(); ++c) {
            if (!finite(poses[c].t) || !cv::checkRange(cv::Mat(poses[c].R))
                || std::abs(cv::determinant(poses[c].R) - 1.0) > 1.0e-6) {
                result.reason = "invalid recovered camera pose";
                result.cameras.clear();
                return result;
            }
            result.cameras.push_back({cameras.cameras[c].id, matrix(poses[c])});
        }
        result.reference_camera_id = cameras.cameras[0].id;
        result.baseline_camera_id = cameras.cameras[baseline].id;
        result.reason = "relative geometry only; metric scale and floor remain unresolved";
        result.ok = true;
    } catch (const std::exception& error) {
        result.ok = false;
        result.cameras.clear();
        result.reason = std::string("markerless geometry failed: ") + error.what();
    }
    return result;
}

}  // namespace fitra::lift
