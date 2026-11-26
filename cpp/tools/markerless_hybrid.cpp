// One-shot external calibration using static scene geometry and raw Halpe26 observations.
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>

#include "lift/calib_io.hpp"
#include "lift/keypoint_format.hpp"
#include "lift/markerless_io.hpp"
#include "lift/triangulator.hpp"

namespace {
namespace fs = std::filesystem;
using namespace fitra::lift;
using Vec = cv::Vec3d;
using Mat = cv::Matx33d;
using Pixels = std::array<cv::Point2d, 3>;
using People = std::array<fitra::infer::Person, 3>;
using Rig = std::array<cv::Matx44d, 3>;

void require(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error(message);
}

double percentile(std::vector<double> values, double fraction) {
    require(!values.empty(), "empty sample");
    std::sort(values.begin(), values.end());
    const double at = fraction * (values.size() - 1);
    const auto lo = static_cast<std::size_t>(at);
    const auto hi = std::min(lo + 1, values.size() - 1);
    return values[lo] + (at - lo) * (values[hi] - values[lo]);
}

Vec unit(const Vec& v) {
    const double length = cv::norm(v);
    require(length > 1e-9 && std::isfinite(length), "undefined direction");
    return v / length;
}

Mat rot(const cv::Matx44d& T) { return T.get_minor<3, 3>(0, 0); }
Vec trans(const cv::Matx44d& T) { return {T(0, 3), T(1, 3), T(2, 3)}; }
cv::Matx44d transform(const Mat& R, const Vec& t) {
    auto T = cv::Matx44d::eye();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j)
            T(i, j) = R(i, j);
        T(i, 3) = t[i];
    }
    return T;
}

cv::Point2d normalized(const Intrinsics& lens, const cv::Point2d& pixel) {
    std::vector<cv::Point2d> input{pixel}, output;
    if (lens.is_fisheye())
        cv::fisheye::undistortPoints(input, output, lens.K, lens.dist);
    else
        cv::undistortPoints(input, output, lens.K, lens.dist);
    return output.at(0);
}

cv::Point2d project(const Intrinsics& lens, const Vec& local) {
    const std::vector<cv::Point3d> object{{local[0], local[1], local[2]}};
    std::vector<cv::Point2d> result;
    if (lens.is_fisheye())
        cv::fisheye::projectPoints(object, result, Vec{}, Vec{}, lens.K, lens.dist);
    else
        cv::projectPoints(object, Vec{}, Vec{}, lens.K, lens.dist, result);
    return result.at(0);
}

struct Scene {
    Rig rig;
    std::vector<Pixels> fit, check;
};

Scene load_scene(const std::string& path, const CalibrationSet& intr) {
    cv::FileStorage in(path, cv::FileStorage::READ);
    require(in.isOpened() && std::string(in["schema"]) == "fitra_markerless_background_v1",
            "invalid background file");
    require(intr.cameras.size() == 3, "expected three cameras");
    Scene scene;
    for (int i = 0; i < 3; ++i) {
        const auto suffix = std::to_string(i);
        const auto& c = intr.cameras[i];
        const auto& lens = c.intrinsics;
        cv::Mat T, K, D;
        in["T" + suffix] >> T;
        in["K" + suffix] >> K;
        in["D" + suffix] >> D;
        require(std::string(in["id" + suffix]) == c.id && c.id == "cam" + suffix &&
                    std::string(in["lens" + suffix]) == lens.distortion_model &&
                    int(in["width" + suffix]) == lens.width && int(in["height" + suffix]) == lens.height &&
                    K.size() == lens.K.size() && D.total() == lens.dist.total() &&
                    cv::norm(K, lens.K, cv::NORM_INF) < 1e-9 &&
                    cv::norm(D.reshape(1, 1), lens.dist.reshape(1, 1), cv::NORM_INF) < 1e-9,
                "background camera identity or intrinsics mismatch");
        require(T.type() == CV_64F && T.rows == 4 && T.cols == 4 && cv::checkRange(T),
                "invalid background transform");
        scene.rig[i] = cv::Matx44d(T);
        const auto R = rot(scene.rig[i]);
        require(cv::norm(R * R.t() - Mat::eye()) < 1e-6 && std::abs(cv::determinant(R) - 1) < 1e-6 &&
                    cv::norm(T.row(3), cv::Mat(cv::Vec4d(0, 0, 0, 1)).t()) < 1e-8,
                "non-rigid background pose");
    }
    require(cv::norm(scene.rig[0] - cv::Matx44d::eye()) < 1e-7 &&
                std::abs(cv::norm(trans(scene.rig[1])) - 1) < 1e-6,
            "expected camera 0 and unit 01 baseline");
    cv::Mat points;
    in["background_tracks"] >> points;
    require(points.type() == CV_64F && points.cols == 6 && points.rows >= 12 && points.rows <= 20000 &&
                cv::checkRange(points),
            "invalid static tracks");
    for (int n = 0; n < points.rows; ++n) {
        Pixels triplet;
        for (int c = 0; c < 3; ++c) {
            triplet[c] = {points.at<double>(n, 2 * c), points.at<double>(n, 2 * c + 1)};
            const auto& l = intr.cameras[c].intrinsics;
            require(triplet[c].x >= 0 && triplet[c].x < l.width && triplet[c].y >= 0 &&
                        triplet[c].y < l.height,
                    "background observation outside image");
        }
        const int cell = int(triplet[0].x / 64) + 3 * int(triplet[0].y / 64);
        (cell % 5 == 0 ? scene.check : scene.fit).push_back(triplet);
    }
    require(scene.check.size() >= 4 && scene.fit.size() >= 8,
            "background lacks spatial validation coverage");
    return scene;
}

std::vector<People> body(const std::vector<MarkerlessFrameSet>& sets, const CalibrationSet& intr) {
    std::vector<People> result;
    for (const auto& set : sets) {
        People group{};
        for (const auto& frame : set.frames) {
            auto it = std::find_if(intr.cameras.begin(), intr.cameras.end(),
                                   [&](const auto& c) { return c.id == frame.camera_id; });
            require(it != intr.cameras.end(), "unknown body camera");
            auto& person = group[static_cast<std::size_t>(it - intr.cameras.begin())];
            person.kp_count = 26;
            for (const auto& px : frame.keypoints)
                person.kpts.at(px.joint_index) = {float(px.pixel.x), float(px.pixel.y), float(px.score)};
        }
        result.push_back(group);
    }
    return result;
}

std::vector<Pixels> body_tracks(const std::vector<People>& input) {
    std::vector<Pixels> result;
    for (const auto& people : input) {
        for (const auto k : {5, 6, 7, 8, 11, 12, 13, 14}) {
            Pixels p{};
            bool ok = true;
            for (int i = 0; i < 3; ++i) {
                const auto& kp = people[i].kpts[k];
                ok = ok && kp.score >= 0.55f;
                p[i] = {kp.x, kp.y};
            }
            if (ok)
                result.push_back(p);
        }
    }
    return result;
}

// Intersect two observed rays. The third camera is excluded from this 3D estimate.
bool prediction(const CalibrationSet& intr, const Rig& rig, const Pixels& pixels, int held, Vec& point,
                double& error) {
    const int a = (held + 1) % 3, b = (held + 2) % 3;
    const auto ray = [&](int i) {
        auto p = normalized(intr.cameras[i].intrinsics, pixels[i]);
        return unit(rot(rig[i]).t() * Vec(p.x, p.y, 1));
    };
    const Vec ra = ray(a), rb = ray(b), ca = -rot(rig[a]).t() * trans(rig[a]),
              cb = -rot(rig[b]).t() * trans(rig[b]);
    const Vec between = cb - ca;
    const double cosine = ra.dot(rb), denom = 1. - cosine * cosine;
    if (denom < 0.001)
        return false;
    const double da = (between.dot(ra) - cosine * between.dot(rb)) / denom;
    const double db = (cosine * between.dot(ra) - between.dot(rb)) / denom;
    if (da <= 0 || db <= 0)
        return false;
    point = (ca + da * ra + cb + db * rb) * .5;
    const Vec local = rot(rig[held]) * point + trans(rig[held]);
    if (local[2] <= 0)
        return false;
    error = cv::norm(project(intr.cameras[held].intrinsics, local) - pixels[held]);
    return std::isfinite(error);
}

struct Error {
    double median, p90;
};
Error errors(const CalibrationSet& intr, const Rig& rig, const std::vector<Pixels>& tracks) {
    std::vector<double> e;
    for (const auto& p : tracks) {
        for (int i = 0; i < 3; ++i) {
            Vec X;
            double err = 0;
            e.push_back(prediction(intr, rig, p, i, X, err) ? err : 10000.);
        }
    }
    return {percentile(e, .5), percentile(e, .9)};
}

Rig candidate(const CalibrationSet& intr, const Scene& scene, const std::vector<Pixels>& joints) {
    Rig output = scene.rig;
    for (int i = 1; i < 3; ++i) {
        std::vector<cv::Point3d> object;
        std::vector<cv::Point2d> image;
        const auto append = [&](const Pixels& p, double cap) {
            Vec X;
            double e;
            if (prediction(intr, scene.rig, p, i, X, e) && e < cap) {
                object.emplace_back(X);
                image.push_back(normalized(intr.cameras[i].intrinsics, p[i]));
            }
        };
        for (const auto& p : scene.fit)
            append(p, 5.);
        const auto budget = std::min(joints.size(), object.size() / 2);
        for (std::size_t n = 0; n < budget; ++n)
            append(joints[n * joints.size() / budget], 12.);
        if (object.size() < 12 || budget < 4)
            return scene.rig;
        Vec rv, t = trans(scene.rig[i]);
        cv::Rodrigues(rot(scene.rig[i]), rv);
        if (!cv::solvePnP(object, image, Mat::eye(), cv::noArray(), rv, t, true, cv::SOLVEPNP_ITERATIVE))
            return scene.rig;
        Mat R;
        cv::Rodrigues(rv, R);
        const Vec old_center = -rot(scene.rig[i]).t() * trans(scene.rig[i]);
        Vec delta;
        cv::Rodrigues(R * rot(scene.rig[i]).t(), delta);
        if (!cv::checkRange(cv::Mat(rv)) || !cv::checkRange(cv::Mat(t)) || cv::norm(delta) > CV_PI / 120. ||
            cv::norm(-R.t() * t - old_center) > .03)
            return scene.rig;
        output[i] = transform(R, t);
    }
    // Preserve the seed gauge even when camera 1's center was adjusted.
    const double b = cv::norm(trans(output[1]));
    for (auto& pose : output)
        pose = transform(rot(pose), trans(pose) / b);
    return output;
}

CalibrationSet calibration(const CalibrationSet& input, const Rig& rig) {
    auto out = input;
    for (std::size_t i = 0; i < out.cameras.size(); ++i) {
        auto& c = out.cameras[i];
        c.has_extrinsics = true;
        c.extrinsics.T_cw = cv::Mat(rig[i]).clone();
        c.extrinsics.camera_center_w = -rot(rig[i]).t() * trans(rig[i]);
        c.extrinsics.method = "markerless_scene_and_body";
    }
    return out;
}

struct Stance {
    Vec head, left, right;
};
std::vector<Stance> stances(const CalibrationSet& intr, const Rig& rig, const std::vector<People>& input) {
    Triangulator lift(calibration(intr, rig), {.45f, 8.f});
    std::vector<Stance> result;
    for (const auto& p : input) {
        const auto s = lift.triangulate({{0, &p[0]}, {1, &p[1]}, {2, &p[2]}});
        const auto& j = s.skeleton.joints;
        const auto xyz = [&](int k) { return Vec(j[k].x, j[k].y, j[k].z); };
        const auto good = [&](int k) {
            return j[k].valid && s.view_count[k] >= 2 && s.max_ray_angle_deg[k] >= 3.;
        };
        if (!good(17))
            continue;
        Stance step;
        step.head = xyz(17);
        bool upright = true;
        for (int side = 0; side < 2; ++side) {
            const int hip = 11 + side, knee = 13 + side, ankle = 15 + side;
            if (!good(hip) || !good(knee) || !good(ankle)) {
                upright = false;
                break;
            }
            const auto thigh = xyz(hip) - xyz(knee), shank = xyz(knee) - xyz(ankle);
            if (cv::norm(thigh) < 1e-5 || cv::norm(shank) < 1e-5 || unit(thigh).dot(unit(shank)) < .78) {
                upright = false;
                break;
            }
            std::vector<Vec> foot;
            for (int k : {20 + side, 22 + side, 24 + side})
                if (good(k))
                    foot.push_back(xyz(k));
            if (foot.size() < 2) {
                upright = false;
                break;
            }
            Vec center{};
            for (const auto& x : foot)
                center += x;
            (side == 0 ? step.left : step.right) = center / double(foot.size());
        }
        if (upright && cv::norm(step.head - (step.left + step.right) * .5) > 1e-5)
            result.push_back(step);
    }
    return result;
}

struct Ground {
    Vec origin, normal;
    double scale, height, inlier_fraction, span_ratio;
};
Ground fit_ground(const std::vector<Stance>& poses, double height) {
    require(poses.size() >= 6, "too few upright, two-foot observations for floor and stature");
    std::vector<Vec> feet;
    std::vector<double> lengths;
    Vec up{};
    for (const auto& p : poses) {
        const Vec axis = p.head - (p.left + p.right) * .5;
        lengths.push_back(cv::norm(axis));
        up += unit(axis);
        feet.push_back(p.left);
        feet.push_back(p.right);
    }
    up = unit(up);
    const double typical = percentile(lengths, .5), band = .016 * typical;
    std::mt19937 rng(66);
    std::uniform_int_distribution<std::size_t> idx(0, feet.size() - 1);
    std::vector<Vec> best;
    double best_cost = std::numeric_limits<double>::infinity();
    for (int iter = 0; iter < 500; ++iter) {
        const Vec p = feet[idx(rng)], q = feet[idx(rng)], r = feet[idx(rng)];
        Vec n = (q - p).cross(r - p);
        if (cv::norm(n) < typical * typical * .0001)
            continue;
        n = unit(n);
        if (std::abs(n.dot(up)) < .94)
            continue;
        std::vector<Vec> support;
        double cost = 0;
        for (const auto& x : feet) {
            const double distance = std::abs((x - p).dot(n));
            if (distance <= band) {
                support.push_back(x);
                cost += distance * distance;
            }
        }
        if (support.size() > best.size() || (support.size() == best.size() && cost < best_cost)) {
            best = std::move(support);
            best_cost = cost;
        }
    }
    require(best.size() >= 6 && best.size() >= feet.size() * .60,
            "no supported floor plane from planted feet");
    Vec center{};
    for (const auto& x : best)
        center += x;
    center /= double(best.size());
    Mat covariance = Mat::zeros();
    for (const auto& x : best) {
        const Vec v = x - center;
        covariance += v * v.t();
    }
    cv::Mat eigenvalues, axes;
    cv::eigen(cv::Mat(covariance) / double(best.size()), eigenvalues, axes);
    const double spread = std::sqrt(std::max(0., eigenvalues.at<double>(1))) / typical;
    require(spread > .02, "foot positions do not span two floor directions");
    Vec normal(axes.at<double>(2, 0), axes.at<double>(2, 1), axes.at<double>(2, 2));
    if (normal.dot(up) < 0)
        normal = -normal;
    require(normal.dot(up) > .94, "fitted floor disagrees with upright body direction");
    std::vector<double> heights;
    for (const auto& p : poses) {
        if (std::max(std::abs(normal.dot(p.left - center)), std::abs(normal.dot(p.right - center))) >
            2 * band)
            continue;
        heights.push_back(normal.dot(p.head - center));
    }
    require(heights.size() >= 5, "too few grounded head-to-floor measurements");
    const double raw = percentile(heights, .5);
    require(raw > .01 && percentile(heights, .9) - percentile(heights, .1) < .12 * raw,
            "inconsistent stature in fitting poses");
    return {center, normal, height / raw, height, double(best.size()) / feet.size(), spread};
}

Rig metres(const Rig& relative, const Ground& g) {
    // Align world X with camera 0's horizontal image axis projected onto the floor.
    Vec x = Vec(1, 0, 0) - g.normal * g.normal[0];
    x = unit(x);
    const Vec y = g.normal.cross(x);
    const Mat axes(x[0], y[0], g.normal[0], x[1], y[1], g.normal[1], x[2], y[2], g.normal[2]);
    Rig result;
    for (int i = 0; i < 3; ++i)
        result[i] = transform(rot(relative[i]) * axes,
                              g.scale * (rot(relative[i]) * g.origin + trans(relative[i])));
    return result;
}

struct VerifiedBody {
    double floor_p90, height_p90;
};
VerifiedBody check_metric(const std::vector<Stance>& samples, const Ground& ground) {
    require(samples.size() >= 4, "too few independent upright samples");
    std::vector<double> feet, heads;
    for (const auto& p : samples) {
        for (const auto& x : {p.left, p.right})
            feet.push_back(std::abs(ground.scale * ground.normal.dot(x - ground.origin)));
        heads.push_back(std::abs(ground.scale * ground.normal.dot(p.head - ground.origin) - ground.height));
    }
    return {percentile(feet, .9), percentile(heads, .9)};
}

MarkerlessReplay read_body(const fs::path& path, const CalibrationSet& intr) {
    std::ifstream stream(path);
    require(bool(stream), "cannot open body observations");
    auto input = read_markerless_observations(stream, intr, {1024, 20.});
    require(input.input_clean() && input.collection.resets == 0 && input.collection.evicted == 0 &&
                input.frame_sets.size() >= 4,
            "unclean or insufficient body observations: " + path.string() +
                (input.diagnostics.empty() ? "" : " (" + input.diagnostics.front() + ")"));
    return input;
}

void check_split(const std::vector<MarkerlessFrameSet>& fit, const std::vector<MarkerlessFrameSet>& hold) {
    const auto& end = fit.back();
    const auto& begin = hold.front();
    require(end.continuity_id == begin.continuity_id, "different subjects or sessions across time blocks");
    for (const auto& f : end.frames) {
        const auto it = std::find_if(begin.frames.begin(), begin.frames.end(),
                                     [&](const auto& c) { return c.camera_id == f.camera_id; });
        require(it != begin.frames.end() && it->stream_epoch == f.stream_epoch &&
                    it->timestamp_kind == f.timestamp_kind && it->source_seq > f.source_seq &&
                    it->capture_mono_ns > f.capture_mono_ns &&
                    it->capture_mono_ns - f.capture_mono_ns >= 500000000LL,
                "fitting and validation blocks overlap or lack time separation");
    }
}

void run(const char* intrinsic, const char* background, const char* fitting, const char* held,
         double height, const fs::path& output) {
    require(std::isfinite(height) && height > .4 && height < 2.7, "invalid entered height in metres");
    require(!fs::exists(output), "output must be a new directory");
    auto intr = load_calibration(intrinsic);
    // Output slots and the background triplets are named cam0, cam1 and cam2.
    std::sort(intr.cameras.begin(), intr.cameras.end(),
              [](const auto& lhs, const auto& rhs) { return lhs.id < rhs.id; });
    validate_calibration(intr);
    const auto scene = load_scene(background, intr);
    const auto a = read_body(fitting, intr), b = read_body(held, intr);
    check_split(a.frame_sets, b.frame_sets);
    auto people = body(a.frame_sets, intr), test = body(b.frame_sets, intr);
    const auto fitting_tracks = body_tracks(people), checking_tracks = body_tracks(test);
    require(checking_tracks.size() >= 12, "insufficient three-view body validation support");
    const auto refined = candidate(intr, scene, fitting_tracks);
    const auto scene_before = errors(intr, scene.rig, scene.check),
               scene_after = errors(intr, refined, scene.check);
    const auto body_before = errors(intr, scene.rig, checking_tracks),
               body_after = errors(intr, refined, checking_tracks);
    const bool improved =
        body_after.median + .1 < body_before.median && body_after.p90 <= body_before.p90 &&
        scene_after.median <= scene_before.median + .25 && scene_after.p90 <= scene_before.p90 + .5;
    const Rig& relative = improved ? refined : scene.rig;
    const auto selected = improved ? body_after : body_before;
    const auto bg = improved ? scene_after : scene_before;
    std::cout << "body_validation_px median/p90 " << body_before.median << "/" << body_before.p90 << " -> "
              << body_after.median << "/" << body_after.p90 << " background " << scene_before.median << "/"
              << scene_before.p90 << " -> " << scene_after.median << "/" << scene_after.p90
              << " corrected=" << improved << '\n';
    require(bg.median <= 4. && bg.p90 <= 8. && selected.median <= 12. && selected.p90 <= 50.,
            "independent image geometry rejected");
    const auto standing = stances(intr, relative, people), validation = stances(intr, relative, test);
    const auto ground = fit_ground(standing, height);
    const auto checks = check_metric(validation, ground);
    std::cout << "upright_fit=" << standing.size() << " upright_check=" << validation.size()
              << " baseline_m=" << ground.scale << " floor_p90_m=" << checks.floor_p90
              << " height_p90_m=" << checks.height_p90 << '\n';
    require(checks.floor_p90 <= .08 && checks.height_p90 <= .12,
            "metric stature or floor inconsistent on separate time block");
    auto output_calib = calibration(intr, metres(relative, ground));
    output_calib.unit = "m";
    output_calib.coordinate_system = "world: x/y on floor, z up (FitraWorld); extrinsics are T_cw";
    validate_calibration(output_calib);
    cv::FileStorage report("report.json",
                           cv::FileStorage::WRITE | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
    require(report.isOpened(), "cannot write report");
    report << "schema"
           << "fitra_markerless_hybrid_report_v1"
           << "background_fit_tracks" << int(scene.fit.size()) << "background_validation_tracks"
           << int(scene.check.size()) << "validation_body_tracks" << int(checking_tracks.size())
           << "height_m" << height << "baseline01_m" << ground.scale << "body_extrinsic_correction"
           << int(improved) << "fit_sets" << int(people.size()) << "validation_sets" << int(test.size())
           << "fit_stances" << int(standing.size()) << "validation_stances" << int(validation.size())
           << "body_fixed_p50_px" << body_before.median << "body_fixed_p90_px" << body_before.p90
           << "body_candidate_p50_px" << body_after.median << "body_candidate_p90_px" << body_after.p90
           << "background_fixed_p50_px" << scene_before.median << "background_fixed_p90_px"
           << scene_before.p90 << "background_candidate_p50_px" << scene_after.median
           << "background_candidate_p90_px" << scene_after.p90 << "floor_support_fraction"
           << ground.inlier_fraction << "floor_minor_spread_over_stature" << ground.span_ratio
           << "validation_foot_p90_m" << checks.floor_p90 << "validation_stature_p90_m"
           << checks.height_p90;
    const auto diagnostics = report.releaseAndGetString();
    const auto pending = output / "calibration-pending.yml";
    const auto report_path = output / "report.json";
    require(fs::create_directory(output), "cannot reserve new calibration output");
    try {
        write_calibration(pending.string(), output_calib);
        const auto saved = load_calibration(pending.string());
        validate_calibration(saved);
        require(saved.cameras.size() == 3 && std::all_of(saved.cameras.begin(), saved.cameras.end(),
                                                         [](const auto& c) { return c.has_extrinsics; }),
                "incomplete calibration file");
        write_markerless_report_new(report_path.string(), diagnostics);
        fs::rename(pending, output / "calibration.yaml");
    } catch (...) {
        std::error_code error;
        fs::remove(pending, error);
        fs::remove(report_path, error);
        fs::remove(output, error);
        throw;
    }
}
} // namespace

int main(int argc, char** argv) {
    if (argc != 7) {
        std::cerr
            << "markerless_hybrid INTRINSICS BACKGROUND FIT_JSONL VALIDATION_JSONL HEIGHT_M NEW_DIR\n";
        return argc == 2 && std::string(argv[1]) == "--help" ? 0 : 2;
    }
    try {
        fitra::lift::set_active_keypoint_format(fitra::lift::KeypointFormat::Halpe26);
        std::size_t n = 0;
        const std::string arg = argv[5];
        const double height = std::stod(arg, &n);
        require(n == arg.size(), "invalid height");
        run(argv[1], argv[2], argv[3], argv[4], height, argv[6]);
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "markerless_hybrid: " << ex.what() << '\n';
        return 1;
    }
}
