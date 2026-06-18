#include "lift/floor_extrinsic_solver.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>

namespace fitra::lift {

namespace {

// Std deviation along the minimum principal axis of a 3D point set — a measure
// of how "thick" the cloud is out of its best-fit plane. ~0 means coplanar.
double min_axis_thickness(const std::vector<cv::Point3f>& pts) {
    if (pts.size() < 3) return 0.0;
    cv::Vec3d mean(0, 0, 0);
    for (const auto& p : pts) mean += cv::Vec3d(p.x, p.y, p.z);
    mean *= 1.0 / static_cast<double>(pts.size());
    cv::Matx33d cov = cv::Matx33d::zeros();
    for (const auto& p : pts) {
        cv::Vec3d d(p.x - mean[0], p.y - mean[1], p.z - mean[2]);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) cov(r, c) += d[r] * d[c];
    }
    cov *= 1.0 / static_cast<double>(pts.size());
    cv::Vec3d eigvals;
    cv::eigen(cov, eigvals);  // descending; [2] is the smallest
    double smallest = eigvals[2] < 0.0 ? 0.0 : eigvals[2];
    return std::sqrt(smallest);
}

}  // namespace

FloorExtrinsicSolution solve_floor_extrinsics(
    const std::vector<FloorCameraInput>& cams,
    const FloorTagMap& map,
    const FloorSolverOptions& opts) {
    FloorExtrinsicSolution out;
    out.ok = true;

    if (cams.empty()) {
        out.ok = false;
        out.message = "no cameras";
        return out;
    }

    for (const auto& cam : cams) {
        FloorCameraSolution sol;
        sol.cam_index = cam.cam_index;

        std::vector<cv::Point3f> objp;
        std::vector<cv::Point2f> imgp;
        int n_tags = 0;
        for (const auto& ob : cam.obs) {
            const FloorTag* tag = map.find(ob.id);
            if (!tag) continue;  // a detected tag not in the map — ignore
            auto wc = map.world_corners(*tag);
            for (int i = 0; i < 4; ++i) {
                objp.emplace_back(static_cast<float>(wc[i].v[0]),
                                  static_cast<float>(wc[i].v[1]),
                                  static_cast<float>(wc[i].v[2]));
                imgp.push_back(ob.corners[i]);
            }
            ++n_tags;
        }
        sol.n_tags = n_tags;
        sol.n_points = static_cast<int>(objp.size());

        if (n_tags < opts.min_tags || sol.n_points < opts.min_points ||
            cam.K.empty()) {
            out.ok = false;
            out.message += "cam" + std::to_string(cam.cam_index) +
                           ": too few tag correspondences; ";
            out.cameras.push_back(std::move(sol));
            continue;
        }

        sol.plane_thickness_m = min_axis_thickness(objp);
        sol.planar_degenerate =
            sol.plane_thickness_m < opts.planar_warn_thickness_m;

        // For fisheye, undistort to normalised image coordinates and solve with
        // an identity camera and no distortion (pinhole PnP assumes a Brown
        // model, so the fisheye model is removed up front).
        cv::Mat rvec, tvec;
        bool ok = false;
        if (cam.fisheye) {
            std::vector<cv::Point2f> norm;
            cv::fisheye::undistortPoints(imgp, norm, cam.K, cam.dist);
            ok = cv::solvePnP(objp, norm, cv::Mat::eye(3, 3, CV_64F), cv::Mat(),
                              rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        } else {
            ok = cv::solvePnP(objp, imgp, cam.K, cam.dist, rvec, tvec, false,
                              cv::SOLVEPNP_ITERATIVE);
        }
        if (!ok) {
            out.ok = false;
            out.message += "cam" + std::to_string(cam.cam_index) +
                           ": solvePnP failed; ";
            out.cameras.push_back(std::move(sol));
            continue;
        }

        cv::Mat R;
        cv::Rodrigues(rvec, R);
        cv::Matx44d raw = cv::Matx44d::eye();
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) raw(r, c) = R.at<double>(r, c);
            raw(r, 3) = tvec.at<double>(r);
        }
        sol.T_cam_world = geom::T_cam_world::from_raw(raw);

        // Reprojection RMS over all corners (in the camera's own distortion
        // model, so we reproject the original — distorted — pixels).
        std::vector<cv::Point2f> proj;
        if (cam.fisheye) {
            cv::fisheye::projectPoints(objp, proj, rvec, tvec, cam.K, cam.dist);
        } else {
            cv::projectPoints(objp, rvec, tvec, cam.K, cam.dist, proj);
        }
        double s2 = 0.0;
        for (size_t i = 0; i < proj.size(); ++i) {
            cv::Point2f d = proj[i] - imgp[i];
            s2 += static_cast<double>(d.x) * d.x + static_cast<double>(d.y) * d.y;
        }
        sol.reproj_rms_px = std::sqrt(s2 / static_cast<double>(proj.size()));
        sol.solved = sol.reproj_rms_px <= opts.max_reproj_px;
        if (!sol.solved) {
            out.ok = false;
            out.message += "cam" + std::to_string(cam.cam_index) +
                           ": reproj " + std::to_string(sol.reproj_rms_px) +
                           "px > max; ";
        }
        out.cameras.push_back(std::move(sol));
    }

    if (out.ok) out.message = "ok";
    return out;
}

}  // namespace fitra::lift
