#include "lift/extrinsic_solver.hpp"

#include "geom/frames.hpp"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <map>

namespace fitra::lift {

namespace {

// The SE(3) helpers these used to define live in geom now (single shared
// implementation across the typed Transform layer and this solver). See
// docs/design/pose-3d-typed-coordinate-frames.md.
using geom::average_rotation;
using geom::compose;
using geom::invert_rigid;
using geom::rot_of;
using geom::trans_of;

struct GroupKey {
    int cam, face;
    bool operator<(const GroupKey& o) const {
        return cam != o.cam ? cam < o.cam : face < o.face;
    }
};

}  // namespace

// Public API kept stable; forwards to the shared geom implementation.
cv::Matx44d pose_from_pos_quat(double x, double y, double z,
                               double qx, double qy, double qz, double qw) {
    return geom::pose_from_pos_quat(x, y, z, qx, qy, qz, qw);
}

double rotation_angle_deg(const cv::Matx44d& a, const cv::Matx44d& b) {
    return geom::rotation_angle_deg(a, b);
}

cv::Matx44d average_poses(const std::vector<cv::Matx44d>& poses) {
    return geom::average_poses(poses);
}

ExtrinsicSolution solve_extrinsics(const std::vector<ExtrinsicSample>& samples,
                                   const ExtrinsicSolverOptions& opts) {
    ExtrinsicSolution out;

    // Group by (camera, face), preserving sample order.
    std::map<GroupKey, std::vector<const ExtrinsicSample*>> groups;
    for (const auto& s : samples) groups[GroupKey{s.cam_index, s.face_id}].push_back(&s);

    const cv::RobotWorldHandEyeCalibrationMethod method = (opts.method == 1)
        ? cv::CALIB_ROBOT_WORLD_HAND_EYE_LI
        : cv::CALIB_ROBOT_WORLD_HAND_EYE_SHAH;

    int solved_groups = 0;
    for (const auto& [key, items] : groups) {
        FaceSolution fs;
        fs.cam_index = key.cam;
        fs.face_id   = key.face;
        fs.n_samples = static_cast<int>(items.size());

        // Rotation diversity of the controller poses (observability proxy):
        // max geodesic angle of any B_i relative to B_0.
        double span = 0.0;
        for (std::size_t i = 1; i < items.size(); ++i) {
            span = std::max(span, rotation_angle_deg(items[0]->T_world_controller,
                                                     items[i]->T_world_controller));
        }
        fs.rotation_span_deg = span;

        if (static_cast<int>(items.size()) < std::max(3, opts.min_samples_per_group)) {
            out.faces.push_back(fs);
            continue;
        }

        // A_i = T_cam←marker (world2cam), B_i = T_world←controller (base2gripper).
        std::vector<cv::Mat> R_w2c, t_w2c, R_b2g, t_b2g;
        R_w2c.reserve(items.size());
        for (const auto* s : items) {
            cv::Matx33d Ra = rot_of(s->T_cam_marker);
            cv::Vec3d   ta = trans_of(s->T_cam_marker);
            cv::Matx33d Rb = rot_of(s->T_world_controller);
            cv::Vec3d   tb = trans_of(s->T_world_controller);
            R_w2c.push_back(cv::Mat(Ra));
            t_w2c.push_back(cv::Mat(ta));
            R_b2g.push_back(cv::Mat(Rb));
            t_b2g.push_back(cv::Mat(tb));
        }

        cv::Mat R_b2w, t_b2w, R_g2c, t_g2c;
        try {
            cv::calibrateRobotWorldHandEye(R_w2c, t_w2c, R_b2g, t_b2g,
                                           R_b2w, t_b2w, R_g2c, t_g2c, method);
        } catch (const cv::Exception& e) {
            out.message += "group(cam=" + std::to_string(key.cam) +
                           ",face=" + std::to_string(key.face) + ") solve failed; ";
            out.faces.push_back(fs);
            continue;
        }

        cv::Mat R_g2c64, R_b2w64, t_g2c64, t_b2w64;
        R_g2c.convertTo(R_g2c64, CV_64F);
        R_b2w.convertTo(R_b2w64, CV_64F);
        t_g2c.reshape(1, 3).convertTo(t_g2c64, CV_64F);
        t_b2w.reshape(1, 3).convertTo(t_b2w64, CV_64F);
        cv::Matx33d Rz(R_g2c64.ptr<double>());  // Z = T_cam←world rotation
        cv::Vec3d   tz(t_g2c64.ptr<double>());
        cv::Matx33d Rx(R_b2w64.ptr<double>());  // X = T_marker←controller rotation
        cv::Vec3d   tx(t_b2w64.ptr<double>());
        fs.T_cam_world         = compose(Rz, tz);
        fs.T_marker_controller = compose(Rx, tx);

        // Residual: measured A_i vs predicted Z·B_i·X⁻¹.
        cv::Matx44d Xinv = invert_rigid(fs.T_marker_controller);
        double sum_t2 = 0.0, sum_a2 = 0.0;
        for (const auto* s : items) {
            cv::Matx44d pred = fs.T_cam_world * s->T_world_controller * Xinv;
            cv::Vec3d dt = trans_of(s->T_cam_marker) - trans_of(pred);
            sum_t2 += dt.dot(dt);
            double da = rotation_angle_deg(s->T_cam_marker, pred);
            sum_a2 += da * da;
        }
        fs.residual_trans_rms_m = std::sqrt(sum_t2 / items.size());
        fs.residual_rot_rms_deg = std::sqrt(sum_a2 / items.size());
        fs.solved = true;
        ++solved_groups;
        out.faces.push_back(fs);
    }

    // Aggregate per camera across solved faces.
    std::map<int, std::vector<const FaceSolution*>> by_cam;
    for (const auto& fs : out.faces) {
        if (fs.solved) by_cam[fs.cam_index].push_back(&fs);
    }
    for (const auto& [cam, faces] : by_cam) {
        CameraExtrinsic ce;
        ce.cam_index = cam;
        ce.n_faces   = static_cast<int>(faces.size());

        std::vector<cv::Matx33d> rots;
        cv::Vec3d tsum(0, 0, 0);
        for (const auto* fs : faces) {
            rots.push_back(rot_of(fs->T_cam_world));
            tsum += trans_of(fs->T_cam_world);
            ce.n_samples += fs->n_samples;
        }
        cv::Matx33d Rmean = average_rotation(rots);
        cv::Vec3d   tmean = tsum / static_cast<double>(faces.size());
        ce.T_cam_world = compose(Rmean, tmean);

        // Cross-face spread relative to the mean (quality metric).
        for (const auto* fs : faces) {
            double dt = cv::norm(trans_of(fs->T_cam_world) - tmean);
            ce.face_spread_trans_m = std::max(ce.face_spread_trans_m, dt);
            double da = rotation_angle_deg(ce.T_cam_world, fs->T_cam_world);
            ce.face_spread_rot_deg = std::max(ce.face_spread_rot_deg, da);
        }
        out.cameras.push_back(ce);
    }

    out.ok = solved_groups > 0;
    if (!out.ok && out.message.empty()) {
        out.message = "no (camera, face) group met the minimum sample count";
    }
    return out;
}

}  // namespace fitra::lift
