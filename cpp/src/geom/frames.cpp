#include "geom/frames.hpp"

#include <algorithm>
#include <cmath>

namespace fitra::geom {

namespace {
constexpr double kRad2Deg = 57.295779513082323;

// Geodesic angle (rad) of a rotation matrix.
double rot_angle_rad(const cv::Matx33d& R) {
    double tr = R(0, 0) + R(1, 1) + R(2, 2);
    double c = std::clamp((tr - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(c);
}
}  // namespace

cv::Matx33d rot_of(const cv::Matx44d& T) {
    return cv::Matx33d(T(0, 0), T(0, 1), T(0, 2),
                       T(1, 0), T(1, 1), T(1, 2),
                       T(2, 0), T(2, 1), T(2, 2));
}

cv::Vec3d trans_of(const cv::Matx44d& T) {
    return cv::Vec3d(T(0, 3), T(1, 3), T(2, 3));
}

cv::Matx44d compose(const cv::Matx33d& R, const cv::Vec3d& t) {
    cv::Matx44d T = cv::Matx44d::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) T(r, c) = R(r, c);
        T(r, 3) = t(r);
    }
    return T;
}

cv::Matx44d invert_rigid(const cv::Matx44d& T) {
    cv::Matx33d R = rot_of(T);
    cv::Vec3d   t = trans_of(T);
    cv::Matx33d Rt = R.t();
    return compose(Rt, -(Rt * t));
}

// Rotation matrix -> unit quaternion (w, x, y, z). Standard Shepperd method.
cv::Vec4d mat_to_quat(const cv::Matx33d& R) {
    double t = R(0, 0) + R(1, 1) + R(2, 2);
    double w, x, y, z;
    if (t > 0.0) {
        double s = std::sqrt(t + 1.0) * 2.0;
        w = 0.25 * s;
        x = (R(2, 1) - R(1, 2)) / s;
        y = (R(0, 2) - R(2, 0)) / s;
        z = (R(1, 0) - R(0, 1)) / s;
    } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
        double s = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;
        w = (R(2, 1) - R(1, 2)) / s;
        x = 0.25 * s;
        y = (R(0, 1) + R(1, 0)) / s;
        z = (R(0, 2) + R(2, 0)) / s;
    } else if (R(1, 1) > R(2, 2)) {
        double s = std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;
        w = (R(0, 2) - R(2, 0)) / s;
        x = (R(0, 1) + R(1, 0)) / s;
        y = 0.25 * s;
        z = (R(1, 2) + R(2, 1)) / s;
    } else {
        double s = std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;
        w = (R(1, 0) - R(0, 1)) / s;
        x = (R(0, 2) + R(2, 0)) / s;
        y = (R(1, 2) + R(2, 1)) / s;
        z = 0.25 * s;
    }
    cv::Vec4d q(w, x, y, z);
    double n = cv::norm(q);
    return n > 0 ? q / n : cv::Vec4d(1, 0, 0, 0);
}

cv::Matx33d quat_to_mat(const cv::Vec4d& q) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    double n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n <= 0) return cv::Matx33d::eye();
    w /= n; x /= n; y /= n; z /= n;
    return cv::Matx33d(
        1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),
        2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
        2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y));
}

// Chordal mean of rotations via sign-aligned quaternion averaging.
cv::Matx33d average_rotation(const std::vector<cv::Matx33d>& rots) {
    cv::Vec4d acc(0, 0, 0, 0);
    cv::Vec4d ref = mat_to_quat(rots.front());
    for (const auto& R : rots) {
        cv::Vec4d q = mat_to_quat(R);
        if (q.dot(ref) < 0) q = -q;  // hemisphere-align
        acc += q;
    }
    double nrm = cv::norm(acc);
    if (nrm <= 0) return rots.front();
    return quat_to_mat(acc / nrm);
}

cv::Matx44d pose_from_pos_quat(double x, double y, double z,
                               double qx, double qy, double qz, double qw) {
    // mat_to/quat use (w,x,y,z) order internally.
    cv::Matx33d R = quat_to_mat(cv::Vec4d(qw, qx, qy, qz));
    return compose(R, cv::Vec3d(x, y, z));
}

double rotation_angle_deg(const cv::Matx44d& a, const cv::Matx44d& b) {
    cv::Matx33d Rrel = rot_of(a).t() * rot_of(b);
    return rot_angle_rad(Rrel) * kRad2Deg;
}

cv::Matx44d average_poses(const std::vector<cv::Matx44d>& poses) {
    if (poses.empty()) return cv::Matx44d::eye();
    if (poses.size() == 1) return poses.front();
    std::vector<cv::Matx33d> rots;
    rots.reserve(poses.size());
    cv::Vec3d tsum(0, 0, 0);
    for (const auto& T : poses) {
        rots.push_back(rot_of(T));
        tsum += trans_of(T);
    }
    return compose(average_rotation(rots),
                   tsum / static_cast<double>(poses.size()));
}

}  // namespace fitra::geom
