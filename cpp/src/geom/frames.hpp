#pragma once
//
// Type-level coordinate-frame distinction for the pose-3d / vr-output tracks.
//
// See docs/design/pose-3d-typed-coordinate-frames.md. 3D math used to flow as
// bare cv::Matx44d / cv::Vec3d with frame semantics living only in variable
// names + comments, which let the 2026-06-09 "split-brain" regression compile
// (a VMT Y-up extrinsic treated as fitra Z-up). `Transform<To, From>` makes the
// to/from chain a compile-time invariant: only middle-frame-matching products
// instantiate, and the world kind (FitraWorld vs VmtWorld) is part of the type.
//
// Scope is the SE(3) layer (extrinsic solver, triangulation, calib I/O, calib
// session). Leaf I/O (infer::Joint3D, kalman/IK state, wire structs) stays raw
// and is fixed to fitra Z-up by the design-doc invariant, not by these types.

#include <vector>

#include <opencv2/core.hpp>

namespace fitra::geom {

// --- frame tags (empty; never instantiated) --------------------------------
namespace frame {
struct FitraWorld {};  // Z-up RH, X-right, Y-forward (the one internal world)
struct VmtWorld   {};  // Y-up RH, X-right, Z-back (SteamVR/VMT Driver)
struct Camera     {};  // per-camera: X-right, Y-down, Z-forward
struct Marker     {};  // AprilTag face object frame
struct Controller {};  // VR controller local frame
}  // namespace frame

// --- raw SE(3) helpers (frame-agnostic; OpenCV-boundary use) ----------------
// Promoted here from extrinsic_solver.cpp's anonymous namespace so the typed
// layer and the solver share one implementation. Quaternions are (w, x, y, z).
cv::Matx33d rot_of(const cv::Matx44d& T);
cv::Vec3d   trans_of(const cv::Matx44d& T);
cv::Matx44d compose(const cv::Matx33d& R, const cv::Vec3d& t);
cv::Matx44d invert_rigid(const cv::Matx44d& T);
cv::Vec4d   mat_to_quat(const cv::Matx33d& R);  // -> (w,x,y,z), normalized
cv::Matx33d quat_to_mat(const cv::Vec4d& q);    // (w,x,y,z) -> R
cv::Matx33d average_rotation(const std::vector<cv::Matx33d>& rots);

// Build a pose (any frame) from position + xyzw quaternion (normalized inside).
cv::Matx44d pose_from_pos_quat(double x, double y, double z,
                               double qx, double qy, double qz, double qw);
// Geodesic angle (degrees) between the rotation parts of two SE(3) poses.
double rotation_angle_deg(const cv::Matx44d& a, const cv::Matx44d& b);
// Arithmetic mean of translations + chordal (sign-aligned quaternion) mean of
// rotations. `poses` empty -> identity. Used for burst averaging.
cv::Matx44d average_poses(const std::vector<cv::Matx44d>& poses);

// --- Point3<Frame>: a 3D point that lives in a specific frame ---------------
template <class F>
struct Point3 {
    cv::Vec3d v{0.0, 0.0, 0.0};
    Point3() = default;
    explicit Point3(const cv::Vec3d& p) : v(p) {}
    Point3(double x, double y, double z) : v(x, y, z) {}
};

// --- Transform<To, From>: a rigid SE(3) transform To <- From ----------------
template <class To, class From>
class Transform {
public:
    Transform() : m_(cv::Matx44d::eye()) {}

    // OpenCV-boundary escape hatch: wrap a raw 4x4 / hand it to OpenCV. Keep
    // .raw() to the single line that calls into OpenCV; do not reuse the
    // unwrapped value (that reopens the frame-safety hole this type closes).
    static Transform from_raw(const cv::Matx44d& m) { return Transform(m); }
    const cv::Matx44d& raw() const { return m_; }

    cv::Matx33d rot()   const { return rot_of(m_); }
    cv::Vec3d   trans() const { return trans_of(m_); }

    Transform<From, To> inverse() const {
        return Transform<From, To>::from_raw(invert_rigid(m_));
    }

private:
    explicit Transform(const cv::Matx44d& m) : m_(m) {}
    cv::Matx44d m_;
};

// Compose: To <- Mid times Mid <- From = To <- From. Only instantiates when the
// middle frames match — this is where chain mistakes become compile errors.
template <class To, class Mid, class From>
Transform<To, From> operator*(const Transform<To, Mid>& a,
                              const Transform<Mid, From>& b) {
    return Transform<To, From>::from_raw(a.raw() * b.raw());
}

// Transform<To, From> * Point3<From> -> Point3<To>.
template <class To, class From>
Point3<To> operator*(const Transform<To, From>& T, const Point3<From>& p) {
    const cv::Matx44d& m = T.raw();
    cv::Vec4d h(p.v[0], p.v[1], p.v[2], 1.0);
    cv::Vec4d r = m * h;
    return Point3<To>(cv::Vec3d(r[0], r[1], r[2]));
}

// --- convenience aliases for the frames actually used -----------------------
using T_cam_world         = Transform<frame::Camera, frame::FitraWorld>;
using T_cam_vmtworld      = Transform<frame::Camera, frame::VmtWorld>;
using T_world_controller  = Transform<frame::VmtWorld, frame::Controller>;
using T_cam_marker        = Transform<frame::Camera, frame::Marker>;
using T_marker_controller = Transform<frame::Marker, frame::Controller>;

}  // namespace fitra::geom
