#pragma once
//
// The single source of truth for the fitra Z-up <-> VMT Y-up world basis change.
//
// See docs/design/pose-3d-typed-coordinate-frames.md. This used to be hand-
// written in three places (vmt_protocol::world_pos_to_vmt, the calib session's
// kVmtWorldToFitra, and firmware_protocol's Slime variant). The wire helpers
// stay as-is (their tests cross-check them against this basis); SE(3)-layer code
// composes the typed Transforms below so the conversion direction is checked.
//
//   fitra (Z-up, X-right, Y-forward) -> VMT (Y-up, X-right, Z-back)
//   is (x, y, z) -> (x, z, -y), i.e. a +90deg rotation about X mapping the
//   fitra axes onto the VMT axes (the rotation part of vmt::world_pos_to_vmt).

#include "geom/frames.hpp"

namespace fitra::geom {

// VmtWorld <- FitraWorld basis change: maps a fitra-frame point to its VMT
// coordinates, (x, y, z) -> (x, z, -y). This is exactly the rotation of
// vmt::world_pos_to_vmt, and the matrix equals the old kVmtWorldToFitra
// constant. Right-composed onto a Transform<Camera, VmtWorld> it yields a
// Transform<Camera, FitraWorld> (re-expressing the world axes without moving
// the camera): T_cam_world = T_cam_vmtworld * fitra_to_vmt_basis().
inline Transform<frame::VmtWorld, frame::FitraWorld> fitra_to_vmt_basis() {
    return Transform<frame::VmtWorld, frame::FitraWorld>::from_raw(cv::Matx44d(
        1,  0, 0, 0,
        0,  0, 1, 0,
        0, -1, 0, 0,
        0,  0, 0, 1));
}

// FitraWorld <- VmtWorld basis change: maps a VMT-frame point to its fitra
// coordinates, (x, y, z) -> (x, -z, y). The inverse (transpose) of the above.
inline Transform<frame::FitraWorld, frame::VmtWorld> vmt_to_fitra_basis() {
    return fitra_to_vmt_basis().inverse();
}

}  // namespace fitra::geom
