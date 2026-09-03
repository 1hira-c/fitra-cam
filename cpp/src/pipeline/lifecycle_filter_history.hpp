#pragma once

#include "lift/floor_contact_stabilizer.hpp"
#include "lift/kalman.hpp"
#include "pipeline/pose_gate.hpp"

namespace fitra::pipeline {

// Reset every temporal filter owned by the 3D loop as one lifecycle action.
// The caller is the loop's single writer, so the boundary cannot be observed
// between the individual resets. Returns true when `source_state` is a
// destructive boundary and the reset was performed.
bool reset_lifecycle_filter_history_if_boundary(
    PoseGateSourceState source_state,
    lift::SkeletonKalman& kalman,
    lift::FloorContactStabilizer& floor_contact,
    lift::FloorContactReport& last_floor_report,
    bool& has_last_3d_update);

}  // namespace fitra::pipeline
