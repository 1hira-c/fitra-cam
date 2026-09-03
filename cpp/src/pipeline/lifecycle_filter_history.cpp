#include "pipeline/lifecycle_filter_history.hpp"

namespace fitra::pipeline {

bool reset_lifecycle_filter_history_if_boundary(
    PoseGateSourceState source_state,
    lift::SkeletonKalman& kalman,
    lift::FloorContactStabilizer& floor_contact,
    lift::FloorContactReport& last_floor_report,
    bool& has_last_3d_update) {
    if (!pose_gate_is_lifecycle_boundary(source_state)) return false;

    kalman.reset();
    floor_contact.reset();
    last_floor_report = {};
    has_last_3d_update = false;
    return true;
}

}  // namespace fitra::pipeline
