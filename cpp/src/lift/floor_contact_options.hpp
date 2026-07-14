#pragma once

#include <cmath>

namespace fitra::lift {

struct FloorContactOptions {
    double floor_z_m = 0.0;
    double enter_height_m = 0.04;
    double exit_height_m = 0.08;
    double enter_speed_mps = 0.35;
    double exit_speed_mps = 1.00;
    double xy_anchor_tau_s = 0.25;
    double max_xy_correction_m = 0.04;
    double max_z_correction_m = 0.08;
    int missing_grace_frames = 4;
    // Retain the latch through short sync-miss bursts and low-rate offline
    // sources. Longer gaps are treated as discontinuities.
    double reset_dt_s = 0.50;
    // Time constant for returning the last rigid correction to zero after
    // contact release. This avoids a one-frame XY/Z snap.
    double release_tau_s = 0.05;
    // Require a height/speed/XY exit signal to persist before unlatching.
    // Oversized Z corrections still release immediately as a safety bound.
    double exit_grace_s = 0.05;
};

inline const char* floor_contact_options_error(const FloorContactOptions& o) {
    const auto finite = [](double value) { return std::isfinite(value); };
    if (!finite(o.floor_z_m)
        || !finite(o.enter_height_m) || !finite(o.exit_height_m)
        || !finite(o.enter_speed_mps) || !finite(o.exit_speed_mps)
        || !finite(o.xy_anchor_tau_s)
        || !finite(o.max_xy_correction_m) || !finite(o.max_z_correction_m)
        || !finite(o.reset_dt_s) || !finite(o.release_tau_s)
        || !finite(o.exit_grace_s)) {
        return "floor-contact settings must be finite";
    }
    if (o.enter_height_m < 0.0 || o.exit_height_m <= o.enter_height_m) {
        return "floor contact heights require 0 <= enter < exit";
    }
    if (o.enter_speed_mps < 0.0 || o.exit_speed_mps <= o.enter_speed_mps) {
        return "floor contact speeds require 0 <= enter < exit";
    }
    if (o.xy_anchor_tau_s <= 0.0 || o.release_tau_s <= 0.0
        || o.max_xy_correction_m <= 0.0 || o.max_z_correction_m <= 0.0
        || o.reset_dt_s <= 0.0) {
        return "floor contact time constants and correction limits must be > 0";
    }
    if (o.exit_grace_s < 0.0) {
        return "floor contact exit grace must be >= 0";
    }
    if (o.missing_grace_frames < 0 || o.missing_grace_frames > 10) {
        return "floor contact missing grace frames must be in [0, 10]";
    }
    return nullptr;
}

}  // namespace fitra::lift
