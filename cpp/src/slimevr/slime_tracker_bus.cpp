#include "slimevr/slime_tracker_bus.hpp"

#include <cstdio>

namespace fitra::slimevr {

namespace {

void append_float(std::string& out, float v, int precision = 6) {
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.*g", precision, static_cast<double>(v));
    out += buf;
}

}  // namespace

void SlimeTrackerBus::publish(const std::array<SlimeTracker, kTrackerCount>& trackers,
                              const SlimeTrackerStats&                       stats) {
    std::lock_guard<std::mutex> lk{mu_};
    snapshot_.trackers = trackers;
    snapshot_.stats    = stats;
    snapshot_.seq += 1;
    snapshot_.ts = std::chrono::system_clock::now();
    snapshot_.has_data = true;
}

SlimeTrackerSnapshot SlimeTrackerBus::snapshot() const {
    std::lock_guard<std::mutex> lk{mu_};
    return snapshot_;
}

const char* tracker_role_name(TrackerRole role) {
    switch (role) {
        case TrackerRole::LeftUpperArm:  return "LeftUpperArm";
        case TrackerRole::RightUpperArm: return "RightUpperArm";
        case TrackerRole::Chest:         return "Chest";
        case TrackerRole::Waist:         return "Waist";
        case TrackerRole::LeftUpperLeg:  return "LeftUpperLeg";
        case TrackerRole::RightUpperLeg: return "RightUpperLeg";
        case TrackerRole::LeftLowerLeg:  return "LeftLowerLeg";
        case TrackerRole::RightLowerLeg: return "RightLowerLeg";
        case TrackerRole::LeftFoot:      return "LeftFoot";
        case TrackerRole::RightFoot:     return "RightFoot";
        case TrackerRole::Count:         break;
    }
    return "Unknown";
}

std::string make_tracker_bundle_fragment(const SlimeTrackerBus& bus) {
    auto snap = bus.snapshot();
    std::string out;
    out.reserve(1024);
    out += "\"trackers\":[";
    if (snap.has_data) {
        const auto& s = snap.stats;
        for (std::size_t i = 0; i < snap.trackers.size(); ++i) {
            if (i) out += ",";
            const auto& t = snap.trackers[i];
            out += "{\"role\":\"";
            out += tracker_role_name(t.role);
            out += "\",\"pos\":[";
            append_float(out, t.pos[0]); out += ",";
            append_float(out, t.pos[1]); out += ",";
            append_float(out, t.pos[2]);
            out += "],\"quat_wxyz\":[";
            append_float(out, t.quat_wxyz[0]); out += ",";
            append_float(out, t.quat_wxyz[1]); out += ",";
            append_float(out, t.quat_wxyz[2]); out += ",";
            append_float(out, t.quat_wxyz[3]);
            out += "],\"valid\":";
            out += (t.valid ? "true" : "false");
            out += ",\"roll_confidence\":";
            append_float(out, t.roll_confidence, 4);
            // Phase 13 M2 per-tracker rolling stats. Indexing is parallel
            // to the trackers array (TrackerRole enum order).
            out += ",\"stats\":{";
            out += "\"ang_vel_p50\":";   append_float(out, s.angular_velocity_rad_s_p50[i], 4);
            out += ",\"ang_vel_p95\":";  append_float(out, s.angular_velocity_rad_s_p95[i], 4);
            out += ",\"conf_avg\":";     append_float(out, s.roll_confidence_avg[i], 4);
            out += ",\"leakage_pct\":";  append_float(out, s.leakage_pct[i], 4);
            out += ",\"freeze_pct\":";   append_float(out, s.freeze_pct[i], 4);
            out += ",\"freeze_current_ms\":";
            out += std::to_string(s.freeze_current_ms[i]);
            out += ",\"freeze_max_ms\":";
            out += std::to_string(s.freeze_max_ms[i]);
            out += ",\"dropouts\":";
            out += std::to_string(static_cast<long long>(s.dropout_count[i]));
            out += "}}";
        }
    }
    out += "]";
    if (snap.has_data) {
        out += ",\"tracker_stats_window_frames\":";
        out += std::to_string(snap.stats.window_frames);
    }
    return out;
}

}  // namespace fitra::slimevr
