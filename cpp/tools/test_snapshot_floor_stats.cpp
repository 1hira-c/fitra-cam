#include <cstdio>
#include <stdexcept>
#include <string>

#include "pipeline/snapshot.hpp"

namespace {

void require_field(const std::string& json,
                   const std::string& field) {
    if (json.find(field) == std::string::npos) {
        throw std::runtime_error("missing JSON field: " + field);
    }
}

}  // namespace

int main() {
    try {
        fitra::pipeline::Skeleton3DBus bus;
        fitra::pipeline::Skeleton3DSnapshot snapshot;
        snapshot.stats.floor_stability_enabled = true;
        snapshot.stats.floor_z_m = 0.125;
        snapshot.stats.floor_contact_fresh = true;
        snapshot.stats.floor_contact_left = true;
        snapshot.stats.floor_contact_right = false;
        snapshot.stats.floor_evidence_left = true;
        snapshot.stats.floor_evidence_right = false;
        snapshot.stats.floor_correction_left_m = 0.0125;
        snapshot.stats.floor_correction_right_m = 0.025;
        snapshot.stats.raw_3d_source = true;
        snapshot.stats.kalman_enabled = false;
        snapshot.stats.ik_enabled = false;
        bus.update(snapshot);

        const std::string json = bus.make_bundle_json();
        require_field(json, "\"floor_stability_enabled\":true");
        require_field(json, "\"floor_z_m\":0.125");
        require_field(json, "\"floor_contact_fresh\":true");
        require_field(json, "\"floor_contact_left\":true");
        require_field(json, "\"floor_contact_right\":false");
        require_field(json, "\"floor_evidence_left\":true");
        require_field(json, "\"floor_evidence_right\":false");
        require_field(json, "\"floor_correction_left_m\":0.0125");
        require_field(json, "\"floor_correction_right_m\":0.025");
        require_field(json, "\"raw_3d_source\":true");
        require_field(json, "\"kalman_enabled\":false");
        require_field(json, "\"ik_enabled\":false");

        std::puts("test_snapshot_floor_stats ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_snapshot_floor_stats failed: %s\n", e.what());
        return 1;
    }
}
