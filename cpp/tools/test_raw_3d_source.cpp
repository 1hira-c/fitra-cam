#include <cstdio>
#include <stdexcept>

#include "pipeline/multi_pipeline.hpp"

namespace {

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main() {
    try {
        using Config = fitra::pipeline::MultiCameraDriver::ThreeDConfig;

        Config normal;
        normal.kalman_enabled = true;
        normal.ik_enabled = false;
        normal.floor_contact_stability = true;
        const auto normal_stages = normal.effective_stages();
        check(!normal_stages.raw_3d_source, "normal mode must not report raw source");
        check(normal_stages.kalman_enabled, "normal mode must retain Kalman");
        check(!normal_stages.ik_enabled, "normal mode must retain individual IK off");
        check(normal_stages.floor_contact_stability,
              "normal mode must retain individual floor on");

        Config raw;
        raw.kalman_enabled = true;
        raw.ik_enabled = true;
        raw.floor_contact_stability = true;
        raw.raw_3d_source = true;
        const auto raw_stages = raw.effective_stages();
        check(raw_stages.raw_3d_source, "raw source marker must be true");
        check(!raw_stages.kalman_enabled,
              "raw source must bypass Kalman even when configured on");
        check(!raw_stages.ik_enabled,
              "raw source must bypass IK even when configured on");
        check(!raw_stages.floor_contact_stability,
              "raw source must bypass floor stabilization even when configured on");
        check(raw.kalman_enabled && raw.ik_enabled && raw.floor_contact_stability,
              "raw source must not overwrite normal-mode stage preferences");

        std::puts("test_raw_3d_source ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_raw_3d_source failed: %s\n", e.what());
        return 1;
    }
}
