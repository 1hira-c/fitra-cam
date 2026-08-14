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

        for (unsigned mask = 0; mask < 8; ++mask) {
            Config cfg;
            cfg.kalman_enabled = (mask & 1U) != 0;
            cfg.ik_enabled = (mask & 2U) != 0;
            cfg.floor_contact_stability = (mask & 4U) != 0;

            const auto normal = cfg.effective_stages();
            check(!normal.raw_3d_source,
                  "normal mode must not report raw source");
            check(normal.kalman_enabled == cfg.kalman_enabled,
                  "normal mode must retain the Kalman preference");
            check(normal.ik_enabled == cfg.ik_enabled,
                  "normal mode must retain the IK preference");
            check(normal.floor_contact_stability == cfg.floor_contact_stability,
                  "normal mode must retain the floor preference");

            cfg.raw_3d_source = true;
            const auto raw = cfg.effective_stages();
            check(raw.raw_3d_source, "raw source marker must be true");
            check(!raw.kalman_enabled, "raw source must always bypass Kalman");
            check(!raw.ik_enabled, "raw source must always bypass IK");
            check(!raw.floor_contact_stability,
                  "raw source must always bypass floor stabilization");
            check(cfg.kalman_enabled == ((mask & 1U) != 0)
                      && cfg.ik_enabled == ((mask & 2U) != 0)
                      && cfg.floor_contact_stability == ((mask & 4U) != 0),
                  "raw source must not overwrite normal-mode stage preferences");
        }

        std::puts("test_raw_3d_source ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_raw_3d_source failed: %s\n", e.what());
        return 1;
    }
}
