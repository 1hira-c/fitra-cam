#include "app/stats_loop.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

#include "util/logging.hpp"

namespace fitra::app {

void run_stats_loop(pipeline::MultiCameraDriver& driver,
                    double log_every_s,
                    std::atomic<bool>& stop) {
    auto last_log = std::chrono::steady_clock::now();
    while (!stop.load()) {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_log).count();
        if (dt >= log_every_s) {
            for (std::size_t i = 0; i < driver.camera_count(); ++i) {
                const auto& s = driver.stats_for(i);
                char buf[256];
                std::snprintf(buf, sizeof(buf),
                              "cam%zu: recv=%5.2f avg_pose=%5.2f recent_pose=%5.2f "
                              "stage_ms=%6.1f processed=%llu pending=%llu",
                              i, driver.recv_fps_for(i),
                              s.avg_pose_fps, s.recent_pose_fps,
                              s.last_stage_ms,
                              static_cast<unsigned long long>(s.processed_count),
                              static_cast<unsigned long long>(driver.pending_for(i)));
                FITRA_LOG_INFO("{}", buf);
            }
            last_log = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

}  // namespace fitra::app
