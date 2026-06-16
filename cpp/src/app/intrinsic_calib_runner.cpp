#include "app/intrinsic_calib_runner.hpp"

#include <chrono>
#include <thread>

#include "pipeline/intrinsic_calib_session.hpp"

namespace fitra::app {

void run_intrinsic_calib_loop(pipeline::ExcalInputSource& input,
                              pipeline::IntrinsicCalibSession& session,
                              std::atomic<bool>& stop) {
    pipeline::ExcalInputItem item;
    while (!stop.load()) {
        if (input.exhausted()) break;
        if (!input.next(item)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        session.on_frame(item.cam_idx, item.bgr, item.ctrl.ts_ms);
    }
}

}  // namespace fitra::app
