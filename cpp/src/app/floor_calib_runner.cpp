#include "app/floor_calib_runner.hpp"

#include <chrono>
#include <thread>

#include "pipeline/floor_calib_session.hpp"

namespace fitra::app {

void run_floor_calib_loop(pipeline::ExcalInputSource& input,
                          pipeline::FloorCalibSession& session,
                          std::atomic<bool>& stop) {
    pipeline::ExcalInputItem item;
    while (!stop.load()) {
        if (input.exhausted()) break;
        if (!input.next(item)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        // The floor path ignores the paired controller pose; only the frame
        // timestamp is used (for the live "age" display). on_frame ignores
        // frames unless the session is collecting, so feed unconditionally.
        session.on_frame(item.cam_idx, item.bgr, item.ctrl.ts_ms);
    }
}

}  // namespace fitra::app
