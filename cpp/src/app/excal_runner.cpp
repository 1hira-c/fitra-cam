#include "app/excal_runner.hpp"

#include <chrono>
#include <thread>

#include "pipeline/extrinsic_calib_session.hpp"

namespace fitra::app {

void run_excal_loop(pipeline::ExcalInputSource& input,
                    pipeline::ExtrinsicCalibSession& session,
                    std::atomic<bool>& stop) {
    pipeline::ExcalInputItem item;
    while (!stop.load()) {
        if (input.exhausted()) break;
        if (!input.next(item)) {
            // Live source with no fresh frame yet; 2 ms keeps poll latency
            // well under a frame period without spinning.
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        // on_frame ignores frames unless the session is collecting, so the
        // loop can feed unconditionally.
        session.on_frame(item.cam_idx, item.bgr, item.ctrl);
    }
}

}  // namespace fitra::app
