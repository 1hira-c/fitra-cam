#include "app/floor_live_input.hpp"

namespace fitra::app {

FloorLiveInput::FloorLiveInput(
    std::vector<std::unique_ptr<camera::FrameSource>> sources)
    : sources_{std::move(sources)} {}

FloorLiveInput::~FloorLiveInput() { stop(); }

void FloorLiveInput::start() {
    for (auto& s : sources_) s->start();
}

void FloorLiveInput::stop() {
    for (auto& s : sources_) s->stop();
}

bool FloorLiveInput::next(pipeline::ExcalInputItem& out) {
    const std::size_t n = sources_.size();
    for (std::size_t k = 0; k < n; ++k) {
        const std::size_t i = (next_cam_ + k) % n;
        camera::DecodedFrame df;
        if (!sources_[i]->try_pop_latest_decoded(df)) continue;
        next_cam_ = (i + 1) % n;

        if (!t0_set_) {
            t0_ = df.captured_at;
            t0_set_ = true;
        }
        pipeline::ControllerObservation c;  // default: unused by the floor path
        c.ts_ms = std::chrono::duration<double, std::milli>(
                      df.captured_at - t0_).count();

        out.cam_idx = i;
        out.bgr     = std::move(df.bgr);
        out.ctrl    = c;
        return true;
    }
    return false;
}

}  // namespace fitra::app
