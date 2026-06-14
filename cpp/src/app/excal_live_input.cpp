#include "app/excal_live_input.hpp"

namespace fitra::app {

ExcalLiveInput::ExcalLiveInput(
    std::vector<std::unique_ptr<camera::FrameSource>> sources,
    vmt::ControllerPoseBus& bus,
    double controller_stale_ms)
    : sources_{std::move(sources)},
      bus_{bus},
      stale_ms_{controller_stale_ms} {}

ExcalLiveInput::~ExcalLiveInput() { stop(); }

void ExcalLiveInput::start() {
    for (auto& s : sources_) s->start();
}

void ExcalLiveInput::stop() {
    for (auto& s : sources_) s->stop();
}

bool ExcalLiveInput::next(pipeline::ExcalInputItem& out) {
    const std::size_t n = sources_.size();
    for (std::size_t k = 0; k < n; ++k) {
        const std::size_t i = (next_cam_ + k) % n;
        camera::DecodedFrame df;
        if (!sources_[i]->try_pop_latest_decoded(df)) continue;
        next_cam_ = (i + 1) % n;

        // One t0 shared across cameras, anchored on the first popped frame so
        // ts starts at 0 (the driver tap anchored on its first tap call; only
        // deltas matter to the gate/burst logic either way).
        if (!t0_set_) {
            t0_ = df.captured_at;
            t0_set_ = true;
        }
        auto snap = bus_.snapshot(stale_ms_);
        pipeline::ControllerObservation c;
        c.running_ok = !snap.stale && snap.pose.running_ok();
        c.x = snap.pose.x; c.y = snap.pose.y; c.z = snap.pose.z;
        c.qx = snap.pose.qx; c.qy = snap.pose.qy;
        c.qz = snap.pose.qz; c.qw = snap.pose.qw;
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
