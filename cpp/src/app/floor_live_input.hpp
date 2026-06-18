#pragma once
//
// Live ExcalInputSource for the floor-AprilTag path: polls decode-only
// FrameSources round-robin and emits frames with a session-relative timestamp.
// Unlike ExcalLiveInput there is no controller pose to pair — the floor session
// only needs (cam_idx, bgr, ts_ms) — so this carries no ControllerPoseBus and
// can live with a default ControllerObservation in the item.
//
// ts_ms is the frame's V4L2 capture timestamp relative to the first popped
// frame (same semantics as tools/excal_record), used only for the UI "age".

#include <chrono>
#include <memory>
#include <vector>

#include "camera/frame_source.hpp"
#include "pipeline/excal_input_source.hpp"

namespace fitra::app {

class FloorLiveInput : public pipeline::ExcalInputSource {
public:
    explicit FloorLiveInput(
        std::vector<std::unique_ptr<camera::FrameSource>> sources);
    ~FloorLiveInput() override;

    void start();
    void stop();

    std::size_t camera_count() const { return sources_.size(); }

    bool next(pipeline::ExcalInputItem& out) override;
    bool exhausted() const override { return false; }

private:
    std::vector<std::unique_ptr<camera::FrameSource>> sources_;
    std::size_t next_cam_ = 0;
    bool        t0_set_ = false;
    std::chrono::steady_clock::time_point t0_{};
};

}  // namespace fitra::app
