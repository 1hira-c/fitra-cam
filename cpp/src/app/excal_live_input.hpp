#pragma once
//
// Live ExcalInputSource: polls decode-only FrameSources round-robin and pairs
// each popped frame with a ControllerPoseBus snapshot taken at pop time — the
// pairing the MultiCameraDriver frame tap performed in main.cpp before
// calib-extrinsic became a dedicated mode (no driver, no TensorRT).
//
// ts_ms is the frame's V4L2 capture timestamp relative to the first popped
// frame: the same session-relative semantics as tools/excal_record and the
// old driver tap, and the only clock the session's motion gate / burst logic
// ever sees.
//
// Lives in app/ (not pipeline/) because ControllerPoseBus is fitra_vmt and
// fitra_vmt already links fitra_pipeline — the dependency must point this way.

#include <chrono>
#include <memory>
#include <vector>

#include "camera/frame_source.hpp"
#include "pipeline/excal_input_source.hpp"
#include "vmt/controller_pose_receiver.hpp"

namespace fitra::app {

class ExcalLiveInput : public pipeline::ExcalInputSource {
public:
    // Sources must be decode-only (Yolox=nullptr) so DecodedFrame::bgr is
    // populated. The bus reference must outlive this object.
    ExcalLiveInput(std::vector<std::unique_ptr<camera::FrameSource>> sources,
                   vmt::ControllerPoseBus& bus,
                   double controller_stale_ms);
    ~ExcalLiveInput() override;

    void start();   // start all capture/decode workers
    void stop();

    std::size_t camera_count() const { return sources_.size(); }

    bool next(pipeline::ExcalInputItem& out) override;
    bool exhausted() const override { return false; }

private:
    std::vector<std::unique_ptr<camera::FrameSource>> sources_;
    vmt::ControllerPoseBus& bus_;
    double      stale_ms_;
    std::size_t next_cam_ = 0;
    bool        t0_set_ = false;
    std::chrono::steady_clock::time_point t0_{};
};

}  // namespace fitra::app
