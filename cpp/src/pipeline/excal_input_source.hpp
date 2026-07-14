#pragma once
//
// Input abstraction for the calib-extrinsic capture loop.
//
// ExtrinsicCalibSession::on_frame() consumes (cam_idx, BGR frame, paired
// ControllerObservation) triples; this interface is the seam that produces
// them. Implementations:
//   - live   (app/excal_live_input) — decode-only FrameSource poll paired
//     with a ControllerPoseBus snapshot at pop time.
//   - replay (M4) — a tools/excal_record session (JPEG sequence +
//     frames.jsonl with the frame↔pose pairing fixed at record time).
// The interface stays free of camera/vmt types so fitra_pipeline can host the
// replay implementation without depending on fitra_vmt (which would be
// circular: fitra_vmt → fitra_tracking → fitra_pipeline).
// See docs/design/pose-3d-calib-mode-separation.md.

#include <cstddef>

#include <opencv2/core.hpp>

#include "pipeline/extrinsic_calib_session.hpp"

namespace fitra::pipeline {

struct ExcalInputItem {
    std::size_t           cam_idx = 0;
    cv::Mat               bgr;    // decoded BGR; valid until the next next() call
    ControllerObservation ctrl;   // frame↔pose pairing fixed by the producer
};

class ExcalInputSource {
public:
    virtual ~ExcalInputSource() = default;

    // Produce the next paired frame. Returns false when nothing is available
    // right now (live: poll again; replay: check exhausted()).
    virtual bool next(ExcalInputItem& out) = 0;

    // True once the source will never produce another item (end of a replay
    // session). Live sources always return false.
    virtual bool exhausted() const = 0;
};

}  // namespace fitra::pipeline
