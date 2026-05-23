#pragma once
//
// YOLOX person detector wrapper around a TrtEngine.
//
// Matches the preprocessing/postprocessing in
// python/scripts/pose_pipeline.py::YoloxOnnx so that the C++ TRT pipeline
// produces numerically equivalent boxes to the Python ORT reference.
//
// Spec (mmdeploy YOLOX humanart family — tiny / S / M / X share the same I/O):
//   input  : "input"  (1, 3, S, S)     float32, BGR raw values, NO normalization
//                                       S = 416 (tiny) or 640 (S/M/X)
//   output : "dets"   (1, N, 5)        float32  [x1, y1, x2, y2, score]
//   output : "labels" (1, N)           int64    class id (person == 0)
//
// Letterbox: r = min(S/h, S/w); resize INTER_LINEAR to (nh, nw); pad to
// SxS with value 114 (top-left aligned). Bboxes scaled back by /r.
//
// Options::input_size is auto-overwritten from the engine binding at
// construction time, so callers may leave the default 416 and load any of
// the four humanart variants without touching this struct.

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "infer/trt_engine.hpp"
#include "infer/types.hpp"

namespace fitra::infer {

class Yolox {
public:
    struct Options {
        std::string input_name  = "input";
        std::string dets_name   = "dets";
        std::string labels_name = "labels";
        int   input_size = 416;
        float score_thr  = 0.5f;
        int   person_label = 0;
    };

    explicit Yolox(TrtEngine& engine);
    Yolox(TrtEngine& engine, Options opts);

    // Run inference and return person bboxes scaled back to `frame_bgr`'s
    // pixel coordinates.
    std::vector<Bbox> infer(const cv::Mat& frame_bgr);

private:
    TrtEngine& engine_;
    Options    opts_;

    // Reusable host buffers
    cv::Mat letterbox_;                 // 416x416x3 uint8
    std::vector<float>  input_blob_;    // CHW float32
    std::vector<float>  dets_host_;
    std::vector<std::int64_t> labels_host_;
};

}  // namespace fitra::infer
