#include "camera/frame_source.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "util/logging.hpp"

namespace fitra::camera {

FrameSource::FrameSource(std::unique_ptr<V4l2Capture> capture,
                         std::unique_ptr<infer::Yolox> yolox,
                         Options opts,
                         const infer::RtmPose::Options* rtmpose_opts)
    : capture_{std::move(capture)},
      yolox_{std::move(yolox)},
      opts_{std::move(opts)} {
    if (rtmpose_opts) {
        rtmpose_enabled_ = true;
        rtmpose_opts_    = *rtmpose_opts;
    }
}

FrameSource::~FrameSource() {
    try { stop(); } catch (...) {}
}

void FrameSource::start() {
    capture_->start();
    stop_.store(false);
    worker_ = std::thread{&FrameSource::decode_loop, this};
}

void FrameSource::stop() {
    if (!worker_.joinable() && !capture_) return;
    stop_.store(true);
    // decode_loop may be parked in capture_->wait_pop_latest; wake it so it
    // observes stop_ without waiting out the timeout. Order: flag -> wake -> join.
    if (capture_) capture_->wake();
    if (worker_.joinable()) worker_.join();
    if (capture_) capture_->stop();
}

void FrameSource::decode_loop() {
    cv::Mat scratch;
    while (!stop_.load()) {
        Frame raw;
        // Event-driven: block until the capture worker publishes a new frame
        // (or stop_ is set, or the 100ms safety timeout fires). Replaces the
        // old 2ms poll-sleep; wait_pop_latest already dedups on seq.
        if (!capture_->wait_pop_latest(raw, stop_, std::chrono::milliseconds(100))) {
            continue;
        }
        if (capture_->options().pixel_format == PixFmt::Yuyv) {
            // Packed YUV422 -> BGR. No entropy decode; just a color convert.
            const auto& o = capture_->options();
            if (static_cast<int>(raw.data.size()) < o.width * o.height * 2) {
                FITRA_LOG_WARN("frame_source: short YUYV frame for seq={} ({} bytes)",
                               raw.seq, raw.data.size());
                continue;
            }
            cv::Mat yuy2(o.height, o.width, CV_8UC2,
                         const_cast<std::uint8_t*>(raw.data.data()));
            cv::cvtColor(yuy2, scratch, cv::COLOR_YUV2BGR_YUYV);
        } else {
            if (!decoder_.decode(raw.data, scratch)) {
                FITRA_LOG_WARN("frame_source: jpeg decode failed for seq={}", raw.seq);
                continue;
            }
        }
        auto t_decode = std::chrono::steady_clock::now();

        const bool calib_recording =
            opts_.calib_recording_flag
            && opts_.calib_recording_flag->load(std::memory_order_relaxed);

        // YOLOX runs on this thread (one IExecutionContext per FrameSource),
        // so all cameras detect in parallel. Skipped during calib recording —
        // raw mp4 capture is the priority, and dump_keypoints_3d re-runs
        // detection offline on the resulting clips anyway.
        if (yolox_ && !calib_recording) {
            bool do_detect = (frame_idx_ % opts_.det_frequency == 0)
                          || cached_bboxes_.empty();
            if (do_detect) {
                auto dets = yolox_->infer(scratch);
                if (opts_.single_person && dets.size() > 1) {
                    auto largest = std::max_element(
                        dets.begin(), dets.end(),
                        [](const auto& a, const auto& b) {
                            float aa = (a.x2 - a.x1) * (a.y2 - a.y1);
                            float bb = (b.x2 - b.x1) * (b.y2 - b.y1);
                            return aa < bb;
                        });
                    infer::Bbox keep = *largest;
                    dets.clear();
                    dets.push_back(keep);
                }
                cached_bboxes_ = std::move(dets);
            }
        }
        auto t_detect = std::chrono::steady_clock::now();

        if (opts_.fake_bbox_if_empty && cached_bboxes_.empty()) {
            infer::Bbox fake{};
            float w = static_cast<float>(scratch.cols);
            float h = static_cast<float>(scratch.rows);
            fake.x1 = 0.2f * w;
            fake.y1 = 0.2f * h;
            fake.x2 = 0.8f * w;
            fake.y2 = 0.8f * h;
            fake.score = 1.0f;
            cached_bboxes_.push_back(fake);
        }

        DecodedFrame df;
        df.seq         = raw.seq;
        df.captured_at = raw.captured_at;
        df.t_decode    = t_decode;
        df.t_detect    = t_detect;
        // During calib recording we drop bboxes too — the central thread sees
        // bboxes.empty() and naturally skips RTMPose. (Without this the
        // "missing prebake" warning would spam.)
        if (!calib_recording) {
            df.bboxes  = cached_bboxes_;  // copy of current cache
        }

        if (rtmpose_enabled_ && !df.bboxes.empty()) {
            // Preprocess each (frame, bbox) into the contiguous CHW block
            // here on the per-camera worker thread, shifting the dominant
            // CPU cost off the central inference thread.
            const std::size_t per_item =
                infer::RtmPose::blob_floats_per_item(rtmpose_opts_);
            df.chw_concat.resize(df.bboxes.size() * per_item);
            df.M_invs.resize(df.bboxes.size());
            for (std::size_t i = 0; i < df.bboxes.size(); ++i) {
                infer::RtmPose::preprocess_to_blob(
                    rtmpose_opts_, scratch, df.bboxes[i],
                    df.chw_concat.data() + i * per_item,
                    df.M_invs[i]);
            }
        }
        // t_prebake closes the per-camera CPU stage. When the prebake block
        // above was skipped (no bbox / no RTMPose) this equals t_detect so the
        // det->bake delta is 0 rather than a garbage epoch-based value.
        df.t_prebake = std::chrono::steady_clock::now();
        if (!rtmpose_enabled_ || opts_.retain_bgr || calib_recording) {
            df.bgr = scratch.clone();
        }

        {
            std::lock_guard<std::mutex> lk{slot_mu_};
            latest_ = std::move(df);
            slot_cv_.notify_one();  // wake the central loop if parked in wait_available
        }
        ++frame_idx_;
    }
}

bool FrameSource::try_pop_latest_decoded(DecodedFrame& out) {
    std::lock_guard<std::mutex> lk{slot_mu_};
    if (!latest_) return false;
    if (latest_->seq == last_returned_seq_) return false;
    last_returned_seq_ = latest_->seq;
    out = *latest_;
    return true;
}

bool FrameSource::wait_available(std::atomic<bool>& consumer_stop,
                                 std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lk{slot_mu_};
    slot_cv_.wait_for(lk, timeout, [&] {
        return consumer_stop.load(std::memory_order_relaxed)
            || (latest_ && latest_->seq != last_returned_seq_);
    });
    return latest_ && latest_->seq != last_returned_seq_;
}

void FrameSource::wake() {
    std::lock_guard<std::mutex> lk{slot_mu_};
    slot_cv_.notify_all();
}

}  // namespace fitra::camera
