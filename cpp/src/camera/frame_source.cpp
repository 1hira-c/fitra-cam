#include "camera/frame_source.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <string>
#include <utility>

#include <cuda_runtime_api.h>
#include <opencv2/imgproc.hpp>

#include "util/logging.hpp"

namespace fitra::camera {

namespace {
// BGR ImageNet mean / inverse-std for the RTMPose preprocess kernel. Must match
// the constants in infer/rtmpose.cpp (preprocess_to_blob).
constexpr float kMeanBgr[3]   = {103.53f, 116.28f, 123.675f};
constexpr float kInvStdBgr[3] = {1.0f / 57.375f, 1.0f / 57.12f, 1.0f / 58.395f};

// Cheap structural sanity check for an MJPEG payload before it reaches the HW
// NVJPEG decoder. USB bandwidth saturation (multi-camera) produces truncated /
// garbage frames; CPU cv::imdecode rejects those gracefully, but the HW NVJPEG
// block SEGFAULTS on a malformed stream (observed: "Not a JPEG file: starts with
// 0xff 0xd7", "Premature end of JPEG file"). Require a leading SOI (FF D8) and a
// trailing EOI (FF D9) within the last bytes (tolerating a little driver
// padding). This catches garbage-start and truncation -- the crash cases -- so
// we can drop the frame instead of feeding it to the HW decoder.
bool looks_like_jpeg(const std::vector<std::uint8_t>& d) {
    if (d.size() < 4) return false;
    if (d[0] != 0xFF || d[1] != 0xD8) return false;          // SOI
    const std::size_t window = std::min<std::size_t>(d.size(), 64);
    for (std::size_t k = 2; k <= window; ++k) {              // scan from the end
        const std::size_t i = d.size() - k;
        if (d[i] == 0xFF && d[i + 1] == 0xD9) return true;   // EOI
    }
    return false;
}
}  // namespace

DeviceChwPool::DeviceChwPool() : state_{std::make_shared<State>()} {}

DeviceChwPool::State::~State() {
    for (auto& b : free)
        if (b.ptr) cudaFree(b.ptr);
}

std::shared_ptr<DeviceChwBuf> DeviceChwPool::acquire(std::size_t floats) {
    DeviceChwBuf buf;
    {
        std::lock_guard<std::mutex> lk{state_->mu};
        for (auto it = state_->free.begin(); it != state_->free.end(); ++it) {
            if (it->capacity >= floats) { buf = *it; state_->free.erase(it); break; }
        }
    }
    bool newly_allocated = false;
    if (!buf.ptr) {
        if (cudaMalloc(reinterpret_cast<void**>(&buf.ptr),
                       floats * sizeof(float)) != cudaSuccess)
            return nullptr;
        buf.capacity = floats;
        newly_allocated = true;
    }
    auto st = state_;  // keep pool state alive as long as any buffer is out
    try {
        return std::shared_ptr<DeviceChwBuf>(new DeviceChwBuf(buf), [st](DeviceChwBuf* b) {
            { std::lock_guard<std::mutex> lk{st->mu}; st->free.push_back(*b); }
            delete b;
        });
    } catch (...) {
        // The control-block/DeviceChwBuf allocation threw: don't leak the device
        // buffer — free a freshly-cudaMalloc'd one, or recycle a pooled one.
        if (newly_allocated) {
            cudaFree(buf.ptr);
        } else {
            std::lock_guard<std::mutex> lk{state_->mu};
            state_->free.push_back(buf);
        }
        throw;
    }
}

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
    const bool use_hw = (capture_->options().pixel_format == PixFmt::Nvjpeg);
    if (use_hw) {
        // Construct here so the dlopen + HW init happen on this decode thread.
        try {
            hw_decoder_ = std::make_unique<NvJpegHwDecoder>();
        } catch (const std::exception& e) {
            FITRA_LOG_ERROR("frame_source: HW NVJPEG decoder unavailable: {}", e.what());
            return;  // decode thread exits; --pixel-format nvjpeg cannot proceed
        }
        // All-GPU front-end (M2): when the .so exposes the device entry points
        // and we are prebaking pose, run the RTMPose preprocess on the GPU
        // straight from the EGL-bridged decode output (no per-person CPU warp,
        // no H2D for the pose input). Falls back per-frame if a buffer can't be
        // acquired.
        device_pose_ = rtmpose_enabled_ && hw_decoder_->device_capable();
        // Bench A/B: force the CPU prebake path even when the GPU one is
        // available, to compare CPU load / latency of the two front-ends.
        if (const char* e = std::getenv("FITRA_DISABLE_GPU_PREPROCESS");
            e && *e && std::string(e) != "0") {
            device_pose_ = false;
        }
        // M3: YOLOX preprocess on the GPU too. Only meaningful alongside the
        // RTMPose device path; when both are on we can skip the BGR cvtColor.
        yolox_device_ = device_pose_ && hw_decoder_->yolox_device_capable();
        if (device_pose_)
            FITRA_LOG_INFO("frame_source: all-GPU preprocess enabled (RTMPose{})",
                           yolox_device_ ? " + YOLOX" : "");
    }
    // Downscaling cameras capture at a higher resolution (full sensor FOV) and
    // are resized to the common output resolution. On the all-GPU nvjpeg path the
    // VIC transform does the downscale in its YUV->RGBA pass (decode_to_device
    // target_w/h), so the camera stays fully on the GPU front-end at the runtime
    // resolution -- no CPU resize, no CPU prebake. The CPU/mjpeg and HW-BGR
    // fallback paths still resize `scratch` below. See
    // docs/design/core-pipeline-per-camera-capture-downscale.md.
    const bool downscaling = capture_->options().downscaling();
    const int  out_w = capture_->options().width;
    const int  out_h = capture_->options().height;

    // Software auto-exposure assist init (no-op unless ExposureMode::Assist).
    if (capture_->options().exposure_mode == V4l2Options::ExposureMode::Assist) {
        const auto& o = capture_->options();
        ae_enabled_  = true;
        ae_target_   = o.ae_target;
        ae_gain_min_ = capture_->gain_min();
        ae_gain_max_ = capture_->gain_max();
        // fps-safe exposure cap: keep exposure well under the frame period so a
        // bright->dark swing can't push pacing past the budget. 85% of period.
        const int fps = o.fps > 0 ? o.fps : 60;
        ae_exp_cap_  = static_cast<int>((1.0e6 / fps) * 0.85 / 100.0);  // 100us units
        // Seed current state from the configured initial exposure/gain.
        ae_cur_exp_  = o.exposure_us100 > 0 ? o.exposure_us100 : ae_exp_cap_;
        ae_cur_gain_ = o.gain >= 0 ? o.gain : (ae_gain_min_ + ae_gain_max_) / 2;
        FITRA_LOG_INFO("frame_source: software-AE assist on (target_luma={} gain[{},{}] "
                       "exp[{},{}]x100us start exp={} gain={})",
                       ae_target_, ae_gain_min_, ae_gain_max_, ae_exp_min_, ae_exp_cap_,
                       ae_cur_exp_, ae_cur_gain_);
    }

    // Declared OUTSIDE the loop so its payload vector retains capacity across
    // frames: wait_pop_latest does `raw = *latest_`, a vector copy-assign that
    // reuses raw.data's storage when large enough. A fresh `Frame raw` per
    // iteration would heap-allocate the (2.46MB) YUYV payload every frame
    // (glibc mmap path), mirroring the capture-thread cap fixed in
    // v4l2_capture.cpp -- it would otherwise cap the decode thread at ~53fps.
    Frame raw;
    // Exchanged with decoded_slot_ instead of reconstructed every iteration.
    // Once the central consumer returns its previous frame, the large host CHW
    // vector and optional BGR Mat are reused by this producer (no steady-state
    // allocation/copy in the decoded-frame handoff).
    DecodedFrame df;
    bool was_idle = false;   // idle/standby edge tracker (force re-detect on resume)
    while (!stop_.load()) {
        // Event-driven: block until the capture worker publishes a new frame
        // (or stop_ is set, or the 100ms safety timeout fires). Replaces the
        // old 2ms poll-sleep; wait_pop_latest already dedups on seq.
        if (!capture_->wait_pop_latest(raw, stop_, std::chrono::milliseconds(100))) {
            continue;
        }
        // calib recording forces a BGR copy (raw mp4 tap); pose is skipped.
        const bool calib_recording =
            opts_.calib_recording_flag
            && opts_.calib_recording_flag->load(std::memory_order_relaxed);
        // Idle/standby (issue #37): skip YOLOX + RTMPose pre-bake to drop the
        // bulk of the GPU load, but keep decoding so resume is the next frame.
        // Unlike calib_recording it does NOT force a BGR copy.
        const bool idle =
            opts_.idle_flag
            && opts_.idle_flag->load(std::memory_order_relaxed);
        // On resume (idle->active) the cached bbox is stale (the subject may
        // have moved/left while detection was paused). Force a fresh YOLOX
        // detection on the first active frame so RTMPose never runs on a stale
        // crop (ghost pose) and we don't wait out the det_frequency schedule.
        const bool just_resumed = was_idle && !idle;
        was_idle = idle;

        // Guard the HW NVJPEG decoder against malformed frames (it segfaults on
        // them; the CPU cv::imdecode path tolerates them on its own). Drop the
        // frame here, same as a decode failure.
        if (use_hw && !looks_like_jpeg(raw.data)) {
            FITRA_LOG_WARN("frame_source: dropping corrupt MJPEG seq={} "
                           "(HW decode unsafe)", raw.seq);
            continue;
        }

        bool gpu_decode_ok = false;     // RGBA CUDA buffer retained for GPU prebake
        bool scratch_valid = false;     // `scratch` holds a BGR frame
        int  fw = 0, fh = 0;            // decoded frame dimensions
        if (use_hw) {
            // Full-GPU front-end: YOLOX + RTMPose both run from the RGBA CUDA
            // buffer, so decode straight to device with NO RGBA->BGR cvtColor.
            // BGR is still produced (decode_keep_device) when something on the
            // CPU needs it: calibration recording or retain_bgr, or when YOLOX
            // can't run on the GPU (older .so) and would need a BGR frame.
            const bool need_bgr =
                opts_.retain_bgr || calib_recording || (yolox_ && !yolox_device_);
            // VIC downscale target for the device path (0 = native). The BGR
            // device-retain path can't scale, so a downscaling camera that needs
            // BGR (calib/retain) falls through to plain decode() + cv::resize.
            const int tgt_w = downscaling ? out_w : 0;
            const int tgt_h = downscaling ? out_h : 0;
            if (device_pose_ && !need_bgr) {
                gpu_decode_ok =
                    hw_decoder_->decode_to_device(raw.data.data(), raw.data.size(),
                                                  fw, fh, tgt_w, tgt_h);
            } else if (device_pose_ && !downscaling) {
                gpu_decode_ok =
                    hw_decoder_->decode_keep_device(raw.data.data(), raw.data.size(), scratch);
                scratch_valid = gpu_decode_ok;
            }
            // Plain HW decode when not on the device path, or as a fallback if
            // the device decode/EGL bridge failed — keep the frame on the CPU
            // prebake path rather than dropping it.
            if (!gpu_decode_ok) {
                if (!hw_decoder_->decode(raw.data.data(), raw.data.size(), scratch)) {
                    FITRA_LOG_WARN("frame_source: HW nvjpeg decode failed for seq={}", raw.seq);
                    continue;
                }
                scratch_valid = true;
            }
            if (scratch_valid) { fw = scratch.cols; fh = scratch.rows; }
        } else if (capture_->options().pixel_format == PixFmt::Yuyv) {
            // Packed YUV422 -> BGR. No entropy decode; just a color convert.
            // Interpret the raw buffer at the *capture* dims (may exceed the
            // output dims for a downscaling camera).
            const auto& o = capture_->options();
            const int cw = o.capture_w(), ch = o.capture_h();
            if (static_cast<int>(raw.data.size()) < cw * ch * 2) {
                FITRA_LOG_WARN("frame_source: short YUYV frame for seq={} ({} bytes)",
                               raw.seq, raw.data.size());
                continue;
            }
            cv::Mat yuy2(ch, cw, CV_8UC2,
                         const_cast<std::uint8_t*>(raw.data.data()));
            cv::cvtColor(yuy2, scratch, cv::COLOR_YUV2BGR_YUYV);
        } else {
            if (!decoder_.decode(raw.data, scratch)) {
                FITRA_LOG_WARN("frame_source: jpeg decode failed for seq={}", raw.seq);
                continue;
            }
        }
        if (!use_hw) { scratch_valid = true; fw = scratch.cols; fh = scratch.rows; }
        // Downscale the full-sensor capture to the common output resolution on
        // the CPU / fallback paths (YUYV, CPU mjpeg, HW-BGR). The all-GPU nvjpeg
        // path already downscales on the device via VIC (decode_to_device
        // target_w/h) and keeps no BGR scratch, so this is skipped there.
        // INTER_AREA is the right filter for shrinking. After this, every
        // downstream coordinate (bbox, M_inv, keypoints, drawer, triangulation)
        // is in the output resolution space.
        if (downscaling && scratch_valid && (fw != out_w || fh != out_h)) {
            cv::resize(scratch, scratch, cv::Size(out_w, out_h), 0, 0, cv::INTER_AREA);
            fw = out_w;
            fh = out_h;
        }

        // Software AE assist: slow deadband controller on the decoded frame's
        // mean luma. Needs a BGR frame; the pure all-GPU device path has none
        // (scratch empty) -> assist is unavailable there (warn once). For the
        // YUYV / CPU / HW-BGR paths (incl. the recommended cam1=YUYV) scratch
        // is always valid.
        if (ae_enabled_) {
            if (!scratch_valid) {
                if (!ae_warned_no_bgr_) {
                    FITRA_LOG_WARN("frame_source: AE assist needs a BGR frame but this "
                                   "camera is on the pure all-GPU device path; AE inactive");
                    ae_warned_no_bgr_ = true;
                }
            } else if (++ae_frames_ >= ae_interval_) {
                ae_frames_ = 0;
                const cv::Scalar m = cv::mean(scratch);  // BGR
                const int luma = static_cast<int>(0.114 * m[0] + 0.587 * m[1] + 0.299 * m[2]);
                const int err  = luma - ae_target_;
                // Only advance the internal gain/exposure state when the ioctl
                // actually succeeds: a camera lacking V4L2_CID_GAIN /
                // EXPOSURE_ABSOLUTE (or rejecting it) would otherwise let assist
                // walk a virtual range that doesn't reflect the sensor. A knob
                // whose set fails is marked unavailable so the controller falls
                // through to the other knob instead of getting stuck on it.
                if (err < -ae_deadband_) {            // too dark: gain first, then exposure
                    if (ae_gain_avail_ && ae_cur_gain_ < ae_gain_max_) {
                        const int v = std::min(ae_cur_gain_ + ae_gain_step_, ae_gain_max_);
                        if (capture_->set_gain(v)) ae_cur_gain_ = v; else ae_gain_avail_ = false;
                    } else if (ae_exp_avail_ && ae_cur_exp_ < ae_exp_cap_) {
                        const int v = std::min(ae_cur_exp_ + ae_exp_step_, ae_exp_cap_);
                        if (capture_->set_exposure_us100(v)) ae_cur_exp_ = v; else ae_exp_avail_ = false;
                    }
                } else if (err > ae_deadband_) {      // too bright: drop gain first, then exposure
                    if (ae_gain_avail_ && ae_cur_gain_ > ae_gain_min_) {
                        const int v = std::max(ae_cur_gain_ - ae_gain_step_, ae_gain_min_);
                        if (capture_->set_gain(v)) ae_cur_gain_ = v; else ae_gain_avail_ = false;
                    } else if (ae_exp_avail_ && ae_cur_exp_ > ae_exp_min_) {
                        const int v = std::max(ae_cur_exp_ - ae_exp_step_, ae_exp_min_);
                        if (capture_->set_exposure_us100(v)) ae_cur_exp_ = v; else ae_exp_avail_ = false;
                    }
                }
            }
        }
        auto t_decode = std::chrono::steady_clock::now();

        // YOLOX runs on this thread (one IExecutionContext per FrameSource),
        // so all cameras detect in parallel. Skipped during calib recording —
        // raw mp4 capture is the priority, and dump_keypoints_3d re-runs
        // detection offline on the resulting clips anyway.
        if (yolox_ && !calib_recording && !idle) {
            // Detect on the decimation schedule only. The old `||
            // cached_bboxes_.empty()` clause forced YOLOX EVERY frame whenever
            // nothing was detected -- so an empty/no-person scene ran the
            // heaviest GPU op (YOLOX_s 640, ~18ms) on all cameras at full frame
            // rate, saturating the shared GPU and capping pose at ~48fps (idle
            // was MORE expensive than tracking). The modulo already re-detects
            // every det_frequency frames when empty (~167ms @60fps/det=10 to
            // acquire a person entering frame), which is plenty responsive, so
            // drop the per-frame retry. Camera detects at its staggered phase
            // slot within the period (det_phase) so the cameras' YOLOX bursts
            // don't all land on the same frame and stall the shared GPU. The
            // det_frequency <= 0 short-circuit guards the modulo against a
            // zero/negative frequency (config bug) -> detect every frame.
            bool do_detect = just_resumed || (opts_.det_frequency <= 0) ||
                ((frame_idx_ % opts_.det_frequency) == (opts_.det_phase % opts_.det_frequency));
            if (do_detect) {
                // GPU YOLOX: the preprocess kernel fills the engine input from
                // the retained RGBA device buffer (on the engine's stream); the
                // kernel and enqueue are ordered on that stream. CPU path used
                // when the GPU decode fell back (BGR scratch available).
                std::vector<infer::Bbox> dets;
                if (gpu_decode_ok && yolox_device_) {
                    dets = yolox_->infer_device([&](float* dst, cudaStream_t st) {
                        float r = 1.0f;
                        if (!hw_decoder_->preprocess_yolox_into(
                                yolox_->input_size(), 114.0f, dst, st, &r)) {
                            FITRA_LOG_WARN("frame_source: GPU YOLOX preprocess failed seq={}",
                                           raw.seq);
                            return -1.0f;  // tell infer_device to skip the enqueue
                        }
                        return r;
                    });
                } else {
                    dets = yolox_->infer(scratch);  // scratch_valid holds here
                }
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
            float w = static_cast<float>(fw);  // frame dims (scratch may be empty
            float h = static_cast<float>(fh);  // on the pure-device path)
            fake.x1 = 0.2f * w;
            fake.y1 = 0.2f * h;
            fake.x2 = 0.8f * w;
            fake.y2 = 0.8f * h;
            fake.score = 1.0f;
            cached_bboxes_.push_back(fake);
        }

        // `df` may contain the consumer's previous frame after publish_exchange.
        // Clear logical contents while retaining reusable allocations.
        df.bboxes.clear();
        df.chw_concat.clear();
        df.chw_dev.reset();
        df.M_invs.clear();
        df.seq         = raw.seq;
        df.captured_at = raw.captured_at;
        df.captured_mono_ns = raw.captured_mono_ns;
        df.t_decode    = t_decode;
        df.t_detect    = t_detect;
        // During calib recording (and idle/standby) we drop bboxes too — the
        // central thread sees bboxes.empty() and naturally skips RTMPose.
        // (Without this the "missing prebake" warning would spam.)
        if (!calib_recording && !idle) {
            df.bboxes  = cached_bboxes_;  // copy of current cache
        }

        if (rtmpose_enabled_ && !df.bboxes.empty()) {
            const std::size_t per_item =
                infer::RtmPose::blob_floats_per_item(rtmpose_opts_);
            const std::size_t nb = df.bboxes.size();

            // All-GPU path: run the preprocess kernel from the retained RGBA
            // device buffer into a pooled device CHW buffer. The CPU does only
            // the (cheap) inverse-affine per bbox; no per-person warp/normalize.
            // Gated on gpu_decode_ok so a device-decode fallback this frame
            // (RGBA buffer not retained) takes the CPU prebake path below.
            bool gpu_done = false;
            if (gpu_decode_ok) {
                if (auto buf = chw_pool_.acquire(nb * per_item)) {
                    df.M_invs.resize(nb);
                    bool ok = true;
                    for (std::size_t i = 0; i < nb; ++i) {
                        df.M_invs[i] = infer::RtmPose::compute_m_inv(rtmpose_opts_, df.bboxes[i]);
                        if (!hw_decoder_->preprocess_into(
                                df.M_invs[i].ptr<double>(),
                                rtmpose_opts_.input_w, rtmpose_opts_.input_h,
                                kMeanBgr, kInvStdBgr, buf->ptr + i * per_item)) {
                            ok = false;
                            break;
                        }
                    }
                    if (ok) { df.chw_dev = std::move(buf); gpu_done = true; }
                }
                if (!gpu_done) {
                    FITRA_LOG_WARN("frame_source: GPU preprocess failed seq={}, "
                                   "falling back to CPU prebake", raw.seq);
                    df.M_invs.clear();
                }
            }

            // CPU prebake (default path, and fallback for the GPU path):
            // warp + normalize + HWC->CHW on this per-camera worker thread.
            // Requires a BGR `scratch`; on the pure-device path it is empty, so
            // if the GPU prebake failed there we skip pose for this frame (the
            // central thread handles a missing prebake gracefully) rather than
            // dereferencing an empty Mat.
            if (!gpu_done && scratch_valid) {
                df.chw_concat.resize(nb * per_item);
                df.M_invs.resize(nb);
                for (std::size_t i = 0; i < nb; ++i) {
                    infer::RtmPose::preprocess_to_blob(
                        rtmpose_opts_, scratch, df.bboxes[i],
                        df.chw_concat.data() + i * per_item,
                        df.M_invs[i]);
                }
            } else if (!gpu_done) {
                FITRA_LOG_WARN("frame_source: no GPU prebake and no BGR fallback "
                               "for seq={}; skipping pose this frame", raw.seq);
                df.M_invs.clear();  // has_prebaked_pose_inputs() -> false
            }
        }
        // t_prebake closes the per-camera CPU stage. When the prebake block
        // above was skipped (no bbox / no RTMPose) this equals t_detect so the
        // det->bake delta is 0 rather than a garbage epoch-based value.
        df.t_prebake = std::chrono::steady_clock::now();
        if ((!rtmpose_enabled_ || opts_.retain_bgr || calib_recording) && scratch_valid) {
            scratch.copyTo(df.bgr);
        } else {
            df.bgr.release();
        }

        decoded_slot_.publish_exchange(df);
        ++frame_idx_;
    }
    // NOTE: the HW decoder is deliberately NOT reset here. Tearing it down on
    // the worker thread (cuGraphicsUnregisterResource + NvJPEGDecoder/NvMM/EGL
    // destruction) segfaults — it races the rest of driver shutdown. Destroying
    // it in ~FrameSource (main thread) is clean: the main thread holds the CUDA
    // primary context (TRT binds it process-wide) and the destructor runs at a
    // well-ordered, single-threaded point after the driver has stopped.
}

bool FrameSource::try_pop_latest_decoded(DecodedFrame& out) {
    return decoded_slot_.try_pop(out);
}

bool FrameSource::wait_available(std::atomic<bool>& consumer_stop,
                                 std::chrono::milliseconds timeout) {
    return decoded_slot_.wait_available(consumer_stop, timeout);
}

void FrameSource::wake() {
    decoded_slot_.wake();
}

}  // namespace fitra::camera
