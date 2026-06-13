#pragma once
//
// Replay ExcalInputSource: feeds a tools/excal_record session (JPEG sequence
// + frames.jsonl) into the same capture loop the live path uses, so
// calib-extrinsic can run collect→solve with no camera / SteamVR attached
// (docs/design/pose-3d-calib-mode-separation.md, オフライン replay).
//
// Determinism contract:
//  - Records are replayed in frames.jsonl FILE ORDER (= global record-time
//    order). The session's velocity estimator keeps one prev-controller state
//    across all cameras, so per-camera splitting or timestamp re-sorting
//    would change gate decisions and break live↔replay equivalence.
//  - The frame↔pose pairing (including running_ok) was fixed at record time;
//    nothing is re-judged against a wall clock here.
//  - next() decodes the recorded JPEG bytes with cv::imdecode — the same
//    byte stream and decoder family as the live MJPEG path.

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "pipeline/excal_input_source.hpp"

namespace fitra::pipeline {

struct ExcalReplayRecord {
    std::size_t cam = 0;
    std::string file;             // relative JPEG path, e.g. "cam0/000001.jpg"
    ControllerObservation ctrl;   // ts_ms = the record's top-level ts_ms
};

// Strict parser for one frames.jsonl line (the format tools/excal_record
// writes). Returns false on a missing/malformed required key (cam, file,
// ts_ms, ctrl.{running_ok,x,y,z,qx,qy,qz,qw}); the diagnostic-only fields
// (seq, stale, age_ms, tracking_result, timestamp_s) are ignored.
bool parse_excal_frame_line(std::string_view line, ExcalReplayRecord& out);

class ExcalReplayInput : public ExcalInputSource {
public:
    // Loads + parses <session_dir>/frames.jsonl up-front. Throws
    // std::runtime_error on a missing dir/file, unsupported meta.json
    // version, or any unparsable line (with its line number).
    explicit ExcalReplayInput(const std::string& session_dir);

    bool next(ExcalInputItem& out) override;
    bool exhausted() const override { return idx_ >= records_.size(); }

    std::size_t size() const { return records_.size(); }
    // max cam index + 1 (0 for an empty session) — the intrinsics file must
    // cover at least this many cameras.
    std::size_t camera_count() const;

private:
    std::filesystem::path dir_;
    std::vector<ExcalReplayRecord> records_;
    std::size_t idx_ = 0;
};

}  // namespace fitra::pipeline
