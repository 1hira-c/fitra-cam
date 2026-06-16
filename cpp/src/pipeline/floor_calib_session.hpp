#pragma once
//
// Floor-AprilTag extrinsic calibration collection loop (VR-free path).
//
// See docs/design/pose-3d-floor-apriltag-extrinsic.md. A separate class from
// ExtrinsicCalibSession by design: the floor path has no VR controller, no
// motion gate, and recovers T_cam←world directly in the fitra Z-up world (no
// VMT basis change). It only shares AprilTag detection (with CLAHE) and the
// calib_io YAML writer.
//
// Cameras and tags are static, so there is no motion to gate against: we simply
// accumulate each (camera, tag) corner observation across frames and arithmetic-
// average them (averaging static observations reduces detector noise). On solve
// each camera localises against the FloorTagMap via multi-tag PnP.
//
// Threading: on_frame() runs on the single capture thread; the read methods
// (state/state_json) and solve_and_write() may run on the Crow / main thread.
// All shared state is mutex-guarded.

#include <array>
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "lift/apriltag_marker.hpp"
#include "lift/calib_io.hpp"
#include "lift/floor_extrinsic_solver.hpp"
#include "lift/floor_tag_map.hpp"

namespace fitra::pipeline {

enum class FloorCalibState {
    kIdle,
    kCollecting,
    kSolving,
    kSolved,
    kFailed,
};

const char* floor_calib_state_name(FloorCalibState s);

struct FloorCalibConfig {
    // Intrinsics used for tag detection + PnP. Camera index in on_frame() maps
    // to intrinsics.cameras[idx]. These are the CALIBRATION-resolution
    // intrinsics. Required.
    lift::CalibrationSet intrinsics;
    // Intrinsics written to the output YAML (the RUNTIME resolution the
    // triangulator will use). If empty, `intrinsics` is written instead — the
    // recovered T_cw is resolution-independent, so either set pairs validly.
    lift::CalibrationSet out_intrinsics;
    // Number of cameras actually captured this session. 0 → fall back to the
    // intrinsics file count. Set this to the live/replay camera count so that
    // a calibration file with MORE cameras than are in use does not drag empty
    // (unused) cameras into the solve and fail it.
    std::size_t num_cams = 0;
    // Known tag layout = the world frame. Required.
    lift::FloorTagMap    map;
    // Detector config. If faces is empty the ctor fills it from `map`. CLAHE is
    // forced on (the floor feasibility study found it necessary).
    lift::MarkerBoardConfig board;
    // Whether the intrinsics use the fisheye distortion model.
    bool   fisheye = false;

    // A (cam, tag) group emits an averaged observation once it reaches
    // burst_min frames; accumulation stops contributing past burst_max.
    int    burst_min = 10;
    int    burst_max = 60;

    // Per-frame detection reprojection filter (px): a tag PnP above this is
    // dropped as a likely mis-decode before it pollutes the average.
    double max_pnp_reproj_px = 3.0;

    lift::FloorSolverOptions solver;
    std::string out_path = "calibrations/extrinsics.yaml";
};

class FloorCalibSession {
public:
    explicit FloorCalibSession(FloorCalibConfig cfg);

    FloorCalibSession(const FloorCalibSession&) = delete;
    FloorCalibSession& operator=(const FloorCalibSession&) = delete;

    void start();            // → kCollecting
    void stop_collecting();  // → kIdle (accumulators retained)

    // Capture frame tap. `bgr` need not be retained past the call. ts_ms is a
    // frame timestamp used only for the live UI "age" display.
    void on_frame(std::size_t cam_idx, const cv::Mat& bgr, double ts_ms = 0.0);

    // Testable core: feed pre-computed tag corners (no image / detection) into
    // the accumulator. Returns true if buffered (state == kCollecting).
    bool ingest(std::size_t cam_idx, int tag_id,
                const std::array<cv::Point2f, 4>& corners);

    bool solve_and_write(std::string& err);

    void set_on_solved(std::function<void()> fn) { on_solved_ = std::move(fn); }

    FloorCalibState state() const;
    std::size_t     ready_group_count() const;  // (cam,tag) groups ≥ burst_min
    std::string     state_json() const;         // self-contained, for Crow

    const lift::FloorExtrinsicSolution& last_solution() const { return solution_; }

private:
    struct GroupKey {
        std::size_t cam;
        int         tag;
        bool operator<(const GroupKey& o) const {
            return cam != o.cam ? cam < o.cam : tag < o.tag;
        }
    };
    struct CornerAccum {
        std::array<cv::Point2d, 4> sum{};   // running corner sums
        int count = 0;
    };
    struct LiveDetection {
        int    tag_id        = 0;
        double reproj_rms_px = 0.0;
        bool   pose_ok       = false;
    };
    struct CamLive {
        double last_ts_ms = 0.0;
        std::vector<LiveDetection> dets;
    };

    FloorCalibConfig cfg_;

    mutable std::mutex mu_;
    FloorCalibState state_ = FloorCalibState::kIdle;

    std::map<GroupKey, CornerAccum> accum_;
    std::map<std::size_t, CamLive>  cam_live_;
    double last_frame_ts_ms_ = 0.0;

    lift::FloorExtrinsicSolution solution_;
    std::string                  last_error_;
    std::function<void()>        on_solved_;

    std::unique_ptr<lift::AprilTagDetector> detector_;
};

}  // namespace fitra::pipeline
