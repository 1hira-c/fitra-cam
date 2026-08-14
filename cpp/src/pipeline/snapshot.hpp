#pragma once
//
// Snapshot bus: per-camera latest YOLOX/RTMPose result, atomically
// readable by the WebSocket publisher.
//
// The bundle JSON schema must match python/scripts/dual_rtmpose_web.py
// so that web/dual_rtmpose/app.js works unchanged.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "infer/types.hpp"

namespace fitra::pipeline {

struct CameraSnapshot {
    int                                    id = 0;
    int                                    w  = 0;
    int                                    h  = 0;
    std::uint64_t                          seq = 0;
    std::chrono::steady_clock::time_point  captured_at{};
    std::chrono::system_clock::time_point  captured_wall{};  // wall-clock for ts_ms
    std::vector<infer::Person>             persons;
    std::vector<infer::Bbox>               bboxes;
    // Stats (mirror python dual_rtmpose_web.py "stats" object)
    double      recv_fps         = 0.0;
    double      recent_pose_fps  = 0.0;
    double      avg_pose_fps     = 0.0;
    std::uint64_t processed      = 0;
    std::uint64_t pending        = 0;
    double      stage_ms         = 0.0;
};

class SnapshotBus {
public:
    explicit SnapshotBus(std::size_t n_cameras);

    // Atomically replace the snapshot for cam_id.
    void update(const CameraSnapshot& s);

    // Build the JSON bundle as a single string. seq increments on each call.
    // Schema:
    //   {"seq":N,"ts_ms":int,"cameras":[{...},{...}]}
    std::string make_bundle_json();

    // Number of cameras the bus was sized for.
    std::size_t size() const { return snapshots_.size(); }

private:
    mutable std::mutex          mu_;
    std::vector<CameraSnapshot> snapshots_;
    std::uint64_t               bundle_seq_ = 0;
};

struct Skeleton3DStats {
    bool enabled = false;
    bool ik_locked = false;
    int valid_joints = 0;
    double tri_fps = 0.0;
    double reproj_err_med_px = 0.0;
    double bone_len_drift_pct = 0.0;
    double sync_dt_ms = 0.0;
    double stage_ms = 0.0;
    double subject_height_m = 0.0;
    bool profile_loaded = false;
    std::string subject_id;
    std::string profile_quality_status;
    std::uint64_t processed = 0;
    std::uint64_t sync_miss = 0;
    // Effective runtime source/stages, not merely the requested config. In raw
    // source mode the marker is true and the three postprocess stages are false,
    // while ik_locked retains its legacy meaning (whether the solver/profile
    // has locked), so downstream consumers never infer provenance from it.
    bool raw_3d_source = false;
    bool kalman_enabled = false;
    bool ik_enabled = false;
    bool floor_stability_enabled = false;
    double floor_z_m = 0.0;
    // False on sync-miss/idle snapshots. Contact booleans retain the last
    // state so consumers do not misinterpret a transport gap as both feet
    // becoming airborne.
    bool floor_contact_fresh = false;
    bool floor_contact_left = false;
    bool floor_contact_right = false;
    bool floor_evidence_left = false;
    bool floor_evidence_right = false;
    double floor_correction_left_m = 0.0;
    double floor_correction_right_m = 0.0;
    // Internal world-frame translations used to keep VR FootAnchor geometry
    // on the ungrounded leg while restoring grounding to foot positions.
    std::array<cv::Vec3f, 2> floor_corrections_m{};
};

// Static camera placement in the fitra Z-up world frame, surfaced to the 3D
// viewer so it can draw a frustum per camera. pos = camera center in world,
// quat_wxyz = camera->world rotation (w,x,y,z).
struct CameraPose3D {
    std::string id;
    double pos[3] = {0.0, 0.0, 0.0};
    double quat_wxyz[4] = {1.0, 0.0, 0.0, 0.0};
};

struct Skeleton3DSnapshot {
    std::uint64_t seq = 0;
    std::chrono::system_clock::time_point ts{};
    // steady_clock capture time of the OLDEST contributing camera frame
    // (min over the synced cameras). Lets VR publishers measure end-to-end
    // capture->send latency. Default (epoch) means "no valid capture time";
    // consumers must guard against it before computing a delta.
    std::chrono::steady_clock::time_point t_capture_oldest{};
    std::vector<infer::Skeleton3D> persons;
    // Static camera placements (world frame). Resent every frame; a few cameras
    // is negligible wire cost and lets the viewer build frustums lazily.
    std::vector<CameraPose3D> cameras;
    Skeleton3DStats stats;
};

class Skeleton3DBus {
public:
    Skeleton3DBus();

    void update(const Skeleton3DSnapshot& s);
    // Build the WebSocket JSON bundle. `extra_fields_json` (optional) is
    // inserted at the top level as a sibling of `stats` — caller supplies
    // a comma-less fragment of the form `"key":value[,"key":value]`.
    // Used by CrowServer to embed tracker snapshots without creating a
    // circular dependency on the tracker library.
    std::string make_bundle_json(const std::string& extra_fields_json = "");

    // Lock-protected value-copy of the latest snapshot. Used by external
    // consumers that want the raw Skeleton3DSnapshot (Joint3D + stats) rather
    // than the JSON wire form -- in particular the VMT publisher,
    // which need to run their own quaternion compose and coordinate transform
    // on the snapshot before serializing.
    Skeleton3DSnapshot snapshot() const;

    // Block until update() is called (i.e. a new triangulation result, valid
    // or sync-miss), `consumer_stop` is set, or `timeout` elapses. `last_seen`
    // is the caller's last-observed internal update counter; it is refreshed to
    // the current value on return. Returns true if a genuinely new update was
    // observed (false on timeout/stop). Lets the tracker extractor react to
    // each 3D frame instead of polling at a fixed cadence -- removing the
    // extractor's contribution to capture->send latency.
    bool wait_for_update(std::uint64_t& last_seen,
                         std::atomic<bool>& consumer_stop,
                         std::chrono::milliseconds timeout);

    // Wake any thread parked in wait_for_update (so a consumer stop flag is
    // observed immediately). Notifies under the bus lock.
    void wake();

private:
    mutable std::mutex mu_;
    std::condition_variable cv_;
    Skeleton3DSnapshot snapshot_;
    std::uint64_t bundle_seq_ = 0;
    // Monotonic counter bumped on every update(); used by wait_for_update to
    // detect new data without relying on snapshot_.seq (which is 0 for
    // sync-miss snapshots and would alias).
    std::uint64_t update_seq_ = 0;
};

std::string make_disabled_3d_json();

}  // namespace fitra::pipeline
