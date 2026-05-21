#pragma once
//
// Snapshot bus: per-camera latest YOLOX/RTMPose result, atomically
// readable by the WebSocket publisher.
//
// The bundle JSON schema must match python/scripts/dual_rtmpose_web.py
// so that web/dual_rtmpose/app.js works unchanged.

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

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
};

struct Skeleton3DSnapshot {
    std::uint64_t seq = 0;
    std::chrono::system_clock::time_point ts{};
    std::vector<infer::Skeleton3D> persons;
    Skeleton3DStats stats;
};

class Skeleton3DBus {
public:
    Skeleton3DBus();

    void update(const Skeleton3DSnapshot& s);
    std::string make_bundle_json();

    // Lock-protected value-copy of the latest snapshot. Used by external
    // consumers that want the raw Skeleton3DSnapshot (Joint3D + stats) rather
    // than the JSON wire form -- in particular the Phase 11 SlimeVR/VMC
    // publisher, which needs to run its own quaternion compose and coordinate
    // transform on the snapshot before serializing.
    Skeleton3DSnapshot snapshot() const;

private:
    mutable std::mutex mu_;
    Skeleton3DSnapshot snapshot_;
    std::uint64_t bundle_seq_ = 0;
};

std::string make_disabled_3d_json();

}  // namespace fitra::pipeline
