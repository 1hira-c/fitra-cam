#include "pipeline/snapshot.hpp"

#include <cstdio>
#include <ctime>
#include <sstream>

#include "lift/keypoint_format.hpp"

namespace fitra::pipeline {

namespace {

void append_float(std::string& out, double v, int precision = 6) {
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.*g", precision, v);
    out += buf;
}

void append_json_string(std::string& out, const std::string& value) {
    out += "\"";
    for (char ch : value) {
        switch (ch) {
            case '\\': out += "\\\\"; break;
            case '"': out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out += ch; break;
        }
    }
    out += "\"";
}

}  // namespace

SnapshotBus::SnapshotBus(std::size_t n_cameras) : snapshots_(n_cameras) {
    for (std::size_t i = 0; i < n_cameras; ++i) {
        snapshots_[i].id = static_cast<int>(i);
    }
}

void SnapshotBus::update(const CameraSnapshot& s) {
    if (static_cast<std::size_t>(s.id) >= snapshots_.size()) return;
    std::lock_guard<std::mutex> lk{mu_};
    snapshots_[static_cast<std::size_t>(s.id)] = s;
}

std::string SnapshotBus::make_bundle_json() {
    using clock = std::chrono::system_clock;
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      clock::now().time_since_epoch()).count();
    std::lock_guard<std::mutex> lk{mu_};
    ++bundle_seq_;

    std::string out;
    out.reserve(2048);
    out += "{\"seq\":";
    out += std::to_string(bundle_seq_);
    out += ",\"ts_ms\":";
    out += std::to_string(static_cast<long long>(now_ms));
    // Surface the active topology so the frontend knows whether the keypoint
    // array has 17 (COCO17) or 26 (Halpe26) entries and which edge table to
    // draw.
    out += ",\"kp_format\":\"";
    out += fitra::lift::keypoint_format_name(fitra::lift::active_keypoint_format());
    out += "\"";
    out += ",\"cameras\":[";
    for (std::size_t i = 0; i < snapshots_.size(); ++i) {
        if (i) out += ",";
        const auto& s = snapshots_[i];
        out += "{\"id\":";
        out += std::to_string(s.id);
        out += ",\"w\":";
        out += std::to_string(s.w);
        out += ",\"h\":";
        out += std::to_string(s.h);

        out += ",\"persons\":[";
        for (std::size_t pi = 0; pi < s.persons.size(); ++pi) {
            if (pi) out += ",";
            const auto& p = s.persons[pi];
            out += "{\"kpts\":[";
            const std::size_t emit_n = std::min<std::size_t>(
                p.kp_count, p.kpts.size());
            for (std::size_t k = 0; k < emit_n; ++k) {
                if (k) out += ",";
                out += "[";
                append_float(out, p.kpts[k].x);     out += ",";
                append_float(out, p.kpts[k].y);     out += ",";
                append_float(out, p.kpts[k].score);
                out += "]";
            }
            out += "]";
            // bbox aligned with infer::Person::bbox (the Python publisher
            // includes the bbox alongside kpts when available).
            if (pi < s.bboxes.size()) {
                const auto& bb = s.bboxes[pi];
                out += ",\"bbox\":[";
                append_float(out, bb.x1);    out += ",";
                append_float(out, bb.y1);    out += ",";
                append_float(out, bb.x2);    out += ",";
                append_float(out, bb.y2);    out += ",";
                append_float(out, bb.score); out += "]";
            }
            out += "}";
        }
        out += "]";

        out += ",\"stats\":{";
        out += "\"recv_fps\":";       append_float(out, s.recv_fps, 4);
        out += ",\"recent_pose_fps\":"; append_float(out, s.recent_pose_fps, 4);
        out += ",\"avg_pose_fps\":";    append_float(out, s.avg_pose_fps, 4);
        out += ",\"pending\":";         out += std::to_string(static_cast<long long>(s.pending));
        out += ",\"stage_ms\":";        append_float(out, s.stage_ms, 4);
        out += ",\"processed\":";       out += std::to_string(static_cast<long long>(s.processed));
        out += ",\"captured_at_ms\":";
        auto cap_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          s.captured_wall.time_since_epoch()).count();
        out += std::to_string(static_cast<long long>(cap_ms));
        out += "}";
        out += "}";
    }
    out += "]}";
    return out;
}

Skeleton3DBus::Skeleton3DBus() {
    snapshot_.stats.enabled = true;
    snapshot_.ts = std::chrono::system_clock::now();
}

void Skeleton3DBus::update(const Skeleton3DSnapshot& s) {
    std::lock_guard<std::mutex> lk{mu_};
    snapshot_ = s;
    snapshot_.stats.enabled = true;
    ++update_seq_;
    cv_.notify_all();  // wake an extractor parked in wait_for_update
}

Skeleton3DSnapshot Skeleton3DBus::snapshot() const {
    std::lock_guard<std::mutex> lk{mu_};
    return snapshot_;
}

bool Skeleton3DBus::wait_for_update(std::uint64_t& last_seen,
                                    std::atomic<bool>& consumer_stop,
                                    std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lk{mu_};
    bool got = cv_.wait_for(lk, timeout, [&] {
        return consumer_stop.load(std::memory_order_relaxed)
            || update_seq_ != last_seen;
    });
    bool fresh = update_seq_ != last_seen;
    last_seen = update_seq_;
    return got && fresh;
}

void Skeleton3DBus::wake() {
    std::lock_guard<std::mutex> lk{mu_};
    cv_.notify_all();
}

std::string Skeleton3DBus::make_bundle_json(const std::string& extra_fields_json) {
    using clock = std::chrono::system_clock;
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      clock::now().time_since_epoch()).count();
    std::lock_guard<std::mutex> lk{mu_};
    ++bundle_seq_;

    const auto& s = snapshot_;
    std::string out;
    out.reserve(2048);
    out += "{\"seq\":";
    out += std::to_string(bundle_seq_);
    out += ",\"ts_ms\":";
    out += std::to_string(static_cast<long long>(now_ms));
    out += ",\"enabled\":true";
    out += ",\"kp_format\":\"";
    out += fitra::lift::keypoint_format_name(fitra::lift::active_keypoint_format());
    out += "\"";
    out += ",\"persons_3d\":[";
    for (std::size_t pi = 0; pi < s.persons.size(); ++pi) {
        if (pi) out += ",";
        out += "{\"id\":";
        out += std::to_string(static_cast<int>(pi));
        out += ",\"joints\":[";
        const auto& skel = s.persons[pi];
        const std::size_t emit_n = std::min<std::size_t>(
            skel.kp_count, skel.joints.size());
        for (std::size_t k = 0; k < emit_n; ++k) {
            if (k) out += ",";
            const auto& j = skel.joints[k];
            out += "[";
            append_float(out, j.x); out += ",";
            append_float(out, j.y); out += ",";
            append_float(out, j.z); out += ",";
            append_float(out, j.score); out += ",";
            out += (j.valid ? "true" : "false");
            out += "]";
        }
        out += "]}";
    }
    out += "]";
    // Static camera placements (world frame): position + camera->world quat.
    // Lets the 3D viewer draw a frustum per camera.
    out += ",\"cameras\":[";
    for (std::size_t ci = 0; ci < s.cameras.size(); ++ci) {
        if (ci) out += ",";
        const auto& cam = s.cameras[ci];
        out += "{\"id\":";
        append_json_string(out, cam.id);
        out += ",\"pos\":[";
        append_float(out, cam.pos[0]); out += ",";
        append_float(out, cam.pos[1]); out += ",";
        append_float(out, cam.pos[2]);
        out += "],\"quat_wxyz\":[";
        append_float(out, cam.quat_wxyz[0]); out += ",";
        append_float(out, cam.quat_wxyz[1]); out += ",";
        append_float(out, cam.quat_wxyz[2]); out += ",";
        append_float(out, cam.quat_wxyz[3]);
        out += "]}";
    }
    out += "]";
    out += ",\"stats\":{";
    out += "\"enabled\":true";
    out += ",\"tri_fps\":"; append_float(out, s.stats.tri_fps, 4);
    out += ",\"reproj_err_med_px\":"; append_float(out, s.stats.reproj_err_med_px, 4);
    out += ",\"bone_len_drift_pct\":"; append_float(out, s.stats.bone_len_drift_pct, 4);
    out += ",\"valid_joints\":"; out += std::to_string(s.stats.valid_joints);
    out += ",\"sync_dt_ms\":"; append_float(out, s.stats.sync_dt_ms, 4);
    out += ",\"stage_ms\":"; append_float(out, s.stats.stage_ms, 4);
    out += ",\"subject_height_m\":"; append_float(out, s.stats.subject_height_m, 4);
    out += ",\"profile_loaded\":"; out += (s.stats.profile_loaded ? "true" : "false");
    out += ",\"subject_id\":"; append_json_string(out, s.stats.subject_id);
    out += ",\"quality_status\":"; append_json_string(out, s.stats.profile_quality_status);
    out += ",\"processed\":"; out += std::to_string(static_cast<long long>(s.stats.processed));
    out += ",\"sync_miss\":"; out += std::to_string(static_cast<long long>(s.stats.sync_miss));
    out += ",\"ik_locked\":"; out += (s.stats.ik_locked ? "true" : "false");
    out += ",\"raw_3d_source\":";
    out += (s.stats.raw_3d_source ? "true" : "false");
    out += ",\"kalman_enabled\":";
    out += (s.stats.kalman_enabled ? "true" : "false");
    out += ",\"ik_enabled\":";
    out += (s.stats.ik_enabled ? "true" : "false");
    out += ",\"floor_stability_enabled\":";
    out += (s.stats.floor_stability_enabled ? "true" : "false");
    out += ",\"floor_z_m\":"; append_float(out, s.stats.floor_z_m, 4);
    out += ",\"floor_contact_fresh\":";
    out += (s.stats.floor_contact_fresh ? "true" : "false");
    out += ",\"floor_contact_left\":";
    out += (s.stats.floor_contact_left ? "true" : "false");
    out += ",\"floor_contact_right\":";
    out += (s.stats.floor_contact_right ? "true" : "false");
    out += ",\"floor_evidence_left\":";
    out += (s.stats.floor_evidence_left ? "true" : "false");
    out += ",\"floor_evidence_right\":";
    out += (s.stats.floor_evidence_right ? "true" : "false");
    out += ",\"floor_correction_left_m\":";
    append_float(out, s.stats.floor_correction_left_m, 4);
    out += ",\"floor_correction_right_m\":";
    append_float(out, s.stats.floor_correction_right_m, 4);
    out += "}";   // close stats
    if (!extra_fields_json.empty()) {
        out += ",";
        out += extra_fields_json;
    }
    out += "}";   // close outer
    return out;
}

std::string make_disabled_3d_json() {
    using clock = std::chrono::system_clock;
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      clock::now().time_since_epoch()).count();
    std::string out;
    out += "{\"seq\":0,\"ts_ms\":";
    out += std::to_string(static_cast<long long>(now_ms));
    out += ",\"enabled\":false,\"persons_3d\":[],\"stats\":{\"enabled\":false}}";
    return out;
}

}  // namespace fitra::pipeline
