#pragma once
// Latest-only live adapter; numerical filtering stays in temporal_candidate.hpp.
#include <algorithm>
#include <array>
#include <memory>
#include <optional>
#include <string>
#include "pipeline/snapshot.hpp"
#include "temporal_candidate.hpp"

namespace fitra::temporal_experiment {
inline Trackers unavailable_trackers() {
    Trackers ts{};
    for (std::size_t i=0; i<ts.size(); ++i) {
        ts[i].role = static_cast<tracking::TrackerRole>(i);
        ts[i].quat_wxyz = {1,0,0,0};
    }
    return ts;
}

class LiveTrialState {
    std::array<std::unique_ptr<Candidate>,5> candidates_;
    std::optional<pipeline::TrackerAxisLineage> lineage_;
    std::chrono::steady_clock::time_point last_capture_{};
    std::array<std::chrono::steady_clock::time_point,26> accepted_at_{};
    std::uint64_t watermark_ = 0, last_publish_ = 0;
    bool active_ = false;
    int selected_ = 0;
    void reset() {
        for (auto& c : candidates_) c->reset();
        lineage_.reset(); last_capture_ = {}; active_ = false;
        trackers = unavailable_trackers();
        presentation = {};
        diagnostics = {};
        accepted_at_ = {};
        output_changed = true;
    }
    void expire_compensation(std::chrono::steady_clock::time_point now) {
        if (selected_!=4 || !active_) return;
        for (int k=0;k<26;++k) {
            auto& d=diagnostics.joint_gate[k];
            if (d.state!=JointAdmission::Predicted || accepted_at_[k].time_since_epoch().count()==0
                || std::chrono::duration<double>(now-accepted_at_[k]).count()<=JointOutlierGate::max_compensation_s+1e-9)
                continue;
            // Receipt-time expiry also runs without another source report.
            // It does not step filters or turn a duplicate into evidence.
            d.state=JointAdmission::Unavailable;
            diagnostics.post_gate.joints[k]={};
            presentation.joints[k]={};
            constexpr std::array<std::array<int,4>,10> dependencies{{
                {5,7,9,-1},{6,8,10,-1},{5,6,18,19},{11,12,18,19},
                {11,13,15,20},{12,14,16,21},{11,13,15,20},{12,14,16,21},
                {13,15,20,-1},{14,16,21,-1}}};
            for (std::size_t t=0;t<trackers.size();++t)
                if (std::find(dependencies[t].begin(),dependencies[t].end(),k)!=dependencies[t].end())
                    trackers[t].valid=false;
            output_changed=true;
        }
    }
public:
    inline static constexpr const char* names[] = {"minimal", "spatial", "distance2", "distance5", "guard5"};
    Trackers trackers = unavailable_trackers();
    infer::Skeleton3D presentation;
    Diagnostics diagnostics;
    bool output_changed = false;
    std::uint64_t processed = 0, generation = 0;
    explicit LiveTrialState(const lift::IkSolver::Options& io) {
        candidates_[0] = std::make_unique<Candidate>(false,false,io);
        candidates_[1] = std::make_unique<Candidate>(false,false,io,4);
        candidates_[2] = std::make_unique<Candidate>(false,false,io,4,.002);
        candidates_[3] = std::make_unique<Candidate>(false,false,io,4,.005);
        candidates_[4] = std::make_unique<Candidate>(false,false,io,4,.005,true);
    }
    const char* selected() const { return names[selected_]; }
    bool active() const { return active_; }
    bool select(const std::string& name) {
        for (int i=0; i<int(candidates_.size()); ++i) if (name==names[i]) {
            selected_=i; reset(); ++generation;
            // Do not process an old source again when switching during a stall.
            watermark_ = std::max(watermark_,last_publish_);
            return true;
        }
        return false;
    }
    void boundary(std::uint64_t publish_ns) {
        watermark_=std::max(watermark_,publish_ns);
        reset();
    }
    bool advance(const pipeline::Skeleton3DSnapshot& snap,
                 std::chrono::steady_clock::time_point now) {
        output_changed=false;
        expire_compensation(now);
        const auto* l = snap.tracker_axis_lineage ? &*snap.tracker_axis_lineage : nullptr;
        const double age = std::chrono::duration<double>(now-snap.t_capture_oldest).count();
        if (!l || l->event_type != pipeline::FusionPoseEventType::Pose ||
            l->source_state != pipeline::FusionPoseSourceState::Fresh ||
            l->source_publish_mono_ns <= watermark_ || !snap.stats.enabled ||
            snap.persons.size()!=1 || snap.persons[0].kp_count!=26 ||
            snap.t_capture_oldest.time_since_epoch().count()==0 || age<0 || age>.1) {
            boundary(l ? l->source_publish_mono_ns : last_publish_);
            return false;
        }
        if (l->source_publish_mono_ns == last_publish_) return false;
        if (l->source_publish_mono_ns < last_publish_) { boundary(last_publish_); return false; }
        if (lineage_ && (lineage_->stream_id!=l->stream_id ||
            lineage_->subject_track_id!=l->subject_track_id ||
            lineage_->coordinate_epoch!=l->coordinate_epoch ||
            lineage_->continuity_epoch!=l->continuity_epoch)) reset();
        double dt=1./60;
        if (last_capture_.time_since_epoch().count()!=0) {
            dt=std::chrono::duration<double>(snap.t_capture_oldest-last_capture_).count();
            if (dt<=0 || dt>.5) { reset(); dt=1./60; }
        }
        for (int i=0; i<int(candidates_.size()); ++i) {
            auto t = candidates_[i]->update(snap.persons[0],dt,1./60,i==selected_ ? &diagnostics : nullptr,
                                            i==selected_ ? &presentation : nullptr);
            if (i==selected_) trackers=t;
        }
        for (const auto& t:trackers) if (t.valid) {
            for (float v:t.pos.val) if (!std::isfinite(v)) { boundary(l->source_publish_mono_ns); return false; }
            for (float v:t.quat_wxyz.val) if (!std::isfinite(v)) { boundary(l->source_publish_mono_ns); return false; }
        }
        last_publish_=l->source_publish_mono_ns;
        last_capture_=snap.t_capture_oldest;
        lineage_=*l; active_=true; ++processed;
        if (selected_==4) for (int k=0;k<26;++k)
            if (diagnostics.joint_gate[k].state==JointAdmission::Accepted) accepted_at_[k]=now;
        expire_compensation(now);
        output_changed=true;
        return true;
    }
};
} // namespace fitra::temporal_experiment
