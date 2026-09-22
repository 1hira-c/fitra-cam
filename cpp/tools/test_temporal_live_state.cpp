#include <iostream>
#include "temporal_live_state.hpp"

using namespace fitra;
using namespace fitra::temporal_experiment;
using Clock=std::chrono::steady_clock;
namespace {
void check(bool ok,const char* why) { if (!ok) throw std::runtime_error(why); }
pipeline::Skeleton3DSnapshot frame(std::uint64_t n,float dx=0) {
    pipeline::Skeleton3DSnapshot s;
    s.seq=n; s.stats.enabled=true;
    s.t_capture_oldest=Clock::time_point{std::chrono::nanoseconds(1000000000+n*16666667)};
    pipeline::TrackerAxisLineage l;
    l.source_publish_mono_ns=1005000000+n*16666667;
    l.event_type=pipeline::FusionPoseEventType::Pose;
    l.source_state=pipeline::FusionPoseSourceState::Fresh;
    l.stream_id="test"; l.subject_track_id="subject";
    s.tracker_axis_lineage=l;
    infer::Skeleton3D p; p.kp_count=26;
    auto put=[&](int i,float x,float y,float z) {p.joints[i]={x+dx,y,z,.95f,true};};
    put(0,0,.08,1.62); put(17,0,0,1.7); put(18,0,0,1.45); put(19,0,0,.95);
    for (int side=0; side<2; ++side) {
        const float a=side ? -1:1;
        put(5+side,a*.18,0,1.42); put(7+side,a*.46,.03,1.37); put(9+side,a*.70,.1,1.32);
        put(11+side,a*.1,0,.95); put(13+side,a*.1,.025,.52); put(15+side,a*.1,0,.09);
        put(20+side,a*.1,.17,.01); put(22+side,a*.15,.13,.01); put(24+side,a*.1,-.05,.01);
    }
    s.persons={p}; return s;
}
auto received(const pipeline::Skeleton3DSnapshot& s) { return s.t_capture_oldest+std::chrono::milliseconds(25); }
void same(const Trackers& a,const Trackers& b) {
    for (std::size_t i=0;i<a.size();++i) {
        check(a[i].role==b[i].role && a[i].valid==b[i].valid,"role/availability mismatch");
        check(a[i].pos==b[i].pos && a[i].quat_wxyz==b[i].quat_wxyz,"numerical mismatch");
        check(a[i].roll_confidence==b[i].roll_confidence && a[i].swing_confidence==b[i].swing_confidence,
              "confidence mismatch");
    }
}
void empty(const LiveTrialState& state) {
    check(!state.active(),"stale active flag");
    for (const auto& t:state.trackers) check(!t.valid,"stale valid tracker");
}
}
int main() {
    try {
        lift::set_active_keypoint_format(lift::KeypointFormat::Halpe26);
        lift::IkSolver::Options io; io.subject_height_m=1.7;
        // All four choices use exactly the replay recurrence, once per source.
        for (int c=0;c<4;++c) {
            LiveTrialState state(io); state.select(LiveTrialState::names[c]);
            Candidate oracle(false,false,io,c==0?0:4,c==2?.002:(c==3?.005:0));
            for (std::uint64_t n=0;n<25;++n) {
                auto s=frame(n,float(n%7)*.003f);
                check(state.advance(s,received(s)),"fresh rejected");
                const auto want=oracle.update(s.persons[0],n ? 16666667e-9:1./60,1./60);
                same(state.trackers,want);
                check(!state.advance(s,received(s)+std::chrono::milliseconds(3)),"duplicate advanced filter");
                same(state.trackers,want);
            }
            check(state.processed==25,"duplicate counted");
        }
        LiveTrialState state(io);
        auto a=frame(1); check(state.advance(a,received(a)),"initial fresh");
        check(!state.select("unknown") && state.active(),"invalid selection mutated state");
        check(state.select("distance5"),"selection rejected"); empty(state);
        check(!state.advance(a,received(a)),"selection reused old report");
        auto b=frame(2,.04f); check(state.advance(b,received(b)),"selection did not recover");
        LiveTrialState fresh(io); fresh.select("distance5"); fresh.advance(b,received(b));
        same(state.trackers,fresh.trackers);

        // A queued boundary may outrun the latest-only raw snapshot.
        state.boundary(b.tracker_axis_lineage->source_publish_mono_ns+1); empty(state);
        check(!state.advance(b,received(b)),"boundary reseeded stale snapshot"); empty(state);
        auto d=frame(4,.02f); check(state.advance(d,received(d)),"boundary recovery failed");
        LiveTrialState reset(io); reset.select("distance5"); reset.advance(d,received(d)); same(state.trackers,reset.trackers);

        auto changed=frame(5,.03f); changed.tracker_axis_lineage->subject_track_id="other";
        check(state.advance(changed,received(changed)),"new lifecycle rejected");
        LiveTrialState identity(io); identity.select("distance5"); identity.advance(changed,received(changed));
        same(state.trackers,identity.trackers);

        // Stale input, missing proof, explicit non-Fresh, absent pose and time reversal fail closed.
        check(!state.advance(changed,changed.t_capture_oldest+std::chrono::milliseconds(101)),"stale accepted"); empty(state);
        check(!state.advance(changed,received(changed)),"stale report reseeded");
        for (int reason=0;reason<5;++reason) {
            auto s=frame(10+reason);
            if (reason==0) s.tracker_axis_lineage.reset();
            if (reason==1) s.tracker_axis_lineage->source_state=pipeline::FusionPoseSourceState::Unavailable;
            if (reason==2) s.persons.clear();
            if (reason==3) s.stats.enabled=false;
            const auto now=reason==4 ? s.t_capture_oldest-std::chrono::milliseconds(1):received(s);
            check(!state.advance(s,now),"inadmissible source accepted"); empty(state);
        }
        auto recovered=frame(20,.01f); check(state.advance(recovered,received(recovered)),"recovery rejected");
        auto gap=frame(60,.015f); check(state.advance(gap,received(gap)),"gap recovery rejected");
        LiveTrialState gap_oracle(io); gap_oracle.select("distance5"); gap_oracle.advance(gap,received(gap));
        same(state.trackers,gap_oracle.trackers);
        auto backwards=frame(59); check(!state.advance(backwards,received(backwards)),"backwards publish accepted"); empty(state);
        std::cout << "temporal live state: recurrence/duplicates/switch/boundaries/identity/stale/gaps passed\n";
        return 0;
    } catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
