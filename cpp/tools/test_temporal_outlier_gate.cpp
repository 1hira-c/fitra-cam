#include <iostream>
#include <limits>
#include "temporal_candidate.hpp"

using namespace fitra;
using namespace fitra::temporal_experiment;
namespace {
void check(bool ok,const char* message) { if (!ok) throw std::runtime_error(message); }
infer::Skeleton3D pose(float dx=0) {
    infer::Skeleton3D s; s.kp_count=26;
    auto put=[&](int k,float x,float y,float z) { s.joints[k]={x+dx,y,z,.95f,true}; };
    put(0,0,.08,1.62); put(17,0,0,1.7); put(18,0,0,1.45); put(19,0,0,.95);
    for (int side=0;side<2;++side) {
        const float sign=side ? -1:1;
        put(5+side,sign*.18,0,1.42); put(7+side,sign*.46,.03,1.37); put(9+side,sign*.70,.1,1.32);
        put(11+side,sign*.1,0,.95); put(13+side,sign*.1,.025,.52); put(15+side,sign*.1,0,.09);
        put(20+side,sign*.1,.17,.01); put(22+side,sign*.15,.13,.01); put(24+side,sign*.1,-.05,.01);
    }
    return s;
}
void near(float a,float b,const char* why) { check(std::abs(a-b)<2e-6,why); }
}
int main() {
    try {
        JointOutlierGate gate;
        auto normal=pose(); gate.update(normal,1./60);
        auto spike=normal; spike.joints[13].x+=.4f; spike.joints[13].score=1.5;
        const auto saved=spike;
        auto result=gate.update(spike,1./60);
        check(gate.diagnostics[13].rejected,"high-score spike accepted");
        check(gate.diagnostics[13].state==JointAdmission::Predicted,"spike not predicted");
        near(result.joints[13].x,normal.joints[13].x,"spike entered predictor");
        check(result.joints[13].score==0,"prediction assigned measured confidence");
        near(spike.joints[13].x,saved.joints[13].x,"raw input modified");
        for (int k:{11,12,14,15,16,19}) {
            check(gate.diagnostics[k].state==JointAdmission::Accepted,"unaffected joint rejected");
            near(result.joints[k].x,normal.joints[k].x,"unaffected joint changed");
        }
        result=gate.update(normal,1./60);
        check(gate.diagnostics[13].state==JointAdmission::Accepted,"normal recovery rejected");
        auto missing=normal; missing.joints[13].valid=false;
        result=gate.update(missing,1./60);
        near(result.joints[13].x,normal.joints[13].x,"rejected spike polluted recovery velocity");

        gate.reset(); gate.update(pose(),.01); gate.update(pose(.01),.01);
        missing=pose(); missing.joints[13].valid=false;
        result=gate.update(missing,.025); near(result.joints[13].x,.135,"constant-velocity prediction failed");
        result=gate.update(missing,.025); check(result.joints[13].valid,"50ms endpoint expired early");
        near(result.joints[13].x,.16,"50ms prediction drift");
        result=gate.update(missing,.000001); check(!result.joints[13].valid,"50ms deadline exceeded");
        spike=pose(); spike.joints[13].x+=1;
        result=gate.update(spike,.01); check(!result.joints[13].valid,"expiry silently reseeded first outlier");

        gate.reset(); gate.update(normal,1./60);
        for (int n=0;n<8;++n) {
            spike=normal; spike.joints[13].x+=(n%2 ? -.4f:.4f);
            result=gate.update(spike,1./60);
            check(gate.diagnostics[13].rejected,"alternating spike accepted");
            if (n>=3) check(!result.joints[13].valid,"rejection extended compensation deadline");
        }
        gate.reset(); gate.update(normal,1./60);
        spike=normal; spike.joints[13].x+=.4;
        for (int n=0;n<3;++n) {
            result=gate.update(spike,1./60);
            check(gate.diagnostics[13].reacquired==(n==2),"persistent-shift admission timing");
        }
        missing=normal; missing.joints[13].valid=false;
        result=gate.update(missing,1./60);
        near(result.joints[13].x,spike.joints[13].x,"reseed retained old velocity");
        gate.reset();
        gate.update(pose(),1./60);
        gate.update(pose(.05),1./60);
        gate.update(pose(),1./60);
        check(gate.diagnostics[15].state==JointAdmission::Accepted && !gate.diagnostics[15].rejected,
              "noisy velocity quarantined a small measured reversal");
        gate.reset();
        for (int n=0;n<30;++n) {
            gate.update(pose(float(n)*2.5f/60),1./60);
            check(gate.diagnostics[13].state==JointAdmission::Accepted,"normal fast translation delayed");
        }
        gate.reset();
        gate.update(pose(),.01); gate.update(pose(.04),.01);
        missing=pose(); missing.joints[13].valid=false;
        gate.update(missing,.01);
        gate.update(pose(.045),.01); // A raw return across a hole, not a velocity measurement.
        result=gate.update(missing,.01);
        near(result.joints[13].x,.145,"recovery across a hole seeded untrusted velocity");
        gate.update(pose(.045),.01); gate.update(pose(.055),.01);
        result=gate.update(missing,.01);
        near(result.joints[13].x,.165,"consecutive accepted reports did not reseed velocity");
        gate.reset(); auto invalid=normal;
        invalid.joints[13].x=std::numeric_limits<float>::quiet_NaN();
        check(!gate.update(invalid,.01).joints[13].valid,"nonfinite observation seeded history");
        check(gate.diagnostics[13].age_s<0,"unseeded age fabricated");

        lift::set_active_keypoint_format(lift::KeypointFormat::Halpe26);
        lift::IkSolver::Options io; io.subject_height_m=1.7;
        Candidate candidate(false,false,io,4,.005,true);
        candidate.update(normal,1./60,1./60);
        check(candidate.extract.foot_anchors[0].valid,"foot anchor not initially seeded");
        const auto anchor=candidate.extract.foot_anchors[0];
        spike=normal; spike.joints[13].x+=.4;
        candidate.update(spike,1./60,1./60);
        check(candidate.extract.foot_anchors[0].knee_to_ankle_dir==anchor.knee_to_ankle_dir,
              "predicted knee contaminated foot anchor");
        check(candidate.extract.extension_prev_flex_deg[2]<0
              && candidate.extract.extension_transition_frames[2]==0,
              "predicted chain retained consecutive extension evidence");
        Candidate floor_guard(false,false,io,1,.005,true); // Bypass IK to isolate floor admission.
        auto deep=normal; deep.joints[20].z=-.2;
        const auto floor_ts=floor_guard.update(deep,1./60,1./60);
        check(floor_guard.outlier_gate.diagnostics[20].state==JointAdmission::Accepted,
              "floor fixture was rejected upstream");
        check(!floor_ts[8].valid && !floor_guard.extract.foot_anchors[0].valid,
              "floor-rejected toe reached orientation/anchor learning");
        candidate.reset(); candidate.update(normal,1./60,1./60);
        missing=normal; missing.joints[20].valid=false;
        for (int n=0;n<8;++n) {
            const auto ts=candidate.update(missing,1./60,1./60);
            if (n>=3) {
                check(!ts[8].valid,"expired toe resurrected by indefinite FK");
                check(!candidate.extract.foot_anchors[0].valid,"expired foot anchor survived");
            }
            check(ts[9].valid,"unaffected right foot disabled");
        }
        candidate.reset();
        const auto ts=candidate.update(pose(.8),1./60,1./60);
        check(candidate.outlier_gate.diagnostics[13].state==JointAdmission::Accepted,"reset retained gate state");
        check(ts[8].valid,"reset retained expired FK state");
        std::cout<<"outlier admission: spikes, deadlines, recovery, floor rejection, extension gaps and FK expiry passed\n";
        return 0;
    } catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
