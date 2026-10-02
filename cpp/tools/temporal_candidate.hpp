#pragma once
// Experimental numerical path shared by offline replay and opt-in VMT trials.
// Never used by the normal main / production TrackerExtractor.
#include "lift/kalman.hpp"
#include "lift/ik.hpp"
#include "lift/floor_contact_stabilizer.hpp"
#include "tracking/tracker_extractor.hpp"
#include "temporal_weighted_bones.hpp"
#include "temporal_distance_filter.hpp"
#include "temporal_outlier_gate.hpp"

namespace fitra::temporal_experiment {
using Trackers = std::array<tracking::TrackerPose, tracking::kTrackerCount>;
struct Diagnostics {
    infer::Skeleton3D post_kalman, post_ik;
    Trackers pre_smoothing{};
    infer::Skeleton3D post_gate;
    std::array<JointGateDiagnostic, infer::kMaxKeypoints> joint_gate{};
};

struct Candidate {
    bool use_kalman, use_euro;
    int experimental_ik = 0;
    std::vector<Bone> bones;
    lift::SkeletonKalman kalman;
    lift::IkSolver ik;
    lift::FloorContactStabilizer floor;
    tracking::TrackerExtractorOptions opts;
    tracking::ExtractContext extract;
    tracking::PosSmoothingContext pos;
    tracking::QuatSmoothingContext quat;
    std::array<cv::Vec3f, tracking::kTrackerCount> prev_pos{};
    std::array<cv::Vec4f, tracking::kTrackerCount> prev_quat{};
    double distance_cap_m = 0;
    DistanceFilter distance_filter;
    bool use_outlier_gate = false;
    JointOutlierGate outlier_gate;

    Candidate(bool k, bool e, const lift::IkSolver::Options& io, int experiment = 0, double cap_m = 0,
              bool guard = false)
        : use_kalman(k), use_euro(e), experimental_ik(experiment),
          bones(experiment ? bone_targets(io) : std::vector<Bone>{}),
          ik(io), distance_cap_m(cap_m), use_outlier_gate(guard) { reset(); }

    void reset() {
        kalman.reset();
        floor.reset();
        extract = {}; pos = {}; quat = {};
        prev_pos = {};
        distance_filter.reset();
        outlier_gate.reset();
        for (auto& q : prev_quat) q = {1, 0, 0, 0};
    }

    Trackers update(infer::Skeleton3D s, double dt, double nominal, Diagnostics* diagnostic = nullptr,
                    infer::Skeleton3D* presentation = nullptr) {
        if (use_outlier_gate) {
            s = outlier_gate.update(s,dt);
            if (diagnostic) {
                diagnostic->post_gate = s;
                diagnostic->joint_gate = outlier_gate.diagnostics;
            }
        }
        if (use_kalman) s = kalman.update(s, dt);
        if (diagnostic) diagnostic->post_kalman = s;
        if (experimental_ik == 0) s = ik.update(s);
        else if (experimental_ik >= 2)
            s = soft_bones(s, bones,
                (experimental_ik == 3 || experimental_ik == 5) ? .1 : 1., experimental_ik >= 4);
        if (diagnostic) diagnostic->post_ik = s;
        lift::FloorContactReport report;
        if (!use_outlier_gate) report = floor.update(s, dt);
        else {
            // Floor evidence comes only from accepted observations. A synthesized
            // sole does not earn another contact-observation/grace interval.
            auto observed = s;
            for (std::size_t k=0; k<26; ++k)
                if (outlier_gate.diagnostics[k].state != JointAdmission::Accepted)
                    observed.joints[k].valid = false;
            report = floor.update(observed,dt);
            for (std::size_t k=0; k<26; ++k)
                if (outlier_gate.diagnostics[k].state == JointAdmission::Accepted) {
                    // Preserve floor's independent sole-outlier rejection.
                    // Its masked predicted evidence must not erase geometry.
                    s.joints[k].valid = observed.joints[k].valid;
                    s.joints[k].score = observed.joints[k].score;
                }
            for (int side=0; side<2; ++side) for (int k : {15+side,20+side,22+side,24+side}) {
                auto& j = s.joints[k];
                if (!j.valid) continue;
                const auto c = report.feet[side].correction_m;
                j.x += c[0]; j.y += c[1]; j.z += c[2];
            }
        }
        if (presentation) *presentation = s;
        const auto observed_extract = extract;
        if (use_outlier_gate) for (int side=0; side<2; ++side)
            for (int k : {13+side,15+side,20+side})
                if (outlier_gate.diagnostics[k].state == JointAdmission::Unavailable || !s.joints[k].valid)
                    extract.foot_anchors[side] = {};
        auto t = tracking::extract_trackers_with_floor_corrections(
            s, {report.feet[0].correction_m, report.feet[1].correction_m},
            &extract, opts.foot_pos_mode, opts.chest_height_frac,
            opts.waist_height_frac, opts.limb_extension);
        if (use_outlier_gate) {
            for (int side=0; side<2; ++side) {
                bool measured = true, unavailable = false;
                for (int k : {13+side,15+side,20+side}) {
                    const auto state = outlier_gate.diagnostics[k].state;
                    measured &= state == JointAdmission::Accepted && s.joints[k].valid;
                    unavailable |= state == JointAdmission::Unavailable || !s.joints[k].valid;
                }
                // No anchor learning from synthetic joints, nor indefinite FK
                // after the gate's bounded compensation has expired.
                if (!measured) extract.foot_anchors[side] = unavailable
                    ? tracking::FootAnchor{} : observed_extract.foot_anchors[side];
                if (unavailable) t[8+side].valid = false;
            }
            for (int limb=0; limb<4; ++limb) {
                const int side = limb%2;
                bool measured = true;
                for (int k : limb<2 ? std::array<int,3>{5+side,7+side,9+side}
                                    : std::array<int,3>{11+side,13+side,15+side})
                    measured &= outlier_gate.diagnostics[k].state == JointAdmission::Accepted;
                if (!measured) {
                    extract.extension_latched[limb] = observed_extract.extension_latched[limb];
                    extract.extension_flex_extreme_deg[limb] = observed_extract.extension_flex_extreme_deg[limb];
                    // A consecutive raw-flexion test cannot cross synthesis.
                    extract.extension_prev_flex_deg[limb] = -1;
                    extract.extension_transition_frames[limb] = 0;
                }
            }
        }
        if (diagnostic) diagnostic->pre_smoothing = t;
        pos.dt_s = static_cast<float>(dt);
        const auto& hip = s.joints[19];
        pos.hip_valid = hip.valid;
        pos.current_hip_pos = {hip.x, hip.y, hip.z};
        if (use_euro) {
            tracking::apply_quat_smoothing(t, prev_quat, quat, opts.quat_one_euro, dt, nominal);
            tracking::apply_pos_smoothing(t, prev_pos, pos, opts.pos_one_euro, nominal);
        } else {
            tracking::apply_quat_smoothing(t, prev_quat, 1.0f, dt, nominal);
            tracking::apply_pos_smoothing(t, prev_pos, pos, 1.0f, nominal);
        }
        if (distance_cap_m > 0) distance_filter.update(t, distance_cap_m);
        return t;
    }
};
} // namespace fitra::temporal_experiment
