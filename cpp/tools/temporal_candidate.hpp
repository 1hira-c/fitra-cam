#pragma once
// Experimental numerical path shared by offline replay and opt-in VMT trials.
// Never used by the normal main / production TrackerExtractor.
#include "lift/kalman.hpp"
#include "lift/ik.hpp"
#include "lift/floor_contact_stabilizer.hpp"
#include "tracking/tracker_extractor.hpp"
#include "temporal_weighted_bones.hpp"
#include "temporal_distance_filter.hpp"

namespace fitra::temporal_experiment {
using Trackers = std::array<tracking::TrackerPose, tracking::kTrackerCount>;
struct Diagnostics {
    infer::Skeleton3D post_kalman, post_ik;
    Trackers pre_smoothing{};
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

    Candidate(bool k, bool e, const lift::IkSolver::Options& io, int experiment = 0, double cap_m = 0)
        : use_kalman(k), use_euro(e), experimental_ik(experiment),
          bones(experiment ? bone_targets(io) : std::vector<Bone>{}),
          ik(io), distance_cap_m(cap_m) { reset(); }

    void reset() {
        kalman.reset();
        floor.reset();
        extract = {}; pos = {}; quat = {};
        prev_pos = {};
        distance_filter.reset();
        for (auto& q : prev_quat) q = {1, 0, 0, 0};
    }

    Trackers update(infer::Skeleton3D s, double dt, double nominal, Diagnostics* diagnostic = nullptr,
                    infer::Skeleton3D* presentation = nullptr) {
        if (use_kalman) s = kalman.update(s, dt);
        if (diagnostic) diagnostic->post_kalman = s;
        if (experimental_ik == 0) s = ik.update(s);
        else if (experimental_ik >= 2)
            s = soft_bones(s, bones,
                (experimental_ik == 3 || experimental_ik == 5) ? .1 : 1., experimental_ik >= 4);
        if (diagnostic) diagnostic->post_ik = s;
        const auto report = floor.update(s, dt);
        if (presentation) *presentation = s;
        auto t = tracking::extract_trackers_with_floor_corrections(
            s, {report.feet[0].correction_m, report.feet[1].correction_m},
            &extract, opts.foot_pos_mode, opts.chest_height_frac,
            opts.waist_height_frac, opts.limb_extension);
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
