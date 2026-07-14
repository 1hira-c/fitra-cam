#pragma once
//
// 3D lifting stack (triangulator + buses + subject profile resolution) and
// MultiCameraDriver construction. Used by run (3D optional) and calib-subject
// (3D required); calib-extrinsic has no 3D at all.

#include <memory>
#include <vector>

#include "config/main_config.hpp"
#include "infer/rtmpose.hpp"
#include "lift/subject_profile.hpp"
#include "lift/triangulator.hpp"
#include "pipeline/multi_pipeline.hpp"
#include "pipeline/snapshot.hpp"
#include "tracking/tracker_bus.hpp"

namespace fitra::app {

struct ThreeDSet {
    std::shared_ptr<lift::Triangulator>        triangulator;
    std::unique_ptr<pipeline::Skeleton3DBus>   bus3d;
    // Tracker snapshot bus. Always alive when 3D is on so the WebUI
    // orientation viz works without VMT output.
    std::unique_ptr<tracking::TrackerBus>  tracker_bus;
    lift::SubjectProfile subject_profile;
    bool   has_subject_profile = false;
    // Resolved height: profile value wins over --subject-height-m.
    double subject_height_m = 0.0;
};

// Loads the calibration + optional subject profile per opts. Throws on a
// missing/invalid calibration YAML (run keeps its current behavior of failing
// the boot; 2D-only runs simply don't call this).
ThreeDSet make_threed(const config::MainOptions& opts,
                      std::size_t n_cams,
                      double subject_height_m);

// threed == nullptr → 2D-only driver. Does not start() the driver.
std::unique_ptr<pipeline::MultiCameraDriver> make_driver(
    std::vector<std::unique_ptr<camera::FrameSource>> sources,
    infer::RtmPose& rtmpose,
    pipeline::SnapshotBus& bus,
    const config::MainOptions& opts,
    const ThreeDSet* threed);

}  // namespace fitra::app
