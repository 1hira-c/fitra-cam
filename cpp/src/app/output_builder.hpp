#pragma once
//
// VR output construction. The TrackerExtractor feeds the WebUI viz as well as
// the publishers, so run and calib-subject both build it (whenever 3D is on);
// the publishers + continuous aligner are run-mode-only — setup modes never
// emit tracker output (docs/design/pose-3d-calib-mode-separation.md).

#include <atomic>
#include <memory>

#include "config/main_config.hpp"
#include "pipeline/snapshot.hpp"
#include "slimevr/native_publisher.hpp"
#include "slimevr/slime_tracker_bus.hpp"
#include "slimevr/tracker_extractor.hpp"
#include "vmt/continuous_aligner.hpp"
#include "vmt/hmd_pose_receiver.hpp"
#include "vmt/vmt_publisher.hpp"

namespace fitra::app {

// Started on return. Single producer for the smoothed tracker stream shared
// by SlimeVR/VMT publishers and the WebUI.
// `idle_flag` (issue #37): non-null wires the standby gate so the extractor
// resets its smoothing on resume. Run mode only; calib passes null.
std::unique_ptr<slimevr::TrackerExtractor> make_tracker_extractor(
    const config::MainOptions& opts,
    pipeline::Skeleton3DBus& bus3d,
    slimevr::SlimeTrackerBus& tracker_bus,
    const std::atomic<bool>* idle_flag = nullptr);

struct RunOutputs {
    std::unique_ptr<slimevr::NativePublisher> slime_pub;
    std::unique_ptr<vmt::VmtPublisher>        vmt_pub;
    std::unique_ptr<vmt::ContinuousAligner>   aligner;

    void stop() {
        // The aligner reads vmt_pub + the buses, so it stops first.
        if (aligner)   aligner->stop();
        if (slime_pub) slime_pub->stop();
        if (vmt_pub)   vmt_pub->stop();
    }
};

// Each output starts on construction; a member stays nullptr when its flag is
// off or its socket/handshake failed (warn-and-continue, pose pipeline
// unaffected). `hmd_bus` may be null — the aligner additionally requires it.
RunOutputs make_run_outputs(const config::MainOptions& opts,
                            pipeline::Skeleton3DBus* bus3d,
                            slimevr::SlimeTrackerBus* tracker_bus,
                            vmt::HmdPoseBus* hmd_bus);

}  // namespace fitra::app
