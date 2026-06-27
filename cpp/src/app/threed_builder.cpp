#include "app/threed_builder.hpp"

#include "app/paths.hpp"
#include "lift/calib_io.hpp"
#include "util/logging.hpp"

namespace fitra::app {

ThreeDSet make_threed(const config::MainOptions& opts,
                      std::size_t n_cams,
                      double subject_height_m) {
    ThreeDSet t;
    t.subject_height_m = subject_height_m;

    lift::Triangulator::Options tri_opts;
    tri_opts.kp_conf_thresh = opts.kp_conf_thresh;
    tri_opts.max_reproj_px  = opts.max_reproj_px;

    // Read resolves to the extrinsic write target (= latest) when three_d.calib
    // is unset (pose-3d-calib-latest-resolution.md).
    const std::string calib_path = config::effective_extrinsics_path(opts);
    FITRA_LOG_INFO("loading calibration: {}", calib_path);
    auto calib = lift::load_calibration(calib_path);
    t.triangulator = std::make_shared<lift::Triangulator>(calib, tri_opts);
    t.triangulator->require_camera_ids(expected_camera_ids(n_cams));
    t.bus3d = std::make_unique<pipeline::Skeleton3DBus>();
    t.tracker_bus = std::make_unique<slimevr::SlimeTrackerBus>();
    FITRA_LOG_INFO("3D lifting enabled ({} calibrated cameras, sync_window={}ms)",
                   t.triangulator->camera_count(), opts.sync_window_ms);
    if (t.subject_height_m > 0.0) {
        FITRA_LOG_INFO("3D IK subject height prior enabled: {} m", t.subject_height_m);
    }

    std::string profile_path = opts.subject_profile;
    if (profile_path.empty() && !opts.subject_id.empty()) {
        profile_path =
            lift::default_subject_profile_path(opts.subjects_dir, opts.subject_id);
    }
    if (!profile_path.empty()) {
        FITRA_LOG_INFO("loading subject profile: {}", profile_path);
        t.subject_profile = lift::load_subject_profile(profile_path);
        if (t.subject_profile.subject_height_m > 0.0) {
            t.subject_height_m = t.subject_profile.subject_height_m;
        } else if (t.subject_height_m > 0.0) {
            t.subject_profile.subject_height_m = t.subject_height_m;
        }
        t.has_subject_profile = true;
        FITRA_LOG_INFO("3D IK subject profile enabled: id={} quality={}",
                       t.subject_profile.subject_id,
                       t.subject_profile.quality_status);
    }
    return t;
}

std::unique_ptr<pipeline::MultiCameraDriver> make_driver(
    std::vector<std::unique_ptr<camera::FrameSource>> sources,
    infer::RtmPose& rtmpose,
    pipeline::SnapshotBus& bus,
    const config::MainOptions& opts,
    const ThreeDSet* threed) {
    if (!threed) {
        return std::make_unique<pipeline::MultiCameraDriver>(
            std::move(sources), rtmpose, bus);
    }
    pipeline::MultiCameraDriver::ThreeDConfig cfg;
    cfg.triangulator        = threed->triangulator;
    cfg.bus                 = threed->bus3d.get();
    cfg.sync_window_ms      = opts.sync_window_ms;
    cfg.kalman_enabled      = opts.kalman_3d;
    cfg.ik_enabled          = opts.ik_3d;
    cfg.bone_calib_frames   = opts.bone_calib_frames;
    cfg.subject_height_m    = threed->subject_height_m;
    cfg.has_subject_profile = threed->has_subject_profile;
    cfg.subject_profile     = threed->subject_profile;
    return std::make_unique<pipeline::MultiCameraDriver>(
        std::move(sources), rtmpose, bus, cfg);
}

}  // namespace fitra::app
