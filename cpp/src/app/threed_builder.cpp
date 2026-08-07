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
    // A stage may use FEWER cameras than the calibration covers — subject
    // calibration triangulates from cam0+cam1 only (a 2-view bone-length profile
    // is valid for an N-view run), yet the rig's extrinsics file carries all N
    // cameras. The file may also store cameras in a different order than
    // cam0..cam{n-1}. Normalize to exactly the expected runtime views in order so
    // the Triangulator is built for those views and require_camera_ids matches;
    // extra cameras are dropped (not an error) and a genuinely missing id is
    // caught by require_camera_ids below. Same normalization as dump_keypoints_3d.
    if (calib.cameras.size() > n_cams) {
        FITRA_LOG_INFO("calibration has {} cameras; using the first {} for this stage",
                       calib.cameras.size(), n_cams);
    }
    calib = lift::select_calib_cameras(calib, expected_camera_ids(n_cams));
    t.triangulator = std::make_shared<lift::Triangulator>(calib, tri_opts);
    t.triangulator->require_camera_ids(expected_camera_ids(n_cams));
    t.bus3d = std::make_unique<pipeline::Skeleton3DBus>();
    t.tracker_bus = std::make_unique<tracking::TrackerBus>();
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
        // The subject profile is an OPTIONAL IK accuracy input (bone lengths). A
        // missing / schema-incompatible (wrong keypoint topology, e.g. a v1/COCO17
        // profile under halpe26) / corrupt profile must NOT take run down — run
        // works with the height prior + default bone lengths. Warn and continue;
        // re-run subject calibration to enable per-subject bone lengths. (The
        // daemon's normal chain routes incompatible profiles to CalibSubject via
        // subject_profile_compatible; this guard also covers the crash-fallback
        // and manual-run paths that bypass that routing.)
        try {
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
        } catch (const std::exception& e) {
            FITRA_LOG_WARN("subject profile unavailable ({}): running without it "
                           "(height prior + default bone lengths; re-run subject "
                           "calibration to enable per-subject IK)", e.what());
            t.has_subject_profile = false;
        }
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
    cfg.floor_contact_stability = opts.floor_contact_stability;
    cfg.raw_3d_source      = opts.no_3d_postprocess;
    cfg.floor_contact       = config::floor_contact_options(opts);
    cfg.bone_calib_frames   = opts.bone_calib_frames;
    cfg.subject_height_m    = threed->subject_height_m;
    cfg.has_subject_profile = threed->has_subject_profile;
    cfg.subject_profile     = threed->subject_profile;
    return std::make_unique<pipeline::MultiCameraDriver>(
        std::move(sources), rtmpose, bus, cfg);
}

}  // namespace fitra::app
