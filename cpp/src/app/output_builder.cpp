#include "app/output_builder.hpp"

#include <cstdint>

#include "util/logging.hpp"

namespace fitra::app {

std::unique_ptr<tracking::TrackerExtractor> make_tracker_extractor(
    const config::MainOptions& opts,
    pipeline::Skeleton3DBus& bus3d,
    tracking::TrackerBus& tracker_bus,
    const std::atomic<bool>* idle_flag,
    tracking::TrackerAxisBus* tracker_axis_bus,
    pipeline::TrackerAxisLineageBus* lineage_bus) {
    tracking::TrackerExtractorOptions tex_opts;
    tex_opts.extract_rate_hz = opts.vmt_rate_hz;
    tex_opts.quat_smooth     = static_cast<float>(opts.vr_quat_smooth);
    // Fixed-EMA fallback alphas feed the WebUI viz as well as VMT, so they run
    // regardless of --vmt-out. One Euro ignores both while enabled.
    tex_opts.pos_smooth      = static_cast<float>(opts.vmt_pos_smooth);
    tex_opts.event_driven    = opts.vr_extract_event_driven;
    // One Euro (speed-adaptive) smoothing — default path. When on,
    // quat_smooth/pos_smooth above are ignored. Feeds both outputs +
    // WebUI (single producer).
    tex_opts.one_euro        = opts.vr_one_euro;
    tex_opts.pos_one_euro    = {static_cast<float>(opts.vr_pos_mincutoff),
                                static_cast<float>(opts.vr_pos_beta),
                                static_cast<float>(opts.vr_pos_dcutoff)};
    tex_opts.quat_one_euro   = {static_cast<float>(opts.vr_quat_mincutoff),
                                static_cast<float>(opts.vr_quat_beta),
                                static_cast<float>(opts.vr_quat_dcutoff)};
    // Foot tracker position: "ankle" (default) | "midpoint". validate_options
    // already restricted the string to these two values.
    tex_opts.foot_pos_mode   = (opts.vr_foot_pos_mode == "midpoint")
                                   ? tracking::FootPosMode::Midpoint
                                   : tracking::FootPosMode::Ankle;
    // Chest / Waist tracker height up the spine ([0,1], validated). Position
    // only — see extract_trackers().
    tex_opts.chest_height_frac = static_cast<float>(opts.vr_chest_height_frac);
    tex_opts.waist_height_frac = static_cast<float>(opts.vr_waist_height_frac);
    tex_opts.limb_extension.snap = opts.limb_extension_snap_3d;
    tex_opts.limb_extension.toe_direction = opts.extended_leg_toe_direction_3d;
    tex_opts.limb_extension.enter_flex_deg =
        static_cast<float>(opts.extension_snap_enter_deg);
    tex_opts.limb_extension.exit_flex_deg =
        static_cast<float>(opts.extension_snap_exit_deg);
    auto extractor = std::make_unique<tracking::TrackerExtractor>(
        bus3d, tracker_bus, tex_opts, tracker_axis_bus, lineage_bus);
    extractor->set_idle_gate(idle_flag);  // before start(); null = no idling
    extractor->start();
    return extractor;
}

RunOutputs make_run_outputs(const config::MainOptions& opts,
                            pipeline::Skeleton3DBus* bus3d,
                            tracking::TrackerBus* tracker_bus,
                            vmt::HmdPoseBus* hmd_bus,
                            const vmt::DiscoveryEndpointBus* disc_bus) {
    RunOutputs out;
    if (!bus3d || !tracker_bus) return out;  // 2D-only run: no outputs

    if (opts.vmt_out) {
        vmt::VmtPublisherOptions vopts;
        vopts.host         = opts.vmt_host;
        vopts.port         = static_cast<std::uint16_t>(opts.vmt_port);
        vopts.send_rate_hz = opts.vmt_rate_hz;
        vopts.index_base   = opts.vmt_index_base;
        // Tracker preset (which roles to publish). validate_options already
        // restricted the string to p3|p6|p8|full; default P8 defends in depth.
        vmt::VmtTrackerPreset preset = vmt::VmtTrackerPreset::P8;
        vmt::parse_vmt_preset(opts.vmt_tracker_preset, preset);
        vopts.preset       = preset;
        vopts.disable_below_floor = opts.vmt_disable_below_floor;
        if (!vmt::parse_degen_mode(opts.vmt_degeneracy_mode, vopts.degeneracy_mode)) {
            // validate_options should have caught this, but defend in depth.
            vopts.degeneracy_mode = vmt::DegenMode::Hold;
        }
        out.vmt_pub = std::make_unique<vmt::VmtPublisher>(
            *bus3d, *tracker_bus, vopts);
        // Empty host -> resolve the destination via discovery at runtime; a
        // non-empty host pins it manually and the bus is ignored. Must be set
        // before start() (it decides manual vs discovery).
        out.vmt_pub->set_discovery_bus(disc_bus);
        if (!out.vmt_pub->start()) {
            out.vmt_pub.reset();
        }
    }

    // Continuous (always-on) HMD-driven alignment refinement. Read-only
    // consumer of bus3d + the HMD bus; nudges vmt_pub's alignment over time.
    if (out.vmt_pub && hmd_bus && opts.hmd_listen_enabled
        && opts.vmt_continuous_align) {
        vmt::ContinuousAlignerConfig cacfg;
        cacfg.enabled          = true;
        cacfg.sample_hz        = opts.vmt_continuous_sample_hz;
        cacfg.resolve_period_s = opts.vmt_continuous_resolve_s;
        cacfg.blend_alpha      = static_cast<float>(opts.vmt_continuous_blend);
        cacfg.hmd_forward_offset_m = static_cast<float>(opts.vmt_align_hmd_forward_m);
        out.aligner = std::make_unique<vmt::ContinuousAligner>(
            *bus3d, *hmd_bus, *out.vmt_pub, opts.hmd_stale_ms, cacfg);
        out.aligner->start();
        FITRA_LOG_INFO("continuous HMD alignment: enabled "
                       "(sample {} Hz, resolve {} s, blend {})",
                       opts.vmt_continuous_sample_hz,
                       opts.vmt_continuous_resolve_s,
                       opts.vmt_continuous_blend);
    }
    return out;
}

}  // namespace fitra::app
