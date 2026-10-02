// Explicit, temporary live audition. The normal main and its settings are untouched.
#include <atomic>
#include <cerrno>
#include <csignal>
#include <iostream>
#include <poll.h>
#include <unistd.h>
#include "app/camera_builder.hpp"
#include "app/output_builder.hpp"
#include "app/pose_relay_builder.hpp"
#include "app/server_builder.hpp"
#include "app/threed_builder.hpp"
#include "app/trt_stack.hpp"
#include "temporal_live_state.hpp"
#include "util/logging.hpp"

using namespace fitra;
namespace {
std::atomic<bool> stop_requested{false};
void on_signal(int) { stop_requested.store(true); }
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }

// Keep the publisher's transport gate open even when the pose is absent: with
// degen=disable it sends explicit enable=0, never a stale held tracker.
void publish_disabled(pipeline::Skeleton3DBus& skeleton, tracking::TrackerBus& trackers) {
    trackers.publish(temporal_experiment::unavailable_trackers(),{});
    pipeline::Skeleton3DSnapshot disabled;
    disabled.stats.enabled=true;
    disabled.stats.ik_locked=true; // Transport-only; no persons or valid trackers.
    skeleton.update(disabled);
}
}

int main(int argc, char** argv) {
    if (argc==2 && std::string(argv[1])=="--help") {
        std::cout << "temporal_vmt_trial --config TRIAL.yaml\n"
                     "Opt-in live VMT test; never starts the normal TrackerExtractor.\n"
                     "stdin: minimal | spatial | distance2 | distance5 | guard5 | freeze-align | quit\n"
                     "guard5 adds raw-joint outlier admission and at most 50ms causal prediction.\n"
                     "A closed stdin or SIGINT/SIGTERM stops and disables trackers.\n"
                     "Requires fixed Halpe26 subject profile and 60 Hz VMT output.\n";
        return 0;
    }
    try {
        require(argc==3 && std::string(argv[1])=="--config","expected --config TRIAL.yaml");
        config::MainOptions opts;
        config::load_main_config(argv[2],opts);
        require(opts.keypoint_format=="halpe26","trial requires Halpe26");
        lift::set_active_keypoint_format(lift::KeypointFormat::Halpe26);
        config::validate_options(opts);
        require(config::run_mode(opts)==config::RunMode::Run && !opts.daemon && !opts.flow_managed,
                "trial only supports standalone run mode");
        require(opts.enable_3d && opts.vmt_out && opts.hmd_listen_enabled && !opts.multi_person && !opts.bench_fake_bbox,
                "trial requires single-person 3D, VMT and HMD return channel");
        require(opts.keypoint_format=="halpe26" && opts.vmt_rate_hz==60,
                "trial requires Halpe26 and 60 Hz nominal output");
        require(opts.vr_foot_pos_mode=="ankle" && opts.vr_chest_height_frac==.65 &&
                opts.vr_waist_height_frac==.15 && opts.limb_extension_snap_3d &&
                opts.extended_leg_toe_direction_3d && opts.extension_snap_enter_deg==20 &&
                opts.extension_snap_exit_deg==12,
                "trial fixes geometry to the offline candidates' defaults");
        lift::set_active_keypoint_format(lift::KeypointFormat::Halpe26);
        std::signal(SIGINT,on_signal);
        std::signal(SIGTERM,on_signal);
        // Upstream is raw only. These are private copies, never saved to YAML.
        opts.kalman_3d=false; opts.ik_3d=false; opts.floor_contact_stability=false;
        opts.vr_one_euro=false; opts.idle_enabled=false;
        opts.vmt_degeneracy_mode="disable";
        FITRA_LOG_WARN("[TRIAL] raw upstream; fixed replay geometry/floor; no OneEuro; "
                       "degen=disable; no D50 wire; five candidate copies per source report");
        auto trt=app::make_trt_stack(opts);
        auto cams=app::make_frame_sources(opts,trt.get(),nullptr,nullptr);
        require(cams.sources.size()>=2,"trial requires at least two calibrated cameras");
        auto threed=app::make_threed(opts,cams.sources.size(),opts.subject_height_m);
        require(threed.has_subject_profile,"trial requires a valid fixed subject profile");
        pipeline::SnapshotBus camera_bus{cams.sources.size()};
        auto driver=app::make_driver(std::move(cams.sources),*trt->rtmpose,camera_bus,opts,&threed);
        lift::IkSolver::Options io;
        io.subject_height_m=threed.subject_height_m;
        io.subject_profile=threed.subject_profile;
        io.has_subject_profile=true;
        temporal_experiment::LiveTrialState trial{io};
        pipeline::Skeleton3DBus presentation_bus;
        tracking::TrackerBus tracker_bus;
        publish_disabled(presentation_bus,tracker_bus);
        auto relay=app::make_pose_relay(opts,true);
        require(relay.receiver!=nullptr,"cannot start HMD return channel");
        auto outputs=app::make_run_outputs(opts,&presentation_bus,&tracker_bus,relay.hmd_bus.get(),
                           relay.beacon ? &relay.beacon->endpoint_bus() : nullptr);
        require(outputs.vmt_pub!=nullptr,"cannot start VMT publisher");
        struct Stop {
            app::RunOutputs& outputs; app::PoseRelay& relay;
            pipeline::MultiCameraDriver& driver;
            pipeline::Skeleton3DBus& skeleton; tracking::TrackerBus& trackers;
            ~Stop() {
                driver.stop();
                if (outputs.aligner) outputs.aligner->stop();
                publish_disabled(skeleton,trackers);
                std::this_thread::sleep_for(std::chrono::milliseconds(70));
                outputs.stop(); relay.stop();
            }
        } cleanup{outputs,relay,*driver,presentation_bus,tracker_bus};
        app::FlowControl flow{stop_requested,false};
        auto server=app::make_server(opts,config::RunMode::Run,camera_bus,&presentation_bus,&flow);
        if (server) {
            server->set_vmt_publisher(outputs.vmt_pub.get());
            server->set_tracker_bus(&tracker_bus);
            server->set_hmd_pose_bus(relay.hmd_bus.get(),opts.hmd_stale_ms);
            server->set_align_hmd_forward_m(static_cast<float>(opts.vmt_align_hmd_forward_m));
            if (relay.beacon) server->set_discovery_beacon(relay.beacon.get());
            if (outputs.aligner) server->set_continuous_aligner(outputs.aligner.get());
            server->start();
        }
        driver->start();
        FITRA_LOG_WARN("[TRIAL] candidate=minimal generation=0; stdin switches enabled; "
                       "capture-age cutoff=100ms; this is not an E2E latency measurement");
        std::string command;
        std::uint64_t seen=0;
        auto next_log=std::chrono::steady_clock::now();
        while (!stop_requested.load()) {
            pollfd fd{STDIN_FILENO,POLLIN,0};
            const int status=::poll(&fd,1,0);
            if (status<0 && errno!=EINTR) throw std::runtime_error("stdin poll failed");
            if (status>0 && (fd.revents&(POLLIN|POLLHUP|POLLERR|POLLNVAL))) {
                char bytes[128];
                const auto n=::read(STDIN_FILENO,bytes,sizeof(bytes));
                if (n==0) { stop_requested.store(true); break; }
                if (n<0 && errno!=EINTR) throw std::runtime_error("stdin read failed");
                for (ssize_t i=0;i<n;++i) {
                    if (bytes[i]=='\r') continue;
                    if (bytes[i]!='\n') {
                        require(command.size()<64,"trial command too long");
                        command+=bytes[i]; continue;
                    }
                    if (command=="quit") stop_requested.store(true);
                    else if (command=="freeze-align") {
                        if (outputs.aligner) outputs.aligner->set_enabled(false);
                        FITRA_LOG_WARN("[TRIAL] alignment frozen (runtime only)");
                    } else if (trial.select(command)) {
                        publish_disabled(presentation_bus,tracker_bus);
                        FITRA_LOG_WARN("[TRIAL] candidate={} generation={}; history reset; allow 2s settling",
                                       trial.selected(),trial.generation);
                    } else if (!command.empty()) FITRA_LOG_WARN("[TRIAL] rejected command: {}",command);
                    command.clear();
                }
            }
            if (stop_requested.load()) break;
            threed.bus3d->wait_for_update(seen,stop_requested,std::chrono::milliseconds(10));
            auto raw=threed.bus3d->snapshot();
            for (const auto& b:threed.tracker_axis_lineage_bus->drain_boundaries())
                trial.boundary(b.source_publish_mono_ns);
            const auto now=std::chrono::steady_clock::now();
            const bool fresh=trial.advance(raw,now);
            if (fresh || (trial.active() && trial.output_changed)) {
                // This stream's metadata and pose refer to the same consumed raw report.
                // The raw bus and FusionPose evidence are never overwritten.
                auto view=raw;
                view.persons={trial.presentation};
                view.tracker_axis_lineage.reset();
                view.stats.ik_locked=true;
                tracker_bus.publish(trial.trackers,{}); // Window 0 = rolling stats not measured here.
                presentation_bus.update(view);
            } else if (!trial.active()) publish_disabled(presentation_bus,tracker_bus);
            if (now>=next_log) {
                next_log=now+std::chrono::seconds(2);
                const auto v=outputs.vmt_pub->stats();
                const double age=raw.t_capture_oldest.time_since_epoch().count()!=0 ?
                    std::chrono::duration<double,std::milli>(now-raw.t_capture_oldest).count():-1;
                FITRA_LOG_INFO("[TRIAL] candidate={} gen={} active={} processed={} source_seq={} tri_fps={} "
                               "source_age_ms={} sent={} disabled={} host_send_age_ema_ms={}",
                               trial.selected(),trial.generation,trial.active(),trial.processed,raw.seq,
                               raw.stats.tri_fps,age,v.sent_bundles,v.disabled_count,v.e2e_capture_to_send_ms);
                if (std::string(trial.selected())=="guard5" && trial.active()) {
                    int accepted=0,predicted=0,unavailable=0,rejected=0,reacquired=0;
                    for (const auto& d:trial.diagnostics.joint_gate) {
                        accepted += d.state==temporal_experiment::JointAdmission::Accepted;
                        predicted += d.state==temporal_experiment::JointAdmission::Predicted;
                        unavailable += d.state==temporal_experiment::JointAdmission::Unavailable;
                        rejected += d.rejected; reacquired += d.reacquired;
                    }
                    FITRA_LOG_INFO("[TRIAL] current joint admission: accepted={} predicted={} unavailable={} "
                                   "rejected={} reacquired={}; prediction is not fresh observation",
                                   accepted,predicted,unavailable,rejected,reacquired);
                }
            }
        }
        if (server) server->stop();
        FITRA_LOG_INFO("[TRIAL] stopping; disabling trackers; no config or calibration was saved");
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "temporal_vmt_trial: " << e.what() << '\n';
        return 1;
    }
}
