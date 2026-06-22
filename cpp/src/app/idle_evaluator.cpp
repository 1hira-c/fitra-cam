#include "app/idle_evaluator.hpp"

#include <chrono>

#include "app/idle_state.hpp"
#include "util/logging.hpp"
#include "vmt/hmd_pose_receiver.hpp"

namespace fitra::app {

IdleEvaluator::IdleEvaluator(IdleState& state, const vmt::HmdPoseBus* hmd_bus,
                             Config cfg)
    : state_{state}, hmd_bus_{hmd_bus}, cfg_{cfg} {
    // vr_observable is constant for the process lifetime (derived from the
    // launch config), so publish it once up front for the status surface.
    state_.vr_observable.store(
        idle_vr_observable(cfg_.has_vr_output, cfg_.hmd_listen_enabled),
        std::memory_order_relaxed);
}

IdleEvaluator::~IdleEvaluator() {
    try { stop(); } catch (...) {}
}

void IdleEvaluator::start() {
    stop_.store(false);
    thread_ = std::thread{&IdleEvaluator::loop, this};
}

void IdleEvaluator::stop() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
}

void IdleEvaluator::loop() {
    using clock = std::chrono::steady_clock;
    // Fixed ~10Hz evaluation cadence — fine-grained relative to enter_after_s
    // and independent of the (coarser) driver throttle idle_tick_hz.
    const auto period = std::chrono::milliseconds(100);
    const bool vr_observable =
        idle_vr_observable(cfg_.has_vr_output, cfg_.hmd_listen_enabled);

    auto last = clock::now();
    bool prev_idle = false;
    while (!stop_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(period);
        if (stop_.load(std::memory_order_relaxed)) break;

        auto now = clock::now();
        const double dt_s = std::chrono::duration<double>(now - last).count();
        last = now;

        // When idle is disabled we still keep the status fields fresh but never
        // confirm idle.
        bool vr_live = false;
        if (hmd_bus_) {
            auto snap = hmd_bus_->snapshot(cfg_.hmd_stale_ms);
            vr_live = snap.have_any && !snap.stale;
        }
        state_.vr_peer_live.store(vr_live, std::memory_order_relaxed);

        const bool ws_present =
            state_.ws_client_count.load(std::memory_order_relaxed) > 0;
        const bool any_consumer = idle_consumer_present(
            ws_present, vr_live, vr_observable, cfg_.has_vr_output);

        bool idle = false;
        if (cfg_.enabled) {
            idle = decision_.step(any_consumer, dt_s, cfg_.enter_after_s);
        }
        state_.idle.store(idle, std::memory_order_relaxed);

        if (idle != prev_idle) {
            if (idle) {
                FITRA_LOG_INFO(
                    "idle: entering standby (no consumer for {:.1f}s; "
                    "ws={} vr_peer_live={} vr_observable={})",
                    cfg_.enter_after_s, ws_present, vr_live, vr_observable);
            } else {
                FITRA_LOG_INFO(
                    "idle: resuming (ws={} vr_peer_live={})",
                    ws_present, vr_live);
            }
            prev_idle = idle;
        }
    }
}

}  // namespace fitra::app
