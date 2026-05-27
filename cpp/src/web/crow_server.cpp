#include "web/crow_server.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <mutex>
#include <set>
#include <sstream>
#include <string_view>
#include <thread>
#include <utility>

#define CROW_MAIN
#include <crow.h>

#include "slimevr/native_publisher.hpp"
#include "slimevr/slime_tracker_bus.hpp"
#include "vmt/vmt_publisher.hpp"
#include "vmt/hmd_pose_receiver.hpp"
#include "vmt/auto_alignment.hpp"
#include "util/logging.hpp"

namespace fitra::web {

namespace {

struct WsClients {
    std::mutex                       mu;
    std::set<crow::websocket::connection*> conns;
};

crow::response json_response(std::string body, int code = 200) {
    crow::response resp{code, std::move(body)};
    resp.set_header("Content-Type", "application/json; charset=utf-8");
    return resp;
}

std::string read_file(const std::filesystem::path& p) {
    std::ifstream f(p, std::ios::binary);
    if (!f.is_open()) return {};
    std::ostringstream oss;
    oss << f.rdbuf();
    return oss.str();
}

std::string guess_content_type(const std::filesystem::path& p) {
    auto ext = p.extension().string();
    if (ext == ".html") return "text/html; charset=utf-8";
    if (ext == ".js")   return "application/javascript; charset=utf-8";
    if (ext == ".css")  return "text/css; charset=utf-8";
    if (ext == ".json") return "application/json; charset=utf-8";
    if (ext == ".png")  return "image/png";
    if (ext == ".jpg" || ext == ".jpeg") return "image/jpeg";
    return "application/octet-stream";
}

std::string json_escape(std::string_view s) {
    std::string out;
    out.reserve(s.size() + 2);
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b";  break;
            case '\f': out += "\\f";  break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    constexpr char hex[] = "0123456789abcdef";
                    out += "\\u00";
                    out += hex[(c >> 4) & 0xf];
                    out += hex[c & 0xf];
                } else {
                    out += c;
                }
        }
    }
    return out;
}

void append_vmt_alignment_json(std::ostringstream& out,
                               const vmt::VmtAlignment& a) {
    out << "{\"x\":" << a.x
        << ",\"y\":" << a.y
        << ",\"z\":" << a.z
        << ",\"yaw_deg\":" << a.yaw_deg
        << "}";
}

std::string make_vmt_stats_fragment(const vmt::VmtPublisher& publisher) {
    auto s = publisher.stats();
    const auto& o = publisher.options();
    std::ostringstream out;
    out << "\"vmt\":{\"sent_bundles\":"          << s.sent_bundles
        << ",\"sent_trackers\":"                  << s.sent_trackers
        << ",\"disabled_count\":"                 << s.disabled_count
        << ",\"skipped_invalid_bundles\":"        << s.skipped_invalid_bundles
        << ",\"last_send_ms\":"                   << s.last_send_ms
        << ",\"rate_hz\":"                        << o.send_rate_hz
        << ",\"port\":"                           << o.port
        << ",\"index_base\":"                     << o.index_base
        << ",\"host\":\""                         << json_escape(o.host) << "\""
        << ",\"degeneracy_mode\":\""              << json_escape(vmt::degen_mode_name(o.degeneracy_mode)) << "\""
        << ",\"alignment\":";
    append_vmt_alignment_json(out, publisher.alignment());
    out << "}";
    return out.str();
}

std::string make_hmd_status_fragment(const vmt::HmdPoseSnapshot& snap,
                                      bool enabled) {
    std::ostringstream out;
    out << "\"hmd\":{\"enabled\":" << (enabled ? "true" : "false")
        << ",\"have_any\":" << (snap.have_any ? "true" : "false")
        << ",\"stale\":"    << (snap.stale ? "true" : "false");
    if (snap.have_any) {
        // age_ms can be +inf if have_any == false but the snapshot interface
        // also returns inf in pathological cases; guard for JSON validity.
        double age = snap.age_ms;
        if (!std::isfinite(age)) age = -1.0;
        out << ",\"valid\":" << (snap.pose.valid ? "true" : "false")
            << ",\"age_ms\":" << age
            << ",\"timestamp_s\":" << snap.pose.timestamp_s
            << ",\"pos\":[" << snap.pose.x << "," << snap.pose.y << ","
            << snap.pose.z << "]"
            << ",\"quat_xyzw\":[" << snap.pose.qx << "," << snap.pose.qy
            << "," << snap.pose.qz << "," << snap.pose.qw << "]"
            << ",\"yaw_deg\":" <<
                (180.0f / 3.14159265358979323846f) *
                vmt::yaw_from_vmt_quat(vmt::VmtQuat{
                    snap.pose.qx, snap.pose.qy, snap.pose.qz, snap.pose.qw});
    }
    out << "}";
    return out.str();
}

// Convert a chest tracker (world Z-up RH frame, see SlimeTracker docs) into
// VMT Driver frame (Y-up RH). Mirrors the per-tracker transform the VMT
// publisher applies before sending /VMT/Room/Driver.
void chest_in_vmt(const slimevr::SlimeTracker& chest,
                  vmt::VmtPos&  out_pos,
                  vmt::VmtQuat& out_quat_xyzw) {
    out_pos       = vmt::world_pos_to_vmt(chest.pos[0],   chest.pos[1],   chest.pos[2]);
    out_quat_xyzw = vmt::world_quat_to_vmt(chest.quat_wxyz[0], chest.quat_wxyz[1],
                                           chest.quat_wxyz[2], chest.quat_wxyz[3]);
}

void append_auto_result_json(std::ostringstream& out,
                              const vmt::AutoAlignmentResult& r,
                              const std::string& mode,
                              int samples_seen) {
    out << "{\"status\":\"" << vmt::status_name(r.status) << "\""
        << ",\"mode\":\"" << json_escape(mode) << "\""
        << ",\"n_samples\":" << r.n_samples
        << ",\"samples_seen\":" << samples_seen
        << ",\"residual_m\":" << r.residual_m
        << ",\"err\":\"" << json_escape(r.err) << "\""
        << ",\"alignment\":";
    append_vmt_alignment_json(out, r.alignment);
    out << "}";
}

bool read_required_number(const crow::json::rvalue& body,
                          const char* key,
                          float& out,
                          std::string& err) {
    if (!body.has(key)) {
        err = std::string("missing field ") + key;
        return false;
    }
    const auto& value = body[key];
    if (value.t() != crow::json::type::Number) {
        err = std::string("invalid field ") + key;
        return false;
    }
    const double v = value.d();
    if (!std::isfinite(v)) {
        err = std::string("invalid field ") + key;
        return false;
    }
    const float f = static_cast<float>(v);
    if (!std::isfinite(f)) {
        err = std::string("invalid field ") + key;
        return false;
    }
    out = f;
    return true;
}

bool role_from_string(const std::string& name, slimevr::TrackerRole& out) {
    for (std::size_t i = 0; i < slimevr::kTrackerCount; ++i) {
        auto role = static_cast<slimevr::TrackerRole>(i);
        if (name == slimevr::tracker_role_name(role)) {
            out = role;
            return true;
        }
    }
    return false;
}

std::string slimevr_corrections_json(slimevr::NativePublisher& publisher) {
    auto corrections = publisher.debug_corrections();
    std::ostringstream o;
    o << "{\"ok\":true,\"preview_no_reset\":"
      << (publisher.options().preview_no_reset ? "true" : "false")
      << ",\"roles\":[";
    for (std::size_t i = 0; i < corrections.size(); ++i) {
        if (i) o << ",";
        auto role = static_cast<slimevr::TrackerRole>(i);
        const auto& c = corrections[i];
        o << "{\"role\":\"" << slimevr::tracker_role_name(role)
          << "\",\"yaw_quarters\":" << c.yaw_quarters
          << ",\"pitch_quarters\":" << c.pitch_quarters
          << ",\"roll_quarters\":" << c.roll_quarters
          << "}";
    }
    o << "]}";
    return o.str();
}

}  // namespace

// In-flight motion calibration state. One AutoAlignSession at a
// time (start while collecting → 409). The collector thread polls the HMD
// bus + tracker bus and accumulates xz samples until the requested
// duration_s elapses or stop is requested; on completion it solves with
// auto_alignment::solve_motion and pushes the result into the publisher's
// VmtAlignment (same channel as the manual UI).
struct AutoAlignSession {
    enum class State { Idle, Collecting, Ok, Err };

    std::mutex                 mu;
    State                      state = State::Idle;
    std::atomic<bool>          stop_requested{false};
    std::thread                worker;

    // Last finalised result (motion or tpose). Held under mu.
    vmt::AutoAlignmentResult   last;
    std::string                last_mode;   // "tpose" | "motion"
    double                     duration_s   = 0.0;
    double                     sample_hz    = 0.0;
    int                        samples_seen = 0;

    void join_if_done() {
        if (worker.joinable() && state != State::Collecting) {
            worker.join();
        }
    }

    // RAII safety net: stop + join the worker on teardown even if
    // CrowServer::stop() took an early-return path. A still-joinable
    // std::thread destructor would otherwise call std::terminate().
    ~AutoAlignSession() {
        stop_requested.store(true);
        if (worker.joinable()) {
            worker.join();
        }
    }
};

struct CrowServer::Impl {
    crow::SimpleApp     app;
    WsClients           clients2d;
    WsClients           clients3d;
    AutoAlignSession    auto_align;
};

CrowServer::CrowServer(pipeline::SnapshotBus& bus, ServerOptions opts)
    : CrowServer(bus, nullptr, std::move(opts)) {}

CrowServer::CrowServer(pipeline::SnapshotBus& bus,
                       pipeline::Skeleton3DBus* bus3d,
                       ServerOptions opts)
    : bus_{bus}, bus3d_{bus3d}, opts_{std::move(opts)}, impl_{std::make_unique<Impl>()} {}

CrowServer::~CrowServer() {
    try { stop(); } catch (...) {}
}

void CrowServer::set_calibration_session(pipeline::CalibrationSession* session,
                                          pipeline::CalibPreflight defaults) {
    calib_session_  = session;
    calib_defaults_ = std::move(defaults);
}

void CrowServer::set_native_publisher(slimevr::NativePublisher* publisher) {
    native_publisher_ = publisher;
}

void CrowServer::set_vmt_publisher(vmt::VmtPublisher* publisher) {
    vmt_publisher_ = publisher;
}

void CrowServer::set_hmd_pose_bus(vmt::HmdPoseBus* bus, double stale_threshold_ms) {
    hmd_pose_bus_ = bus;
    if (stale_threshold_ms > 0.0) hmd_stale_ms_ = stale_threshold_ms;
}

void CrowServer::set_tracker_bus(slimevr::SlimeTrackerBus* tracker_bus) {
    tracker_bus_ = tracker_bus;
}

void CrowServer::start() {
    auto& app     = impl_->app;
    auto& clients2d = impl_->clients2d;
    auto& clients3d = impl_->clients3d;

    // WS /ws — register first so the catch-all HTTP route below does not
    // shadow upgrade requests (Crow's BaseRule::handle_upgrade returns
    // 404 without writing it, which the client sees as a closed socket).
    CROW_WEBSOCKET_ROUTE(app, "/ws")
    .onopen([&clients2d](crow::websocket::connection& c) {
        std::lock_guard<std::mutex> lk{clients2d.mu};
        clients2d.conns.insert(&c);
    })
    .onclose([&clients2d](crow::websocket::connection& c,
                       const std::string& /*reason*/,
                       uint16_t /*code*/) {
        std::lock_guard<std::mutex> lk{clients2d.mu};
        clients2d.conns.erase(&c);
    })
    .onmessage([](crow::websocket::connection& /*c*/,
                  const std::string& /*data*/,
                  bool /*is_binary*/) {
        // ignore client messages (ping etc.)
    });

    CROW_WEBSOCKET_ROUTE(app, "/ws3d")
    .onopen([&clients3d](crow::websocket::connection& c) {
        std::lock_guard<std::mutex> lk{clients3d.mu};
        clients3d.conns.insert(&c);
    })
    .onclose([&clients3d](crow::websocket::connection& c,
                       const std::string& /*reason*/,
                       uint16_t /*code*/) {
        std::lock_guard<std::mutex> lk{clients3d.mu};
        clients3d.conns.erase(&c);
    })
    .onmessage([](crow::websocket::connection& /*c*/,
                  const std::string& /*data*/,
                  bool /*is_binary*/) {
        // ignore client messages (ping etc.)
    });

    // GET /stats — current bundle as JSON
    CROW_ROUTE(app, "/stats")
    ([this]() {
        crow::response resp{bus_.make_bundle_json()};
        resp.set_header("Content-Type", "application/json; charset=utf-8");
        return resp;
    });

    CROW_ROUTE(app, "/stats3d")
    ([this]() {
        // When a tracker bus is attached, embed the smoothed SlimeVR
        // tracker snapshot (role/pos/quat/valid/roll_confidence) as a
        // top-level field of the bundle so the WebUI can render axes.
        std::string trackers_fragment;
        if (tracker_bus_) {
            trackers_fragment = slimevr::make_tracker_bundle_fragment(*tracker_bus_);
        }
        std::string body = bus3d_ ? bus3d_->make_bundle_json(trackers_fragment)
                                  : pipeline::make_disabled_3d_json();
        // When the native SlimeVR publisher is wired up, splice its send
        // counters into the bundle JSON. The splice only relies on
        // `body.back() == '}'` (the outer message close), so it stays
        // correct even when `extra_fields_json` injects e.g. `]` before it.
        if (native_publisher_) {
            auto s = native_publisher_->stats();
            std::ostringstream extra;
            extra << ",\"slimevr\":{\"sent_handshakes\":"  << s.sent_handshakes
                  << ",\"sent_sensor_info\":"              << s.sent_sensor_info
                  << ",\"sent_rotations\":"                << s.sent_rotations
                  << ",\"sent_heartbeats\":"               << s.sent_heartbeats
                  << ",\"skipped_invalid\":"               << s.skipped_invalid
                  << ",\"ping_count\":"                    << s.ping_count
                  << ",\"last_send_ms\":"                  << s.last_send_ms
                  << "}}";
            if (!body.empty() && body.back() == '}') {
                body.pop_back();
                body += extra.str();
            }
        }
        // Splice VMT publisher stats. Stacks on top of the slimevr splice
        // (body now ends in `}` again after that), or applies fresh if
        // slimevr is not attached.
        if (vmt_publisher_) {
            std::ostringstream extra;
            extra << "," << make_vmt_stats_fragment(*vmt_publisher_) << "}";
            if (!body.empty() && body.back() == '}') {
                body.pop_back();
                body += extra.str();
            }
        }
        // HMD status block. Always present iff a bus is attached;
        // when have_any=false the consumer (WebUI) shows "no hmd".
        if (hmd_pose_bus_) {
            auto snap = hmd_pose_bus_->snapshot(hmd_stale_ms_);
            std::ostringstream extra;
            extra << "," << make_hmd_status_fragment(snap, true) << "}";
            if (!body.empty() && body.back() == '}') {
                body.pop_back();
                body += extra.str();
            }
        }
        crow::response resp{std::move(body)};
        resp.set_header("Content-Type", "application/json; charset=utf-8");
        return resp;
    });

    CROW_ROUTE(app, "/api/vmt/alignment")
    ([this]() {
        std::ostringstream out;
        out << "{\"enabled\":" << (vmt_publisher_ ? "true" : "false")
            << ",\"alignment\":";
        append_vmt_alignment_json(out,
            vmt_publisher_ ? vmt_publisher_->alignment() : vmt::VmtAlignment{});
        out << "}";
        return json_response(out.str());
    });

    CROW_ROUTE(app, "/api/vmt/alignment").methods(crow::HTTPMethod::POST)
    ([this](const crow::request& req) {
        if (!vmt_publisher_) {
            return json_response("{\"ok\":false,\"err\":\"vmt publisher disabled\"}", 409);
        }
        auto body = crow::json::load(req.body);
        if (!body) {
            return json_response("{\"ok\":false,\"err\":\"invalid json\"}", 400);
        }
        vmt::VmtAlignment a;
        std::string err;
        if (!read_required_number(body, "x", a.x, err)
            || !read_required_number(body, "y", a.y, err)
            || !read_required_number(body, "z", a.z, err)
            || !read_required_number(body, "yaw_deg", a.yaw_deg, err)) {
            std::ostringstream out;
            out << "{\"ok\":false,\"err\":\"" << json_escape(err) << "\"}";
            return json_response(out.str(), 400);
        }
        vmt_publisher_->set_alignment(a);
        std::ostringstream out;
        out << "{\"ok\":true,\"enabled\":true,\"alignment\":";
        append_vmt_alignment_json(out, a);
        out << "}";
        return json_response(out.str());
    });

    // ----------------------------------------------------------------------
    // HMD-driven auto alignment.
    //
    // tpose:        single-shot from a paired HMD/chest snapshot.
    // motion/start: background thread accumulates HMD/chest xz pairs for
    //               `duration_s` at `sample_hz`, then solves on completion.
    //               One session at a time.
    // motion/stop:  request the worker to stop early and return.
    // status:       last-finalised result + state.
    // ----------------------------------------------------------------------
    CROW_ROUTE(app, "/api/vmt/alignment/auto/tpose").methods(crow::HTTPMethod::POST)
    ([this](const crow::request& /*req*/) {
        if (!vmt_publisher_) {
            return json_response("{\"ok\":false,\"err\":\"vmt publisher disabled\"}", 409);
        }
        if (!hmd_pose_bus_) {
            return json_response("{\"ok\":false,\"err\":\"hmd pose bus not attached\"}", 409);
        }
        if (!tracker_bus_) {
            return json_response("{\"ok\":false,\"err\":\"tracker bus not attached\"}", 409);
        }

        // Block tpose while a motion session is collecting, and reap a
        // finished motion worker. Move the thread out under the lock and
        // join it outside, so concurrent requests never join() the same
        // std::thread object (UB) nor join() while holding sess.mu (the
        // worker grabs sess.mu on exit → deadlock).
        auto& sess = impl_->auto_align;
        std::thread old_worker;
        {
            std::lock_guard<std::mutex> g(sess.mu);
            if (sess.state == AutoAlignSession::State::Collecting) {
                return json_response(
                    "{\"ok\":false,\"err\":\"motion session in progress\"}", 409);
            }
            if (sess.worker.joinable()) old_worker = std::move(sess.worker);
        }
        if (old_worker.joinable()) old_worker.join();

        auto hmd_snap = hmd_pose_bus_->snapshot(hmd_stale_ms_);
        if (!hmd_snap.have_any) {
            return json_response(
                "{\"ok\":false,\"err\":\"no hmd packets yet\",\"status\":\"no_hmd\"}", 409);
        }
        if (hmd_snap.stale) {
            return json_response(
                "{\"ok\":false,\"err\":\"hmd packet stale\",\"status\":\"stale_hmd\"}", 409);
        }

        auto trk = tracker_bus_->snapshot();
        if (!trk.has_data) {
            return json_response(
                "{\"ok\":false,\"err\":\"no tracker data yet\"}", 409);
        }
        const auto& chest = trk.trackers[static_cast<std::size_t>(
            slimevr::TrackerRole::Chest)];
        if (!chest.valid) {
            return json_response(
                "{\"ok\":false,\"err\":\"chest tracker invalid\"}", 409);
        }

        vmt::VmtPos  cpos;
        vmt::VmtQuat cquat;
        chest_in_vmt(chest, cpos, cquat);
        auto r = vmt::solve_tpose(hmd_snap.pose, cpos, cquat);

        {
            std::lock_guard<std::mutex> g(sess.mu);
            sess.last           = r;
            sess.last_mode      = "tpose";
            sess.samples_seen   = 1;
            sess.state = (r.status == vmt::AutoAlignmentStatus::Ok)
                ? AutoAlignSession::State::Ok
                : AutoAlignSession::State::Err;
        }
        if (r.status == vmt::AutoAlignmentStatus::Ok) {
            vmt_publisher_->set_alignment(r.alignment);
        }

        std::ostringstream out;
        out << "{\"ok\":" << (r.status == vmt::AutoAlignmentStatus::Ok ? "true" : "false")
            << ",\"result\":";
        append_auto_result_json(out, r, "tpose", 1);
        out << "}";
        return json_response(out.str());
    });

    CROW_ROUTE(app, "/api/vmt/alignment/auto/motion/start").methods(crow::HTTPMethod::POST)
    ([this](const crow::request& req) {
        if (!vmt_publisher_) {
            return json_response("{\"ok\":false,\"err\":\"vmt publisher disabled\"}", 409);
        }
        if (!hmd_pose_bus_) {
            return json_response("{\"ok\":false,\"err\":\"hmd pose bus not attached\"}", 409);
        }
        if (!tracker_bus_) {
            return json_response("{\"ok\":false,\"err\":\"tracker bus not attached\"}", 409);
        }

        double duration_s = 3.0;
        double sample_hz  = 30.0;
        if (!req.body.empty()) {
            // .d() throws if the field is present but non-numeric; map that to
            // 400 rather than letting it surface as a 500.
            try {
                auto body = crow::json::load(req.body);
                if (!body) {
                    return json_response("{\"ok\":false,\"err\":\"invalid json\"}", 400);
                }
                if (body.has("duration_s")) {
                    double v = body["duration_s"].d();
                    if (v < 0.5 || v > 30.0) {
                        return json_response(
                            "{\"ok\":false,\"err\":\"duration_s must be in [0.5, 30]\"}", 400);
                    }
                    duration_s = v;
                }
                if (body.has("sample_hz")) {
                    double v = body["sample_hz"].d();
                    if (v < 5.0 || v > 120.0) {
                        return json_response(
                            "{\"ok\":false,\"err\":\"sample_hz must be in [5, 120]\"}", 400);
                    }
                    sample_hz = v;
                }
            } catch (const std::exception&) {
                return json_response(
                    "{\"ok\":false,\"err\":\"duration_s/sample_hz must be numbers\"}", 400);
            }
        }

        // Claim the session and move out any finished worker under one lock,
        // then join outside the lock. Setting Collecting here makes concurrent
        // start/tpose requests bail before they can touch sess.worker.
        auto& sess = impl_->auto_align;
        std::thread old_worker;
        {
            std::lock_guard<std::mutex> g(sess.mu);
            if (sess.state == AutoAlignSession::State::Collecting) {
                return json_response(
                    "{\"ok\":false,\"err\":\"motion session already in progress\"}", 409);
            }
            sess.state        = AutoAlignSession::State::Collecting;
            sess.last_mode    = "motion";
            sess.duration_s   = duration_s;
            sess.sample_hz    = sample_hz;
            sess.samples_seen = 0;
            sess.last         = vmt::AutoAlignmentResult{};
            if (sess.worker.joinable()) old_worker = std::move(sess.worker);
        }
        if (old_worker.joinable()) old_worker.join();
        sess.stop_requested.store(false);

        vmt::HmdPoseBus*           hmd     = hmd_pose_bus_;
        const double               stale   = hmd_stale_ms_;
        slimevr::SlimeTrackerBus*  tracker = tracker_bus_;
        vmt::VmtPublisher*         pub     = vmt_publisher_;

        std::thread new_worker([&sess, hmd, stale, tracker, pub,
                                   duration_s, sample_hz]() {
            using namespace std::chrono;
            const auto period = duration<double>(1.0 / sample_hz);
            const auto t0     = steady_clock::now();
            std::vector<vmt::MotionSample> samples;
            samples.reserve(static_cast<std::size_t>(duration_s * sample_hz + 8));
            auto next_tick = t0;
            while (!sess.stop_requested.load()) {
                if (steady_clock::now() - t0 >= duration<double>(duration_s)) break;
                auto h = hmd->snapshot(stale);
                if (h.have_any && !h.stale && h.pose.valid && tracker) {
                    auto trk = tracker->snapshot();
                    const auto& chest = trk.trackers[static_cast<std::size_t>(
                        slimevr::TrackerRole::Chest)];
                    if (trk.has_data && chest.valid) {
                        vmt::VmtPos  cpos;
                        vmt::VmtQuat cquat;
                        chest_in_vmt(chest, cpos, cquat);
                        samples.push_back({h.pose.x, h.pose.z, cpos.x, cpos.z});
                    }
                }
                {
                    std::lock_guard<std::mutex> g(sess.mu);
                    sess.samples_seen = static_cast<int>(samples.size());
                }
                next_tick += duration_cast<steady_clock::duration>(period);
                std::this_thread::sleep_until(next_tick);
            }

            auto result = vmt::solve_motion(samples);
            {
                std::lock_guard<std::mutex> g(sess.mu);
                sess.last      = result;
                sess.last_mode = "motion";
                sess.state = (result.status == vmt::AutoAlignmentStatus::Ok)
                    ? AutoAlignSession::State::Ok
                    : AutoAlignSession::State::Err;
            }
            if (result.status == vmt::AutoAlignmentStatus::Ok && pub) {
                pub->set_alignment(result.alignment);
            }
        });

        // Publish the running thread under the lock; /stop and CrowServer::stop
        // also touch sess.worker only while holding sess.mu.
        {
            std::lock_guard<std::mutex> g(sess.mu);
            sess.worker = std::move(new_worker);
        }

        std::ostringstream out;
        out << "{\"ok\":true,\"state\":\"collecting\""
            << ",\"duration_s\":" << duration_s
            << ",\"sample_hz\":" << sample_hz << "}";
        return json_response(out.str());
    });

    CROW_ROUTE(app, "/api/vmt/alignment/auto/motion/stop").methods(crow::HTTPMethod::POST)
    ([this](const crow::request& /*req*/) {
        auto& sess = impl_->auto_align;
        sess.stop_requested.store(true);
        std::thread old_worker;
        {
            std::lock_guard<std::mutex> g(sess.mu);
            if (sess.worker.joinable()) old_worker = std::move(sess.worker);
        }
        if (old_worker.joinable()) old_worker.join();

        std::lock_guard<std::mutex> g(sess.mu);
        std::ostringstream out;
        out << "{\"ok\":true,\"state\":\""
            << (sess.state == AutoAlignSession::State::Ok ? "ok" :
                sess.state == AutoAlignSession::State::Err ? "err" : "idle")
            << "\",\"result\":";
        append_auto_result_json(out, sess.last, sess.last_mode, sess.samples_seen);
        out << "}";
        return json_response(out.str());
    });

    CROW_ROUTE(app, "/api/vmt/alignment/auto/status")
    ([this]() {
        auto& sess = impl_->auto_align;
        std::lock_guard<std::mutex> g(sess.mu);
        const char* state_str =
            sess.state == AutoAlignSession::State::Collecting ? "collecting" :
            sess.state == AutoAlignSession::State::Ok         ? "ok"         :
            sess.state == AutoAlignSession::State::Err        ? "err"        : "idle";
        std::ostringstream out;
        out << "{\"state\":\"" << state_str << "\""
            << ",\"samples_seen\":" << sess.samples_seen
            << ",\"duration_s\":" << sess.duration_s
            << ",\"sample_hz\":" << sess.sample_hz
            << ",\"last\":";
        append_auto_result_json(out, sess.last, sess.last_mode, sess.samples_seen);
        out << "}";
        return json_response(out.str());
    });

    CROW_ROUTE(app, "/api/slimevr/corrections")
    ([this]() {
        if (!native_publisher_) {
            return json_response(
                "{\"ok\":false,\"err\":\"slimevr publisher not attached\",\"roles\":[]}",
                503);
        }
        return json_response(slimevr_corrections_json(*native_publisher_));
    });

    CROW_ROUTE(app, "/api/slimevr/corrections").methods(crow::HTTPMethod::POST)
    ([this](const crow::request& req) {
        if (!native_publisher_) {
            return json_response(
                "{\"ok\":false,\"err\":\"slimevr publisher not attached\",\"roles\":[]}",
                503);
        }
        auto body = crow::json::load(req.body);
        if (!body) {
            return json_response("{\"ok\":false,\"err\":\"invalid json\"}", 400);
        }
        bool reset = body.has("reset") && body["reset"].b();
        if (reset && !body.has("role")) {
            native_publisher_->reset_debug_corrections();
            return json_response(slimevr_corrections_json(*native_publisher_));
        }
        if (!body.has("role")) {
            return json_response("{\"ok\":false,\"err\":\"missing role\"}", 400);
        }
        slimevr::TrackerRole role;
        std::string role_name = body["role"].s();
        if (!role_from_string(role_name, role)) {
            return json_response("{\"ok\":false,\"err\":\"unknown role\"}", 400);
        }
        slimevr::NativePublisherDebugCorrection correction;
        if (!reset) {
            auto current = native_publisher_->debug_corrections();
            correction = current[static_cast<std::size_t>(role)];
            if (body.has("yaw_quarters")) {
                correction.yaw_quarters = static_cast<int>(body["yaw_quarters"].i());
            }
            if (body.has("pitch_quarters")) {
                correction.pitch_quarters = static_cast<int>(body["pitch_quarters"].i());
            }
            if (body.has("roll_quarters")) {
                correction.roll_quarters = static_cast<int>(body["roll_quarters"].i());
            }
        }
        native_publisher_->set_debug_correction(role, correction);
        return json_response(slimevr_corrections_json(*native_publisher_));
    });

    // Subject calibration routes. Registered before the catch-all so /calib,
    // /api/calib/* and /artifacts/<path> are not shadowed by the static
    // handler below.
    register_calibration_routes_();

    // Static files under opts_.static_dir
    std::filesystem::path static_root{opts_.static_dir};
    CROW_ROUTE(app, "/")
    ([static_root]() {
        auto body = read_file(static_root / "index.html");
        if (body.empty()) {
            return crow::response{404, "index.html not found"};
        }
        crow::response resp{body};
        resp.set_header("Content-Type", "text/html; charset=utf-8");
        return resp;
    });

    CROW_ROUTE(app, "/<path>")
    ([static_root](const std::string& sub) {
        // Reject anything that escapes the static root.
        std::filesystem::path req = static_root / sub;
        auto canon_req  = std::filesystem::weakly_canonical(req);
        auto canon_root = std::filesystem::weakly_canonical(static_root);
        auto root_str = canon_root.string();
        if (canon_req.string().rfind(root_str, 0) != 0) {
            return crow::response{403, "forbidden"};
        }
        if (!std::filesystem::is_regular_file(canon_req)) {
            return crow::response{404, "not found"};
        }
        auto body = read_file(canon_req);
        crow::response resp{body};
        resp.set_header("Content-Type", guess_content_type(canon_req));
        return resp;
    });

    server_thread_ = std::thread{[this]() {
        impl_->app
            // Crow installs SIGINT/SIGTERM by default and stops itself when
            // they fire. We want our own main() handler to drive the whole
            // shutdown (driver -> server), so clear Crow's handlers.
            .signal_clear()
            .loglevel(crow::LogLevel::Warning)
            .concurrency(static_cast<std::uint16_t>(opts_.crow_threads))
            .port(static_cast<std::uint16_t>(opts_.port))
            .bindaddr(opts_.host)
            .run();
    }};

    stop_.store(false);
    publisher_thread_ = std::thread{&CrowServer::publisher_loop, this};

    FITRA_LOG_INFO("crow listening on http://{}:{}/", opts_.host, opts_.port);
}

void CrowServer::stop() {
    if (!server_thread_.joinable() && !publisher_thread_.joinable()) return;
    stop_.store(true);
    if (impl_) {
        // Tear down the auto-alignment session before app.stop()
        // so the worker thread (if collecting) exits cleanly. Move the
        // thread out under the lock and join outside (the worker grabs
        // sess.mu on exit, so joining under the lock would deadlock).
        auto& sess = impl_->auto_align;
        sess.stop_requested.store(true);
        std::thread old_worker;
        {
            std::lock_guard<std::mutex> g(sess.mu);
            if (sess.worker.joinable()) old_worker = std::move(sess.worker);
        }
        if (old_worker.joinable()) old_worker.join();
        impl_->app.stop();
    }
    if (publisher_thread_.joinable()) publisher_thread_.join();
    if (server_thread_.joinable())    server_thread_.join();
}

void CrowServer::register_calibration_routes_() {
    if (!calib_session_) return;
    auto& app = impl_->app;
    auto* session = calib_session_;
    auto defaults = calib_defaults_;

    std::filesystem::path calib_root{opts_.calib_static_dir};
    CROW_ROUTE(app, "/subject-calib")
    ([calib_root]() {
        auto body = read_file(calib_root / "index.html");
        if (body.empty()) return crow::response{404, "calibration UI not installed"};
        crow::response r{body};
        r.set_header("Content-Type", "text/html; charset=utf-8");
        return r;
    });
    CROW_ROUTE(app, "/subject-calib/<path>")
    ([calib_root](const std::string& sub) {
        std::filesystem::path req = calib_root / sub;
        auto canon_req  = std::filesystem::weakly_canonical(req);
        auto canon_root = std::filesystem::weakly_canonical(calib_root);
        if (canon_req.string().rfind(canon_root.string(), 0) != 0) {
            return crow::response{403, "forbidden"};
        }
        if (!std::filesystem::is_regular_file(canon_req)) {
            return crow::response{404, "not found"};
        }
        crow::response r{read_file(canon_req)};
        r.set_header("Content-Type", guess_content_type(canon_req));
        return r;
    });

    CROW_ROUTE(app, "/api/calib/state")
    ([session]() {
        crow::response r{session->state_json()};
        r.set_header("Content-Type", "application/json; charset=utf-8");
        return r;
    });

    CROW_ROUTE(app, "/api/calib/preflight").methods(crow::HTTPMethod::POST)
    ([session, defaults](const crow::request& req) {
        auto body = crow::json::load(req.body);
        if (!body) return crow::response{400, "{\"ok\":false,\"err\":\"invalid json\"}"};
        pipeline::CalibPreflight in = defaults;
        if (body.has("subject_id"))         in.subject_id       = body["subject_id"].s();
        if (body.has("subject_height_m"))   in.subject_height_m = body["subject_height_m"].d();
        if (body.has("required_hold_sec"))  in.required_hold_sec = body["required_hold_sec"].d();
        if (body.has("recording_frames_per_cam"))
            in.recording_frames_per_cam = static_cast<int>(body["recording_frames_per_cam"].i());
        std::string err;
        bool ok = session->preflight(in, err);
        std::ostringstream o;
        o << "{\"ok\":" << (ok ? "true" : "false")
          << ",\"err\":\"" << err << "\"}";
        crow::response r{o.str()};
        r.set_header("Content-Type", "application/json; charset=utf-8");
        return r;
    });

    CROW_ROUTE(app, "/api/calib/start").methods(crow::HTTPMethod::POST)
    ([session](const crow::request& /*req*/) {
        std::string err;
        bool ok = session->start(err);
        std::ostringstream o;
        o << "{\"ok\":" << (ok ? "true" : "false")
          << ",\"err\":\"" << err << "\"}";
        crow::response r{o.str()};
        r.set_header("Content-Type", "application/json; charset=utf-8");
        return r;
    });

    CROW_ROUTE(app, "/api/calib/retake").methods(crow::HTTPMethod::POST)
    ([session](const crow::request& req) {
        auto body = crow::json::load(req.body);
        std::string pose = body && body.has("pose") ? body["pose"].s() : std::string{};
        std::string err;
        bool ok = session->retake(pose, err);
        std::ostringstream o;
        o << "{\"ok\":" << (ok ? "true" : "false")
          << ",\"err\":\"" << err << "\"}";
        crow::response r{o.str()};
        r.set_header("Content-Type", "application/json; charset=utf-8");
        return r;
    });

    CROW_ROUTE(app, "/api/calib/cancel").methods(crow::HTTPMethod::POST)
    ([session](const crow::request& /*req*/) {
        std::string err;
        bool ok = session->cancel(err);
        std::ostringstream o;
        o << "{\"ok\":" << (ok ? "true" : "false")
          << ",\"err\":\"" << err << "\"}";
        crow::response r{o.str()};
        r.set_header("Content-Type", "application/json; charset=utf-8");
        return r;
    });

    CROW_ROUTE(app, "/api/calib/approve").methods(crow::HTTPMethod::POST)
    ([session](const crow::request& req) {
        auto body = crow::json::load(req.body);
        bool force = body && body.has("force") && body["force"].b();
        std::string err;
        bool ok = session->approve(force, err);
        std::ostringstream o;
        o << "{\"ok\":" << (ok ? "true" : "false")
          << ",\"err\":\"" << err << "\"}";
        crow::response r{o.str()};
        r.set_header("Content-Type", "application/json; charset=utf-8");
        return r;
    });
}

void CrowServer::publisher_loop() {
    using clock = std::chrono::steady_clock;
    auto period = std::chrono::duration<double>(1.0 / std::max(opts_.publish_hz, 1.0));
    auto next = clock::now();
    while (!stop_.load()) {
        next += std::chrono::duration_cast<clock::duration>(period);
        std::this_thread::sleep_until(next);
        if (stop_.load()) break;

        auto msg = bus_.make_bundle_json();
        // Include trackers fragment in the WS broadcast so the 3D viewer
        // can keep AxesHelpers per tracker in sync at publish_hz.
        std::string trackers_fragment;
        if (tracker_bus_) {
            trackers_fragment = slimevr::make_tracker_bundle_fragment(*tracker_bus_);
        }
        std::string extra3d = trackers_fragment;
        if (vmt_publisher_) {
            if (!extra3d.empty()) extra3d += ",";
            extra3d += make_vmt_stats_fragment(*vmt_publisher_);
        }
        auto msg3d = bus3d_ ? bus3d_->make_bundle_json(extra3d)
                            : pipeline::make_disabled_3d_json();
        {
            std::lock_guard<std::mutex> lk{impl_->clients2d.mu};
            for (auto* c : impl_->clients2d.conns) {
                try {
                    c->send_text(msg);
                } catch (...) {
                    // best-effort; client will be reaped on close
                }
            }
        }
        {
            std::lock_guard<std::mutex> lk{impl_->clients3d.mu};
            for (auto* c : impl_->clients3d.conns) {
                try {
                    c->send_text(msg3d);
                } catch (...) {
                    // best-effort; client will be reaped on close
                }
            }
        }
    }
}

}  // namespace fitra::web
