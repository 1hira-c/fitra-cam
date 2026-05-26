#include "web/crow_server.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <mutex>
#include <set>
#include <sstream>

#define CROW_MAIN
#include <crow.h>

#include "slimevr/native_publisher.hpp"
#include "slimevr/slime_tracker_bus.hpp"
#include "vmt/vmt_publisher.hpp"
#include "util/logging.hpp"

namespace fitra::web {

namespace {

struct WsClients {
    std::mutex                       mu;
    std::set<crow::websocket::connection*> conns;
};

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
        << ",\"host\":\""                         << o.host << "\""
        << ",\"degeneracy_mode\":\""              << vmt::degen_mode_name(o.degeneracy_mode) << "\""
        << ",\"alignment\":";
    append_vmt_alignment_json(out, publisher.alignment());
    out << "}";
    return out.str();
}

crow::response json_response(int code, const std::string& body) {
    crow::response resp{code, body};
    resp.set_header("Content-Type", "application/json; charset=utf-8");
    return resp;
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

}  // namespace

struct CrowServer::Impl {
    crow::SimpleApp app;
    WsClients       clients2d;
    WsClients       clients3d;
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
        // Phase 13 M1: when a tracker bus is attached, embed the smoothed
        // SlimeVR tracker snapshot (role/pos/quat/valid/roll_confidence) as
        // a top-level field of the bundle so the WebUI can render axes.
        std::string trackers_fragment;
        if (tracker_bus_) {
            trackers_fragment = slimevr::make_tracker_bundle_fragment(*tracker_bus_);
        }
        std::string body = bus3d_ ? bus3d_->make_bundle_json(trackers_fragment)
                                  : pipeline::make_disabled_3d_json();
        // Phase 11: when the native SlimeVR publisher is wired up, splice
        // its send counters into the bundle JSON. The bundle always ends
        // in a single `}` (the outer message close); Phase 13 weakened
        // the previous "ends in `}}`" invariant since `extra_fields_json`
        // can inject e.g. `]` (trackers array close) right before it. The
        // splice only relies on `body.back() == '}'`, so it stays correct.
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
        // Phase 14: splice VMT publisher stats. Stacks on top of the slimevr
        // splice (body now ends in `}` again after that), or applies fresh if
        // slimevr is not attached.
        if (vmt_publisher_) {
            std::ostringstream extra;
            extra << "," << make_vmt_stats_fragment(*vmt_publisher_) << "}";
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
        return json_response(200, out.str());
    });

    CROW_ROUTE(app, "/api/vmt/alignment").methods(crow::HTTPMethod::POST)
    ([this](const crow::request& req) {
        if (!vmt_publisher_) {
            return json_response(409, "{\"ok\":false,\"err\":\"vmt publisher disabled\"}");
        }
        auto body = crow::json::load(req.body);
        if (!body) {
            return json_response(400, "{\"ok\":false,\"err\":\"invalid json\"}");
        }
        vmt::VmtAlignment a;
        std::string err;
        if (!read_required_number(body, "x", a.x, err)
            || !read_required_number(body, "y", a.y, err)
            || !read_required_number(body, "z", a.z, err)
            || !read_required_number(body, "yaw_deg", a.yaw_deg, err)) {
            std::ostringstream out;
            out << "{\"ok\":false,\"err\":\"" << err << "\"}";
            return json_response(400, out.str());
        }
        vmt_publisher_->set_alignment(a);
        std::ostringstream out;
        out << "{\"ok\":true,\"enabled\":true,\"alignment\":";
        append_vmt_alignment_json(out, a);
        out << "}";
        return json_response(200, out.str());
    });

    // Phase 8 calibration routes. Registered before the catch-all so /calib,
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
    if (impl_) impl_->app.stop();
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
        // Phase 13 M1: include trackers fragment in the WS broadcast so the
        // 3D viewer can keep AxesHelpers per tracker in sync at publish_hz.
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
