#include "web/crow_routes_setup.hpp"

#include <filesystem>
#include <sstream>
#include <string>

#include "pipeline/extrinsic_calib_session.hpp"
#include "vmt/auto_alignment.hpp"   // yaw_from_vmt_quat
#include "vmt/controller_pose_receiver.hpp"
#include "vmt/hmd_pose_receiver.hpp"
#include "vmt/vmt_protocol.hpp"
#include "web/crow_util.hpp"

namespace fitra::web::detail {

namespace {

void append_excal_hmd_pose_json(std::ostringstream& out,
                                const vmt::HmdPoseSnapshot& snap,
                                bool enabled) {
    out << "{\"enabled\":" << (enabled ? "true" : "false")
        << ",\"have_any\":" << (snap.have_any ? "true" : "false")
        << ",\"stale\":" << (snap.stale ? "true" : "false")
        << ",\"valid\":" << (snap.have_any && snap.pose.valid ? "true" : "false")
        << ",\"age_ms\":";
    append_age_ms_json(out, snap.age_ms);
    out << ",\"timestamp_s\":" << (snap.have_any ? snap.pose.timestamp_s : 0.0f);
    if (snap.have_any) {
        out << ",\"pos\":[" << snap.pose.x << "," << snap.pose.y << ","
            << snap.pose.z << "]"
            << ",\"quat_xyzw\":[" << snap.pose.qx << "," << snap.pose.qy
            << "," << snap.pose.qz << "," << snap.pose.qw << "]"
            << ",\"yaw_deg\":" <<
                (180.0f / 3.14159265358979323846f) *
                vmt::yaw_from_vmt_quat(vmt::VmtQuat{
                    snap.pose.qx, snap.pose.qy, snap.pose.qz, snap.pose.qw});
    }
    out << "}";
}

void append_excal_controller_pose_json(std::ostringstream& out,
                                       const vmt::ControllerPoseSnapshot& snap,
                                       bool enabled,
                                       const std::string& role) {
    out << "{\"enabled\":" << (enabled ? "true" : "false")
        << ",\"role\":\"" << json_escape(role) << "\""
        << ",\"have_any\":" << (snap.have_any ? "true" : "false")
        << ",\"stale\":" << (snap.stale ? "true" : "false")
        << ",\"valid\":" << (snap.have_any && snap.pose.valid ? "true" : "false")
        << ",\"running_ok\":"
        << (snap.have_any && snap.pose.running_ok() ? "true" : "false")
        << ",\"tracking_result\":"
        << (snap.have_any ? snap.pose.tracking_result : 0)
        << ",\"age_ms\":";
    append_age_ms_json(out, snap.age_ms);
    out << ",\"timestamp_s\":"
        << (snap.have_any ? snap.pose.timestamp_s : 0.0f);
    if (snap.have_any) {
        out << ",\"pos\":[" << snap.pose.x << "," << snap.pose.y << ","
            << snap.pose.z << "]"
            << ",\"quat_xyzw\":[" << snap.pose.qx << "," << snap.pose.qy
            << "," << snap.pose.qz << "," << snap.pose.qw << "]";
    }
    out << "}";
}

// Subdirectory file handler shared by both static page groups. Rejects
// anything that escapes `root`: prefix match alone allows sibling-directory
// traversal (root "/a/b" would accept "/a/b2/..."), so the match is anchored
// to a directory boundary by appending the separator before comparing.
crow::response serve_static_sub(const std::filesystem::path& root,
                                const std::string& sub) {
    std::filesystem::path req = root / sub;
    auto canon_req  = std::filesystem::weakly_canonical(req);
    auto canon_root = std::filesystem::weakly_canonical(root);
    std::string root_str = canon_root.string();
    if (!root_str.empty() &&
        root_str.back() != std::filesystem::path::preferred_separator) {
        root_str += std::filesystem::path::preferred_separator;
    }
    if (canon_req.string().rfind(root_str, 0) != 0) {
        return crow::response{403, "forbidden"};
    }
    if (!std::filesystem::is_regular_file(canon_req)) {
        return crow::response{404, "not found"};
    }
    crow::response r{read_file(canon_req)};
    r.set_header("Content-Type", guess_content_type(canon_req));
    return r;
}

crow::response serve_static_index(const std::filesystem::path& root,
                                  const char* missing_msg) {
    auto body = read_file(root / "index.html");
    if (body.empty()) return crow::response{404, missing_msg};
    crow::response r{body};
    r.set_header("Content-Type", "text/html; charset=utf-8");
    return r;
}

}  // namespace

void register_calib_routes(crow::SimpleApp& app, const CalibRouteDeps& deps) {
    // No session (run / calib-extrinsic mode): nothing is registered — the
    // wizard page and /api/calib/* fall through to the static catchall.
    if (!deps.session) return;
    auto* session = deps.session;
    auto defaults = deps.defaults;

    std::filesystem::path calib_root{deps.static_dir};
    CROW_ROUTE(app, "/subject-calib")
    ([calib_root]() {
        return serve_static_index(calib_root, "calibration UI not installed");
    });
    CROW_ROUTE(app, "/subject-calib/<path>")
    ([calib_root](const std::string& sub) {
        return serve_static_sub(calib_root, sub);
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

void register_excal_routes(crow::SimpleApp& app, const ExcalRouteDeps& deps) {
    if (!deps.session) return;
    auto* session = deps.session;

    std::filesystem::path excal_root{deps.static_dir};
    CROW_ROUTE(app, "/extrinsic-calib")
    ([excal_root]() {
        return serve_static_index(excal_root, "extrinsic-calib UI not installed");
    });
    CROW_ROUTE(app, "/extrinsic-calib/<path>")
    ([excal_root](const std::string& sub) {
        return serve_static_sub(excal_root, sub);
    });

    CROW_ROUTE(app, "/api/excal/state")
    ([session]() {
        crow::response r{session->state_json()};
        r.set_header("Content-Type", "application/json; charset=utf-8");
        return r;
    });

    CROW_ROUTE(app, "/api/excal/extrinsics")
    ([session]() {
        crow::response r{session->extrinsics_json()};
        r.set_header("Content-Type", "application/json; charset=utf-8");
        return r;
    });

    CROW_ROUTE(app, "/api/excal/poses")
    ([hmd_bus = deps.hmd_bus, hmd_stale = deps.hmd_stale_ms,
      controller_bus = deps.controller_bus,
      controller_stale = deps.controller_stale_ms,
      role = deps.controller_role]() {
        const bool hmd_enabled = hmd_bus != nullptr;
        const bool controller_enabled = controller_bus != nullptr;

        vmt::HmdPoseSnapshot hmd_snap;
        if (hmd_enabled) {
            hmd_snap = hmd_bus->snapshot(hmd_stale);
        }

        vmt::ControllerPoseSnapshot controller_snap;
        if (controller_enabled) {
            controller_snap = controller_bus->snapshot(controller_stale);
        }

        std::ostringstream o;
        o << "{\"hmd\":";
        append_excal_hmd_pose_json(o, hmd_snap, hmd_enabled);
        o << ",\"controller\":";
        append_excal_controller_pose_json(o, controller_snap, controller_enabled,
                                          role);
        o << "}";
        return json_response(o.str());
    });

    CROW_ROUTE(app, "/api/excal/start").methods(crow::HTTPMethod::POST)
    ([session](const crow::request& /*req*/) {
        session->start();
        crow::response r{"{\"ok\":true,\"state\":\""
                         + std::string(pipeline::extrinsic_calib_state_name(session->state()))
                         + "\"}"};
        r.set_header("Content-Type", "application/json; charset=utf-8");
        return r;
    });

    CROW_ROUTE(app, "/api/excal/stop").methods(crow::HTTPMethod::POST)
    ([session](const crow::request& /*req*/) {
        session->stop_collecting();
        crow::response r{"{\"ok\":true,\"samples\":"
                         + std::to_string(session->sample_count()) + "}"};
        r.set_header("Content-Type", "application/json; charset=utf-8");
        return r;
    });

    CROW_ROUTE(app, "/api/excal/solve").methods(crow::HTTPMethod::POST)
    ([session, next_step = deps.next_step](const crow::request& /*req*/) {
        std::string err;
        bool ok = session->solve_and_write(err);
        std::ostringstream o;
        // err can carry paths / OpenCV exception text → JSON-escape so a `"`
        // or backslash from `write failed: …` doesn't break the response body.
        o << "{\"ok\":" << (ok ? "true" : "false")
          << ",\"err\":\"" << json_escape(err) << "\""
          << ",\"state\":\"" << pipeline::extrinsic_calib_state_name(session->state()) << "\"";
        // On success the session's on_solved hook is about to stop the
        // process (auto-exit); tell the client what to run next.
        if (ok && !next_step.empty()) {
            o << ",\"next_step\":\"" << json_escape(next_step) << "\"";
        }
        o << "}";
        crow::response r{o.str()};
        r.set_header("Content-Type", "application/json; charset=utf-8");
        return r;
    });
}

}  // namespace fitra::web::detail
