// Setup-mode route group (docs/design/core-pipeline-setup-mode.md): the
// editable config draft (/api/config*) and, when a camera manager is attached
// (M3), V4L2 enumeration + preview (/api/cameras*). Registered by
// CrowServer::start() only in RunMode::Setup, so these paths 404 in every other
// mode (same conditional-registration pattern as the calib groups).
//
// The JSON <-> MainOptions mapping lives here (not in fitra_config) so the
// store stays Crow-free. GET/POST /api/config exposes an editable SUBSET of the
// config (the fields the Setup wizard touches); fields outside the subset keep
// their seed values and are still written verbatim by write_union (emit covers
// the whole struct).

#include <sstream>
#include <string>

#include <crow.h>

#include "config/main_config.hpp"
#include "config/setup_config_store.hpp"
#include "web/crow_routes_setup.hpp"
#include "web/crow_util.hpp"

namespace fitra::web::detail {

namespace {

using config::MainOptions;

// ---- crow::json typed getters (no-throw; update only when present + typed) --
bool jstr(const crow::json::rvalue& o, const char* k, std::string& v) {
    if (o.has(k) && o[k].t() == crow::json::type::String) { v = o[k].s(); return true; }
    return false;
}
bool jint(const crow::json::rvalue& o, const char* k, int& v) {
    if (o.has(k) && o[k].t() == crow::json::type::Number) { v = static_cast<int>(o[k].i()); return true; }
    return false;
}
bool jflt(const crow::json::rvalue& o, const char* k, float& v) {
    if (o.has(k) && o[k].t() == crow::json::type::Number) { v = static_cast<float>(o[k].d()); return true; }
    return false;
}
bool jbool(const crow::json::rvalue& o, const char* k, bool& v) {
    if (o.has(k) && (o[k].t() == crow::json::type::True ||
                     o[k].t() == crow::json::type::False)) { v = o[k].b(); return true; }
    return false;
}

// Overlay the editable subset from a JSON `config` object onto the draft.
void merge_config(MainOptions& d, const crow::json::rvalue& cfg) {
    if (cfg.has("cameras")) {
        const auto c = cfg["cameras"];
        jstr(c, "cam0", d.cam_paths[0]);
        jstr(c, "cam1", d.cam_paths[1]);
        jstr(c, "cam2", d.cam_paths[2]);
        jint(c, "width", d.width);
        jint(c, "height", d.height);
        jint(c, "fps", d.fps);
        jstr(c, "pixel_format", d.pixel_format);
        jint(c, "n_buffers", d.n_buffers);
    }
    if (cfg.has("inference")) {
        const auto c = cfg["inference"];
        jstr(c, "det_engine", d.det_engine);
        jstr(c, "pose_engine", d.pose_engine);
        jstr(c, "keypoint_format", d.keypoint_format);
        jint(c, "det_frequency", d.det_frequency);
        jflt(c, "det_score", d.det_score);
        jbool(c, "multi_person", d.multi_person);
    }
    if (cfg.has("web")) {
        const auto c = cfg["web"];
        jstr(c, "host", d.host);
        jint(c, "port", d.port);
    }
    if (cfg.has("three_d")) {
        const auto c = cfg["three_d"];
        jbool(c, "enable_3d", d.enable_3d);
        jstr(c, "calib", d.calib);
    }
    if (cfg.has("vmt")) {
        const auto c = cfg["vmt"];
        jbool(c, "vmt_out", d.vmt_out);
        jstr(c, "host", d.vmt_host);
        jint(c, "port", d.vmt_port);
        jbool(c, "hmd_listen_enabled", d.hmd_listen_enabled);
    }
    if (cfg.has("slimevr")) {
        const auto c = cfg["slimevr"];
        jbool(c, "slimevr_out", d.slimevr_out);
        jstr(c, "host", d.slimevr_host);
        jint(c, "port", d.slimevr_port);
    }
    if (cfg.has("intrinsic_calib")) {
        // The daemon STEP selector (not a run-mode flag).
        jbool(cfg["intrinsic_calib"], "enabled", d.intrinsic_step_enabled);
    }
    if (cfg.has("extrinsic_calib")) {
        const auto c = cfg["extrinsic_calib"];
        jstr(c, "method", d.excal_method);
        if (jstr(c, "out", d.excal_out)) {
            // Keep the floor output target coupled to `out`, as the loader does.
            d.floor_out = d.excal_out;
        }
    }
}

std::string b(bool v) { return v ? "true" : "false"; }

// Serialize the editable subset of the draft as a JSON `config` object.
std::string draft_to_json(const MainOptions& d) {
    std::ostringstream o;
    o << "{"
      << "\"cameras\":{"
      << "\"cam0\":\"" << json_escape(d.cam_paths[0]) << "\","
      << "\"cam1\":\"" << json_escape(d.cam_paths[1]) << "\","
      << "\"cam2\":\"" << json_escape(d.cam_paths[2]) << "\","
      << "\"width\":" << d.width << ",\"height\":" << d.height
      << ",\"fps\":" << d.fps
      << ",\"pixel_format\":\"" << json_escape(d.pixel_format) << "\""
      << ",\"n_buffers\":" << d.n_buffers << "},"
      << "\"inference\":{"
      << "\"det_engine\":\"" << json_escape(d.det_engine) << "\","
      << "\"pose_engine\":\"" << json_escape(d.pose_engine) << "\","
      << "\"keypoint_format\":\"" << json_escape(d.keypoint_format) << "\","
      << "\"det_frequency\":" << d.det_frequency
      << ",\"det_score\":" << d.det_score
      << ",\"multi_person\":" << b(d.multi_person) << "},"
      << "\"web\":{\"host\":\"" << json_escape(d.host) << "\",\"port\":" << d.port << "},"
      << "\"three_d\":{\"enable_3d\":" << b(d.enable_3d)
      << ",\"calib\":\"" << json_escape(d.calib) << "\"},"
      << "\"vmt\":{\"vmt_out\":" << b(d.vmt_out)
      << ",\"host\":\"" << json_escape(d.vmt_host) << "\",\"port\":" << d.vmt_port
      << ",\"hmd_listen_enabled\":" << b(d.hmd_listen_enabled) << "},"
      << "\"slimevr\":{\"slimevr_out\":" << b(d.slimevr_out)
      << ",\"host\":\"" << json_escape(d.slimevr_host) << "\",\"port\":" << d.slimevr_port << "},"
      << "\"intrinsic_calib\":{\"enabled\":" << b(d.intrinsic_step_enabled) << "},"
      << "\"extrinsic_calib\":{\"method\":\"" << json_escape(d.excal_method)
      << "\",\"out\":\"" << json_escape(d.excal_out) << "\"}"
      << "}";
    return o.str();
}

std::string named_json(config::SetupConfigStore* store) {
    std::ostringstream o;
    o << "[";
    const auto names = store->list_named();
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (i) o << ",";
        o << "\"" << json_escape(names[i]) << "\"";
    }
    o << "]";
    return o.str();
}

crow::response ok_or_err(bool ok, const std::string& err) {
    std::ostringstream o;
    o << "{\"ok\":" << b(ok);
    if (!ok) o << ",\"err\":\"" << json_escape(err) << "\"";
    o << "}";
    return json_response(o.str());
}

}  // namespace

void register_setup_mode_routes(crow::SimpleApp& app, const SetupRouteDeps& deps) {
    if (deps.store) {
        auto* store = deps.store;

        CROW_ROUTE(app, "/api/config")
        ([store]() {
            std::ostringstream o;
            o << "{\"config\":" << draft_to_json(store->draft())
              << ",\"named\":" << named_json(store) << "}";
            return json_response(o.str());
        });

        CROW_ROUTE(app, "/api/config").methods(crow::HTTPMethod::POST)
        ([store](const crow::request& req) {
            auto body = crow::json::load(req.body);
            if (!body) return json_response("{\"ok\":false,\"err\":\"invalid JSON\"}", 400);
            MainOptions d = store->draft();
            merge_config(d, body.has("config") ? body["config"] : body);
            store->set_draft(d);
            return json_response("{\"ok\":true}");
        });

        CROW_ROUTE(app, "/api/config/validate").methods(crow::HTTPMethod::POST)
        ([store](const crow::request&) {
            std::string err;
            return ok_or_err(store->validate_draft(err), err);
        });

        CROW_ROUTE(app, "/api/config/list")
        ([store]() {
            return json_response("{\"named\":" + named_json(store) + "}");
        });

        CROW_ROUTE(app, "/api/config/save").methods(crow::HTTPMethod::POST)
        ([store](const crow::request& req) {
            auto body = crow::json::load(req.body);
            std::string name;
            if (body && body.has("name") && body["name"].t() == crow::json::type::String) {
                name = body["name"].s();
            }
            std::string err;
            return ok_or_err(store->save_named(name, err), err);
        });

        CROW_ROUTE(app, "/api/config/load").methods(crow::HTTPMethod::POST)
        ([store](const crow::request& req) {
            auto body = crow::json::load(req.body);
            std::string name;
            if (body && body.has("name") && body["name"].t() == crow::json::type::String) {
                name = body["name"].s();
            }
            std::string err;
            if (!store->load_named(name, err)) return ok_or_err(false, err);
            return json_response("{\"ok\":true,\"config\":" + draft_to_json(store->draft()) + "}");
        });
    }

    if (deps.on_proceed) {
        CROW_ROUTE(app, "/api/setup/proceed").methods(crow::HTTPMethod::POST)
        ([on_proceed = deps.on_proceed](const crow::request&) {
            std::string next, err;
            const bool ok = on_proceed(next, err);
            std::ostringstream o;
            o << "{\"ok\":" << b(ok);
            if (ok) o << ",\"next\":\"" << json_escape(next) << "\"";
            else    o << ",\"err\":\"" << json_escape(err) << "\"";
            o << "}";
            return json_response(o.str());
        });
    }

    // Camera enumeration + preview routes are added in M3 (deps.cameras).
    (void)deps.cameras;
}

}  // namespace fitra::web::detail
