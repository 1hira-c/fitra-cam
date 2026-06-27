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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include <crow.h>

#include "camera/setup_camera_manager.hpp"
#include "camera/v4l2_enumerate.hpp"
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
        // Per-camera overrides: cameras.overrides[i] (i = slot 0..2) carries the
        // capture-resolution / pixel-format / exposure overrides. Each field's
        // unset sentinel (0 capture, "" pixel_format/exposure_mode, -1 gain)
        // means "use the global / leave the camera default", matching the YAML
        // cam{N}_* keys and MainOptions arrays.
        if (c.has("overrides") && c["overrides"].t() == crow::json::type::List) {
            const auto ov = c["overrides"];
            const std::size_t n = std::min<std::size_t>(ov.size(), 3);
            for (std::size_t i = 0; i < n; ++i) {
                const auto o = ov[i];
                jint(o, "capture_width",  d.cam_cap_width[i]);
                jint(o, "capture_height", d.cam_cap_height[i]);
                jstr(o, "pixel_format",   d.cam_pixel_format[i]);
                jstr(o, "exposure_mode",  d.cam_exposure_mode[i]);
                jint(o, "exposure",       d.cam_exposure[i]);
                jint(o, "gain",           d.cam_gain[i]);
                jint(o, "ae_target",      d.cam_ae_target[i]);
            }
        }
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
        // Single source of truth: an empty host means runtime zeroconf discovery
        // (the beacon resolves ip:port); a pinned host means manual. Deriving the
        // flag from host-emptiness keeps the persisted config coherent and
        // matches the runtime gate (pose_relay_builder.cpp). It rules out the two
        // incoherent combinations the WebUI radios could otherwise POST:
        //   - discovery=false + host="" -> validate_options rejects it as
        //     "--vmt-out needs a destination" (main_config.cpp), so the setup
        //     proceed/save would fail; a blank host now means auto-detect.
        //   - discovery=true + a pinned host -> a misleading flag the runtime
        //     ignores anyway (a non-empty host always wins and disables the beacon).
        d.vmt_discovery = d.vmt_host.empty();
    }
    if (cfg.has("slimevr")) {
        const auto c = cfg["slimevr"];
        jbool(c, "slimevr_out", d.slimevr_out);
        jstr(c, "host", d.slimevr_host);
        jint(c, "port", d.slimevr_port);
    }
    if (cfg.has("intrinsic_calib")) {
        const auto c = cfg["intrinsic_calib"];
        jbool(c, "enabled", d.intrinsic_step_enabled);  // daemon STEP selector
        jstr(c, "out", d.intrinsic_out);                // where the intrinsic step writes
    }
    if (cfg.has("extrinsic_calib")) {
        const auto c = cfg["extrinsic_calib"];
        jstr(c, "method", d.excal_method);
        if (jstr(c, "out", d.excal_out)) {
            // Keep the floor output target coupled to `out`, as the loader does.
            d.floor_out = d.excal_out;
        }
        // PnP / marker intrinsics + floor tag layout. The floor method requires
        // floor_map; both methods fall back to three_d.calib when their
        // intrinsics field is empty (see config::precheck_mode_switch).
        jstr(c, "intrinsics",       d.excal_intrinsics);   // controller method
        jstr(c, "floor_map",        d.floor_map);
        jstr(c, "floor_intrinsics", d.floor_intrinsics);   // floor method
        jbool(c, "floor_fisheye",   d.floor_fisheye);
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
      << ",\"n_buffers\":" << d.n_buffers
      << ",\"overrides\":[";
    for (int i = 0; i < 3; ++i) {
        if (i) o << ",";
        o << "{\"capture_width\":" << d.cam_cap_width[i]
          << ",\"capture_height\":" << d.cam_cap_height[i]
          << ",\"pixel_format\":\"" << json_escape(d.cam_pixel_format[i]) << "\""
          << ",\"exposure_mode\":\"" << json_escape(d.cam_exposure_mode[i]) << "\""
          << ",\"exposure\":" << d.cam_exposure[i]
          << ",\"gain\":" << d.cam_gain[i]
          << ",\"ae_target\":" << d.cam_ae_target[i] << "}";
    }
    o << "]},"
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
      << ",\"discovery\":" << b(d.vmt_discovery)
      << ",\"host\":\"" << json_escape(d.vmt_host) << "\",\"port\":" << d.vmt_port
      << ",\"hmd_listen_enabled\":" << b(d.hmd_listen_enabled) << "},"
      << "\"slimevr\":{\"slimevr_out\":" << b(d.slimevr_out)
      << ",\"host\":\"" << json_escape(d.slimevr_host) << "\",\"port\":" << d.slimevr_port << "},"
      << "\"intrinsic_calib\":{\"enabled\":" << b(d.intrinsic_step_enabled)
      << ",\"out\":\"" << json_escape(d.intrinsic_out) << "\"},"
      << "\"extrinsic_calib\":{\"method\":\"" << json_escape(d.excal_method) << "\""
      << ",\"out\":\"" << json_escape(d.excal_out) << "\""
      << ",\"intrinsics\":\"" << json_escape(d.excal_intrinsics) << "\""
      << ",\"floor_map\":\"" << json_escape(d.floor_map) << "\""
      << ",\"floor_intrinsics\":\"" << json_escape(d.floor_intrinsics) << "\""
      << ",\"floor_fisheye\":" << b(d.floor_fisheye) << "}"
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

std::string cameras_json() {
    const auto devices = camera::enumerate_v4l2_cameras();
    std::ostringstream o;
    o << "{\"cameras\":[";
    for (std::size_t i = 0; i < devices.size(); ++i) {
        const auto& d = devices[i];
        if (i) o << ",";
        o << "{\"by_path\":\"" << json_escape(d.by_path) << "\""
          << ",\"dev_node\":\"" << json_escape(d.dev_node) << "\""
          << ",\"card\":\"" << json_escape(d.card) << "\""
          << ",\"driver\":\"" << json_escape(d.driver) << "\""
          << ",\"formats\":[";
        for (std::size_t fi = 0; fi < d.formats.size(); ++fi) {
            const auto& f = d.formats[fi];
            if (fi) o << ",";
            o << "{\"fourcc\":\"" << json_escape(f.fourcc) << "\""
              << ",\"description\":\"" << json_escape(f.description) << "\""
              << ",\"sizes\":[";
            for (std::size_t si = 0; si < f.sizes.size(); ++si) {
                const auto& s = f.sizes[si];
                if (si) o << ",";
                o << "{\"width\":" << s.width << ",\"height\":" << s.height
                  << ",\"fps\":[";
                for (std::size_t pi = 0; pi < s.fps.size(); ++pi) {
                    if (pi) o << ",";
                    o << s.fps[pi];
                }
                o << "]}";
            }
            o << "]}";
        }
        o << "]}";
    }
    o << "]}";
    return o.str();
}

// Build a PreviewRequest from a JSON body (device required).
bool parse_preview(const crow::json::rvalue& body, camera::PreviewRequest& req,
                   std::string& err) {
    if (!body || !body.has("device") ||
        body["device"].t() != crow::json::type::String) {
        err = "device required";
        return false;
    }
    req.device = body["device"].s();
    if (body.has("width")  && body["width"].t()  == crow::json::type::Number) req.width  = static_cast<int>(body["width"].i());
    if (body.has("height") && body["height"].t() == crow::json::type::Number) req.height = static_cast<int>(body["height"].i());
    if (body.has("fps")    && body["fps"].t()    == crow::json::type::Number) req.fps    = static_cast<int>(body["fps"].i());
    if (body.has("pixel_format")  && body["pixel_format"].t()  == crow::json::type::String) req.pixel_format  = body["pixel_format"].s();
    if (body.has("exposure_mode") && body["exposure_mode"].t() == crow::json::type::String) req.exposure_mode = body["exposure_mode"].s();
    if (body.has("exposure")  && body["exposure"].t()  == crow::json::type::Number) req.exposure  = static_cast<int>(body["exposure"].i());
    if (body.has("gain")      && body["gain"].t()      == crow::json::type::Number) req.gain      = static_cast<int>(body["gain"].i());
    if (body.has("ae_target") && body["ae_target"].t() == crow::json::type::Number) req.ae_target = static_cast<int>(body["ae_target"].i());
    return true;
}

}  // namespace

void register_setup_mode_routes(crow::SimpleApp& app, const SetupRouteDeps& deps) {
    if (deps.store) {
        auto* store = deps.store;

        // GET /api/setup/check-path?path=<p> — resolve `path` against the CWD (the
        // same base the run/calib children open engines + calib artifacts from) and
        // report existence, so the WebUI can flag a missing engine/calib path on the
        // spot whether the user typed an absolute or a relative path. Inside the
        // deps.store guard so this filesystem-existence probe exists ONLY in the
        // Setup module — never on the run/calib 0.0.0.0 bind, where it would be a
        // CWD-subtree existence oracle for any LAN client.
        CROW_ROUTE(app, "/api/setup/check-path")
        ([](const crow::request& req) {
            const char* p = req.url_params.get("path");
            const std::string path = p ? p : "";
            std::error_code ec;
            std::string abs;
            bool exists = false, is_file = false, allowed = false;
            if (!path.empty()) {
                abs = std::filesystem::weakly_canonical(
                          std::filesystem::absolute(path, ec), ec).string();
                const std::string root = std::filesystem::weakly_canonical(
                          std::filesystem::current_path(ec), ec).string();
                // Containment: only probe paths under the daemon CWD (engines/calib
                // live under outputs/ + configs/). path_within anchors at the
                // directory boundary so a sibling like <root>-secret can't escape.
                allowed = path_within(root, abs);
                if (allowed) {
                    exists  = std::filesystem::exists(abs, ec) && !ec;
                    is_file = exists && std::filesystem::is_regular_file(abs, ec) && !ec;
                }
            }
            std::ostringstream o;
            o << "{\"path\":\"" << json_escape(path) << "\""
              << ",\"abs\":\"" << json_escape(abs) << "\""
              << ",\"allowed\":" << b(allowed)
              << ",\"exists\":" << b(exists)
              << ",\"is_file\":" << b(is_file) << "}";
            return json_response(o.str());
        });

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
        ([store, on_validate = deps.on_validate](const crow::request&) {
            std::string err;
            // on_validate (app layer) is mode-aware and rejects a cameras-but-no-
            // engines draft; fall back to the relaxed range/enum check when unset.
            const bool ok =
                on_validate ? on_validate(err) : store->validate_draft(err);
            return ok_or_err(ok, err);
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

    if (deps.cameras) {
        auto* cams = deps.cameras;

        // Enumeration is fresh per call (cheap, opens read-only + closes).
        CROW_ROUTE(app, "/api/cameras")
        ([]() { return json_response(cameras_json()); });

        CROW_ROUTE(app, "/api/cameras/preview/start").methods(crow::HTTPMethod::POST)
        ([cams](const crow::request& req) {
            camera::PreviewRequest pr;
            std::string err;
            if (!parse_preview(crow::json::load(req.body), pr, err))
                return ok_or_err(false, err);
            return ok_or_err(cams->start(pr, err), err);
        });

        CROW_ROUTE(app, "/api/cameras/preview/stop").methods(crow::HTTPMethod::POST)
        ([cams](const crow::request& req) {
            auto body = crow::json::load(req.body);
            std::string device;
            if (body && body.has("device") &&
                body["device"].t() == crow::json::type::String) {
                device = body["device"].s();
            }
            if (device.empty()) return ok_or_err(false, "device required");
            cams->stop(device);
            return json_response("{\"ok\":true}");
        });

        // GET /api/cameras/preview.jpg?cam=<device> — single JPEG snapshot the
        // browser polls (Crow cannot stream multipart from a handler).
        CROW_ROUTE(app, "/api/cameras/preview.jpg")
        ([cams](const crow::request& req) {
            const char* cam = req.url_params.get("cam");
            if (!cam) return crow::response{400, "cam query param required"};
            std::vector<std::uint8_t> jpeg;
            if (!cams->latest_jpeg(cam, jpeg)) {
                return crow::response{503, "preview not ready"};
            }
            crow::response resp{std::string(jpeg.begin(), jpeg.end())};
            resp.set_header("Content-Type", "image/jpeg");
            resp.set_header("Cache-Control", "no-store");
            return resp;
        });
    }
}

}  // namespace fitra::web::detail
