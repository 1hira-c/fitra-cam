#pragma once
//
// Private JSON / static-file helpers shared by the CrowServer route TUs
// (crow_server.cpp, crow_routes_setup.cpp). Not part of the public web API.

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>

#include <crow.h>

namespace fitra::web::detail {

inline crow::response json_response(std::string body, int code = 200) {
    crow::response resp{code, std::move(body)};
    resp.set_header("Content-Type", "application/json; charset=utf-8");
    return resp;
}

inline std::string read_file(const std::filesystem::path& p) {
    std::ifstream f(p, std::ios::binary);
    if (!f.is_open()) return {};
    std::ostringstream oss;
    oss << f.rdbuf();
    return oss.str();
}

inline std::string guess_content_type(const std::filesystem::path& p) {
    auto ext = p.extension().string();
    if (ext == ".html") return "text/html; charset=utf-8";
    if (ext == ".js")   return "application/javascript; charset=utf-8";
    if (ext == ".css")  return "text/css; charset=utf-8";
    if (ext == ".json") return "application/json; charset=utf-8";
    if (ext == ".png")  return "image/png";
    if (ext == ".jpg" || ext == ".jpeg") return "image/jpeg";
    return "application/octet-stream";
}

inline std::string json_escape(std::string_view s) {
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

inline void append_age_ms_json(std::ostringstream& out, double age_ms) {
    out << (std::isfinite(age_ms) ? age_ms : -1.0);
}

}  // namespace fitra::web::detail
