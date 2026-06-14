#include "pipeline/excal_replay_input.hpp"

#include <algorithm>
#include <charconv>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>

namespace fitra::pipeline {

namespace {

// Find `"key":` at object level and return the offset just past the colon,
// or npos. The format is machine-generated (tools/excal_record) with no
// whitespace around separators and no escaped quotes inside strings, so a
// plain substring search on the quoted key is unambiguous.
std::size_t value_pos(std::string_view s, std::string_view key) {
    std::string needle = "\"";
    needle += key;
    needle += "\":";
    auto p = s.find(needle);
    if (p == std::string_view::npos) return p;
    return p + needle.size();
}

bool parse_double_at(std::string_view s, std::string_view key, double& out) {
    auto p = value_pos(s, key);
    if (p == std::string_view::npos || p >= s.size()) return false;
    std::size_t end = s.find_first_of(",}", p);
    if (end == std::string_view::npos) return false;
    auto num = s.substr(p, end - p);
    if (num.empty()) return false;
    // from_chars is locale-independent (always parses '.' as the decimal
    // point) and needs no NUL-terminated copy — strtod would mis-parse on a
    // locale that uses ',' as the decimal separator.
    double v = 0.0;
    auto [ptr, ec] = std::from_chars(num.data(), num.data() + num.size(), v);
    if (ec != std::errc{} || ptr != num.data() + num.size()) return false;
    out = v;
    return true;
}

bool parse_bool_at(std::string_view s, std::string_view key, bool& out) {
    auto p = value_pos(s, key);
    if (p == std::string_view::npos) return false;
    if (s.compare(p, 4, "true") == 0)  { out = true;  return true; }
    if (s.compare(p, 5, "false") == 0) { out = false; return true; }
    return false;
}

bool parse_string_at(std::string_view s, std::string_view key, std::string& out) {
    auto p = value_pos(s, key);
    if (p == std::string_view::npos || p >= s.size() || s[p] != '"') return false;
    auto end = s.find('"', p + 1);
    if (end == std::string_view::npos) return false;
    out.assign(s.substr(p + 1, end - p - 1));
    // The recorder never escapes paths; a backslash means this is not a
    // format we understand.
    return out.find('\\') == std::string::npos;
}

}  // namespace

bool parse_excal_frame_line(std::string_view line, ExcalReplayRecord& out) {
    double cam = 0.0, ts_ms = 0.0;
    if (!parse_double_at(line, "cam", cam) || cam < 0.0) return false;
    if (!parse_string_at(line, "file", out.file) || out.file.empty()) return false;
    if (!parse_double_at(line, "ts_ms", ts_ms)) return false;

    ControllerObservation c;
    if (!parse_bool_at(line, "running_ok", c.running_ok)) return false;
    if (!parse_double_at(line, "x", c.x)) return false;
    if (!parse_double_at(line, "y", c.y)) return false;
    if (!parse_double_at(line, "z", c.z)) return false;
    if (!parse_double_at(line, "qx", c.qx)) return false;
    if (!parse_double_at(line, "qy", c.qy)) return false;
    if (!parse_double_at(line, "qz", c.qz)) return false;
    if (!parse_double_at(line, "qw", c.qw)) return false;
    c.ts_ms = ts_ms;

    out.cam  = static_cast<std::size_t>(cam);
    out.ctrl = c;
    return true;
}

ExcalReplayInput::ExcalReplayInput(const std::string& session_dir)
    : dir_{session_dir} {
    // meta.json: only the format version is contractual; the image size etc.
    // are determined by the JPEGs themselves.
    {
        std::ifstream mf{dir_ / "meta.json"};
        if (!mf.is_open()) {
            throw std::runtime_error("excal-replay: cannot open "
                                     + (dir_ / "meta.json").string());
        }
        std::ostringstream oss;
        oss << mf.rdbuf();
        const std::string meta = oss.str();
        if (meta.find("\"version\": 1") == std::string::npos &&
            meta.find("\"version\":1") == std::string::npos) {
            throw std::runtime_error(
                "excal-replay: unsupported meta.json version in "
                + (dir_ / "meta.json").string());
        }
    }

    std::ifstream jf{dir_ / "frames.jsonl"};
    if (!jf.is_open()) {
        throw std::runtime_error("excal-replay: cannot open "
                                 + (dir_ / "frames.jsonl").string());
    }
    std::string line;
    std::size_t line_no = 0;
    while (std::getline(jf, line)) {
        ++line_no;
        if (line.empty()) continue;
        ExcalReplayRecord rec;
        if (!parse_excal_frame_line(line, rec)) {
            throw std::runtime_error("excal-replay: bad frames.jsonl line "
                                     + std::to_string(line_no) + " in "
                                     + (dir_ / "frames.jsonl").string());
        }
        records_.push_back(std::move(rec));
    }
}

bool ExcalReplayInput::next(ExcalInputItem& out) {
    if (exhausted()) return false;
    const auto& rec = records_[idx_++];

    const auto path = dir_ / rec.file;
    std::ifstream f{path, std::ios::binary};
    if (!f.is_open()) {
        throw std::runtime_error("excal-replay: cannot open " + path.string());
    }
    std::vector<char> bytes{std::istreambuf_iterator<char>(f),
                            std::istreambuf_iterator<char>()};
    cv::Mat buf(1, static_cast<int>(bytes.size()), CV_8UC1, bytes.data());
    cv::Mat bgr = cv::imdecode(buf, cv::IMREAD_COLOR);
    if (bgr.empty()) {
        throw std::runtime_error("excal-replay: imdecode failed for "
                                 + path.string());
    }

    out.cam_idx = rec.cam;
    out.bgr     = std::move(bgr);
    out.ctrl    = rec.ctrl;
    return true;
}

std::size_t ExcalReplayInput::camera_count() const {
    std::size_t max_cam = 0;
    bool any = false;
    for (const auto& r : records_) {
        max_cam = std::max(max_cam, r.cam);
        any = true;
    }
    return any ? max_cam + 1 : 0;
}

}  // namespace fitra::pipeline
