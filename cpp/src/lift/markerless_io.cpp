#include "lift/markerless_io.hpp"

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <istream>
#include <limits>
#include <ostream>
#include <set>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <unistd.h>

#include <crow/json.h>

namespace fitra::lift {
namespace {

using Json = crow::json::rvalue;
using Out = crow::json::wvalue;
using Type = crow::json::type;
using Number = crow::json::num_type;

[[noreturn]] void invalid(const std::string& reason) {
    throw std::runtime_error("markerless replay: " + reason);
}

// Read with a bound before growing memory, rather than checking getline's
// already allocated unbounded line after the fact.
bool read_line(std::istream& input, std::string& line, std::size_t& total) {
    line.clear();
    char ch;
    while (input.get(ch)) {
        if (++total > markerless_max_jsonl_bytes) invalid("input exceeds 32 MiB");
        if (ch == '\n') return true;
        if (line.size() == markerless_max_jsonl_line_bytes)
            invalid("line exceeds 256 KiB");
        line.push_back(ch);
    }
    if (input.bad() || (!input.eof() && input.fail())) invalid("input read failed");
    return !line.empty();
}

void check_depth(const std::string& text) {
    bool in_string = false, escaped = false;
    int depth = 0;
    for (unsigned char ch : text) {
        // Crow's parser uses a NUL sentinel even for its length-taking load().
        // A raw NUL must not hide trailing bytes after a valid JSON prefix.
        if (ch == 0) invalid("raw NUL byte in JSONL input");
        if (in_string) {
            if (ch < 0x20) invalid("unescaped control character");
            if (escaped) escaped = false;
            else if (ch == '\\') escaped = true;
            else if (ch == '"') in_string = false;
        } else if (ch == '"') in_string = true;
        else if (ch == '{' || ch == '[') {
            if (++depth > 16) invalid("JSON nesting exceeds 16");
        } else if (ch == '}' || ch == ']') --depth;
    }
}

// Crow's integer accessors use stringstream without checking overflow. Validate
// the original number tokens before its parser owns/reorders the object. The
// resulting u()/i() conversions are then known to fit, including uint64's top.
void check_number_tokens(const std::string& text) {
    bool in_string = false, escaped = false;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const char ch = text[i];
        if (in_string) {
            if (escaped) escaped = false;
            else if (ch == '\\') escaped = true;
            else if (ch == '"') in_string = false;
            continue;
        }
        if (ch == '"') { in_string = true; continue; }
        if (ch != '-' && (ch < '0' || ch > '9')) continue;
        const std::size_t first = i;
        while (i < text.size() && text[i] != ',' && text[i] != '}' && text[i] != ']'
               && text[i] != ' ' && text[i] != '\t' && text[i] != '\r' && text[i] != '\n') ++i;
        const char* begin = text.data() + first;
        const char* end = text.data() + i;
        const char* cursor = begin;
        if (cursor != end && *cursor == '-') ++cursor;
        if (cursor == end) invalid("invalid JSON number");
        if (*cursor == '0') ++cursor;
        else if (*cursor >= '1' && *cursor <= '9') {
            do { ++cursor; } while (cursor != end && *cursor >= '0' && *cursor <= '9');
        } else invalid("invalid JSON number");
        if (cursor != end && *cursor == '.') {
            ++cursor;
            if (cursor == end || *cursor < '0' || *cursor > '9') invalid("invalid JSON fraction");
            do { ++cursor; } while (cursor != end && *cursor >= '0' && *cursor <= '9');
        }
        if (cursor != end && (*cursor == 'e' || *cursor == 'E')) {
            ++cursor;
            if (cursor != end && (*cursor == '+' || *cursor == '-')) ++cursor;
            if (cursor == end || *cursor < '0' || *cursor > '9') invalid("invalid JSON exponent");
            do { ++cursor; } while (cursor != end && *cursor >= '0' && *cursor <= '9');
        }
        if (cursor != end) invalid("invalid JSON number token");
        std::from_chars_result parsed;
        if (std::find(begin, end, '.') != end || std::find(begin, end, 'e') != end
            || std::find(begin, end, 'E') != end) {
            double number = 0.0;
            parsed = std::from_chars(begin, end, number);
            if (!std::isfinite(number)) invalid("nonfinite number token");
        } else if (ch == '-') {
            std::int64_t number = 0;
            parsed = std::from_chars(begin, end, number);
        } else {
            std::uint64_t number = 0;
            parsed = std::from_chars(begin, end, number);
        }
        if (parsed.ec != std::errc{} || parsed.ptr != end)
            invalid("invalid or out-of-range number token");
        if (i < text.size()) --i;
    }
}

void check_unique_keys(const Json& value) {
    if (value.t() == Type::Object) {
        std::set<std::string> keys;
        for (const auto& item : value) {
            const std::string key = item.key();
            if (!keys.insert(key).second) invalid("duplicate object key: " + key);
            check_unique_keys(item);
        }
    } else if (value.t() == Type::List) {
        for (const auto& item : value) check_unique_keys(item);
    }
}

Json parse_line(const std::string& line) {
    check_depth(line);
    check_number_tokens(line);
    Json value = crow::json::load(line);
    if (!value || value.t() != Type::Object) invalid("expected a JSON object");
    check_unique_keys(value);
    return value;
}

const Json& field(const Json& object, const char* name) {
    if (object.t() != Type::Object || !object.has(name))
        invalid(std::string("missing field: ") + name);
    return object[name];
}

std::string text(const Json& value, const char* name) {
    if (value.t() != Type::String) invalid(std::string(name) + " must be a string");
    const std::string result = value.s();
    if (result.empty() || result.size() > 256)
        invalid(std::string(name) + " must contain 1..256 bytes");
    return result;
}

std::uint64_t uint64(const Json& value, const char* name) {
    // Crow's i/u APIs also accept strings. Check token kind first to prevent
    // converting strings, exponent/decimal numbers or negative values to IDs.
    if (value.t() != Type::Number || value.nt() != Number::Unsigned_integer)
        invalid(std::string(name) + " must be a nonnegative integer token");
    return value.u();
}

std::int64_t int64(const Json& value, const char* name) {
    if (value.t() != Type::Number) invalid(std::string(name) + " must be an integer");
    if (value.nt() == Number::Signed_integer) return value.i();
    if (value.nt() != Number::Unsigned_integer)
        invalid(std::string(name) + " must be an integer token");
    const auto n = value.u();
    if (n > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
        invalid(std::string(name) + " exceeds int64 range");
    return static_cast<std::int64_t>(n);
}

int integer(const Json& value, const char* name) {
    const auto n = int64(value, name);
    if (n < std::numeric_limits<int>::min() || n > std::numeric_limits<int>::max())
        invalid(std::string(name) + " exceeds int range");
    return static_cast<int>(n);
}

double finite_number(const Json& value, const char* name) {
    if (value.t() != Type::Number) invalid(std::string(name) + " must be a number");
    const double n = value.d();
    if (!std::isfinite(n)) invalid(std::string(name) + " must be finite");
    return n;
}

MarkerlessTimestampKind timestamp(const std::string& name) {
    if (name == "kernel_monotonic_soe") return MarkerlessTimestampKind::KernelMonotonicSoe;
    if (name == "kernel_monotonic_eof") return MarkerlessTimestampKind::KernelMonotonicEof;
    if (name == "synthetic_monotonic") return MarkerlessTimestampKind::SyntheticMonotonic;
    invalid("unknown timestamp_kind");
}

const char* timestamp_name(MarkerlessTimestampKind kind) {
    switch (kind) {
        case MarkerlessTimestampKind::KernelMonotonicSoe: return "kernel_monotonic_soe";
        case MarkerlessTimestampKind::KernelMonotonicEof: return "kernel_monotonic_eof";
        case MarkerlessTimestampKind::SyntheticMonotonic: return "synthetic_monotonic";
        default: invalid("cannot serialize unknown timestamp_kind");
    }
}

MarkerlessFrameSet decode(const Json& value) {
    MarkerlessFrameSet set;
    set.sample_id = uint64(field(value, "sample_id"), "sample_id");
    set.continuity_id = text(field(value, "continuity_id"), "continuity_id");
    const auto& frames = field(value, "frames");
    if (frames.t() != Type::List || frames.size() < 2 || frames.size() > 3)
        invalid("frames must be an array of 2..3 camera frames");
    for (const auto& item : frames) {
        MarkerlessCameraFrame frame;
        frame.camera_id = text(field(item, "camera_id"), "camera_id");
        frame.source_seq = uint64(field(item, "source_seq"), "source_seq");
        frame.stream_epoch = uint64(field(item, "stream_epoch"), "stream_epoch");
        frame.capture_mono_ns = int64(field(item, "capture_mono_ns"), "capture_mono_ns");
        frame.timestamp_kind = timestamp(text(field(item, "timestamp_kind"), "timestamp_kind"));
        frame.width = integer(field(item, "width"), "width");
        frame.height = integer(field(item, "height"), "height");
        frame.person_count = integer(field(item, "person_count"), "person_count");
        const auto& points = field(item, "keypoints");
        if (points.t() != Type::List || points.size() > 26)
            invalid("keypoints must be an array of at most 26 observations");
        for (const auto& point : points) {
            if (point.t() != Type::List || point.size() != 4)
                invalid("keypoint must contain [joint_index,x,y,score]");
            frame.keypoints.push_back({integer(point[0], "joint_index"),
                {finite_number(point[1], "x"), finite_number(point[2], "y")},
                finite_number(point[3], "score")});
        }
        set.frames.push_back(std::move(frame));
    }
    return set;
}

Out encode(const MarkerlessFrameSet& set) {
    Out result;
    result["sample_id"] = set.sample_id;
    result["continuity_id"] = set.continuity_id;
    result["frames"] = Out::list{};
    std::size_t i = 0;
    for (const auto& frame : set.frames) {
        auto& item = result["frames"][i++];
        item["camera_id"] = frame.camera_id;
        item["source_seq"] = frame.source_seq;
        item["stream_epoch"] = frame.stream_epoch;
        item["capture_mono_ns"] = frame.capture_mono_ns;
        item["timestamp_kind"] = timestamp_name(frame.timestamp_kind);
        item["width"] = frame.width;
        item["height"] = frame.height;
        item["person_count"] = frame.person_count;
        item["keypoints"] = Out::list{};
        std::size_t k = 0;
        for (const auto& point : frame.keypoints) {
            if (!std::isfinite(point.pixel.x) || !std::isfinite(point.pixel.y)
                || !std::isfinite(point.score)) invalid("cannot serialize nonfinite observation");
            auto& p = item["keypoints"][k++];
            p[0] = point.joint_index;
            p[1] = point.pixel.x;
            p[2] = point.pixel.y;
            p[3] = point.score;
        }
    }
    return result;
}

Out finite_or_null(double value) {
    if (std::isfinite(value)) return Out(value);
    return Out(nullptr);
}

}  // namespace

MarkerlessReplay read_markerless_observations(std::istream& input,
    const CalibrationSet& intrinsics, const MarkerlessCollectionOptions& options) {
    MarkerlessCollector collector{intrinsics, options};
    std::string line;
    std::size_t total = 0, line_number = 1;
    if (!read_line(input, line, total)) invalid("missing JSONL header");
    const auto header = parse_line(line);
    if (text(field(header, "schema"), "schema") != "fitra_markerless_observations_v1")
        invalid("unsupported observations schema");
    if (text(field(header, "keypoint_format"), "keypoint_format") != "halpe26")
        invalid("keypoint_format must be halpe26");
    MarkerlessReplay replay;
    while (read_line(input, line, total)) {
        ++line_number;
        MarkerlessFrameSet set;
        try { set = decode(parse_line(line)); }
        catch (const std::exception& e) {
            invalid("line " + std::to_string(line_number) + ": " + e.what());
        }
        ++replay.records;
        std::string reason;
        const auto status = collector.ingest(set, reason);
        if (status != MarkerlessIngest::Accepted) {
            ++replay.rejected_records;
            if (replay.diagnostics.size() < 64)
                replay.diagnostics.push_back("line " + std::to_string(line_number) + ": " + reason);
        }
    }
    replay.frame_sets = collector.frame_sets();
    replay.collection = collector.stats();
    return replay;
}

void write_markerless_observations(std::ostream& output,
    const std::vector<MarkerlessFrameSet>& frame_sets) {
    Out header;
    header["schema"] = "fitra_markerless_observations_v1";
    header["keypoint_format"] = "halpe26";
    output << header.dump() << '\n';
    for (const auto& set : frame_sets) {
        const std::string line = encode(set).dump();
        if (line.size() > markerless_max_jsonl_line_bytes) invalid("encoded line exceeds 256 KiB");
        output << line << '\n';
        if (!output) invalid("observation write failed");
    }
    if (!output) invalid("observation write failed");
}

std::string markerless_relative_report_json(const MarkerlessRelativeResult& geometry,
                                           const MarkerlessReplay& replay) {
    Out report;
    report["schema"] = "fitra_markerless_relative_v1";
    report["coordinate_system"] = "reference_camera";
    report["unit"] = "arbitrary_baseline";
    report["ok"] = geometry.ok && replay.input_clean();
    report["geometry_ok"] = geometry.ok;
    report["input_clean"] = replay.input_clean();
    report["diagnostic_only"] = true;
    report["metric_scale_known"] = false;
    report["floor_aligned"] = false;
    report["runtime_ready"] = false;
    report["reason"] = replay.input_clean() ? geometry.reason : "input_rejected_or_reset";
    report["geometry_reason"] = geometry.reason;
    report["reference_camera_id"] = geometry.reference_camera_id;
    report["baseline_camera_id"] = geometry.baseline_camera_id;
    report["training_frame_sets"] = geometry.training_frame_sets;
    report["validation_frame_sets"] = geometry.validation_frame_sets;
    report["inlier_points"] = geometry.inlier_points;
    report["training_reprojection_median_px"] = finite_or_null(geometry.training_reprojection_median_px);
    report["validation_reprojection_median_px"] = finite_or_null(geometry.validation_reprojection_median_px);
    report["validation_reprojection_p95_px"] = finite_or_null(geometry.validation_reprojection_p95_px);
    report["structure_ratio"] = finite_or_null(geometry.structure_ratio);
    report["median_ray_angle_deg"] = finite_or_null(geometry.median_ray_angle_deg);
    auto& counts = report["collection"];
    counts["records"] = replay.records;
    counts["rejected_records"] = replay.rejected_records;
    counts["accepted"] = replay.collection.accepted;
    counts["rejected"] = replay.collection.rejected;
    counts["resets"] = replay.collection.resets;
    counts["evicted"] = replay.collection.evicted;
    counts["retained"] = replay.frame_sets.size();
    counts["max_accepted_skew_ms"] = finite_or_null(replay.collection.max_accepted_skew_ms);
    report["diagnostics"] = replay.diagnostics;
    report["cameras"] = Out::list{};
    std::size_t i = 0;
    for (const auto& camera : geometry.cameras) {
        auto& item = report["cameras"][i++];
        item["id"] = camera.id;
        auto& matrix = item["T_camera_reference"];
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                matrix[r][c] = finite_or_null(camera.T_camera_reference(r,c));
    }
    return report.dump();
}

void write_markerless_report_new(const std::string& path, const std::string& report) {
    const std::filesystem::path output{path};
    if (output.extension() != ".json") invalid("report output must be a new .json file");
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (fd < 0) invalid("cannot create new report: " + std::string(std::strerror(errno)));
    bool closed = false;
    try {
        const std::string bytes = report + '\n';
        std::size_t offset = 0;
        while (offset < bytes.size()) {
            const auto count = ::write(fd, bytes.data() + offset, bytes.size() - offset);
            if (count < 0 && errno == EINTR) continue;
            if (count <= 0) invalid("report write failed");
            offset += static_cast<std::size_t>(count);
        }
        const int result = ::close(fd);
        closed = true;
        if (result != 0) invalid("report close failed");
    } catch (...) {
        if (!closed) ::close(fd);
        ::unlink(path.c_str());
        throw;
    }
}

}  // namespace fitra::lift
