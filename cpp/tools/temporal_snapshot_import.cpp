// CPU-only diagnostic import of asynchronous /stats snapshots. This cannot
// reproduce the original live synchronization or manufacture capture lineage.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "lift/calib_io.hpp"
#include "lift/keypoint_format.hpp"
#include "lift/triangulator.hpp"

namespace {
using namespace fitra;
void require(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}
std::string quote(const std::string& value) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c) << std::dec;
        else out << c;
    }
    out << '"';
    return out.str();
}
std::uint64_t fingerprint(std::istream& in) {
    in.clear(); in.seekg(0);
    require(in.good(), "cannot rewind input");
    std::uint64_t value = 14695981039346656037ULL;
    for (char c; in.get(c);) {
        value ^= static_cast<unsigned char>(c);
        value *= 1099511628211ULL;
    }
    require(in.eof(), "input read failed");
    in.clear(); in.seekg(0);
    return value;
}
template<class T>
void nullable_array(std::ostream& out, const std::array<std::optional<T>, 3>& values) {
    out << '[';
    for (std::size_t c = 0; c < values.size(); ++c) {
        if (c) out << ',';
        if (values[c]) out << *values[c]; else out << "null";
    }
    out << ']';
}
void write_joints(std::ostream& out, const lift::TriangulatedSkeleton& tri) {
    out << '[';
    for (int k = 0; k < 26; ++k) {
        if (k) out << ',';
        const auto& j = tri.skeleton.joints[k];
        if (!j.valid) { out << "null"; continue; }
        require(std::isfinite(j.x) && std::isfinite(j.y) && std::isfinite(j.z)
                    && std::isfinite(j.score), "nonfinite triangulated joint");
        out << '[' << j.x << ',' << j.y << ',' << j.z << ',' << j.score << ']';
    }
    out << ']';
}
void write_quality(std::ostream& out, const lift::TriangulatedSkeleton& tri) {
    out << '[';
    for (int k = 0; k < 26; ++k) {
        if (k) out << ',';
        if (k == 0 || !tri.skeleton.joints[k].valid) { out << "null"; continue; }
        require(std::isfinite(tri.reproj_error_px[k]) && std::isfinite(tri.max_ray_angle_deg[k]),
                "nonfinite triangulation quality");
        out << "{\"views\":" << tri.view_count[k] << ",\"reproj_px\":" << tri.reproj_error_px[k]
            << ",\"ray_angle_deg\":" << tri.max_ray_angle_deg[k] << '}';
    }
    out << ']';
}
double option_number(const std::string& value) {
    std::size_t used = 0;
    const double number = std::stod(value, &used);
    require(used == value.size() && std::isfinite(number), "invalid numeric option");
    return number;
}
} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--help") {
            std::cout << "temporal_snapshot_import INPUT.jsonl CALIB.yaml OUTPUT.jsonl [--max-skew-ms 15] [--omit-camera INDEX] [--height 1.7] [--nominal-hz 30]\n"
                         "CPU diagnostic import using production Triangulator. Asynchronous HTTP snapshots are not original live 3D or kernel-synchronized capture.\n"
                         "--omit-camera is a counterfactual only; all original timestamps and boundary conditions remain. Output must not exist.\n";
            return 0;
        }
        require(argc >= 4 && (argc - 4) % 2 == 0, "usage: temporal_snapshot_import INPUT CALIB OUTPUT [OPTIONS]");
        double max_skew = 15, height = 1.7, nominal_hz = 30;
        int omit = -1;
        std::vector<std::string> seen_options;
        for (int a = 4; a < argc; a += 2) {
            const std::string name = argv[a], value = argv[a + 1];
            require(std::find(seen_options.begin(), seen_options.end(), name) == seen_options.end(), "duplicate option");
            seen_options.push_back(name);
            const double number = option_number(value);
            if (name == "--max-skew-ms") max_skew = number;
            else if (name == "--height") height = number;
            else if (name == "--nominal-hz") nominal_hz = number;
            else if (name == "--omit-camera") {
                require(number >= 0 && number <= 2 && std::floor(number) == number, "omit-camera must be 0, 1, or 2");
                omit = static_cast<int>(number);
            } else throw std::runtime_error("unknown option: " + name);
        }
        require(max_skew >= 0 && max_skew <= 100, "max-skew-ms must be in [0,100]");
        require(height >= 1 && height <= 2.5 && nominal_hz >= 5 && nominal_hz <= 240, "invalid height/cadence");
        require(!std::filesystem::exists(argv[3]), "output exists");
        std::ifstream input(argv[1]), calib_bytes(argv[2], std::ios::binary);
        require(input.good() && calib_bytes.good(), "cannot open source/calibration");
        const auto input_binding = fingerprint(input), calib_binding = fingerprint(calib_bytes);
        const std::vector<std::string> ids{"cam0", "cam1", "cam2"};
        const auto calib = lift::select_calib_cameras(lift::load_calibration(argv[2]), ids);
        lift::Triangulator triangulator(calib);
        triangulator.require_camera_ids(ids);
        lift::set_active_keypoint_format(lift::KeypointFormat::Halpe26);

        // Validate/render before creating the output, so malformed data does not
        // leave an apparently usable partial replay. The trace is diagnostic,
        // not a hot-path recorder, and no input bytes are changed.
        std::ostringstream output;
        output << std::setprecision(9);
        output << "{\"schema\":\"fitra_temporal_input_v1\",\"input_stage\":\"raw_triangulated\","
                  "\"kp_format\":\"halpe26\",\"coordinate_frame\":\"fitra_world_z_up_m\","
                  "\"time_basis\":\"measured_report\",\"reference_kind\":\"none\",\"nominal_hz\":" << nominal_hz
               << ",\"subject_height_m\":" << height << ",\"provenance\":{\"source\":"
               << quote(std::filesystem::absolute(argv[1]).lexically_normal().string())
               << ",\"source_fnv1a64\":\"" << input_binding << "\",\"calibration\":"
               << quote(std::filesystem::absolute(argv[2]).lexically_normal().string())
               << ",\"calibration_fnv1a64\":\"" << calib_binding << "\","
                  "\"observation_kind\":\"approximate_asynchronous_http_snapshot\","
                  "\"time_source\":\"observed_mono_ns_integer_difference\","
                  "\"skew_source\":\"estimated_wall_capture_ms\","
                  "\"camera_sync\":\"not_kernel_or_hardware_synchronized\","
                  "\"original_live_3d_reproduced\":false,\"live_schedule_emulated\":false,"
                  "\"raw_observations_modified\":false,\"counterfactual\":" << (omit >= 0 ? "true" : "false")
               << ",\"options\":{\"max_skew_ms\":" << max_skew << ",\"omit_camera\":";
        if (omit >= 0) output << omit; else output << "null";
        output << ",\"height_m\":" << height << ",\"nominal_hz\":" << nominal_hz << "}}}\n";

        std::optional<std::int64_t> origin, previous_observed;
        std::optional<std::array<std::optional<std::uint64_t>, 3>> previous_processed;
        std::size_t source_rows = 0, emitted = 0, duplicates = 0;
        std::string text;
        while (std::getline(input, text)) {
            const auto row = YAML::Load(text), snapshot = row["snapshot"];
            require(snapshot.IsMap() && snapshot["kp_format"].as<std::string>() == "halpe26", "Halpe26 snapshot required");
            const auto observed = row["observed_mono_ns"].as<std::int64_t>();
            const auto done = row["read_done_mono_ns"].as<std::int64_t>();
            require(observed >= 0 && done >= observed && (!previous_observed || observed > *previous_observed),
                    "observed timestamps must strictly increase and read completion must follow observation");
            if (!origin) origin = observed;
            previous_observed = observed;
            ++source_rows;
            const auto source_seq = snapshot["seq"].as<std::uint64_t>();
            require(snapshot["cameras"].IsSequence(), "camera array required");
            std::array<std::optional<std::uint64_t>, 3> processed{};
            std::array<std::optional<std::int64_t>, 3> capture_ms{};
            std::array<infer::Person, 3> people{};
            std::array<bool, 3> present{}, single_person{};
            std::string boundary;
            auto reject = [&](const char* reason) { if (boundary.empty()) boundary = reason; };
            for (const auto& camera : snapshot["cameras"]) {
                const int c = camera["id"].as<int>();
                require(c >= 0 && c < 3 && !present[c], "unexpected/duplicate camera ID");
                present[c] = true;
                require(camera["w"].as<int>() == calib.cameras[c].intrinsics.width &&
                            camera["h"].as<int>() == calib.cameras[c].intrinsics.height,
                        "snapshot/calibration resolution mismatch");
                const auto stats = camera["stats"];
                processed[c] = stats["processed"].as<std::uint64_t>();
                if (stats["captured_at_ms"] && !stats["captured_at_ms"].IsNull()) {
                    capture_ms[c] = stats["captured_at_ms"].as<std::int64_t>();
                    if (*capture_ms[c] <= 0) reject("missing_capture_timestamp");
                } else reject("missing_capture_timestamp");
                if (previous_processed && (*previous_processed)[c] && *processed[c] < *(*previous_processed)[c])
                    reject("source_counter_regression");
                const auto persons = camera["persons"];
                require(persons.IsSequence(), "persons array required");
                if (persons.size() != 1) {
                    reject(persons.size() == 0 ? "no_person" : "ambiguous_detection");
                    continue;
                }
                single_person[c] = true;
                people[c].kp_count = 26;
                const auto points = persons[0]["kpts"];
                require(points.IsSequence() && points.size() == 26, "expected 26 Halpe keypoints");
                for (int k = 0; k < 26; ++k) {
                    require(points[k].IsSequence() && points[k].size() == 3, "invalid 2D keypoint");
                    const float x = points[k][0].as<float>(), y = points[k][1].as<float>(), score = points[k][2].as<float>();
                    require(std::isfinite(x) && std::isfinite(y) && std::isfinite(score) && score >= 0, "invalid 2D value");
                    people[c].kpts[k] = {x, y, score};
                }
            }
            for (bool have : present) if (!have) reject("missing_camera");
            std::optional<std::int64_t> skew;
            if (std::all_of(capture_ms.begin(), capture_ms.end(), [](const auto& t) { return t && *t > 0; })) {
                auto low = *capture_ms[0], high = low;
                for (const auto& t : capture_ms) { low = std::min(low, *t); high = std::max(high, *t); }
                skew = high - low;
                if (*skew > max_skew) reject("excessive_source_skew");
            }
            lift::TriangulatedSkeleton tri;
            if (boundary.empty()) {
                std::vector<lift::PerCameraObservation> observations;
                for (int c = 0; c < 3; ++c) if (c != omit && single_person[c]) observations.push_back({c, &people[c]});
                tri = triangulator.triangulate(observations);
                if (!tri.valid_joints) reject("no_triangulated_body");
            }
            const bool fresh = boundary.empty();
            // A boundary invalidates downstream history even if camera counters
            // did not move. Only a still-usable duplicate may be skipped.
            if (fresh && previous_processed && processed == *previous_processed) { ++duplicates; continue; }
            previous_processed = processed;
            output << "{\"seq\":" << emitted++ << ",\"t_ns\":" << observed - *origin
                   << ",\"segment\":\"http_snapshot\",\"state\":\"" << (fresh ? "fresh" : "boundary")
                   << "\",\"phase\":\"unlabelled\",\"reference_joints\":null,\"joints\":";
            if (fresh) write_joints(output, tri); else output << "null";
            output << ",\"boundary_reason\":";
            if (fresh) output << "null"; else output << quote(boundary);
            output << ",\"source_snapshot_seq\":" << source_seq << ",\"source_observed_mono_ns\":\"" << observed
                   << "\",\"source_read_done_mono_ns\":\"" << done << "\",\"camera_processed\":";
            nullable_array(output, processed);
            output << ",\"camera_capture_wall_ms\":";
            nullable_array(output, capture_ms);
            output << ",\"source_skew_ms\":";
            if (skew) output << *skew; else output << "null";
            output << ",\"valid_joints\":" << (fresh ? tri.valid_joints : 0) << ",\"median_reproj_px\":";
            if (fresh) output << tri.median_reproj_px; else output << "null";
            output << ",\"joint_quality\":";
            if (fresh) write_quality(output, tri); else output << "null";
            output << "}\n";
        }
        require(input.eof() && source_rows > 0 && emitted > 0, "empty/incomplete snapshot trace");
        require(input_binding == fingerprint(input) && calib_binding == fingerprint(calib_bytes), "source/calibration changed during import");
        const auto encoded = output.str();
        std::FILE* file = std::fopen(argv[3], "wx");
        require(file != nullptr, "cannot exclusively create output");
        const auto count = std::fwrite(encoded.data(), 1, encoded.size(), file);
        const int closed = std::fclose(file);
        require(count == encoded.size() && closed == 0, "output write failed");
        std::cerr << "imported " << emitted << " reports; skipped " << duplicates << " unchanged camera-counter snapshots; diagnostic only\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "temporal_snapshot_import: " << error.what() << '\n';
        return 1;
    }
}
