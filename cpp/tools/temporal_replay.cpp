// Offline, one update per source report. Uses production filter/geometry code;
// does not emulate capture, the paced TrackerExtractor thread, or SteamVR.
#include <cmath>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

#include "lift/kalman.hpp"
#include "lift/ik.hpp"
#include "lift/floor_contact_stabilizer.hpp"
#include "tracking/tracker_extractor.hpp"

using namespace fitra;

namespace {
void require(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}

infer::Skeleton3D skeleton(const YAML::Node& values) {
    require(values.IsSequence() && values.size() == 26, "expected 26 Halpe joints");
    infer::Skeleton3D result;
    result.kp_count = 26;
    for (std::size_t i = 0; i < 26; ++i) {
        const auto v = values[i];
        if (v.IsNull()) continue;
        require(v.IsSequence() && v.size() == 4, "joint must be null or [x,y,z,score]");
        auto& j = result.joints[i];
        j = {v[0].as<float>(), v[1].as<float>(), v[2].as<float>(), v[3].as<float>(), true};
        require(std::isfinite(j.x) && std::isfinite(j.y) && std::isfinite(j.z)
                    && std::isfinite(j.score) && j.score >= 0 && j.score <= 1,
                "nonfinite joint or invalid score");
    }
    return result;
}

using Trackers = std::array<tracking::TrackerPose, tracking::kTrackerCount>;

void write_trackers(std::ostream& out, const Trackers& trackers) {
    out << '[';
    for (std::size_t i = 0; i < trackers.size(); ++i) {
        if (i) out << ',';
        const auto& t = trackers[i];
        if (!t.valid) { out << "null"; continue; }
        out << "{\"position_m\":[";
        for (int c = 0; c < 3; ++c) {
            require(std::isfinite(t.pos[c]), "nonfinite output position");
            if (c) out << ',';
            out << t.pos[c];
        }
        out << "],\"quat_wxyz\":[";
        for (int c = 0; c < 4; ++c) {
            require(std::isfinite(t.quat_wxyz[c]), "nonfinite output rotation");
            if (c) out << ',';
            out << t.quat_wxyz[c];
        }
        out << "]}";
    }
    out << ']';
}

struct Candidate {
    bool use_kalman, use_euro;
    lift::SkeletonKalman kalman;
    lift::IkSolver ik;
    lift::FloorContactStabilizer floor;
    tracking::TrackerExtractorOptions opts;
    tracking::ExtractContext extract;
    tracking::PosSmoothingContext pos;
    tracking::QuatSmoothingContext quat;
    std::array<cv::Vec3f, tracking::kTrackerCount> prev_pos{};
    std::array<cv::Vec4f, tracking::kTrackerCount> prev_quat{};

    Candidate(bool k, bool e, const lift::IkSolver::Options& io)
        : use_kalman(k), use_euro(e), ik(io) { reset(); }

    void reset() {
        kalman.reset();
        floor.reset();
        extract = {}; pos = {}; quat = {};
        prev_pos = {};
        for (auto& q : prev_quat) q = {1, 0, 0, 0};
    }

    Trackers update(infer::Skeleton3D s, double dt, double nominal) {
        if (use_kalman) s = kalman.update(s, dt);
        s = ik.update(s);
        const auto report = floor.update(s, dt);
        auto t = tracking::extract_trackers_with_floor_corrections(
            s, {report.feet[0].correction_m, report.feet[1].correction_m},
            &extract, opts.foot_pos_mode, opts.chest_height_frac,
            opts.waist_height_frac, opts.limb_extension);
        pos.dt_s = static_cast<float>(dt);
        const auto& hip = s.joints[19];
        pos.hip_valid = hip.valid;
        pos.current_hip_pos = {hip.x, hip.y, hip.z};
        if (use_euro) {
            tracking::apply_quat_smoothing(t, prev_quat, quat, opts.quat_one_euro, dt, nominal);
            tracking::apply_pos_smoothing(t, prev_pos, pos, opts.pos_one_euro, nominal);
        } else {
            tracking::apply_quat_smoothing(t, prev_quat, 1.0f, dt, nominal);
            tracking::apply_pos_smoothing(t, prev_pos, pos, 1.0f, nominal);
        }
        return t;
    }
};
} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--help") {
            std::cout << "temporal_replay INPUT.jsonl OUTPUT.jsonl [SUBJECT_PROFILE.yaml]\n"
                         "Input: fitra_temporal_input_v1 header, then timestamped Halpe26 reports.\n"
                         "Without a profile, header.subject_height_m supplies the IK prior.\n"
                         "CPU offline evaluation only; output must not already exist.\n";
            return 0;
        }
        require(argc == 3 || argc == 4, "usage: temporal_replay INPUT OUTPUT [SUBJECT_PROFILE]");
        std::ifstream input(argv[1]);
        require(input.good(), "cannot open input");
        require(!std::ifstream(argv[2]).good(), "output already exists; choose a new path");
        // Bind the replay to its exact input bytes (accidental mix-up detection;
        // the scoring report additionally records SHA-256 artifact hashes).
        std::uint64_t input_fingerprint = 14695981039346656037ULL;
        for (char c; input.get(c);) {
            input_fingerprint ^= static_cast<unsigned char>(c);
            input_fingerprint *= 1099511628211ULL;
        }
        require(input.eof(), "input read failure");
        input.clear(); input.seekg(0);
        std::string line;
        require(static_cast<bool>(std::getline(input, line)), "missing header");
        const auto header = YAML::Load(line);
        require(header["schema"].as<std::string>() == "fitra_temporal_input_v1", "unsupported schema");
        require(header["input_stage"].as<std::string>() == "raw_triangulated", "input must be pre-filter 3D");
        require(header["kp_format"].as<std::string>() == "halpe26", "Halpe26 required");
        require(header["coordinate_frame"].as<std::string>() == "fitra_world_z_up_m", "wrong coordinate frame");
        const auto basis = header["time_basis"].as<std::string>();
        require(basis == "synthetic_nominal" || basis == "nominal_from_legacy_frames" || basis == "measured_report",
                "unsupported time basis");
        const auto reference_kind = header["reference_kind"].as<std::string>();
        require(reference_kind == "synthetic_known" || reference_kind == "pseudo" || reference_kind == "none",
                "unsupported reference kind");
        const double hz = header["nominal_hz"].as<double>();
        require(std::isfinite(hz) && hz >= 5 && hz <= 240, "invalid nominal_hz");
        const double nominal = 1.0 / hz;
        lift::set_active_keypoint_format(lift::KeypointFormat::Halpe26);
        lift::IkSolver::Options io;
        io.subject_height_m = header["subject_height_m"].as<double>();
        require(std::isfinite(io.subject_height_m) && io.subject_height_m >= 1 && io.subject_height_m <= 2.5,
                "invalid subject height");
        if (argc == 4) {
            io.subject_profile = lift::load_subject_profile(argv[3]);
            io.has_subject_profile = true;
        }
        std::array<std::unique_ptr<Candidate>, 3> candidates{
            std::make_unique<Candidate>(false, false, io),
            std::make_unique<Candidate>(true, false, io),
            std::make_unique<Candidate>(true, true, io)};
        const std::array<const char*, 3> names{"minimal", "kalman", "current"};
        tracking::TrackerExtractorOptions opts;
        std::ofstream output(argv[2]);
        require(output.good(), "cannot open output");
        output << std::setprecision(9);
        output << "{\"schema\":\"fitra_temporal_output_v1\",\"schedule\":\"one_step_per_report\","
                  "\"input_fnv1a64\":\"" << input_fingerprint << "\","
                  "\"candidates\":[\"minimal\",\"kalman\",\"current\"],"
                  "\"common\":{\"ik\":true,\"floor\":true,\"confidence_gates\":true},"
                  "\"nominal_hz\":" << hz << ",\"profile_loaded\":" << (argc == 4 ? "true" : "false")
               << ",\"subject_height_prior_m\":" << io.subject_height_m
               << ",\"pos_one_euro\":[" << opts.pos_one_euro.mincutoff << ',' << opts.pos_one_euro.beta << ',' << opts.pos_one_euro.dcutoff
               << "],\"quat_one_euro\":[" << opts.quat_one_euro.mincutoff << ',' << opts.quat_one_euro.beta << ',' << opts.quat_one_euro.dcutoff
               << "],\"foot_position\":\"ankle\",\"chest_height_frac\":" << opts.chest_height_frac
               << ",\"waist_height_frac\":" << opts.waist_height_frac << "}\n";
        std::int64_t previous_time = -1, previous_seq = -1;
        std::string previous_segment;
        bool previous_fresh = false;
        std::size_t rows = 0;
        while (std::getline(input, line)) {
            require(!line.empty(), "blank input row");
            const auto row = YAML::Load(line);
            const auto t = row["t_ns"].as<std::int64_t>();
            const auto seq = row["seq"].as<std::int64_t>();
            const auto segment = row["segment"].as<std::string>();
            const auto state = row["state"].as<std::string>();
            require(t >= 0 && t > previous_time, "timestamps must strictly increase; deduplicate source reports");
            require(seq >= 0 && (segment != previous_segment || seq > previous_seq), "nonincreasing source seq");
            require(!segment.empty(), "empty continuity segment");
            require(state == "fresh" || state == "boundary", "unsupported state");
            const bool fresh = state == "fresh";
            const double elapsed = previous_time < 0 ? nominal : (t - previous_time) * 1e-9;
            const bool reset = !fresh || !previous_fresh || segment != previous_segment || elapsed > 0.5;
            if (reset) {
                for (auto& candidate : candidates) candidate->reset();
            }
            const double dt = reset ? nominal : elapsed;
            // No reference data ever enters candidate state or input normalization.
            Trackers reference{};
            if (fresh && row["reference_joints"] && !row["reference_joints"].IsNull()) {
                require(reference_kind != "none", "reference joints present but reference_kind is none");
                // Reference holes must not be filled by historical FK anchors.
                reference = tracking::extract_trackers(skeleton(row["reference_joints"]), nullptr,
                    opts.foot_pos_mode, opts.chest_height_frac, opts.waist_height_frac, opts.limb_extension);
            }
            output << "{\"seq\":" << seq << ",\"t_ns\":" << t
                   << ",\"reset\":" << (reset ? "true" : "false")
                   << ",\"reference\":";
            write_trackers(output, reference);
            output << ",\"candidates\":{";
            const auto raw = fresh ? skeleton(row["joints"]) : infer::Skeleton3D{};
            std::array<double, 3> processing_ms{};
            for (std::size_t i = 0; i < candidates.size(); ++i) {
                if (i) output << ',';
                output << '"' << names[i] << "\":";
                const auto start = std::chrono::steady_clock::now();
                const auto trackers = fresh ? candidates[i]->update(raw, dt, nominal) : Trackers{};
                processing_ms[i] = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-start).count();
                write_trackers(output, trackers);
            }
            output << "},\"processing_ms\":{";
            for (std::size_t i = 0; i < candidates.size(); ++i) {
                if (i) output << ',';
                output << '"' << names[i] << "\":" << processing_ms[i];
            }
            output << "}}\n";
            require(output.good(), "output write failed");
            previous_time = t; previous_seq = seq; previous_segment = segment; previous_fresh = fresh;
            ++rows;
        }
        require(rows > 0 && input.eof(), "empty data or input read failure");
        output.close();
        require(output.good(), "output close failed");
        std::cerr << "replayed " << rows << " reports (offline, no live latency claim)\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "temporal_replay: " << e.what() << '\n';
        return 1;
    }
}
