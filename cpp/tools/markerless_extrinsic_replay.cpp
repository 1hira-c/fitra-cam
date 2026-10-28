// Fixed-intrinsics raw 2D replay; no cameras, TensorRT or runtime mutation.
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "lift/calib_io.hpp"
#include "lift/markerless_io.hpp"

int main(int argc, char** argv) {
    try {
        std::string intrinsics, observations, output;
        for (int i = 1; i < argc; ++i) {
            const std::string_view argument{argv[i]};
            if (argument == "--help" || argument == "-h") {
                std::puts("markerless_extrinsic_replay --intrinsics PATH --observations PATH [--output NEW.json]\n"
                          "Read raw Halpe26 observations JSONL and estimate relative camera geometry.\n"
                          "No camera/GPU access. Scale and floor remain unknown; diagnostic only.\n"
                          "Existing output files and symlinks are never overwritten.");
                return 0;
            }
            if (i + 1 >= argc) throw std::runtime_error("missing argument value");
            if (argument == "--intrinsics") intrinsics = argv[++i];
            else if (argument == "--observations") observations = argv[++i];
            else if (argument == "--output") output = argv[++i];
            else throw std::runtime_error("unknown argument");
        }
        if (intrinsics.empty() || observations.empty())
            throw std::runtime_error("--intrinsics and --observations are required");
        const auto calibration = fitra::lift::load_calibration(intrinsics);
        std::ifstream input{observations};
        if (!input) throw std::runtime_error("cannot open observations input");
        const auto replay = fitra::lift::read_markerless_observations(input, calibration);
        const auto geometry = fitra::lift::solve_markerless_relative(calibration, replay.frame_sets);
        const auto report = fitra::lift::markerless_relative_report_json(geometry, replay);
        if (!output.empty()) fitra::lift::write_markerless_report_new(output, report);
        std::cout << report << '\n';
        if (!std::cout) throw std::runtime_error("cannot write diagnostic output");
        return geometry.ok && replay.input_clean() ? 0 : 1;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "markerless_extrinsic_replay failed: %s\n", e.what());
        return 1;
    }
}
