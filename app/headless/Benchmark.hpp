#pragma once

#include "salvage/core/simulation/Simulation.hpp"

#include <expected>
#include <filesystem>
#include <iosfwd>
#include <string>
#include <string_view>

namespace salvage::headless {

struct Counts {
    std::size_t asteroids{}, debris{};
    bool operator==(const Counts&) const = default;
};

struct Options {
    std::vector<core::Algorithm> algorithms{core::Algorithm::bruteForce, core::Algorithm::kdTree,
                                            core::Algorithm::sweepAndPrune};
    std::vector<std::size_t> workers{1};
    std::vector<std::uint64_t> seeds{1};
    std::vector<Counts> sizes{{64, 64}};
    std::vector<std::string> scenarios{"uniform"};
    std::size_t warmup{10}, steps{60}, repetitions{3};
    double fixedStep{core::SimulationConfig{}.fixedStep};
    core::Response response{core::Response::resolveContacts};
    bool replay{}, fixedDensity{}, validate{true};
    std::filesystem::path json, csv;
};

// A CLI11 help request or parse error supplies the process exit code.
std::expected<Options, int> parseOptions(int argc, char** argv);
std::string_view name(core::Algorithm algorithm);

struct Trial {
    std::size_t repetition{}, order{};
    core::Duration generation{}, initialization{};
    std::vector<core::SimulationMetrics> samples;
};

struct Variant {
    core::Algorithm algorithm;
    std::size_t workers;
    std::string validation;
    std::string error;
    std::vector<Trial> trials;
};

struct Case {
    std::string scenario;
    core::FieldSettings field;
    std::uint64_t seed;
    core::Duration preparation{};
    std::vector<Variant> variants;
};

Case benchmark(const Options& options, const std::string& scenario, Counts counts, std::uint64_t seed);
bool report(const Options& options, const std::vector<Case>& cases, std::ostream& summary);

}
