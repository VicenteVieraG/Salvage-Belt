#pragma once

#include "salvage-core/simulation/Simulation.hpp"

#include <functional>
#include <span>
#include <stop_token>
#include <string>
#include <string_view>
#include <vector>

namespace salvage::benchmark {

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

struct RunSettings {
    std::span<const core::Algorithm> algorithms;
    std::span<const std::size_t> workers;
    std::size_t warmup{10}, steps{60}, repetitions{3};
    bool replay{}, validate{true};
};

struct RunResult {
    core::Duration preparation{};
    std::vector<Variant> variants;
};

// Scenario-specific setup can supply generation time incurred before Simulation is constructed.
using Setup = std::function<core::SimulationConfig(std::size_t workers, core::Duration& extraGeneration)>;

std::string_view algorithmName(core::Algorithm algorithm);
// A stop request ends the run between ticks. The caller should discard the partial result.
RunResult run(const RunSettings& settings, const Setup& setup, std::stop_token stop = {});

}
