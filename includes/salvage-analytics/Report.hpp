#pragma once

#include "benchmark/Statistics.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <vector>

namespace salvage::core {
enum class Algorithm;
struct SimulationConfig;
}

namespace salvage::analytics {

struct ReportProtocol {
    bool replay{};
    std::string response, scale;
    double fixedStep{};
    std::uint64_t warmup{}, steps{}, repetitions{};
    bool operator==(const ReportProtocol&) const = default;
};

struct Sample {
    std::uint64_t tick;
    double seconds, detectionSeconds;
    std::uint64_t examined, candidates, contacts;
};

struct ReportRun {
    std::uint64_t repetition;
    std::vector<Sample> samples;
};

struct ReportVariant {
    std::string algorithm;
    std::uint64_t workers;
    std::string status, validation, error;
    std::optional<benchmark::Stats> timing;
    std::optional<double> speedupVsBrute;
    std::vector<ReportRun> runs;
};

struct SceneKey {
    std::string scenario;
    std::uint64_t seed, bodies, asteroids, debris;
    bool operator==(const SceneKey&) const = default;
};

struct ReportCase {
    SceneKey scene;
    std::vector<ReportVariant> variants;
};

struct SavedReport {
    std::string filename;
    ReportProtocol protocol;
    std::vector<ReportCase> cases;
};

SavedReport loadReport(const std::filesystem::path& path);
std::optional<SavedReport> generateBenchmark(const core::SimulationConfig& config,
                                             std::span<const core::Algorithm> algorithms, std::stop_token stop);
std::string displayText(std::string text);

}
