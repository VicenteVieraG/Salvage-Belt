#include "Benchmark.hpp"

#include <CLI/CLI.hpp>
#include <algorithm>
#include <array>
#include <charconv>
#include <limits>
#include <ranges>
#include <stdexcept>

namespace salvage::headless {
namespace {
using core::Algorithm;
constexpr std::array algorithms{Algorithm::bruteForce, Algorithm::kdTree, Algorithm::sweepAndPrune};
constexpr std::array<std::string_view, 7> scenarios{
    "uniform", "clustered", "heterogeneous", "fast", "slow", "dense", "aligned"
};

// Matrix options take a single comma-separated argument. Empty or repeated entries are errors.
template<class Convert>
auto matrixList(const std::string& input, Convert convert) {
    std::vector<decltype(convert(std::string_view{}))> values;
    if (input.empty()) throw CLI::ValidationError("Expected a nonempty list");
    for (const auto part : input | std::views::split(',')) {
        std::string_view item(part);
        if (item.empty()) throw CLI::ValidationError("Empty entry in list: " + input);
        auto value = convert(item);
        if (std::ranges::find(values, value) != values.end())
            throw CLI::ValidationError("Duplicate entry in list: " + input);
        values.push_back(std::move(value));
    }
    return values;
}

template<class T>
T unsignedDecimal(std::string_view text) {
    T value{};
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size())
        throw CLI::ValidationError("Expected an unsigned decimal integer, got '" + std::string(text) + "'");
    return value;
}

bool sameOutput(const std::filesystem::path& a, const std::filesystem::path& b) {
    if (a.empty() || b.empty()) return false;
    if (a == "-" || b == "-") return a == b;
    return std::filesystem::weakly_canonical(std::filesystem::absolute(a)) ==
           std::filesystem::weakly_canonical(std::filesystem::absolute(b)) ||
           (std::filesystem::exists(a) && std::filesystem::exists(b) && std::filesystem::equivalent(a, b));
}
}

std::string_view name(Algorithm algorithm) {
    switch (algorithm) {
        case Algorithm::bruteForce: return "brute-force";
        case Algorithm::kdTree: return "kd-tree";
        case Algorithm::sweepAndPrune: return "sweep-and-prune";
    }
    throw std::invalid_argument("Unknown broad phase");
}

std::expected<Options, int> parseOptions(int argc, char** argv) {
    Options options;
    CLI::App app{"Compare fixed-step collision runs on the same seeded field.", "salvage-headless"};
    app.footer("Use commas for lists and '-' for one export on standard output.");
    app.option_defaults()->take_last();

    app.add_option_function<std::string>("--algorithm", [&](const std::string& value) {
        if (value == "all") options.algorithms.assign(algorithms.begin(), algorithms.end());
        else options.algorithms = matrixList(value, [](std::string_view item) {
            const auto it = std::ranges::find(algorithms, item, name);
            if (it == algorithms.end()) throw CLI::ValidationError("--algorithm", "Unknown algorithm: " + std::string(item));
            return *it;
        });
    }, "brute-force, kd-tree, sweep-and-prune, or all")->type_name("LIST")->default_str("all");

    app.add_option_function<std::string>("--workers", [&](const std::string& value) {
        options.workers = matrixList(value, unsignedDecimal<std::size_t>);
        if (std::ranges::contains(options.workers, 0u)) throw CLI::ValidationError("--workers", "Must be positive");
    }, "CPU workers, including the caller")->type_name("LIST")->default_str("1");
    app.add_option_function<std::string>("--seed", [&](const std::string& value) {
        options.seeds = matrixList(value, unsignedDecimal<std::uint64_t>);
    }, "Unsigned 64-bit scene seeds")->type_name("LIST")->default_str("1");
    app.add_option_function<std::string>("--scenario", [&](const std::string& value) {
        options.scenarios = matrixList(value, [](std::string_view item) {
            if (!std::ranges::contains(scenarios, item))
                throw CLI::ValidationError("--scenario", "Unknown scenario: " + std::string(item));
            return std::string(item);
        });
    }, "uniform, clustered, heterogeneous, fast, slow, dense, aligned")->type_name("LIST")->default_str("uniform");

    Counts counts = options.sizes.front();
    auto* bodies = app.add_option_function<std::string>("--bodies", [&](const std::string& value) {
        options.sizes = matrixList(value, [](std::string_view item) {
            auto n = unsignedDecimal<std::size_t>(item);
            return Counts{.asteroids = n - n / 2, .debris = n / 2};
        });
    }, "Total bodies, split between spheres and boxes")->type_name("LIST")->default_str("128");

    // CLI11 recognizes C-style integer prefixes. Benchmark counts use base ten, including leading zeros.
    const auto decimal = [](const std::string& value) { return std::to_string(unsignedDecimal<std::size_t>(value)); };
    auto* asteroids = app.add_option("--asteroids", counts.asteroids, "Sphere count")
        ->capture_default_str()->transform(decimal);
    auto* debris = app.add_option("--debris", counts.debris, "Box count")
        ->capture_default_str()->transform(decimal);
    bodies->excludes(asteroids, debris);
    app.add_option("--warmup", options.warmup, "Unmeasured ticks per cold start")->capture_default_str()->transform(decimal);
    app.add_option("--steps", options.steps, "Measured ticks per repetition")
        ->capture_default_str()->transform(decimal)->check(CLI::PositiveNumber);
    app.add_option("--repetitions", options.repetitions, "Independent cold starts per variant")
        ->capture_default_str()->transform(decimal)->check(CLI::PositiveNumber);
    app.add_option("--fixed-step", options.fixedStep, "Simulated seconds per tick (default 1/120)")
        ->check(CLI::PositiveNumber);

    std::string mode{"simulation"}, response{"resolve"}, scale{"volume"};
    app.add_option("--mode", mode, "Full simulation or detection on prerecorded frames")
        ->check(CLI::IsMember({"simulation", "replay"}))->capture_default_str();
    app.add_option("--response", response, "Contact response")
        ->check(CLI::IsMember({"resolve", "detection-only"}))->capture_default_str();
    app.add_option("--scale", scale, "Hold spawn volume or density fixed")
        ->check(CLI::IsMember({"volume", "density"}))->capture_default_str();
    app.add_flag_callback("--no-validate", [&] { options.validate = false; },
                          "Skip the independent all-pairs correctness check");

    const auto path = [](const std::string& value) -> std::string {
        return value.empty() || value.starts_with("--") ? "Expected an output path or '-'" : "";
    };
    app.add_option("--json", options.json, "Export versioned results, samples, summaries and metadata")
        ->type_name("PATH")->check(path);
    app.add_option("--csv", options.csv, "Export one row per measured tick")->type_name("PATH")->check(path);

    try {
        app.parse(argc, argv);
        options.replay = mode == "replay";
        options.response = response == "resolve" ? core::Response::resolveContacts : core::Response::detectionOnly;
        options.fixedDensity = scale == "density";
        if (*asteroids || *debris) options.sizes = {counts};
        constexpr auto maxBodies = std::numeric_limits<core::BodyId>::max();
        for (const auto n : options.sizes)
            if (n.asteroids > maxBodies || n.debris > maxBodies - n.asteroids)
                throw CLI::ValidationError("Too many bodies for scene identifiers");
        if (options.warmup > std::numeric_limits<std::size_t>::max() - options.steps)
            throw CLI::ValidationError("Warmup plus measured steps exceeds the tick range");
        if (sameOutput(options.json, options.csv))
            throw CLI::ValidationError("JSON and CSV need different destinations");
    } catch (const CLI::ParseError& error) {
        return std::unexpected(app.exit(error));
    }
    return options;
}
}
