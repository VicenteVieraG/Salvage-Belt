#include "salvage-headless/Benchmark.hpp"
#include "salvage-core/execution/Executor.hpp"

#include <chrono>
#include <cmath>
#include <utility>

namespace salvage::headless {
namespace {
using namespace core;
using Clock = std::chrono::steady_clock;

FieldSettings fieldFor(const Options& options, const std::string& scenario, Counts counts) {
    FieldSettings field;
    field.asteroids = counts.asteroids;
    field.debris = counts.debris;
    if (scenario == "heterogeneous") field.extent = {0.1, 2.5};
    if (scenario == "fast") field.velocity = {-40, 40};
    if (scenario == "slow") field.velocity = {-0.1, 0.1};
    if (options.fixedDensity && counts.asteroids + counts.debris > 0) {
        const double scale = std::cbrt(static_cast<double>(counts.asteroids + counts.debris) / 128.0);
        field.spawnBounds.min = field.spawnBounds.min * scale;
        field.spawnBounds.max = field.spawnBounds.max * scale;
    }
    return field;
}

SimulationConfig configFor(const Case& scene, const Options& options, std::size_t workers,
                           Duration& extraGeneration) {
    SimulationConfig config;
    config.workers = workers;
    config.seed = scene.seed;
    config.scene = scene.field;
    config.fixedStep = options.fixedStep;
    config.response = options.response;
    if (scene.scenario == "clustered" || scene.scenario == "aligned" || scene.scenario == "dense") {
        Executor executor(workers);
        const auto start = Clock::now();
        World source(scene.field, scene.seed, executor);
        const auto initial = source.view().bodies;
        std::vector<Body> bodies(initial.begin(), initial.end());
        for (auto& body : bodies) {
            auto p = body.getPosition();
            if (scene.scenario == "aligned") p = {p.x, 0, 0};
            else if (scene.scenario == "dense") p = p * 0.16;
            else {
                p = p * 0.25;
                p.x += std::copysign(scene.field.spawnBounds.max.x * 0.55, p.x);
            }
            body.setPosition(p);
        }
        config.scene = std::move(bodies);
        extraGeneration = Clock::now() - start;
    }
    return config;
}

}

Case benchmark(const Options& options, const std::string& scenario, Counts counts, std::uint64_t seed) {
    Case scene{scenario, fieldFor(options, scenario, counts), seed, {}, {}};
    const salvage::benchmark::RunSettings settings{options.algorithms, options.workers, options.warmup,
        options.steps, options.repetitions, options.replay, options.validate};
    auto result = salvage::benchmark::run(settings, [&](std::size_t workers, core::Duration& extraGeneration) {
        return configFor(scene, options, workers, extraGeneration);
    });
    scene.preparation = result.preparation;
    scene.variants = std::move(result.variants);
    return scene;
}
}
