#include "salvage/core/simulation/Simulation.hpp"
#include "salvage/core/execution/Executor.hpp"

#include <algorithm>
#include <stdexcept>

namespace salvage::core {
namespace {
using Clock = std::chrono::steady_clock;

SimulationConfig validated(SimulationConfig config) {
    if (!(config.fixedStep > 0) || !std::isfinite(config.fixedStep) || !config.gravity.finite())
        throw std::invalid_argument("Invalid fixed step or gravity");
    if (config.response != Response::resolveContacts && config.response != Response::detectionOnly)
        throw std::invalid_argument("Unknown contact response mode");
    validateSolverSettings(config.solver);
    return config;
}

}

struct Simulation::Run {
    SimulationConfig settings;
    Executor executor;
    SimulationMetrics metrics;
    World world;
    std::vector<Body> next;
    std::unique_ptr<IBroadPhase> strategy;
    DetectionResult detection;

    explicit Run(SimulationConfig config)
        : settings(validated(std::move(config))), executor(settings.workers),
          world(generate()), next(world.view().bodies.size()) {
        const auto start = Clock::now();
        strategy = makeBroadPhase(settings.algorithm);
        strategy->update(world.view());
        metrics.initialization = Clock::now() - start;
        metrics.peakCollisionBufferBytes = strategy->indexBytes();
    }

    World generate() {
        const auto start = Clock::now();
        World initial(settings.scene, settings.seed, executor);
        metrics.generation = Clock::now() - start;
        return initial;
    }
};

Simulation::Simulation(SimulationConfig config) { reset(std::move(config)); }
Simulation::~Simulation() = default;

void Simulation::reset(SimulationConfig config) { run_ = std::make_unique<Run>(std::move(config)); }
void Simulation::reset() { reset(run_->settings); }

void Simulation::restartWith(Algorithm algorithm, std::size_t workers) {
    auto config = run_->settings;
    config.algorithm = algorithm;
    config.workers = workers;
    reset(std::move(config));
}

void Simulation::step(std::span<const BodyInput> inputs) {
    auto& run = *run_;
    const auto& config = run.settings;
    const auto start = Clock::now();
    integrate(run.world.view(), run.next, config.fixedStep, config.gravity, inputs, run.executor);
    const auto integrated = Clock::now();
    auto detection = detect({run.next}, *run.strategy, run.executor);
    const auto detected = Clock::now();
    if (config.response == Response::resolveContacts)
        resolve(run.next, detection.contacts, config.solver);

    const auto resolved = Clock::now();
    const auto previousPairBytes = run.detection.candidates.capacity() * sizeof(CollisionPair);
    run.world.bodies_.swap(run.next);
    run.detection = std::move(detection);
    auto& metrics = run.metrics;
    ++metrics.stepIndex;
    metrics.detection = run.detection.metrics;
    metrics.integration = integrated - start;
    metrics.resolution = config.response == Response::resolveContacts ? resolved - detected : Duration{};
    metrics.peakCollisionBufferBytes = std::max(metrics.peakCollisionBufferBytes,
        previousPairBytes + metrics.detection.indexBytes + metrics.detection.pairBufferBytes);
    metrics.totalStep = Clock::now() - start;
}

const SimulationConfig& Simulation::config() const noexcept { return run_->settings; }
WorldView Simulation::world() const noexcept { return run_->world.view(); }
SimulationMetrics Simulation::metrics() const noexcept { return run_->metrics; }
BroadPhaseDebug Simulation::debug() const { return run_->strategy->debug(); }

SimulationSnapshot Simulation::snapshot() const {
    const auto view = world().bodies;
    return {run_->metrics.stepIndex, static_cast<double>(run_->metrics.stepIndex) * run_->settings.fixedStep,
            {view.begin(), view.end()}, run_->detection.candidates, run_->detection.contacts};
}

}
