#pragma once

#include "salvage-core/collision/Detection.hpp"
#include "salvage-core/physics/Dynamics.hpp"

namespace salvage::core {

enum class Response { resolveContacts, detectionOnly };

struct SimulationConfig {
    std::uint64_t seed{1};
    Scene scene{FieldSettings{}};
    Algorithm algorithm{Algorithm::bruteForce};
    std::size_t workers{1};
    double fixedStep{1.0 / 120.0};
    Vec3 gravity{};
    SolverSettings solver{};
    Response response{Response::resolveContacts};
};

struct SimulationSnapshot {
    std::uint64_t stepIndex{};
    double simulationTime{}; ///< Seconds of simulated time.
    std::vector<Body> bodies; ///< Completed post-response state.
    std::vector<CollisionPair> candidates;
    std::vector<Contact> contacts; ///< Geometry at detection, before response.
};

/// Coordinate from a single thread. A snapshot is independently owned and can be passed to a frontend.
class Simulation {
public:
    explicit Simulation(SimulationConfig config = {});
    ~Simulation();
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    /// Synchronously advance exactly one configured tick. Optional inputs apply only to this tick.
    void step(std::span<const BodyInput> inputs = {});
    void reset(); ///< Cold restart with the same settings, seed and strategy.
    void reset(SimulationConfig config); ///< A failed restart preserves the current run.
    void restartWith(Algorithm algorithm, std::size_t workers);

    const SimulationConfig& config() const noexcept;
    WorldView world() const noexcept;
    /// Contacts detected during the latest tick, before response. Borrow until the next step or reset.
    std::span<const Contact> contacts() const noexcept;
    SimulationSnapshot snapshot() const;
    SimulationMetrics metrics() const noexcept;
    BroadPhaseDebug debug() const; ///< Copy the most recently indexed, pre-response geometry on demand.

private:
    struct Run;
    std::unique_ptr<Run> run_;
};

}
