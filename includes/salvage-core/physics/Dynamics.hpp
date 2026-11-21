#pragma once

#include "ContactSolver.hpp"
#include "salvage-core/world/World.hpp"

namespace salvage::core {

/// Applied once, at the beginning of a tick. Force is constant for the fixed step.
struct BodyInput {
    BodyId body{};
    Vec3 force{}, impulse{};
};

void validateSolverSettings(const SolverSettings& settings);

/// Write the integrated state into independent output slots, including refreshed collider bounds.
void integrate(WorldView current, std::span<Body> next, double dt, Vec3 gravity,
               std::span<const BodyInput> inputs, Executor& executor);

/// Apply impulses in stable contact order, then revisit those pairs to reduce penetration.
/// Bodies must be in dense ID order, and contacts must reference valid, distinct bodies.
void resolve(std::span<Body> bodies, std::span<const Contact> contacts, const SolverSettings& settings);

}
