#pragma once

#include "Body.hpp"
#include <cstddef>
#include <span>
#include <vector>

namespace salvage::core {

class Executor;
class Simulation;

/// Dense scene-local IDs match the indices in this view. Borrow until the next step or reset.
struct WorldView {
    std::span<const Body> bodies;
};

struct Range {
    double min, max;
};

struct FieldSettings {
    std::size_t asteroids{64}; ///< Spheres, first in ID order.
    std::size_t debris{64};    ///< Axis-aligned boxes.
    AABB spawnBounds{{-24, -24, -24}, {24, 24, 24}};
    Range extent{0.5, 1.5};    ///< Radius or half-size along each box axis.
    Range velocity{-4, 4};     ///< Independent components, in world units per second.
    Range inverseMass{0.25, 2};
    Material material{0.6, 0.3};
    std::size_t maxPlacementAttempts{128};
};

/// Explicit scenes are copied and assigned fresh IDs in input order.
using Scene = std::variant<FieldSettings, std::vector<Body>>;

class World {
public:
    World(const Scene& scene, std::uint64_t seed, Executor& executor);
    WorldView view() const noexcept { return {bodies_}; }

private:
    friend class Simulation;
    std::vector<Body> bodies_;
};

}
