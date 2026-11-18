/**
 * @file
 * @brief Brute-force enumeration of overlapping body bounds.
 */
#include "algorithm/BruteForceBroadPhase.hpp"

#include <cstddef>

namespace salvage::core {
    void BruteForceBroadPhase::initialize(std::span<const Body>) {}

    void BruteForceBroadPhase::update(std::span<const Body>) {}

    void BruteForceBroadPhase::findCandidates(
        std::span<const Body> bodies, std::vector<CollisionPair>& out) const {
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            const Body& first = bodies[i];
            for (const Body& second : bodies.subspan(i + 1)) {
                if (overlaps(first.aabb(), second.aabb())) {
                    out.emplace_back(first.getId(), second.getId());
                }
            }
        }
    }
}
