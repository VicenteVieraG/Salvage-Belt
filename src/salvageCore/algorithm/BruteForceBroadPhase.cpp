/**
 * @file
 * @brief No-op brute-force strategy lifecycle.
 */
#include "algorithm/BruteForceBroadPhase.hpp"

namespace salvage::core {
    void BruteForceBroadPhase::initialize(std::span<const Body>) {
        // Placeholder: no spatial data is initialized in this iteration.
    }

    void BruteForceBroadPhase::update(std::span<const Body>) {
        // Placeholder: no spatial data is updated in this iteration.
    }

    void BruteForceBroadPhase::findCandidates(
        std::span<const Body>, std::vector<CollisionPair>&) const {
        // Placeholder: leave caller-owned output unchanged; do not enumerate pairs.
    }
}
