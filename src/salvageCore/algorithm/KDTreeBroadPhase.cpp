/**
 * @file
 * @brief No-op k-d tree strategy lifecycle.
 */
#include "algorithm/KDTreeBroadPhase.hpp"

namespace salvage::core {
    void KDTreeBroadPhase::initialize(std::span<const Body>) {
        // Placeholder: no tree is constructed in this iteration.
    }

    void KDTreeBroadPhase::update(std::span<const Body>) {
        // Placeholder: no tree is updated in this iteration.
    }

    void KDTreeBroadPhase::findCandidates(
        std::span<const Body>, std::vector<CollisionPair>&) const {
        // Placeholder: leave caller-owned output unchanged; do not query a tree.
    }
}
