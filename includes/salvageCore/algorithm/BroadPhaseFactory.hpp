/**
 * @file
 * @brief Selection and construction of broad-phase strategy placeholders.
 */
#pragma once

#include <memory>
#include "IBroadPhase.hpp"

namespace salvage::core {
    /// @brief Broad-phase strategies available for interface review.
    enum class Algorithm {
        BruteForce, ///< Select BruteForceBroadPhase; currently a no-op placeholder.
        KDTree ///< Select KDTreeBroadPhase; currently a no-op placeholder.
    };

    /**
     * @brief Create an independently owned strategy for the selected algorithm.
     * @param algorithm Strategy to construct.
     * @return A non-null placeholder strategy; call initialize() before using it.
     * @throws std::invalid_argument If algorithm is not a supported enumerator.
     * @note Does not select or replace the broad phase used by World.
     */
    std::unique_ptr<IBroadPhase> makeBroadPhase(Algorithm algorithm);
}
