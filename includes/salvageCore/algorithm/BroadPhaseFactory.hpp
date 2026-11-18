/**
 * @file
 * @brief Selection and construction of broad-phase strategies.
 */
#pragma once

#include <memory>
#include "IBroadPhase.hpp"

namespace salvage::core {
    /// @brief Broad-phase strategies recognized by the factory.
    enum class Algorithm {
        BruteForce, ///< Select the pairwise AABB scan.
        KDTree, ///< Select the k-d tree AABB index.
        SweepAndPrune ///< Select the single-axis AABB sweep.
    };

    /**
     * @brief Create an independently owned strategy for the selected algorithm.
     * @param algorithm Strategy to construct.
     * @return A non-null strategy. Call initialize() before using it.
     * @throws std::invalid_argument If algorithm is not a supported enumerator.
     */
    std::unique_ptr<IBroadPhase> makeBroadPhase(Algorithm algorithm);
}
