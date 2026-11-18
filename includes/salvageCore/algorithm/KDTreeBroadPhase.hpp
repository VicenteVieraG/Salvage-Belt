/**
 * @file
 * @brief k-d tree strategy placeholder for interface review.
 */
#pragma once

#include "IBroadPhase.hpp"

namespace salvage::core {
    /**
     * @brief Placeholder for a future k-d tree broad-phase strategy.
     * Every operation is a no-op; no tree is built or queried.
     */
    class KDTreeBroadPhase final : public IBroadPhase {
        public:
            /// @copydoc IBroadPhase::initialize()
            void initialize(std::span<const Body> bodies) override;

            /// @copydoc IBroadPhase::update()
            void update(std::span<const Body> bodies) override;

            /// @copydoc IBroadPhase::findCandidates()
            void findCandidates(std::span<const Body> bodies, std::vector<CollisionPair>& out) const override;
    };
}
