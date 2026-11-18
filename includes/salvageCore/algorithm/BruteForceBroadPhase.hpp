/**
 * @file
 * @brief Brute-force strategy placeholder for interface review.
 */
#pragma once

#include "IBroadPhase.hpp"

namespace salvage::core {
    /**
     * @brief Placeholder for a future brute-force broad-phase strategy.
     * Every operation is a no-op. World's existing collision query remains
     * separate until the strategy interface is integrated.
     */
    class BruteForceBroadPhase final : public IBroadPhase {
        public:
            /// @copydoc IBroadPhase::initialize()
            void initialize(std::span<const Body> bodies) override;

            /// @copydoc IBroadPhase::update()
            void update(std::span<const Body> bodies) override;

            /// @copydoc IBroadPhase::findCandidates()
            void findCandidates(std::span<const Body> bodies, std::vector<CollisionPair>& out) const override;
    };
}
