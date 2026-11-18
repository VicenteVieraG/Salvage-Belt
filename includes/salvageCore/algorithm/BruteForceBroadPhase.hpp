/**
 * @file
 * @brief Brute-force broad-phase collision detection.
 */
#pragma once

#include "IBroadPhase.hpp"

namespace salvage::core {
    /**
     * @brief Test each unordered pair of bodies for inclusive AABB overlap.
     * Each query reads the supplied body bounds directly in O(n²) time.
     * The strategy is stateless, so initialize() and update() require no work.
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
