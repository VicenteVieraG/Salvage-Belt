/**
 * @file
 * @brief Sweep-and-prune broad-phase collision detection.
 */
#pragma once

#include <cstddef>
#include "IBroadPhase.hpp"

namespace salvage::core {
    /// @brief Sweep body bounds in order of their minimum x coordinate.
    class SweepAndPruneBroadPhase final : public IBroadPhase {
        private:
            std::vector<std::size_t> order; ///< Indices into the current body collection, in sweep order.

        public:
            /// @copydoc IBroadPhase::initialize()
            void initialize(std::span<const Body> bodies) override;

            /// @copydoc IBroadPhase::update()
            void update(std::span<const Body> bodies) override;

            /// @copydoc IBroadPhase::findCandidates()
            void findCandidates(std::span<const Body> bodies, std::vector<CollisionPair>& out) const override;
    };
}
