/**
 * @file
 * @brief Brute-force broad-phase collision detection.
 */
#pragma once

#include "IBroadPhase.hpp"

namespace salvage::core {
    /**
     * @brief Test each unordered pair of bodies for inclusive AABB overlap.
     * Emits pairs in ascending input-index order, with the earlier body first.
     * Each query reads the supplied body bounds directly in O(n²) time.
     * The strategy stores no body data, so initialize() and update() require no work.
     */
    class BruteForceBroadPhase final : public IBroadPhase {
        private:
            std::size_t workerCount{1};

        public:
            BruteForceBroadPhase() = default;
            /// @brief Limit scan workers. Zero uses hardware concurrency.
            explicit BruteForceBroadPhase(std::size_t workers) : workerCount(workers) {}

            /// @copydoc IBroadPhase::initialize()
            void initialize(std::span<const Body> bodies) override;

            /// @copydoc IBroadPhase::update()
            void update(std::span<const Body> bodies) override;

            /// @copydoc IBroadPhase::findCandidates()
            void findCandidates(std::span<const Body> bodies, std::vector<CollisionPair>& out) const override;
    };
}
