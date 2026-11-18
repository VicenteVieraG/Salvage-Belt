/**
 * @file
 * @brief k-d tree broad-phase collision detection.
 */
#pragma once

#include <cstddef>
#include "IBroadPhase.hpp"

namespace salvage::core {
    /**
     * @brief Index body bounds in a balanced 3D k-d tree.
     * Initialization and updates rebuild the tree. Queries append each pair
     * of overlapping AABBs once, including pairs that touch.
     */
    class KDTreeBroadPhase final : public IBroadPhase {
        private:
            struct Node {
                BodyId id;
                AABB bounds;
                AABB subtreeBounds; ///< Encloses this body and all its descendants.
            };

            std::vector<Node> nodes; ///< Each subtree occupies a range with its root at the median.

            static AABB build(std::span<Node> subtree, std::size_t axis);
            static void findOverlaps(std::span<const Node> subtree, const Body& body, std::vector<CollisionPair>& out);
        public:
            /// @copydoc IBroadPhase::initialize()
            void initialize(std::span<const Body> bodies) override;

            /// @copydoc IBroadPhase::update()
            void update(std::span<const Body> bodies) override;

            /// @copydoc IBroadPhase::findCandidates()
            void findCandidates(std::span<const Body> bodies, std::vector<CollisionPair>& out) const override;
    };
}
