/**
 * @file
 * @brief Brute-force enumeration of overlapping body bounds.
 */
#include "algorithm/BruteForceBroadPhase.hpp"

#include "CandidateScan.hpp"

namespace salvage::core {
    void BruteForceBroadPhase::initialize(std::span<const Body>) {}

    void BruteForceBroadPhase::update(std::span<const Body>) {}

    void BruteForceBroadPhase::findCandidates(std::span<const Body> bodies, std::vector<CollisionPair>& out) const {
        const auto scanRow = [bodies](std::size_t i, std::vector<CollisionPair>& pairs) {
            const Body& first = bodies[i];

            for (const Body& second : bodies.subspan(i + 1)) {
                if (overlaps(first.aabb(), second.aabb())) {
                    pairs.emplace_back(first.getId(), second.getId());
                }
            }
        };
        scanCandidateRows(bodies.size(), workerCount, out, scanRow);
    }
}
