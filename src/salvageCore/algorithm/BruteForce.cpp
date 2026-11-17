/**
 * @file
 * @brief Brute-force broad-phase collision queries.
 */
#include "algorithm/BruteForce.hpp"
#include "physics/AABB.hpp"

#include <cstddef>

namespace salvage::core::BruteForce {
    std::vector<BodyPair> broadPhase(std::span<const Body> bodies) {
        std::vector<BodyPair> pairs;

        for (std::size_t i = 0; i < bodies.size(); i++) {
            for (std::size_t j = i + 1; j < bodies.size(); j++) {
                const Body& first = bodies[i];
                const Body& second = bodies[j];

                if (first.getInverseMass() == 0.0 && second.getInverseMass() == 0.0) continue;
                if (overlaps(first.aabb(), second.aabb())) pairs.emplace_back(first, second);
            }
        }
        return pairs;
    }
}
