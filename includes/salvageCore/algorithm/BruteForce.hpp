/**
 * @file
 * @brief Stateless brute-force collision algorithms.
 *
 * @namespace salvage::core::BruteForce
 * @brief Collision algorithms using exhaustive pair enumeration.
 */
#pragma once

#include <span>
#include <vector>
#include "world/Body.hpp"

namespace salvage::core::BruteForce {
    /**
     * @brief Find broad-phase candidates using inclusive AABB overlap tests.
     * Reports each pair once in input order and omits static/static pairs.
     * Touching bounds count as overlaps. Candidates can be false positives for
     * the actual shapes; no contact resolution is performed.
     * @param bodies Read-only view of the bodies to check.
     * @return Independent body copies; editing them does not change the input.
     * @note Checks all body pairs, giving quadratic time complexity.
     */
    std::vector<BodyPair> broadPhase(std::span<const Body> bodies);
}
