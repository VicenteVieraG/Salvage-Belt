/**
 * @file
 * @brief Axis-aligned bounds and inclusive overlap testing.
 */
#pragma once

#include "Vec3.hpp"

namespace salvage::core {
    /**
     * @brief Axis-aligned bounding box in world coordinates.
     * @pre Each minimum component is no greater than its maximum component.
     */
    struct AABB {
        Vec3 min{}; ///< Minimum corner, inclusive.
        Vec3 max{}; ///< Maximum corner, inclusive.
    };

    /**
     * @brief Test whether two axis-aligned bounds overlap.
     * Touching faces, edges, and corners count as overlaps.
     * @param lhs First bounding box.
     * @param rhs Second bounding box.
     * @return True when the intervals overlap on all three axes.
     */
    constexpr bool overlaps(const AABB& lhs, const AABB& rhs) noexcept {
        return lhs.min.x <= rhs.max.x && lhs.max.x >= rhs.min.x
            && lhs.min.y <= rhs.max.y && lhs.max.y >= rhs.min.y
            && lhs.min.z <= rhs.max.z && lhs.max.z >= rhs.min.z;
    }
}
