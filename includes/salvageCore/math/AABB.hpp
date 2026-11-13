#pragma once

#include "Vec3.hpp"

namespace salvage::core {
    struct AABB {
        Vec3 min{};
        Vec3 max{};
    };

    // Touching faces, edges, and corners count as overlaps.
    constexpr bool overlaps(const AABB& lhs, const AABB& rhs) noexcept {
        return lhs.min.x <= rhs.max.x && lhs.max.x >= rhs.min.x
            && lhs.min.y <= rhs.max.y && lhs.max.y >= rhs.min.y
            && lhs.min.z <= rhs.max.z && lhs.max.z >= rhs.min.z;
    }
};
