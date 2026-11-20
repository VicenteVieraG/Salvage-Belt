#pragma once

#include "salvage/core/world/Body.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace salvage::core::detail {

/// Generated bodies have diameter <= cell width. Overlapping centers must be in adjacent cells.
class PlacementIndex {
    struct Cell {
        int x, y, z;
        bool operator==(const Cell&) const = default;
    };

    struct Hash {
        std::size_t operator()(Cell cell) const noexcept {
            return static_cast<std::size_t>(cell.x) * 73856093u
                 ^ static_cast<std::size_t>(cell.y) * 19349663u
                 ^ static_cast<std::size_t>(cell.z) * 83492791u;
        }
    };

    double width_;
    std::unordered_multimap<Cell, const AABB*, Hash> entries_;

    Cell cellAt(Vec3 p) const {
        return {static_cast<int>(std::floor(p.x / width_)), static_cast<int>(std::floor(p.y / width_)),
                static_cast<int>(std::floor(p.z / width_))};
    }

public:
    PlacementIndex(AABB region, double maxExtent, std::size_t count) {
        const auto& [a, b] = region;
        const double coordinate = std::max({std::abs(a.x), std::abs(a.y), std::abs(a.z),
                                            std::abs(b.x), std::abs(b.y), std::abs(b.z)});
        width_ = std::max(maxExtent * 4, coordinate * 0x1p-28);
        entries_.reserve(count);
    }

    // The caller keeps accepted bodies at stable addresses until placement finishes.
    void add(const Body& body) { entries_.emplace(cellAt(body.getPosition()), &body.aabb()); }

    bool overlapsAny(const Body& body) const {
        const auto c = cellAt(body.getPosition());
        for (int x = c.x - 1; x <= c.x + 1; ++x)
            for (int y = c.y - 1; y <= c.y + 1; ++y)
                for (int z = c.z - 1; z <= c.z + 1; ++z) {
                    auto [it, end] = entries_.equal_range({x, y, z});
                    for (; it != end; ++it)
                        if (overlaps(body.aabb(), *it->second)) return true;
                }
        return false;
    }
};

}
