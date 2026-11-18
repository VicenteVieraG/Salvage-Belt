#pragma once

#include "physics/AABB.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <unordered_map>

namespace salvage::core::detail {
    class PlacementGrid {
        struct Cell {
            std::int32_t x;
            std::int32_t y;
            std::int32_t z;

            bool operator==(const Cell&) const = default;
        };

        struct CellHash {
            std::size_t operator()(const Cell& cell) const noexcept {
                return (static_cast<std::size_t>(cell.x) * 73856093u)
                    ^ (static_cast<std::size_t>(cell.y) * 19349663u)
                    ^ (static_cast<std::size_t>(cell.z) * 83492791u);
            }
        };

        double width;
        std::unordered_multimap<Cell, const AABB*, CellHash> cells;

        Cell cellAt(const Vec3& position) const noexcept {
            return {
                static_cast<std::int32_t>(std::floor(position.x / width)),
                static_cast<std::int32_t>(std::floor(position.y / width)),
                static_cast<std::int32_t>(std::floor(position.z / width))
            };
        }

    public:
        explicit PlacementGrid(double width, std::size_t capacity = 0) : width{width} {
            if (capacity > 0) cells.reserve(capacity);
        }

        void add(const Vec3& clippedMin, const AABB& bounds) {
            cells.emplace(cellAt(clippedMin), &bounds);
        }

        bool overlapsAny(const AABB& bounds) const {
            const Cell min = cellAt(bounds.min);
            const Cell max = cellAt(bounds.max);
            // Entries are stored by their lower corner and can extend into the next cell.
            for (auto x = min.x - 1; x <= max.x; ++x) {
                for (auto y = min.y - 1; y <= max.y; ++y) {
                    for (auto z = min.z - 1; z <= max.z; ++z) {
                        const auto [first, last] = cells.equal_range(Cell{x, y, z});
                        for (auto it = first; it != last; ++it) {
                            if (overlaps(bounds, *it->second)) return true;
                        }
                    }
                }
            }
            return false;
        }
    };

    // Each body occupies a single entry. Larger obstacles use coarser grids so they
    // neither enlarge the main grid's cells nor have to be inserted into many cells.
    class PlacementIndex {
        AABB region;
        double baseWidth;
        PlacementGrid local;
        std::map<double, PlacementGrid> coarse;

        static double cellWidth(const AABB& region, double maxExtent) noexcept {
            const auto& [min, max] = region;
            const double maxCoordinate = std::max({std::abs(min.x), std::abs(min.y), std::abs(min.z),
                std::abs(max.x), std::abs(max.y), std::abs(max.z)});
            // Keep cell coordinates and neighbor arithmetic in range even far from the origin.
            return std::max(4.0 * maxExtent, maxCoordinate * 0x1p-28);
        }

    public:
        PlacementIndex(const AABB& region, double maxExtent, std::size_t count)
            : region{region}, baseWidth{cellWidth(region, maxExtent)}, local{baseWidth, count} {}

        // Bounds must stay at a stable address while indexed. Queries are inside region.
        void add(const AABB& bounds) {
            if (!overlaps(bounds, region)) return;
            const Vec3 min{std::max(bounds.min.x, region.min.x), std::max(bounds.min.y, region.min.y),
                std::max(bounds.min.z, region.min.z)};
            const Vec3 max{std::min(bounds.max.x, region.max.x), std::min(bounds.max.y, region.max.y),
                std::min(bounds.max.z, region.max.z)};
            const Vec3 size = max - min;
            const double span = std::max({size.x, size.y, size.z});

            // Leave room for rounding at cell boundaries. Half a cell can span at most two cells.
            if (span <= baseWidth * 0.5) {
                local.add(min, bounds);
                return;
            }

            const double requiredWidth = span * 2.0;
            double width = requiredWidth;
            if (std::isfinite(requiredWidth)) {
                int exponent{};
                std::frexp(requiredWidth, &exponent);
                width = std::ldexp(1.0, exponent);
            }
            coarse.try_emplace(width, width).first->second.add(min, bounds);
        }

        bool overlapsAny(const AABB& bounds) const {
            if (local.overlapsAny(bounds)) return true;
            return std::ranges::any_of(coarse, [&](const auto& entry) {
                return entry.second.overlapsAny(bounds);
            });
        }
    };
}
