/**
 * @file
 * @brief Median-split k-d tree for inclusive AABB overlap queries.
 */
#include "algorithm/KDTreeBroadPhase.hpp"

#include <algorithm>
#include <array>
#include <numeric>

namespace salvage::core {
    namespace {
        AABB enclosingBounds(const AABB& a, const AABB& b) {
            return {
                .min = {.x = std::min(a.min.x, b.min.x), .y = std::min(a.min.y, b.min.y), .z = std::min(a.min.z, b.min.z)},
                .max = {.x = std::max(a.max.x, b.max.x), .y = std::max(a.max.y, b.max.y), .z = std::max(a.max.z, b.max.z)}
            };
        }
    }

    AABB KDTreeBroadPhase::build(std::span<Node> subtree, std::size_t axis) {
        constexpr std::array axes{&Vec3::x, &Vec3::y, &Vec3::z};
        const auto middle = subtree.size() / 2;
        auto& root = subtree[middle];
        if (subtree.size() == 1) {
            root.subtreeBounds = root.bounds;
            return root.subtreeBounds;
        }

        const auto median = subtree.begin() + static_cast<std::ptrdiff_t>(middle);
        std::ranges::nth_element(subtree, median, {}, [coordinate = axes[axis]](const Node& node) {
            return std::midpoint(node.bounds.min.*coordinate, node.bounds.max.*coordinate);
        });

        const auto nextAxis = (axis + 1) % axes.size();
        root.subtreeBounds = enclosingBounds(root.bounds, build(subtree.first(middle), nextAxis));
        const auto right = subtree.subspan(middle + 1);
        if (!right.empty()) {
            root.subtreeBounds = enclosingBounds(root.subtreeBounds, build(right, nextAxis));
        }
        return root.subtreeBounds;
    }

    void KDTreeBroadPhase::initialize(std::span<const Body> bodies) {
        nodes.reserve(bodies.size());
        nodes.clear();

        for (const Body& body : bodies) nodes.push_back({body.getId(), body.aabb(), {}});
        if (!nodes.empty()) build(nodes, 0);
    }

    void KDTreeBroadPhase::update(std::span<const Body> bodies) {
        initialize(bodies);
    }

    void KDTreeBroadPhase::findOverlaps(std::span<const Node> subtree, const Body& body, std::vector<CollisionPair>& out) {
        if (subtree.empty()) return;

        const auto middle = subtree.size() / 2;
        const Node& root = subtree[middle];
        if (!overlaps(body.aabb(), root.subtreeBounds)) return;

        // Only the query from the smaller ID emits the pair.
        if (body.getId() < root.id && overlaps(body.aabb(), root.bounds)) {
            out.emplace_back(body.getId(), root.id);
        }
        findOverlaps(subtree.first(middle), body, out);
        findOverlaps(subtree.subspan(middle + 1), body, out);
    }

    void KDTreeBroadPhase::findCandidates(std::span<const Body> bodies, std::vector<CollisionPair>& out) const {
        for (const Body& body : bodies) findOverlaps(nodes, body, out);
    }
}
