/**
 * @file
 * @brief Single-axis sweep for inclusive AABB overlaps.
 */
#include "algorithm/SweepAndPruneBroadPhase.hpp"

#include <algorithm>
#include <numeric>

namespace salvage::core {
    void SweepAndPruneBroadPhase::initialize(std::span<const Body> bodies) {
        update(bodies);
    }

    void SweepAndPruneBroadPhase::update(std::span<const Body> bodies) {
        if (order.size() != bodies.size()) {
            order.resize(bodies.size());
            std::iota(order.begin(), order.end(), std::size_t{0});
        }

        const auto minX = [bodies](std::size_t index) { return bodies[index].aabb().min.x; };
        if (!std::ranges::is_sorted(order, {}, minX)) {
            std::ranges::sort(order, {}, minX);
        }
    }

    void SweepAndPruneBroadPhase::findCandidates(std::span<const Body> bodies, std::vector<CollisionPair>& out) const {
        for (auto first = order.begin(); first != order.end(); ++first) {
            const Body& body = bodies[*first];

            for (auto next = first + 1; next != order.end(); ++next) {
                const Body& other = bodies[*next];

                if (other.aabb().min.x > body.aabb().max.x) break;
                if (overlaps(body.aabb(), other.aabb())) {
                    out.emplace_back(body.getId(), other.getId());
                }
            }
        }
    }
}
