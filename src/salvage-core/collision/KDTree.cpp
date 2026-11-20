#include "salvage/core/collision/BroadPhase.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <numeric>

namespace salvage::core {
namespace {

constexpr std::array axes{&Vec3::x, &Vec3::y, &Vec3::z};
constexpr auto none = std::numeric_limits<std::size_t>::max();

class KDTree final : public IBroadPhase {
    struct Node {
        AABB region;
        std::size_t begin, end;
        std::size_t left{none}, right{none};
        unsigned axis{};
        double split{};
    };
    std::vector<BodyId> ids_;
    std::vector<Node> nodes_;

    std::size_t build(WorldView world, std::size_t begin, std::size_t end, AABB region, unsigned axis) {
        if (begin == end) return none;
        const auto index = nodes_.size();
        nodes_.push_back({region, begin, end});
        if (end - begin <= 8) return index;

        const auto coordinate = axes[axis];
        auto ids = std::span(ids_).subspan(begin, end - begin);
        const auto median = ids.begin() + static_cast<std::ptrdiff_t>(ids.size() / 2);
        std::ranges::nth_element(ids, median, {}, [&](BodyId id) {
            return std::pair{world.bodies[id].getPosition().*coordinate, id};
        });
        const auto split = world.bodies[*median].getPosition().*coordinate;
        // Only strictly separated extents descend. All bodies touching or spanning the plane stay here.
        auto left = std::partition(ids.begin(), ids.end(), [&](BodyId id) {
            return world.bodies[id].aabb().max.*coordinate < split;
        });
        auto right = std::partition(left, ids.end(), [&](BodyId id) {
            return world.bodies[id].aabb().min.*coordinate <= split;
        });
        const auto middleBegin = begin + static_cast<std::size_t>(left - ids.begin());
        const auto middleEnd = begin + static_cast<std::size_t>(right - ids.begin());
        nodes_[index] = {region, middleBegin, middleEnd, none, none, axis, split};
        auto leftRegion = region;
        leftRegion.max.*coordinate = split;
        region.min.*coordinate = split;
        const auto nextAxis = (axis + 1) % 3;
        nodes_[index].left = build(world, begin, middleBegin, leftRegion, nextAxis);
        nodes_[index].right = build(world, middleEnd, end, region, nextAxis);
        return index;
    }

    void query(WorldView world, const Body& body, std::size_t index, CandidateBatch& out) const {
        const auto& node = nodes_[index];
        for (auto i = node.begin; i < node.end; ++i) {
            const auto other = ids_[i];
            if (body.getId() >= other) continue;
            ++out.examined;
            if (overlaps(body.aabb(), world.bodies[other].aabb()))
                out.pairs.emplace_back(body.getId(), other);
        }
        if (node.left != none && body.aabb().min.*axes[node.axis] < node.split)
            query(world, body, node.left, out);
        if (node.right != none && body.aabb().max.*axes[node.axis] > node.split)
            query(world, body, node.right, out);
    }

public:
    void update(WorldView world) override {
        nodes_.clear();
        ids_.resize(world.bodies.size());
        std::iota(ids_.begin(), ids_.end(), BodyId{});
        if (ids_.empty()) return;
        auto region = world.bodies.front().aabb();
        for (const auto& body : world.bodies) {
            for (auto component : axes) {
                region.min.*component = std::min(region.min.*component, body.aabb().min.*component);
                region.max.*component = std::max(region.max.*component, body.aabb().max.*component);
            }
        }
        build(world, 0, ids_.size(), region, 0);
    }

    void scan(WorldView world, std::size_t begin, std::size_t end, CandidateBatch& out) const override {
        for (auto i = begin; i < end; ++i) query(world, world.bodies[i], 0, out);
    }

    std::size_t indexBytes() const noexcept override {
        return nodes_.capacity() * sizeof(Node) + ids_.capacity() * sizeof(BodyId);
    }

    BroadPhaseDebug debug() const override {
        BroadPhaseDebug result;
        for (const auto& node : nodes_)
            if (node.left != none || node.right != none)
                result.splits.push_back({node.region, node.axis, node.split});
        return result;
    }
};

}

std::unique_ptr<IBroadPhase> makeKDTree() { return std::make_unique<KDTree>(); }

}
