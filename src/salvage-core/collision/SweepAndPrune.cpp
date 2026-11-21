#include "salvage-core/collision/BroadPhase.hpp"
#include <algorithm>
#include <tuple>

namespace salvage::core {
namespace {

class SweepAndPrune final : public IBroadPhase {
    std::vector<SweepInterval> order_;

public:
    void update(WorldView world) override {
        if (order_.size() != world.bodies.size()) {
            order_.resize(world.bodies.size());
            for (std::size_t i = 0; i < order_.size(); ++i) order_[i].body = static_cast<BodyId>(i);
        }
        for (auto& interval : order_) {
            const auto& bounds = world.bodies[interval.body].aabb();
            interval.min = bounds.min.x;
            interval.max = bounds.max.x;
        }
        const auto less = [](const auto& a, const auto& b) {
            return std::tie(a.min, a.body) < std::tie(b.min, b.body);
        };
        if (!std::ranges::is_sorted(order_, less)) std::ranges::sort(order_, less);
    }

    void scan(WorldView world, std::size_t begin, std::size_t end, CandidateBatch& out) const override {
        for (auto i = begin; i < end; ++i) {
            const auto& a = order_[i];
            for (auto j = i + 1; j < order_.size() && order_[j].min <= a.max; ++j) {
                const auto b = order_[j].body;
                ++out.examined;
                if (overlaps(world.bodies[a.body].aabb(), world.bodies[b].aabb()))
                    out.pairs.emplace_back(a.body, b);
            }
        }
    }

    std::size_t indexBytes() const noexcept override { return order_.capacity() * sizeof(SweepInterval); }
    BroadPhaseDebug debug() const override { return {{}, order_}; }
};

}

std::unique_ptr<IBroadPhase> makeSweepAndPrune() { return std::make_unique<SweepAndPrune>(); }

}
