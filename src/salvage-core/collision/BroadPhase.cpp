#include "salvage-core/collision/BroadPhase.hpp"

#include <stdexcept>

namespace salvage::core {

std::unique_ptr<IBroadPhase> makeKDTree();
std::unique_ptr<IBroadPhase> makeSweepAndPrune();

namespace {
class BruteForce final : public IBroadPhase {
public:
    void update(WorldView) override {}
    std::size_t indexBytes() const noexcept override { return 0; }
    BroadPhaseDebug debug() const override { return {}; }

    void scan(WorldView world, std::size_t begin, std::size_t end, CandidateBatch& out) const override {
        const auto bodies = world.bodies;
        for (auto row = begin; row < end; ++row) {
            // Alternate long and short rows to balance triangular work across contiguous batches.
            const auto i = row % 2 == 0 ? row / 2 : bodies.size() - 1 - row / 2;
            const auto& first = bodies[i];
            for (const Body& second : bodies.subspan(i + 1)) {
                ++out.examined;
                if (overlaps(first.aabb(), second.aabb())) out.pairs.emplace_back(first.getId(), second.getId());
            }
        }
    }
};
}

std::unique_ptr<IBroadPhase> makeBroadPhase(Algorithm algorithm) {
    switch (algorithm) {
    case Algorithm::bruteForce: return std::make_unique<BruteForce>();
    case Algorithm::kdTree: return makeKDTree();
    case Algorithm::sweepAndPrune: return makeSweepAndPrune();
    }
    throw std::invalid_argument("Unknown broad-phase algorithm");
}

}
