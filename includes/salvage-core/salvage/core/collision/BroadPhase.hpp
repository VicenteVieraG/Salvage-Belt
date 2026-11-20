#pragma once

#include "salvage/core/world/World.hpp"
#include <memory>
#include <utility>

namespace salvage::core {

enum class Algorithm { bruteForce, kdTree, sweepAndPrune };
using CollisionPair = std::pair<BodyId, BodyId>;

struct CandidateBatch {
    std::vector<CollisionPair> pairs;
    std::uint64_t examined{};
};

struct SplitPlane {
    AABB region; ///< Region split by this node.
    unsigned axis;
    double coordinate;
};
struct SweepInterval {
    BodyId body;
    double min, max;
};
struct BroadPhaseDebug {
    std::vector<SplitPlane> splits;
    std::vector<SweepInterval> intervals;
};

/// A strategy owns only its index. update() initializes or maintains it for the supplied view.
/// Concurrent scans of disjoint row ranges append to private batches.
class IBroadPhase {
public:
    virtual ~IBroadPhase() = default;
    virtual void update(WorldView world) = 0;
    virtual void scan(WorldView world, std::size_t begin, std::size_t end, CandidateBatch& out) const = 0;
    virtual std::size_t indexBytes() const noexcept = 0;
    virtual BroadPhaseDebug debug() const = 0;
};

std::unique_ptr<IBroadPhase> makeBroadPhase(Algorithm algorithm);

}
