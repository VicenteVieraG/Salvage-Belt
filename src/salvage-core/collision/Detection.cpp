#include "salvage-core/collision/Detection.hpp"
#include "salvage-core/execution/Executor.hpp"

#include <algorithm>
#include <stdexcept>

namespace salvage::core {

DetectionResult detect(WorldView world, IBroadPhase& strategy, Executor& executor) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    DetectionResult result;
    auto& metrics = result.metrics;
    strategy.update(world);
    const auto updated = Clock::now();
    metrics.maintenance = updated - start;

    const auto workers = std::min(world.bodies.size(), executor.workerCount());
    std::vector<CandidateBatch> batches(workers);
    executor.parallelFor(world.bodies.size(), [&](auto slot, auto begin, auto end) {
        strategy.scan(world, begin, end, batches[slot]);
    });
    const auto scanned = Clock::now();
    metrics.search = scanned - updated;

    for (const auto& batch : batches) {
        metrics.rawCandidates += batch.pairs.size();
        metrics.pairsExamined += batch.examined;
        metrics.pairBufferBytes += batch.pairs.capacity() * sizeof(CollisionPair);
    }
    auto& pairs = result.candidates;
    pairs.reserve(metrics.rawCandidates);
    for (const auto& batch : batches) {
        for (auto [a, b] : batch.pairs) {
            if (a >= world.bodies.size() || b >= world.bodies.size() || a == b)
                throw std::logic_error("Broad phase emitted an invalid pair");
            if (a > b) std::swap(a, b);
            pairs.emplace_back(a, b);
        }
    }
    std::ranges::sort(pairs);
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
    metrics.uniqueCandidates = pairs.size();
    metrics.pairBufferBytes += pairs.capacity() * sizeof(CollisionPair);
    const auto normalized = Clock::now();
    metrics.normalization = normalized - scanned;

    std::vector<std::vector<Contact>> confirmed(std::min(executor.workerCount(), pairs.size()));
    executor.parallelFor(pairs.size(), [&](auto slot, auto begin, auto end) {
        for (auto i = begin; i < end; ++i) {
            auto [a, b] = pairs[i];
            if (auto contact = detectContact(world.bodies[a], world.bodies[b]))
                confirmed[slot].push_back(*contact);
        }
    });
    for (const auto& contacts : confirmed) metrics.contacts += contacts.size();
    result.contacts.reserve(metrics.contacts);
    for (const auto& contacts : confirmed)
        result.contacts.insert(result.contacts.end(), contacts.begin(), contacts.end());

    metrics.indexBytes = strategy.indexBytes();
    const auto done = Clock::now();
    metrics.narrowPhase = done - normalized;
    metrics.total = done - start;
    return result;
}

}
