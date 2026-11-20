#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace salvage::core {

using Duration = std::chrono::duration<double>; ///< Wall-clock seconds.

struct DetectionMetrics {
    Duration maintenance{}, search{}, normalization{}, narrowPhase{}, total{};
    std::uint64_t pairsExamined{}; ///< Body-pair AABB tests, excludes index traversal.
    std::size_t rawCandidates{}, uniqueCandidates{}, contacts{};
    std::size_t indexBytes{}, pairBufferBytes{}; ///< Estimated reserved element storage.
};

struct SimulationMetrics {
    std::uint64_t stepIndex{};
    Duration generation{}, initialization{}; ///< Cold-start costs, carried forward for the run.
    Duration integration{}, resolution{}, totalStep{};
    DetectionMetrics detection{};
    std::size_t peakCollisionBufferBytes{}; ///< Run maximum sampled index + pair buffer capacity.
};

}
