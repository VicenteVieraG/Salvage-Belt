#pragma once

#include "BroadPhase.hpp"
#include "Contact.hpp"
#include "salvage-core/metrics/SimulationMetrics.hpp"

namespace salvage::core {

struct DetectionResult {
    std::vector<CollisionPair> candidates; ///< Unique, ascending IDs, sorted by pair.
    std::vector<Contact> contacts;         ///< Pre-response geometry in candidate order.
    DetectionMetrics metrics;
};

/// Update an index and detect contacts on a borrowed, dense-ID world. Useful for headless replay.
DetectionResult detect(WorldView world, IBroadPhase& strategy, Executor& executor);

}
