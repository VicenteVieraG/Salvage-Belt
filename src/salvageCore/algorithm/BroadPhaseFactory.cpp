/**
 * @file
 * @brief Construction of selectable broad-phase strategy placeholders.
 */
#include "algorithm/BroadPhaseFactory.hpp"
#include "algorithm/BruteForceBroadPhase.hpp"
#include "algorithm/KDTreeBroadPhase.hpp"

#include <stdexcept>

namespace salvage::core {
    std::unique_ptr<IBroadPhase> makeBroadPhase(Algorithm algorithm) {
        switch (algorithm) {
            case Algorithm::BruteForce:
                return std::make_unique<BruteForceBroadPhase>();
            case Algorithm::KDTree:
                return std::make_unique<KDTreeBroadPhase>();
        }

        throw std::invalid_argument("Unknown broad-phase algorithm");
    }
}
