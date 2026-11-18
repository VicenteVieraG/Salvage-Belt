/**
 * @file
 * @brief Construction of selectable broad-phase strategies.
 */
#include "algorithm/BroadPhaseFactory.hpp"
#include "algorithm/BruteForceBroadPhase.hpp"
#include "algorithm/KDTreeBroadPhase.hpp"
#include "algorithm/SweepAndPruneBroadPhase.hpp"

#include <stdexcept>

namespace salvage::core {
    std::unique_ptr<IBroadPhase> makeBroadPhase(Algorithm algorithm) {
        switch (algorithm) {
            case Algorithm::BruteForce:
                return std::make_unique<BruteForceBroadPhase>();
            case Algorithm::KDTree:
                return std::make_unique<KDTreeBroadPhase>();
            case Algorithm::SweepAndPrune:
                return std::make_unique<SweepAndPruneBroadPhase>();
        }

        throw std::invalid_argument("Unknown broad-phase algorithm");
    }
}
