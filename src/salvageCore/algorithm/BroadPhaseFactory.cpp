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
        return makeBroadPhase(BroadPhaseOptions{.algorithm = algorithm});
    }

    std::unique_ptr<IBroadPhase> makeBroadPhase(BroadPhaseOptions options) {
        if (options.workerCount != 1 && options.algorithm != Algorithm::BruteForce) {
            throw std::invalid_argument("Parallel execution is supported only for brute force");
        }
        switch (options.algorithm) {
            case Algorithm::BruteForce:
                return std::make_unique<BruteForceBroadPhase>(options.workerCount);
            case Algorithm::KDTree:
                return std::make_unique<KDTreeBroadPhase>();
            case Algorithm::SweepAndPrune:
                return std::make_unique<SweepAndPruneBroadPhase>();
        }

        throw std::invalid_argument("Unknown broad-phase algorithm");
    }
}
