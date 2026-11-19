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
        if (options.execution == ExecutionMode::Parallel) {
            if (options.algorithm == Algorithm::BruteForce) {
                return std::make_unique<BruteForceBroadPhase>(options.workerCount);
            }
            throw std::invalid_argument("Parallel execution is supported only for brute force");
        }
        if (options.execution != ExecutionMode::Sequential) {
            throw std::invalid_argument("Unknown broad-phase execution mode");
        }
        switch (options.algorithm) {
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
