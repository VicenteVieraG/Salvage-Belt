/** @file @brief Schedule independent rows of a broad-phase query. */
#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <exception>
#include <thread>
#include <vector>
#include "algorithm/IBroadPhase.hpp"

namespace salvage::core {
    // Each row owns its pairs. Each block owns its output until all scans finish.
    inline void scanCandidateRows(std::size_t rows, std::size_t workerLimit, std::vector<CollisionPair>& out, const auto& scanRow) {
        constexpr std::size_t rowsPerBlock = 32;
        const std::size_t blockCount = rows / rowsPerBlock + (rows % rowsPerBlock != 0);
        const std::size_t availableWorkers = workerLimit == 1 || blockCount <= 1
            ? 1 : std::max(1u, std::thread::hardware_concurrency());
        const auto requestedWorkers = workerLimit == 0 ? availableWorkers : workerLimit;
        const auto workerCount = std::min({blockCount, availableWorkers, requestedWorkers});
        if (workerCount <= 1) {
            for (std::size_t i = 0; i < rows; ++i) scanRow(i, out);
            return;
        }

        struct BlockResult {
            std::vector<CollisionPair> pairs;
            std::exception_ptr error;
        };
        std::vector<BlockResult> results(blockCount);
        std::atomic<std::size_t> nextBlock{0};
        const auto scanBlocks = [&] {
            // Claim more work as each block finishes to balance uneven scan costs.
            for (auto block = nextBlock.fetch_add(1, std::memory_order_relaxed);
                block < blockCount; block = nextBlock.fetch_add(1, std::memory_order_relaxed)) {
                auto& result = results[block];
                const auto begin = block * rowsPerBlock;
                const auto end = begin + std::min(rowsPerBlock, rows - begin);
                try {
                    for (auto i = begin; i < end; ++i) scanRow(i, result.pairs);
                } catch (...) {
                    result.error = std::current_exception();
                }
            }
        };
        {
            // The caller takes one worker slot. jthread joins before results are read.
            std::vector<std::jthread> threads;
            threads.reserve(workerCount - 1);
            for (std::size_t i = 1; i < workerCount; ++i) threads.emplace_back(scanBlocks);
            scanBlocks();
        }
        for (const auto& result : results) {
            if (result.error) std::rethrow_exception(result.error);
        }
        for (const auto& result : results) {
            out.insert(out.end(), result.pairs.begin(), result.pairs.end());
        }
    }
}
