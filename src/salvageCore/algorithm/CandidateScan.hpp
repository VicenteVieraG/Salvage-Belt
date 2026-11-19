/** @file @brief Schedule independent rows of a broad-phase query. */
#pragma once

#include <algorithm>
#include <cstddef>
#include <exception>
#include <omp.h>
#include "algorithm/IBroadPhase.hpp"

namespace salvage::core {
    // Each row owns its pairs. Each block owns its output until all scans finish.
    inline void scanCandidateRows(std::size_t rows, std::size_t workerLimit,
        std::vector<CollisionPair>& out, const auto& scanRow) {
        constexpr std::size_t rowsPerBlock = 32;
        const auto blocks = static_cast<std::ptrdiff_t>(rows / rowsPerBlock + (rows % rowsPerBlock != 0));
        if (workerLimit == 1 || blocks <= 1) {
            for (std::size_t i = 0; i < rows; ++i) scanRow(i, out);
            return;
        }

        const auto available = static_cast<std::size_t>(omp_get_max_threads());
        const auto workers = static_cast<int>(std::min({available, static_cast<std::size_t>(blocks),
            workerLimit == 0 ? available : workerLimit}));
        struct BlockResult {
            std::vector<CollisionPair> pairs;
            std::exception_ptr error;
        };
        std::vector<BlockResult> results(static_cast<std::size_t>(blocks));
        // Dynamic scheduling balances the longer rows at the start of a brute-force scan.
        #pragma omp parallel for num_threads(workers) schedule(dynamic, 1)
        for (std::ptrdiff_t block = 0; block < blocks; ++block) {
            const auto index = static_cast<std::size_t>(block);
            auto& result = results[index];
            const auto begin = index * rowsPerBlock;
            const auto end = std::min(begin + rowsPerBlock, rows);
            try {
                for (auto i = begin; i < end; ++i) scanRow(i, result.pairs);
            } catch (...) {
                // Exceptions must leave the OpenMP region on the calling thread.
                result.error = std::current_exception();
            }
        }
        for (const auto& result : results) {
            if (result.error) std::rethrow_exception(result.error);
        }
        for (const auto& result : results) {
            out.insert(out.end(), result.pairs.begin(), result.pairs.end());
        }
    }
}
