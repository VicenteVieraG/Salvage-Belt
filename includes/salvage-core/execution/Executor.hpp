#pragma once

#include <cstddef>
#include <functional>
#include <memory>

namespace salvage::core {

/// Persistent CPU workers. The calling thread occupies slot zero.
/// Use from one coordinating thread. Tasks must not recursively submit to the same executor.
class Executor {
public:
    using RangeTask = std::function<void(std::size_t slot, std::size_t begin, std::size_t end)>;

    explicit Executor(std::size_t workerCount = 1);
    ~Executor();
    Executor(const Executor&) = delete;
    Executor& operator=(const Executor&) = delete;

    std::size_t workerCount() const noexcept;
    /// Run disjoint contiguous ranges. All workers finish before returning or rethrowing a task error.
    void parallelFor(std::size_t count, const RangeTask& task);

private:
    struct Pool;
    std::unique_ptr<Pool> pool_;
    std::size_t workers_;
};

}
