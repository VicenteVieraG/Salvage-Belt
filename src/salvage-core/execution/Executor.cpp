#include "salvage/core/execution/Executor.hpp"

#include <algorithm>
#include <stdexcept>
#if SALVAGE_ENABLE_THREADS
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>
#endif

namespace salvage::core {

#if SALVAGE_ENABLE_THREADS
struct Executor::Pool {
    std::mutex mutex;
    std::condition_variable ready, finished;
    std::size_t generation{}, remaining{}, count{}, active{};
    bool stopping{};
    const RangeTask* task{};
    std::exception_ptr failure;
    std::vector<std::thread> threads;

    explicit Pool(std::size_t workers) {
        try {
            for (std::size_t slot = 1; slot < workers; ++slot)
                threads.emplace_back([this, slot] { work(slot); });
        } catch (...) {
            stop();
            throw;
        }
    }

    ~Pool() { stop(); }

    void stop() {
        {
            std::lock_guard lock(mutex);
            stopping = true;
        }
        ready.notify_all();
        for (auto& thread : threads) thread.join();
    }

    void runRange(std::size_t slot) {
        if (slot >= active) return;
        const auto width = count / active;
        const auto extra = count % active;
        const auto begin = slot * width + std::min(slot, extra);
        try {
            (*task)(slot, begin, begin + width + (slot < extra));
        } catch (...) {
            std::lock_guard lock(mutex);
            if (!failure) failure = std::current_exception();
        }
    }

    void work(std::size_t slot) {
        std::size_t seen{};
        std::unique_lock lock(mutex);
        for (;;) {
            ready.wait(lock, [&] { return stopping || generation != seen; });
            if (stopping) return;
            seen = generation;
            lock.unlock();
            runRange(slot);
            lock.lock();
            if (--remaining == 0) finished.notify_one();
        }
    }

    void dispatch(std::size_t size, const RangeTask& action) {
        std::unique_lock lock(mutex);
        count = size;
        active = std::min(size, threads.size() + 1);
        remaining = threads.size();
        task = &action;
        failure = nullptr;
        ++generation;
        lock.unlock();
        ready.notify_all();
        runRange(0);
        lock.lock();
        finished.wait(lock, [&] { return remaining == 0; });
        task = nullptr;
        if (failure) std::rethrow_exception(failure);
    }
};
#else
struct Executor::Pool {};
#endif

Executor::Executor(std::size_t workerCount) : workers_(workerCount) {
    if (!workers_) throw std::invalid_argument("Worker count must be positive");
    if (workers_ == 1) return;
#if SALVAGE_ENABLE_THREADS
    pool_ = std::make_unique<Pool>(workers_);
#else
    throw std::invalid_argument("This build supports one CPU worker");
#endif
}

Executor::~Executor() = default;
std::size_t Executor::workerCount() const noexcept { return workers_; }

void Executor::parallelFor(std::size_t count, const RangeTask& task) {
    if (!count) return;
#if SALVAGE_ENABLE_THREADS
    if (pool_ && count > 1) {
        pool_->dispatch(count, task);
        return;
    }
#endif
    task(0, 0, count);
}

}
