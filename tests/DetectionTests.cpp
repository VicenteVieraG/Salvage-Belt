#include "salvage-core/collision/Detection.hpp"
#include "salvage-core/execution/Executor.hpp"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <array>
#include <random>

using namespace salvage::core;

namespace {

constexpr std::array algorithms{Algorithm::bruteForce, Algorithm::kdTree, Algorithm::sweepAndPrune};
constexpr std::size_t workers = SALVAGE_ENABLE_THREADS ? 4 : 1;

void compareWithAllPairs(WorldView world, const DetectionResult& detection) {
    std::vector<CollisionPair> expected;
    for (std::size_t i = 0; i < world.bodies.size(); ++i)
        for (const auto& second : world.bodies.subspan(i + 1))
            if (detectContact(world.bodies[i], second))
                expected.emplace_back(world.bodies[i].getId(), second.getId());

    std::vector<CollisionPair> actual;
    for (const auto& contact : detection.contacts) actual.emplace_back(contact.first, contact.second);
    REQUIRE(actual == expected);
    REQUIRE(std::ranges::is_sorted(detection.candidates));
    REQUIRE(std::ranges::adjacent_find(detection.candidates) == detection.candidates.end());
    REQUIRE(std::ranges::includes(detection.candidates, expected));
    for (auto [a, b] : detection.candidates) {
        REQUIRE(a < b);
        REQUIRE(b < world.bodies.size());
    }
    REQUIRE(detection.metrics.uniqueCandidates == detection.candidates.size());
    REQUIRE(detection.metrics.contacts == actual.size());
}

}

TEST_CASE("All broad phases agree with exhaustive geometry on boundary and split-plane cases") {
    Executor executor(workers);
    std::vector<Body> bodies;
    for (int x = -4; x <= 4; ++x)
        for (int y = -1; y <= 1; ++y)
            bodies.emplace_back(Vec3{2.0 * x, 2.0 * y, 0}, Vec3{}, 1, BoxCollider{{1, 1, 1}});
    bodies.emplace_back(Vec3{-3, 0, 0}, Vec3{}, 0, BoxCollider{{12, 0, 0}});
    bodies.emplace_back(Vec3{}, Vec3{}, 1, SphereCollider{0});
    bodies.emplace_back(Vec3{}, Vec3{}, 1, SphereCollider{0});
    for (auto algorithm : algorithms) {
        auto strategy = makeBroadPhase(algorithm);
        for (std::size_t count : {std::size_t{0}, std::size_t{1}, bodies.size()}) {
            World world(std::vector<Body>(bodies.begin(), bodies.begin() + count), 1, executor);
            const auto result = detect(world.view(), *strategy, executor);
            compareWithAllPairs(world.view(), result);
            if (algorithm == Algorithm::bruteForce)
                REQUIRE(result.metrics.pairsExamined == (count ? count * (count - 1) / 2 : 0));
            const auto debug = strategy->debug();
            if (algorithm == Algorithm::kdTree && count > 8) REQUIRE_FALSE(debug.splits.empty());
            if (algorithm == Algorithm::sweepAndPrune) REQUIRE(debug.intervals.size() == count);
        }
    }
}

TEST_CASE("Indexed searches remain conservative across abrupt reordering and heterogeneous sizes") {
    Executor executor(workers);
    for (auto algorithm : algorithms) {
        auto strategy = makeBroadPhase(algorithm);
        for (unsigned seed : {17, 93, 1049}) {
            std::mt19937 rng(seed);
            std::uniform_real_distribution<double> position(-9.0, 9.0), extent(0.1, 3.2);
            std::vector<Body> input;
            for (int i = 0; i < 112; ++i) {
                const Vec3 p{position(rng), position(rng), position(rng)};
                const Collider shape = i % 2 ? Collider{SphereCollider{extent(rng)}}
                                             : Collider{BoxCollider{{extent(rng), extent(rng), extent(rng)}}};
                input.emplace_back(p, Vec3{}, 1, shape);
            }
            World world(input, seed, executor);
            compareWithAllPairs(world.view(), detect(world.view(), *strategy, executor));
        }
    }
}

TEST_CASE("Collision pipeline normalizes and deduplicates before shared narrow phase") {
    class DuplicateStrategy final : public IBroadPhase {
    public:
        void update(WorldView) override {}
        void scan(WorldView, std::size_t begin, std::size_t end, CandidateBatch& out) const override {
            for (auto i = begin; i < end; ++i) {
                out.pairs.emplace_back(1, 0);
                out.pairs.emplace_back(0, 1);
            }
        }
        std::size_t indexBytes() const noexcept override { return 0; }
        BroadPhaseDebug debug() const override { return {}; }
    } strategy;
    Executor executor(workers);
    World world(std::vector<Body>(2), 1, executor);
    const auto result = detect(world.view(), strategy, executor);
    compareWithAllPairs(world.view(), result);
    REQUIRE(result.metrics.rawCandidates == 4);
    REQUIRE(result.metrics.uniqueCandidates == 1);
}
