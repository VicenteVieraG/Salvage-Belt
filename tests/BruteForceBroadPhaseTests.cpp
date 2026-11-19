/** @file @brief Brute-force candidate generation through the strategy interface. */
#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include "algorithm/BroadPhaseFactory.hpp"

using namespace salvage::core;

TEST_CASE("Brute force appends each overlapping AABB pair once", "[core][broadphase]") {
    const std::array bodies{
        Body{30, {}, {}, 0.0, SphereCollider{1.0}},
        Body{10, {2.0, 2.0, 2.0}, {}, 0.0, SphereCollider{1.0}},
        Body{20, {1.0, 1.0, 1.0}, {}, 1.0, BoxCollider{{}}},
        Body{40, {0.0, 0.0, 5.0}, {}, 1.0, SphereCollider{1.0}}
    };
    auto strategy = makeBroadPhase(Algorithm::BruteForce);
    strategy->initialize(bodies);
    const CollisionPair prior{30, 10};
    std::vector<CollisionPair> candidates{prior};
    strategy->findCandidates(bodies, candidates);

    REQUIRE(candidates.size() == 4);
    REQUIRE(candidates.front() == prior);
    // Compare only this query's output, independent of pair orientation and order.
    auto appended = std::span{candidates}.subspan(1);
    for (auto& [first, second] : appended) {
        if (first > second) std::swap(first, second);
    }
    std::ranges::sort(appended);
    const std::array<CollisionPair, 3> expected{{{10, 20}, {10, 30}, {20, 30}}};
    REQUIRE(std::ranges::equal(appended, expected));
}

TEST_CASE("Brute force queries the current body collection after updates", "[core][broadphase]") {
    auto strategy = makeBroadPhase(Algorithm::BruteForce);
    std::vector<Body> bodies;
    std::vector<CollisionPair> candidates;
    strategy->initialize(bodies);
    strategy->findCandidates(bodies, candidates);
    REQUIRE(candidates.empty());

    bodies.emplace_back(10, Vec3{}, Vec3{}, 1.0, SphereCollider{1.0});
    strategy->update(bodies);
    strategy->findCandidates(bodies, candidates);
    REQUIRE(candidates.empty());

    bodies.emplace_back(20, Vec3{}, Vec3{}, 1.0, SphereCollider{1.0});
    strategy->update(bodies);
    strategy->findCandidates(bodies, candidates);
    REQUIRE(candidates.size() == 1);

    bodies.back().setPosition({3.0, 0.0, 0.0});
    strategy->update(bodies);
    candidates.clear();
    strategy->findCandidates(bodies, candidates);
    REQUIRE(candidates.empty());
}

TEST_CASE("Brute force appends the same pairs at every worker count", "[core][broadphase][parallel]") {
    std::vector<Body> bodies;
    for (BodyId id = 100; id > 0; --id) {
        // Interleaved clusters overlap across block boundaries, with ties and touching bounds.
        const Vec3 position{2.0 * (id % 7), 0.0, 0.0};
        bodies.emplace_back(id, position, Vec3{}, 0.0, SphereCollider{1.0});
    }

    auto reference = makeBroadPhase(Algorithm::BruteForce);
    reference->initialize(bodies);
    for (std::size_t workers : {0, 1, 2, 4, 128}) {
        CAPTURE(workers);
        auto strategy = makeBroadPhase({.workerCount = workers});
        strategy->initialize(bodies);
        std::vector<CollisionPair> expected{{200, 300}};
        std::vector<CollisionPair> actual = expected;
        reference->findCandidates(bodies, expected);
        strategy->findCandidates(bodies, actual);
        REQUIRE(actual == expected);

        strategy->update({});
        strategy->findCandidates({}, actual);
        REQUIRE(actual == expected);
    }
}

TEST_CASE("Parallel execution rejects unimplemented strategies", "[core][broadphase][parallel]") {
    for (auto algorithm : {Algorithm::KDTree, Algorithm::SweepAndPrune}) {
        REQUIRE(makeBroadPhase({.algorithm = algorithm, .workerCount = 1}) != nullptr);
        REQUIRE_THROWS_AS(makeBroadPhase({.algorithm = algorithm, .workerCount = 0}), std::invalid_argument);
        REQUIRE_THROWS_AS(makeBroadPhase({.algorithm = algorithm, .workerCount = 2}), std::invalid_argument);
    }
}
