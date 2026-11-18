/** @file @brief Compare indexed broad-phase strategies with the pairwise scan. */
#include <algorithm>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include "algorithm/BruteForceBroadPhase.hpp"
#include "algorithm/KDTreeBroadPhase.hpp"
#include "algorithm/SweepAndPruneBroadPhase.hpp"

using namespace salvage::core;

TEMPLATE_TEST_CASE("Indexed broad phase matches brute force across updates", "[core][broadphase]",
    KDTreeBroadPhase, SweepAndPruneBroadPhase) {
    std::vector<Body> bodies;
    // Neighbors touch at faces, edges, and corners. Centers tie along every split axis.
    BodyId id = 1000;
    for (int x = -2; x <= 2; ++x) {
        for (int y = -2; y <= 2; ++y) {
            for (int z = -2; z <= 2; ++z) {
                bodies.emplace_back(id--, Vec3{2.0 * x, 2.0 * y, 2.0 * z}, Vec3{}, 0.0,
                    BoxCollider{{1.0, 1.0, 1.0}});
            }
        }
    }
    bodies.emplace_back(10, Vec3{-20.0, 0.0, 0.0}, Vec3{}, 1.0, BoxCollider{{25.0, 0.0, 0.0}});
    bodies.emplace_back(20, Vec3{}, Vec3{}, 1.0, SphereCollider{0.0});
    bodies.emplace_back(30, Vec3{}, Vec3{}, 1.0, SphereCollider{0.0});

    TestType strategy;
    strategy.initialize(bodies);
    const auto checkPairs = [&] {
        BruteForceBroadPhase reference;
        reference.initialize(bodies);
        std::vector<CollisionPair> expected;
        reference.findCandidates(bodies, expected);

        const CollisionPair prior{10, 20};
        std::vector<CollisionPair> actual{prior};
        strategy.findCandidates(bodies, actual);
        REQUIRE_FALSE(actual.empty());
        REQUIRE(actual.front() == prior);
        auto appended = std::span{actual}.subspan(1);
        const auto sortPairs = [](std::span<CollisionPair> pairs) {
            for (auto& [first, second] : pairs) {
                if (second < first) {
                    std::swap(first, second);
                }
            }
            std::ranges::sort(pairs);
        };
        sortPairs(expected);
        sortPairs(appended);
        REQUIRE(std::ranges::equal(appended, expected));
    };
    checkPairs();

    // Change the sweep order without changing the number of bodies.
    bodies.back().setPosition({-4.0, -4.0, -4.0});
    bodies.front().setCollider(BoxCollider{{12.0, 1.0, 1.0}});
    std::ranges::reverse(bodies);
    strategy.update(bodies);
    checkPairs();

    bodies.erase(bodies.begin(), bodies.begin() + 60);
    bodies.front().setCollider(BoxCollider{{10.0, 0.0, 2.0}});
    bodies.back().setPosition({4.0, -4.0, 4.0});
    bodies.emplace_back(40, Vec3{5.0, -4.0, 4.0}, Vec3{}, 1.0, SphereCollider{1.0});
    strategy.update(bodies);
    checkPairs();

    bodies.resize(1);
    strategy.initialize(bodies);
    checkPairs();
    bodies.clear();
    strategy.update(bodies);
    checkPairs();
}
