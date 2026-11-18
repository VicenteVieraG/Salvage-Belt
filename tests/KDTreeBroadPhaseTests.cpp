/** @file @brief k-d tree candidate generation and index rebuilds. */
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include "algorithm/BruteForceBroadPhase.hpp"
#include "algorithm/KDTreeBroadPhase.hpp"

using namespace salvage::core;

TEST_CASE("K-d tree matches brute force across split planes and rebuilds", "[core][broadphase]") {
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

    KDTreeBroadPhase tree;
    tree.initialize(bodies);
    const auto checkPairs = [&] {
        BruteForceBroadPhase reference;
        reference.initialize(bodies);
        std::vector<CollisionPair> expected;
        reference.findCandidates(bodies, expected);

        const CollisionPair prior{10, 20};
        std::vector<CollisionPair> actual{prior};
        tree.findCandidates(bodies, actual);
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

    bodies.erase(bodies.begin(), bodies.begin() + 60);
    bodies.front().setCollider(BoxCollider{{10.0, 0.0, 2.0}});
    bodies.back().setPosition({4.0, -4.0, 4.0});
    bodies.emplace_back(40, Vec3{5.0, -4.0, 4.0}, Vec3{}, 1.0, SphereCollider{1.0});
    tree.update(bodies);
    checkPairs();

    bodies.resize(1);
    tree.initialize(bodies);
    checkPairs();
    bodies.clear();
    tree.update(bodies);
    checkPairs();
}
