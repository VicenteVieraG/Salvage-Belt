#include <catch2/catch_test_macros.hpp>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include "salvageCore.hpp"

using namespace salvage::core;

static_assert(std::is_same_v<decltype(std::declval<World&>().getBody(1)), const Body*>);
static_assert(std::is_same_v<decltype(std::declval<World&>().getBodies()), std::span<const Body>>);
static_assert(std::is_same_v<decltype(std::declval<const World&>().findPotentialCollisions()),
    std::vector<std::pair<Body, Body>>>);

namespace {
    void requireVec3(const Vec3& actual, const Vec3& expected) {
        REQUIRE(actual.x == expected.x);
        REQUIRE(actual.y == expected.y);
        REQUIRE(actual.z == expected.z);
    }

    void requireBounds(const Body& body, const Vec3& min, const Vec3& max) {
        requireVec3(body.aabb().min, min);
        requireVec3(body.aabb().max, max);
    }

    void requirePairIds(const std::vector<std::pair<Body, Body>>& actual,
        const std::vector<std::pair<BodyId, BodyId>>& expected) {
        REQUIRE(actual.size() == expected.size());
        for (std::size_t i = 0; i < actual.size(); ++i) {
            REQUIRE(actual[i].first.getId() == expected[i].first);
            REQUIRE(actual[i].second.getId() == expected[i].second);
        }
    }
}

TEST_CASE("A new world is empty and has zero gravity", "[core][world]") {
    const World world;
    REQUIRE(world.getBodyCount() == 0);
    REQUIRE(world.getBodies().empty());
    REQUIRE(world.getBody(0) == nullptr);
    REQUIRE(world.getBody(1) == nullptr);
    REQUIRE(world.findPotentialCollisions().empty());
    requireVec3(world.getGravity(), {});
}

TEST_CASE("World creates bodies with unique IDs", "[core][world]") {
    World world;
    const BodyId sphere = world.createBody();
    const BodyId box = world.createBody(Body{
        {3.0, -2.0, 1.0}, {2.0, 4.0, -6.0}, 0.5, BoxCollider{{1.0, 2.0, 3.0}}
    });

    REQUIRE(sphere == 0);
    REQUIRE(box == 1);
    REQUIRE(world.getBodyCount() == 2);
    REQUIRE(world.getBody(sphere) != nullptr);
    REQUIRE(world.getBody(box) != nullptr);
    REQUIRE(world.getBody(sphere)->getId() == sphere);
    REQUIRE(world.getBody(box)->getId() == box);
    REQUIRE(world.getBody(sphere)->getInverseMass() == 1.0);
    REQUIRE(std::holds_alternative<SphereCollider>(world.getBody(sphere)->getCollider()));
    REQUIRE(std::holds_alternative<BoxCollider>(world.getBody(box)->getCollider()));
    REQUIRE(world.getBody(box)->getInverseMass() == 0.5);
    requireVec3(world.getBody(box)->getVelocity(), {2.0, 4.0, -6.0});
    requireBounds(*world.getBody(sphere), {-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0});
    requireBounds(*world.getBody(box), {2.0, -4.0, -2.0}, {4.0, 0.0, 4.0});
    REQUIRE(world.getBodies()[0].getId() == sphere);
    REQUIRE(world.getBodies()[1].getId() == box);
}

TEST_CASE("World copies body state while assigning its own IDs", "[core][world]") {
    World world;
    const BodyId first = world.createBody();
    Body source{first, {3.0, -2.0, 1.0}, {2.0, 4.0, -6.0}, 0.5, BoxCollider{{1.0, 2.0, 3.0}}};
    BodyId copy = world.createBody(source);
    REQUIRE(copy != first);
    REQUIRE(source.getId() == first);
    requireVec3(source.getPosition(), {3.0, -2.0, 1.0});
    source.setPosition({});
    requireVec3(world.getBody(copy)->getPosition(), {3.0, -2.0, 1.0});

    // Copy from the world's own storage repeatedly, including across growth.
    for (int i = 0; i < 64; ++i) {
        const BodyId next = world.createBody(*world.getBody(copy));
        REQUIRE(next > copy);
        copy = next;
        REQUIRE(world.getBody(copy)->getId() == copy);
        requireVec3(world.getBody(copy)->getPosition(), {3.0, -2.0, 1.0});
        requireVec3(world.getBody(copy)->getVelocity(), {2.0, 4.0, -6.0});
        REQUIRE(world.getBody(copy)->getInverseMass() == 0.5);
        requireBounds(*world.getBody(copy), {2.0, -4.0, -2.0}, {4.0, 0.0, 4.0});
    }
}

TEST_CASE("World IDs survive storage growth and removal", "[core][world]") {
    World world;
    std::vector<BodyId> ids;
    for (int i = 0; i < 256; ++i) {
        ids.push_back(world.createBody(Body{{static_cast<double>(i), 0.0, 0.0}}));
    }

    REQUIRE_FALSE(world.removeBody(ids.back() + 1));
    REQUIRE(world.removeBody(ids[127]));
    REQUIRE(world.removeBody(ids.front()));
    REQUIRE(world.removeBody(ids.back()));
    // Missing IDs below, within, and above the remaining sorted range must
    // not select the next body returned by lower_bound.
    for (const BodyId missingId : {ids.front(), ids[127], ids.back(), std::numeric_limits<BodyId>::max()}) {
        REQUIRE(world.getBody(missingId) == nullptr);
        REQUIRE_FALSE(world.removeBody(missingId));
        REQUIRE_FALSE(world.setBodyPosition(missingId, {}));
        REQUIRE_FALSE(world.setBodyVelocity(missingId, {}));
        REQUIRE_FALSE(world.setBodyCollider(missingId, SphereCollider{}));
    }
    REQUIRE(world.getBodyCount() == 253);
    for (std::size_t i = 1; i + 1 < ids.size(); ++i) {
        if (i == 127) {
            continue;
        }
        REQUIRE(world.getBody(ids[i]) != nullptr);
        REQUIRE(world.getBody(ids[i])->getId() == ids[i]);
        REQUIRE(world.getBody(ids[i])->getPosition().x == static_cast<double>(i));
    }
    REQUIRE(world.getBodies().front().getId() == ids[1]);
    REQUIRE(world.getBodies().back().getId() == ids[254]);
    const BodyId appended = world.createBody();
    REQUIRE(appended > ids.back());
    REQUIRE(world.getBody(appended) != nullptr);
    REQUIRE(world.getBody(appended)->getId() == appended);
    REQUIRE(world.setBodyVelocity(ids[128], {1.0, 2.0, 3.0}));
    requireVec3(world.getBody(ids[128])->getVelocity(), {1.0, 2.0, 3.0});
    REQUIRE(world.getBodies().back().getId() == appended);
}

TEST_CASE("Clearing a world preserves gravity without recycling IDs", "[core][world]") {
    World world{{0.0, -2.0, 0.0}};
    const BodyId first = world.createBody();
    const BodyId second = world.createBody();
    world.clear();
    world.clear();

    REQUIRE(world.getBodies().empty());
    REQUIRE(world.getBodyCount() == 0);
    REQUIRE(world.getBody(first) == nullptr);
    REQUIRE(world.getBody(second) == nullptr);
    REQUIRE(world.findPotentialCollisions().empty());
    requireVec3(world.getGravity(), {0.0, -2.0, 0.0});
    REQUIRE(world.createBody() == second + 1);
}

TEST_CASE("World body edits keep bounds synchronized", "[core][world]") {
    World world;
    const BodyId id = world.createBody();

    REQUIRE(world.setBodyPosition(id, {4.0, -2.0, 8.0}));
    requireBounds(*world.getBody(id), {3.0, -3.0, 7.0}, {5.0, -1.0, 9.0});
    REQUIRE(world.setBodyCollider(id, BoxCollider{{2.0, 1.0, 0.0}}));
    requireBounds(*world.getBody(id), {2.0, -3.0, 8.0}, {6.0, -1.0, 8.0});
    REQUIRE(world.setBodyVelocity(id, {-1.0, 2.0, 3.0}));
    requireVec3(world.getBody(id)->getVelocity(), {-1.0, 2.0, 3.0});
    requireBounds(*world.getBody(id), {2.0, -3.0, 8.0}, {6.0, -1.0, 8.0});

    const BodyId missingId = id + 1;
    REQUIRE_FALSE(world.setBodyPosition(missingId, {}));
    REQUIRE_FALSE(world.setBodyVelocity(missingId, {}));
    REQUIRE_FALSE(world.setBodyCollider(missingId, SphereCollider{}));
    REQUIRE(world.removeBody(id));
    REQUIRE_FALSE(world.setBodyPosition(id, {}));
    REQUIRE_FALSE(world.setBodyVelocity(id, {}));
    REQUIRE_FALSE(world.setBodyCollider(id, SphereCollider{}));
}

TEST_CASE("Invalid bodies leave a world and its IDs unchanged", "[core][world]") {
    World world;
    Vec3 position{};
    Vec3 velocity{};
    double inverseMass = 1.0;
    Collider collider = SphereCollider{};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    SECTION("Non-finite position") { position.x = nan; }
    SECTION("Non-finite velocity") { velocity.z = inf; }
    SECTION("Negative inverse mass") { inverseMass = -1.0; }
    SECTION("Infinite inverse mass") { inverseMass = inf; }
    SECTION("NaN inverse mass") { inverseMass = nan; }
    SECTION("Negative radius") { collider = SphereCollider{-1.0}; }
    SECTION("Infinite radius") { collider = SphereCollider{inf}; }
    SECTION("NaN radius") { collider = SphereCollider{nan}; }
    SECTION("Negative box x extent") { collider = BoxCollider{{-1.0, 1.0, 1.0}}; }
    SECTION("Negative box y extent") { collider = BoxCollider{{1.0, -1.0, 1.0}}; }
    SECTION("Negative box z extent") { collider = BoxCollider{{1.0, 1.0, -1.0}}; }
    SECTION("Infinite box extent") { collider = BoxCollider{{1.0, inf, 1.0}}; }
    SECTION("NaN box extent") { collider = BoxCollider{{1.0, 1.0, nan}}; }

    REQUIRE_THROWS_AS(world.createBody(Body{position, velocity, inverseMass, collider}), std::invalid_argument);
    REQUIRE(world.getBodyCount() == 0);
    REQUIRE(world.createBody() == 0);
}

TEST_CASE("Invalid world edits do not change existing state", "[core][world]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    World world{{0.0, -2.0, 0.0}};
    const BodyId id = world.createBody();

    REQUIRE_THROWS_AS((World{Vec3{inf, 0.0, 0.0}}), std::invalid_argument);
    REQUIRE_THROWS_AS(world.setGravity({0.0, nan, 0.0}), std::invalid_argument);
    REQUIRE_THROWS_AS(world.setBodyPosition(id, {0.0, 0.0, -inf}), std::invalid_argument);
    REQUIRE_THROWS_AS(world.setBodyVelocity(id, {nan, 0.0, 0.0}), std::invalid_argument);
    REQUIRE_THROWS_AS(world.setBodyCollider(id, SphereCollider{-1.0}), std::invalid_argument);
    REQUIRE_THROWS_AS(world.setBodyCollider(id, BoxCollider{{0.0, inf, 0.0}}), std::invalid_argument);
    requireVec3(world.getGravity(), {0.0, -2.0, 0.0});
    requireVec3(world.getBody(id)->getPosition(), {});
    requireVec3(world.getBody(id)->getVelocity(), {});
    requireBounds(*world.getBody(id), {-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0});
}

TEST_CASE("Zero-gravity stepping moves dynamic bodies and refreshes bounds", "[core][world][step]") {
    World world;
    const BodyId id = world.createBody(Body{
        {1.0, 2.0, 3.0}, {4.0, -2.0, 6.0}, 1.0, BoxCollider{{0.5, 1.0, 2.0}}
    });
    world.step(0.5);

    requireVec3(world.getBody(id)->getPosition(), {3.0, 1.0, 6.0});
    requireVec3(world.getBody(id)->getVelocity(), {4.0, -2.0, 6.0});
    requireBounds(*world.getBody(id), {2.5, 0.0, 4.0}, {3.5, 2.0, 8.0});
}

TEST_CASE("Gravity uses semi-implicit Euler independently of dynamic mass", "[core][world][step]") {
    World world{{2.0, -4.0, 6.0}};
    const BodyId first = world.createBody();
    const BodyId second = world.createBody(Body{{}, {}, 0.25});
    world.step(0.5);

    for (const BodyId id : {first, second}) {
        requireVec3(world.getBody(id)->getVelocity(), {1.0, -2.0, 3.0});
        requireVec3(world.getBody(id)->getPosition(), {0.5, -1.0, 1.5});
    }
    world.step(0.5);
    requireVec3(world.getBody(first)->getVelocity(), {2.0, -4.0, 6.0});
    requireVec3(world.getBody(first)->getPosition(), {1.5, -3.0, 4.5});
    world.setGravity({});
    world.step(0.5);
    requireVec3(world.getBody(first)->getVelocity(), {2.0, -4.0, 6.0});
    requireVec3(world.getBody(first)->getPosition(), {2.5, -5.0, 7.5});
}

TEST_CASE("Static bodies remain fixed even with velocity and gravity", "[core][world][step]") {
    World world{{0.0, -4.0, 0.0}};
    const BodyId id = world.createBody(Body{
        {1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, 0.0
    });
    world.step(2.0);
    requireVec3(world.getBody(id)->getPosition(), {1.0, 2.0, 3.0});
    requireVec3(world.getBody(id)->getVelocity(), {4.0, 5.0, 6.0});
    requireBounds(*world.getBody(id), {0.0, 1.0, 2.0}, {2.0, 3.0, 4.0});
}

TEST_CASE("Zero and invalid time steps leave bodies unchanged", "[core][world][step]") {
    World world{{0.0, -4.0, 0.0}};
    REQUIRE_NOTHROW(world.step(1.0));
    REQUIRE_THROWS_AS(world.step(-1.0), std::invalid_argument);
    const BodyId id = world.createBody(Body{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}});
    world.step(0.0);
    for (const double dt : {-1.0, std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        REQUIRE_THROWS_AS(world.step(dt), std::invalid_argument);
    }
    requireVec3(world.getBody(id)->getPosition(), {1.0, 2.0, 3.0});
    requireVec3(world.getBody(id)->getVelocity(), {4.0, 5.0, 6.0});
    requireBounds(*world.getBody(id), {0.0, 1.0, 2.0}, {2.0, 3.0, 4.0});
}

TEST_CASE("AABB overlap includes touching bounds and rejects separation on every axis", "[core][world][aabb]") {
    constexpr AABB box{{-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}};
    STATIC_REQUIRE(overlaps(box, box));
    Vec3 center;
    bool expected = false;
    SECTION("Positive x separation") { center = {2.25, 0.0, 0.0}; }
    SECTION("Negative x separation") { center = {-2.25, 0.0, 0.0}; }
    SECTION("Positive y separation") { center = {0.0, 2.25, 0.0}; }
    SECTION("Negative y separation") { center = {0.0, -2.25, 0.0}; }
    SECTION("Positive z separation") { center = {0.0, 0.0, 2.25}; }
    SECTION("Negative z separation") { center = {0.0, 0.0, -2.25}; }
    SECTION("Face touching") { center = {2.0, 0.0, 0.0}; expected = true; }
    SECTION("Edge touching") { center = {2.0, 2.0, 0.0}; expected = true; }
    SECTION("Corner touching") { center = {2.0, 2.0, 2.0}; expected = true; }
    SECTION("Contained bounds") { center = {}; expected = true; }

    const AABB other = computeAABB(center, SphereCollider{1.0});
    REQUIRE(overlaps(box, other) == expected);
    REQUIRE(overlaps(other, box) == expected);
}

TEST_CASE("World reports each potential collision once and omits static pairs", "[core][world][collision]") {
    World world;
    const BodyId first = world.createBody();
    REQUIRE(world.findPotentialCollisions().empty());
    const BodyId second = world.createBody(Body{{}, {}, 1.0, BoxCollider{}});
    const BodyId third = world.createBody(Body{{}, {}, 0.0});
    const BodyId fourth = world.createBody(Body{{}, {}, 0.0});
    world.createBody(Body{{10.0, 0.0, 0.0}});
    const std::vector<std::pair<BodyId, BodyId>> expected{
        {first, second}, {first, third}, {first, fourth}, {second, third}, {second, fourth}
    };
    requirePairIds(world.findPotentialCollisions(), expected);
    requirePairIds(world.findPotentialCollisions(), expected);

    REQUIRE(world.removeBody(second));
    const std::vector<std::pair<BodyId, BodyId>> afterRemoval{{first, third}, {first, fourth}};
    requirePairIds(world.findPotentialCollisions(), afterRemoval);
}

TEST_CASE("Potential collisions reflect movement and collider changes immediately", "[core][world][collision]") {
    World world;
    const BodyId first = world.createBody();
    const BodyId second = world.createBody(Body{{4.0, 0.0, 0.0}, {-2.0, 0.0, 0.0}});
    const std::vector<std::pair<BodyId, BodyId>> pair{{first, second}};
    REQUIRE(world.findPotentialCollisions().empty());
    world.step(1.0);
    requirePairIds(world.findPotentialCollisions(), pair);
    REQUIRE(world.setBodyPosition(second, {3.0, 0.0, 0.0}));
    REQUIRE(world.findPotentialCollisions().empty());
    REQUIRE(world.setBodyCollider(second, SphereCollider{2.0}));
    requirePairIds(world.findPotentialCollisions(), pair);

    // AABB candidates deliberately include false positives for actual spheres.
    REQUIRE(world.setBodyCollider(second, SphereCollider{1.0}));
    REQUIRE(world.setBodyPosition(second, {2.0, 2.0, 2.0}));
    requirePairIds(world.findPotentialCollisions(), pair);
}

TEST_CASE("Point colliders participate in potential collisions", "[core][world][collision]") {
    World world;
    const BodyId first = world.createBody(Body{{}, {}, 1.0, SphereCollider{0.0}});
    const BodyId second = world.createBody(Body{{}, {}, 1.0, BoxCollider{{0.0, 0.0, 0.0}}});
    const std::vector<std::pair<BodyId, BodyId>> pair{{first, second}};
    requirePairIds(world.findPotentialCollisions(), pair);
    REQUIRE(world.setBodyPosition(second, {0.0, 0.0, 0.25}));
    REQUIRE(world.findPotentialCollisions().empty());
}

TEST_CASE("Potential collision pairs own independent body snapshots", "[core][world][collision]") {
    World world;
    const BodyId sphere = world.createBody(Body{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, 0.5, SphereCollider{2.0}});
    const BodyId box = world.createBody(Body{{2.0, 2.0, 3.0}, {}, 0.0, BoxCollider{{1.0, 2.0, 3.0}}});
    auto pairs = world.findPotentialCollisions();
    requirePairIds(pairs, {{sphere, box}});

    pairs[0].first.setPosition({});
    requireVec3(world.getBody(sphere)->getPosition(), {1.0, 2.0, 3.0});
    REQUIRE(world.setBodyPosition(box, {10.0, 20.0, 30.0}));
    world.clear();

    // Returned bodies retain all their state even after the World is cleared.
    requirePairIds(pairs, {{sphere, box}});
    requireVec3(pairs[0].first.getPosition(), {});
    requireVec3(pairs[0].first.getVelocity(), {4.0, 5.0, 6.0});
    REQUIRE(pairs[0].first.getInverseMass() == 0.5);
    REQUIRE(std::get<SphereCollider>(pairs[0].first.getCollider()).radius == 2.0);
    requireBounds(pairs[0].first, {-2.0, -2.0, -2.0}, {2.0, 2.0, 2.0});
    requireVec3(pairs[0].second.getPosition(), {2.0, 2.0, 3.0});
    requireVec3(pairs[0].second.getVelocity(), {});
    REQUIRE(pairs[0].second.getInverseMass() == 0.0);
    requireVec3(std::get<BoxCollider>(pairs[0].second.getCollider()).halfExtents, {1.0, 2.0, 3.0});
    requireBounds(pairs[0].second, {1.0, 0.0, 0.0}, {3.0, 4.0, 6.0});
}
