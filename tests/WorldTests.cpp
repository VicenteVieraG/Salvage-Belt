#include <catch2/catch_test_macros.hpp>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include "salvageCore.hpp"

using namespace salvage::core;

static_assert(std::is_same_v<decltype(std::declval<World&>().getBody(1)), const Body*>);
static_assert(std::is_same_v<decltype(std::declval<World&>().getBodies()), std::span<const Body>>);

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

TEST_CASE("World creates bodies from definitions with unique IDs", "[core][world]") {
    World world;
    const BodyId sphere = world.createBody();
    const BodyId box = world.createBody({
        .position = {3.0, -2.0, 1.0},
        .velocity = {2.0, 4.0, -6.0},
        .inverseMass = 0.5,
        .collider = BoxCollider{{1.0, 2.0, 3.0}}
    });

    REQUIRE(sphere != 0);
    REQUIRE(box != sphere);
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

TEST_CASE("World IDs survive storage growth and removal", "[core][world]") {
    World world;
    std::vector<BodyId> ids;
    for (int i = 0; i < 256; ++i) {
        ids.push_back(world.createBody({.position = {static_cast<double>(i), 0.0, 0.0}}));
    }

    REQUIRE_FALSE(world.removeBody(0));
    REQUIRE(world.removeBody(ids[127]));
    REQUIRE(world.removeBody(ids.front()));
    REQUIRE(world.removeBody(ids.back()));
    REQUIRE_FALSE(world.removeBody(ids[127]));
    REQUIRE(world.getBody(ids[127]) == nullptr);
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
    REQUIRE(world.createBody() > ids.back());
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
    REQUIRE(world.createBody() > second);
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

    REQUIRE_FALSE(world.setBodyPosition(0, {}));
    REQUIRE_FALSE(world.setBodyVelocity(0, {}));
    REQUIRE_FALSE(world.setBodyCollider(0, SphereCollider{}));
    REQUIRE(world.removeBody(id));
    REQUIRE_FALSE(world.setBodyPosition(id, {}));
    REQUIRE_FALSE(world.setBodyVelocity(id, {}));
    REQUIRE_FALSE(world.setBodyCollider(id, SphereCollider{}));
}

TEST_CASE("Invalid body definitions leave a world and its IDs unchanged", "[core][world]") {
    World world;
    BodyDefinition definition;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    SECTION("Non-finite position") { definition.position.x = nan; }
    SECTION("Non-finite velocity") { definition.velocity.z = inf; }
    SECTION("Negative inverse mass") { definition.inverseMass = -1.0; }
    SECTION("Infinite inverse mass") { definition.inverseMass = inf; }
    SECTION("NaN inverse mass") { definition.inverseMass = nan; }
    SECTION("Negative radius") { definition.collider = SphereCollider{-1.0}; }
    SECTION("Infinite radius") { definition.collider = SphereCollider{inf}; }
    SECTION("NaN radius") { definition.collider = SphereCollider{nan}; }
    SECTION("Negative box x extent") { definition.collider = BoxCollider{{-1.0, 1.0, 1.0}}; }
    SECTION("Negative box y extent") { definition.collider = BoxCollider{{1.0, -1.0, 1.0}}; }
    SECTION("Negative box z extent") { definition.collider = BoxCollider{{1.0, 1.0, -1.0}}; }
    SECTION("Infinite box extent") { definition.collider = BoxCollider{{1.0, inf, 1.0}}; }
    SECTION("NaN box extent") { definition.collider = BoxCollider{{1.0, 1.0, nan}}; }

    REQUIRE_THROWS_AS(world.createBody(definition), std::invalid_argument);
    REQUIRE(world.getBodyCount() == 0);
    REQUIRE(world.createBody() == 1);
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
    const BodyId id = world.createBody({
        .position = {1.0, 2.0, 3.0},
        .velocity = {4.0, -2.0, 6.0},
        .collider = BoxCollider{{0.5, 1.0, 2.0}}
    });
    world.step(0.5);

    requireVec3(world.getBody(id)->getPosition(), {3.0, 1.0, 6.0});
    requireVec3(world.getBody(id)->getVelocity(), {4.0, -2.0, 6.0});
    requireBounds(*world.getBody(id), {2.5, 0.0, 4.0}, {3.5, 2.0, 8.0});
}

TEST_CASE("Gravity uses semi-implicit Euler independently of dynamic mass", "[core][world][step]") {
    World world{{2.0, -4.0, 6.0}};
    const BodyId first = world.createBody();
    const BodyId second = world.createBody({.inverseMass = 0.25});
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
    const BodyId id = world.createBody({
        .position = {1.0, 2.0, 3.0}, .velocity = {4.0, 5.0, 6.0}, .inverseMass = 0.0
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
    const BodyId id = world.createBody({.position = {1.0, 2.0, 3.0}, .velocity = {4.0, 5.0, 6.0}});
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
    const BodyId second = world.createBody({.collider = BoxCollider{}});
    const BodyId third = world.createBody({.inverseMass = 0.0});
    const BodyId fourth = world.createBody({.inverseMass = 0.0});
    world.createBody({.position = {10.0, 0.0, 0.0}});
    const std::vector<BodyPair> expected{
        {first, second}, {first, third}, {first, fourth}, {second, third}, {second, fourth}
    };
    REQUIRE(world.findPotentialCollisions() == expected);
    REQUIRE(world.findPotentialCollisions() == expected);

    REQUIRE(world.removeBody(second));
    const std::vector<BodyPair> afterRemoval{{first, third}, {first, fourth}};
    REQUIRE(world.findPotentialCollisions() == afterRemoval);
}

TEST_CASE("Potential collisions reflect movement and collider changes immediately", "[core][world][collision]") {
    World world;
    const BodyId first = world.createBody();
    const BodyId second = world.createBody({.position = {4.0, 0.0, 0.0}, .velocity = {-2.0, 0.0, 0.0}});
    const std::vector<BodyPair> pair{{first, second}};
    REQUIRE(world.findPotentialCollisions().empty());
    world.step(1.0);
    REQUIRE(world.findPotentialCollisions() == pair);
    REQUIRE(world.setBodyPosition(second, {3.0, 0.0, 0.0}));
    REQUIRE(world.findPotentialCollisions().empty());
    REQUIRE(world.setBodyCollider(second, SphereCollider{2.0}));
    REQUIRE(world.findPotentialCollisions() == pair);

    // AABB candidates deliberately include false positives for actual spheres.
    REQUIRE(world.setBodyCollider(second, SphereCollider{1.0}));
    REQUIRE(world.setBodyPosition(second, {2.0, 2.0, 2.0}));
    REQUIRE(world.findPotentialCollisions() == pair);
}

TEST_CASE("Point colliders participate in potential collisions", "[core][world][collision]") {
    World world;
    const BodyId first = world.createBody({.collider = SphereCollider{0.0}});
    const BodyId second = world.createBody({.collider = BoxCollider{{0.0, 0.0, 0.0}}});
    const std::vector<BodyPair> pair{{first, second}};
    REQUIRE(world.findPotentialCollisions() == pair);
    REQUIRE(world.setBodyPosition(second, {0.0, 0.0, 0.25}));
    REQUIRE(world.findPotentialCollisions().empty());
}
