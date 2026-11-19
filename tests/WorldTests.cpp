/**
 * @file
 * @brief World ownership, validation, integration, and collision-candidate tests.
 */
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include "salvageCore.hpp"

using namespace salvage::core;

static_assert(std::is_same_v<decltype(std::declval<World&>().getBody(1)), const Body*>);
static_assert(std::is_same_v<decltype(std::declval<World&>().getBodies()), std::span<const Body>>);
static_assert(std::is_same_v<decltype(std::declval<const World&>().findPotentialCollisions()),
    std::vector<BodyPair>>);

namespace {
    /**
     * @brief Assert equality of every vector component.
     * @param actual Observed vector.
     * @param expected Required vector.
     */
    void requireVec3(const Vec3& actual, const Vec3& expected) {
        REQUIRE(actual.x == expected.x);
        REQUIRE(actual.y == expected.y);
        REQUIRE(actual.z == expected.z);
    }

    /**
     * @brief Assert the cached bounds of a body.
     * @param body Body to inspect.
     * @param min Required minimum corner.
     * @param max Required maximum corner.
     */
    void requireBounds(const Body& body, const Vec3& min, const Vec3& max) {
        requireVec3(body.aabb().min, min);
        requireVec3(body.aabb().max, max);
    }

    /**
     * @brief Assert the ordered IDs in collision candidates.
     * @param actual Observed body pairs.
     * @param expected Required identifier pairs.
     */
    void requirePairIds(const std::vector<BodyPair>& actual,
        const std::vector<std::pair<BodyId, BodyId>>& expected) {
        REQUIRE(actual.size() == expected.size());
        for (std::size_t i = 0; i < actual.size(); ++i) {
            REQUIRE(actual[i].first.getId() == expected[i].first);
            REQUIRE(actual[i].second.getId() == expected[i].second);
        }
    }
}

/**
 * @brief A new world is empty and has zero gravity.
 * @test A new world is empty and has zero gravity.
 */
TEST_CASE("A new world is empty and has zero gravity", "[core][world]") {
    const World world;
    REQUIRE(world.getBodyCount() == 0);
    REQUIRE(world.getBodies().empty());
    REQUIRE(world.getBody(0) == nullptr);
    REQUIRE(world.getBody(1) == nullptr);
    REQUIRE(world.findPotentialCollisions().empty());
    requireVec3(world.getGravity(), {});
}

/**
 * @brief World creates bodies with unique IDs.
 * @test World creates bodies with unique IDs.
 */
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

TEST_CASE("Random bodies repeat from a seed and fit within configured bounds", "[core][world][generation]") {
    const RandomBodySettings settings{
        .spawnBounds = {{-10.0, -12.0, -14.0}, {12.0, 14.0, 16.0}},
        .extent = {0.25, 1.0},
        .velocity = {-2.0, 1.0},
        .inverseMass = {0.5, 1.5},
        .restitution = {0.1, 0.8},
        .friction = {0.2, 0.9}
    };
    const auto inRange = [](double value, const RandomBodySettings::Range& range) {
        return range.min <= value && value <= range.max;
    };

    World world;
    World repeated;
    constexpr std::size_t count = 48;
    world.generateRandomBodies(count, 42, settings);
    repeated.generateRandomBodies(count, 42, settings);
    REQUIRE(world.getBodyCount() == count);
    REQUIRE(repeated.getBodyCount() == count);
    REQUIRE(world.findPotentialCollisions().empty());

    World different;
    different.generateRandomBodies(1, 43, settings);
    REQUIRE(different.getBodies().front().getPosition().x != world.getBodies().front().getPosition().x);

    for (std::size_t i = 0; i < count; ++i) {
        const auto& body = world.getBodies()[i];
        const auto& copy = repeated.getBodies()[i];
        const Vec3 extents = std::visit([](const auto& shape) { return getExtents(shape); }, body.getCollider());
        REQUIRE(body.getId() == i);
        REQUIRE(body.getCollider().index() == copy.getCollider().index());
        requireVec3(body.getPosition(), copy.getPosition());
        requireVec3(body.getVelocity(), copy.getVelocity());
        requireBounds(body, copy.aabb().min, copy.aabb().max);
        for (const auto axis : {&Vec3::x, &Vec3::y, &Vec3::z}) {
            REQUIRE(body.aabb().min.*axis >= settings.spawnBounds.min.*axis);
            REQUIRE(body.aabb().max.*axis <= settings.spawnBounds.max.*axis);
            REQUIRE(inRange(extents.*axis, settings.extent));
            REQUIRE(inRange(body.getVelocity().*axis, settings.velocity));
        }
        REQUIRE(inRange(body.getInverseMass(), settings.inverseMass));
        REQUIRE(body.getInverseMass() == copy.getInverseMass());
        REQUIRE(inRange(body.getMaterial().restitution, settings.restitution));
        REQUIRE(body.getMaterial().restitution == copy.getMaterial().restitution);
        REQUIRE(inRange(body.getMaterial().friction, settings.friction));
        REQUIRE(body.getMaterial().friction == copy.getMaterial().friction);
    }
}

TEST_CASE("Crowded random placement preserves the world and its next body ID", "[core][world][generation]") {
    const RandomBodySettings settings{
        .spawnBounds = {{-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}},
        .extent = {1.0, 1.0},
        .velocity = {0.0, 0.0},
        .inverseMass = {0.0, 0.0},
        .maxPlacementAttempts = 2
    };
    World world;
    const BodyId existing = world.createBody(Body{{4.0, 0.0, 0.0}});
    REQUIRE_THROWS_AS(world.generateRandomBodies(2, 42, settings), std::runtime_error);
    REQUIRE(world.getBodyCount() == 1);
    requireVec3(world.getBody(existing)->getPosition(), {4.0, 0.0, 0.0});

    world.generateRandomBodies(1, 42, settings);
    REQUIRE(world.getBodyCount() == 2);
    REQUIRE(world.getBodies().back().getId() == existing + 1);
    REQUIRE(world.getBodies().back().getInverseMass() == 0.0);
    requireVec3(world.getBodies().back().getVelocity(), {});
    requireBounds(world.getBodies().back(), settings.spawnBounds.min, settings.spawnBounds.max);

    REQUIRE_THROWS_AS(world.generateRandomBodies(1, 42, settings), std::runtime_error);
    REQUIRE(world.getBodyCount() == 2);
    REQUIRE(world.createBody() == existing + 2);
}

TEST_CASE("Random generation rejects invalid limits before adding bodies", "[core][world][generation]") {
    World world;
    RandomBodySettings settings;
    SECTION("Reversed extent range") { settings.extent = {2.0, 1.0}; }
    SECTION("Non-finite velocity range") { settings.velocity.max = std::numeric_limits<double>::infinity(); }
    SECTION("Restitution outside the material limits") { settings.restitution.max = 1.1; }
    SECTION("Reversed spawn bounds") { settings.spawnBounds.min.z = settings.spawnBounds.max.z + 1.0; }
    SECTION("Zero placement attempts") { settings.maxPlacementAttempts = 0; }

    REQUIRE_NOTHROW(world.generateRandomBodies(0, 42, settings));
    REQUIRE_THROWS_AS(world.generateRandomBodies(1, 42, settings), std::invalid_argument);
    REQUIRE(world.getBodyCount() == 0);
    REQUIRE(world.createBody() == 0);
}

TEST_CASE("Random spawn space scales with body count and spacing", "[core][world][generation]") {
    const auto small = RandomBodySettings::forBodyCount(8, 5.0);
    const auto large = RandomBodySettings::forBodyCount(64, 5.0);
    const auto wider = RandomBodySettings::forBodyCount(8, 10.0);
    REQUIRE(small.spawnBounds.max.x == Catch::Approx(5.0));
    REQUIRE(large.spawnBounds.min.y == Catch::Approx(-10.0));
    REQUIRE(wider.spawnBounds.max.z == Catch::Approx(10.0));
    REQUIRE_THROWS_AS(RandomBodySettings::forBodyCount(8, 0.0), std::invalid_argument);

    World world;
    world.generateRandomBodies(8'000, 42);
    REQUIRE(world.getBodyCount() == 8'000);
    bool outsideOriginalBounds = false;
    const auto bounds = RandomBodySettings::forBodyCount(8'000).spawnBounds;
    for (const Body& body : world.getBodies()) {
        REQUIRE(body.aabb().min.x >= bounds.min.x);
        REQUIRE(body.aabb().max.x <= bounds.max.x);
        outsideOriginalBounds |= body.aabb().max.x > 50.0;
    }
    REQUIRE(outsideOriginalBounds);
}

TEST_CASE("Random placement checks large obstacles extending outside the spawn region", "[core][world][generation]") {
    const RandomBodySettings settings{.extent = {0.25, 0.25}};
    World reference;
    reference.generateRandomBodies(1, 42, settings);
    const Body& candidate = reference.getBodies().front();
    const Body obstacle{candidate.getPosition() + Vec3{128.0, 0.0, 0.0}, {}, 0.0,
        BoxCollider{{128.0, 4.0, 4.0}}};
    REQUIRE(obstacle.getPosition().x > settings.spawnBounds.max.x);
    REQUIRE(overlaps(candidate.aabb(), obstacle.aabb()));

    World world;
    world.createBody(obstacle);
    world.generateRandomBodies(128, 42, settings);
    REQUIRE(world.getBodyCount() == 129);
    REQUIRE(world.findPotentialCollisions().empty());
}

/**
 * @brief World copies body state while assigning its own IDs.
 * @test World copies body state while assigning its own IDs. Copy from the world's own storage repeatedly, including across growth.
 */
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

/**
 * @brief World IDs survive storage growth and removal.
 * @test World IDs survive storage growth and removal. Missing IDs below, within, and above the remaining sorted range must not select the next body returned by lower_bound.
 */
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

/**
 * @brief Clearing a world preserves gravity without recycling IDs.
 * @test Clearing a world preserves gravity without recycling IDs.
 */
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

/**
 * @brief World body edits keep bounds synchronized.
 * @test World body edits keep bounds synchronized.
 */
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

/**
 * @brief Invalid bodies leave a world and its IDs unchanged.
 * @test Invalid bodies leave a world and its IDs unchanged.
 */
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

/**
 * @brief Invalid world edits do not change existing state.
 * @test Invalid world edits do not change existing state.
 */
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

/**
 * @brief Zero-gravity stepping moves dynamic bodies and refreshes bounds.
 * @test Zero-gravity stepping moves dynamic bodies and refreshes bounds.
 */
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

TEST_CASE("A generated body rebounds from a wall on the step it reaches it", "[core][world][step][generation]") {
    World world;
    const auto wall = world.createBody(Body{{4.0, 0.0, 0.0}, {}, 0.0, BoxCollider{{1.0, 2.0, 2.0}}});
    // Leave a gap between IDs so a candidate ID cannot double as a storage index.
    const auto removed = world.createBody();
    REQUIRE(world.removeBody(removed));
    world.generateRandomBodies(1, 42, {
        .spawnBounds = {{-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}},
        .extent = {1.0, 1.0},
        .velocity = {0.0, 0.0},
        .inverseMass = {1.0, 1.0},
        .restitution = {1.0, 1.0}
    });
    const auto moving = world.getBodies().back().getId();
    REQUIRE(world.findPotentialCollisions().empty());
    REQUIRE(world.addBodyForce(moving, {4.0, 0.0, 0.0}));

    world.step(1.0);
    requirePairIds(world.findPotentialCollisions(), {{wall, moving}});
    requireVec3(world.getBody(moving)->getPosition(), {2.0, 0.0, 0.0});
    requireVec3(world.getBody(moving)->getVelocity(), {-4.0, 0.0, 0.0});

    world.step(0.5);
    requireVec3(world.getBody(moving)->getPosition(), {});
    REQUIRE(world.findPotentialCollisions().empty());
}

TEST_CASE("Position correction discovers a new pair on the next pass", "[core][world][step]") {
    World world;
    world.setSolverSettings({.positionIterations = 2, .penetrationSlop = 0.0, .positionCorrection = 1.0});
    const auto fixed = world.createBody(Body{{}, {}, 0.0});
    const auto middle = world.createBody(Body{{1.0, 0.0, 0.0}});
    const auto last = world.createBody(Body{{3.5, 0.0, 0.0}});
    requirePairIds(world.findPotentialCollisions(), {{fixed, middle}});

    // Moving the middle sphere out of the fixed sphere pushes it into the last sphere.
    world.step(0.1);
    REQUIRE(world.getBody(middle)->getPosition().x == 1.75);
    REQUIRE(world.getBody(last)->getPosition().x == 3.75);
}

TEST_CASE("World steps and queries use the selected broad phase", "[core][world][step][broadphase]") {
    for (const auto algorithm : {Algorithm::BruteForce, Algorithm::KDTree, Algorithm::SweepAndPrune}) {
        CAPTURE(algorithm);
        World world{algorithm, {-2.0, 0.0, 0.0}};
        world.setSolverSettings({.penetrationSlop = 0.0, .positionCorrection = 1.0});
        const auto fixed = world.createBody(Body{{}, {}, 0.0});
        const auto moving = world.createBody(Body{{2.0, 0.0, 0.0}, {}, 1.0, SphereCollider{}, Material{1.0, 0.0}});

        requirePairIds(world.findPotentialCollisions(), {{fixed, moving}});
        REQUIRE(world.findContacts().size() == 1);

        world.step(1.0);
        REQUIRE(world.getBody(moving)->getVelocity().x == 2.0);
        REQUIRE(world.getBody(moving)->getPosition().x == 2.0);
    }
    REQUIRE_THROWS_AS(World(static_cast<Algorithm>(-1)), std::invalid_argument);
}

TEST_CASE("Copied worlds keep their strategy and simulate independently", "[core][world][broadphase]") {
    World original{Algorithm::KDTree};
    const auto fixed = original.createBody(Body{{}, {}, 0.0});
    const auto moving = original.createBody(Body{{2.0, 0.0, 0.0}});
    REQUIRE(original.addBodyForce(moving, {-2.0, 0.0, 0.0}));
    World copy = original;
    World assigned;
    assigned = original;

    const auto expected = original.findContacts().size();
    REQUIRE(copy.findContacts().size() == expected);
    REQUIRE(assigned.findContacts().size() == expected);
    REQUIRE(copy.removeBody(fixed));
    copy.step(1.0);
    REQUIRE(copy.getBody(moving)->getVelocity().x == -2.0);
    REQUIRE(original.getBody(moving)->getVelocity().x == 0.0);

    World moved = std::move(assigned);
    moved.step(1.0);
    original.step(1.0);
    requireVec3(moved.getBody(moving)->getPosition(), original.getBody(moving)->getPosition());
    requireVec3(moved.getBody(moving)->getVelocity(), original.getBody(moving)->getVelocity());

    assigned.clear();
    const auto fresh = assigned.createBody(Body{{}, {1.0, 0.0, 0.0}});
    assigned.step(0.5);
    REQUIRE(assigned.getBody(fresh)->getPosition().x == 0.5);
}

/**
 * @brief Gravity uses Velocity Verlet independently of dynamic mass.
 * @test Gravity uses Velocity Verlet independently of dynamic mass.
 */
TEST_CASE("Gravity uses Velocity Verlet independently of dynamic mass", "[core][world][step]") {
    World world{{2.0, -4.0, 6.0}};
    const BodyId first = world.createBody();
    // Keep these bodies apart: this test isolates gravity from contact response.
    const BodyId second = world.createBody(Body{{10.0, 0.0, 0.0}, {}, 0.25});
    world.step(0.5);

    for (const BodyId id : {first, second}) {
        requireVec3(world.getBody(id)->getVelocity(), {1.0, -2.0, 3.0});
        const double offset = id == first ? 0.0 : 10.0;
        requireVec3(world.getBody(id)->getPosition(), {offset + 0.25, -0.5, 0.75});
        requireBounds(*world.getBody(id), {offset - 0.75, -1.5, -0.25}, {offset + 1.25, 0.5, 1.75});
    }
    world.step(0.5);
    requireVec3(world.getBody(first)->getVelocity(), {2.0, -4.0, 6.0});
    requireVec3(world.getBody(first)->getPosition(), {1.0, -2.0, 3.0});
    world.setGravity({});
    world.step(0.5);
    requireVec3(world.getBody(first)->getVelocity(), {2.0, -4.0, 6.0});
    requireVec3(world.getBody(first)->getPosition(), {2.0, -4.0, 6.0});
}

/**
 * @brief Velocity Verlet follows a ballistic trajectory across different time steps.
 * @test Velocity Verlet follows a ballistic trajectory across different time steps.
 */
TEST_CASE("Velocity Verlet follows a ballistic trajectory across different time steps", "[core][world][step]") {
    World world{{2.0, -4.0, 6.0}};
    const BodyId id = world.createBody(Body{{1.0, -2.0, 3.0}, {4.0, 2.0, -2.0}});

    SECTION("Single step") {
        world.step(2.0);
    }

    SECTION("Several unequal steps with the same total duration") {
        world.step(0.25);
        world.step(0.5);
        world.step(1.25);
    }

    requireVec3(world.getBody(id)->getPosition(), {13.0, -6.0, 11.0});
    requireVec3(world.getBody(id)->getVelocity(), {8.0, -6.0, 10.0});
    requireBounds(*world.getBody(id), {12.0, -7.0, 10.0}, {14.0, -5.0, 12.0});
}

/**
 * @brief Static bodies remain fixed even with velocity and gravity.
 * @test Static bodies remain fixed even with velocity and gravity.
 */
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

/**
 * @brief Zero and invalid time steps leave bodies unchanged.
 * @test Zero and invalid time steps leave bodies unchanged.
 */
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

/**
 * @brief AABB overlap includes touching bounds and rejects separation on every axis.
 * @test AABB overlap includes touching bounds and rejects separation on every axis.
 */
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

/**
 * @brief World reports each potential collision once and omits static pairs.
 * @test World reports each potential collision once and omits static pairs.
 */
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

/**
 * @brief Potential collisions reflect movement and collider changes immediately.
 * @test Potential collisions reflect movement and collider changes immediately. AABB candidates deliberately include false positives for actual spheres.
 */
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

    REQUIRE(world.setBodyCollider(second, SphereCollider{1.0}));
    REQUIRE(world.setBodyPosition(second, {2.0, 2.0, 2.0}));
    requirePairIds(world.findPotentialCollisions(), pair);
}

/**
 * @brief Point colliders participate in potential collisions.
 * @test Point colliders participate in potential collisions.
 */
TEST_CASE("Point colliders participate in potential collisions", "[core][world][collision]") {
    World world;
    const BodyId first = world.createBody(Body{{}, {}, 1.0, SphereCollider{0.0}});
    const BodyId second = world.createBody(Body{{}, {}, 1.0, BoxCollider{{0.0, 0.0, 0.0}}});
    const std::vector<std::pair<BodyId, BodyId>> pair{{first, second}};
    requirePairIds(world.findPotentialCollisions(), pair);
    REQUIRE(world.setBodyPosition(second, {0.0, 0.0, 0.25}));
    REQUIRE(world.findPotentialCollisions().empty());
}

/**
 * @brief Potential collision pairs own independent body snapshots.
 * @test Potential collision pairs own independent body snapshots. Returned bodies retain all their state even after the World is cleared.
 */
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

TEST_CASE("Parallel world queries and steps match sequential contact resolution", "[core][world][parallel]") {
    World sequential;
    World parallel{BroadPhaseOptions{.workerCount = 3}};
    for (World* world : {&sequential, &parallel}) {
        world->createBody(Body{{}, {}, 0.0});
        for (int i = 1; i < 80; ++i) world->createBody(Body{{10.0 * i, 0.0, 0.0}});
        world->createBody(Body{{1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}});
        world->createBody(Body{{3.1, 0.0, 0.0}});
    }
    requirePairIds(parallel.findPotentialCollisions(), {{0, 80}});
    const auto contacts = parallel.findContacts();
    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts[0].first == 0);
    REQUIRE(contacts[0].second == 80);

    for (int step = 0; step < 3; ++step) {
        sequential.step(0.1);
        parallel.step(0.1);
        for (const Body& expected : sequential.getBodies()) {
            const Body& actual = *parallel.getBody(expected.getId());
            requireVec3(actual.getPosition(), expected.getPosition());
            requireVec3(actual.getVelocity(), expected.getVelocity());
        }
    }
}
