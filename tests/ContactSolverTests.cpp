/** @file @brief Collision response, stability, and validated configuration tests. */
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "salvageCore.hpp"

using namespace salvage::core;
using Catch::Approx;

TEST_CASE("Elastic collisions conserve linear momentum and kinetic energy", "[core][solver]") {
    World world;
    const auto first = world.createBody(Body{{-1.1, 0.0, 0.0}, {1.0, 0.0, 0.0}, 1.0, SphereCollider{}, Material{1.0, 0.0}});
    const auto second = world.createBody(Body{{1.1, 0.0, 0.0}, {-1.0, 0.0, 0.0}, 0.5, SphereCollider{}, Material{1.0, 0.0}});
    world.step(0.2);
    const double firstVelocity = world.getBody(first)->getVelocity().x;
    const double secondVelocity = world.getBody(second)->getVelocity().x;
    REQUIRE(firstVelocity == Approx(-5.0 / 3.0));
    REQUIRE(secondVelocity == Approx(1.0 / 3.0));
    REQUIRE(firstVelocity + 2.0 * secondVelocity == Approx(-1.0));
    REQUIRE(0.5 * firstVelocity * firstVelocity + secondVelocity * secondVelocity == Approx(1.5));
    world.step(0.1);
    REQUIRE(world.getBody(first)->getVelocity().x == Approx(firstVelocity));
    REQUIRE(world.getBody(second)->getVelocity().x == Approx(secondVelocity));
}

TEST_CASE("Inelastic collisions equalize normal velocities without changing tangential motion", "[core][solver]") {
    World world;
    const Material smooth{0.0, 0.0};
    const auto first = world.createBody(Body{{-1.0, 0.0, 0.0}, {2.0, 3.0, 0.0}, 1.0, BoxCollider{{1.0, 2.0, 2.0}}, smooth});
    const auto second = world.createBody(Body{{1.0, 0.0, 0.0}, {-1.0, -2.0, 0.0}, 0.5, BoxCollider{{1.0, 2.0, 2.0}}, smooth});
    world.step(0.01);
    REQUIRE(world.getBody(first)->getVelocity().x == Approx(0.0).margin(1e-12));
    REQUIRE(world.getBody(second)->getVelocity().x == Approx(0.0).margin(1e-12));
    REQUIRE(world.getBody(first)->getVelocity().y == 3.0);
    REQUIRE(world.getBody(second)->getVelocity().y == -2.0);
}

TEST_CASE("Static surfaces reflect moving bodies but retain their own state", "[core][solver]") {
    for (bool reversed : {false, true}) {
        World world;
        const Body moving{{0.0, 1.0, 0.0}, {0.0, -2.0, 0.0}, 1.0, SphereCollider{}, Material{1.0, 0.0}};
        // Stored velocity on a static body must not turn it into a moving surface.
        const Body floor{{0.0, -0.5, 0.0}, {4.0, 50.0, -3.0}, 0.0, BoxCollider{{10.0, 0.5, 10.0}}};
        const auto first = world.createBody(reversed ? moving : floor);
        const auto second = world.createBody(reversed ? floor : moving);
        const auto dynamicId = reversed ? first : second;
        const auto staticId = reversed ? second : first;
        world.step(0.01);
        REQUIRE(world.getBody(dynamicId)->getVelocity().y == Approx(2.0));
        REQUIRE(world.getBody(dynamicId)->getPosition().y == Approx(1.0).margin(2e-5));
        REQUIRE(world.getBody(staticId)->getPosition().y == -0.5);
        REQUIRE(world.getBody(staticId)->getVelocity().y == 50.0);
        REQUIRE(world.getBody(dynamicId)->aabb().min.y == world.getBody(dynamicId)->getPosition().y - 1.0);
    }
}

TEST_CASE("Separating overlaps receive correction without attractive impulses", "[core][solver]") {
    World world;
    const auto first = world.createBody(Body{{-0.5, 0.0, 0.0}, {-1.0, 0.0, 0.0}, 1.0, SphereCollider{}, Material{1.0, 0.0}});
    const auto second = world.createBody(Body{{0.5, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.5, SphereCollider{}, Material{1.0, 0.0}});
    world.step(0.01);
    REQUIRE(world.getBody(first)->getVelocity().x == -1.0);
    REQUIRE(world.getBody(second)->getVelocity().x == 1.0);
    const double firstCorrection = -0.51 - world.getBody(first)->getPosition().x;
    const double secondCorrection = world.getBody(second)->getPosition().x - 0.51;
    REQUIRE(firstCorrection == Approx(2.0 * secondCorrection));
    REQUIRE(world.getBody(second)->getPosition().x - world.getBody(first)->getPosition().x == Approx(2.0).margin(2e-5));
}

TEST_CASE("Friction obeys the normal impulse limit and can stop slow sliding", "[core][solver]") {
    for (double speed : {0.1, 2.0}) {
        World world{{0.0, -10.0, 0.0}};
        // Unequal coefficients mix to sqrt(0.125 * 2) = 0.5.
        world.createBody(Body{{0.0, -0.5, 0.0}, {}, 0.0, BoxCollider{{10.0, 0.5, 10.0}}, Material{0.0, 0.125}});
        const auto box = world.createBody(Body{{0.0, 0.5, 0.0}, {speed, 0.0, 0.0}, 1.0, BoxCollider{}, Material{0.0, 2.0}});
        world.step(0.1);
        const double expected = speed < 0.5 ? 0.0 : speed - 0.5;
        REQUIRE(world.getBody(box)->getVelocity().x == Approx(expected).margin(1e-12));
        REQUIRE(world.getBody(box)->getVelocity().y == Approx(0.0).margin(1e-12));
    }
}

TEST_CASE("Restitution threshold suppresses low speed bouncing", "[core][solver]") {
    World world;
    world.createBody(Body{{0.0, -0.5, 0.0}, {}, 0.0, BoxCollider{{10.0, 0.5, 10.0}}});
    const auto sphere = world.createBody(Body{{0.0, 1.0, 0.0}, {0.0, -0.1, 0.0}, 1.0, SphereCollider{}, Material{1.0, 0.0}});
    world.step(0.01);
    REQUIRE(world.getBody(sphere)->getVelocity().y == Approx(0.0).margin(1e-12));
}

TEST_CASE("Spheres and small box stacks remain at rest under gravity", "[core][solver][stability]") {
    World world{{0.0, -9.81, 0.0}};
    world.createBody(Body{{0.0, -0.5, 0.0}, {}, 0.0, BoxCollider{{10.0, 0.5, 10.0}}});
    const auto first = world.createBody(Body{{0.0, 0.5, 0.0}, {}, 1.0, BoxCollider{}});
    const auto second = world.createBody(Body{{0.0, 1.5, 0.0}, {}, 1.0, BoxCollider{}});
    const auto sphere = world.createBody(Body{{4.0, 1.0, 0.0}});
    for (int step = 0; step < 600; ++step) {
        world.step(1.0 / 120.0);
        REQUIRE(world.getBody(first)->getPosition().y == Approx(0.5).margin(1e-3));
        REQUIRE(world.getBody(second)->getPosition().y == Approx(1.5).margin(1e-3));
        REQUIRE(world.getBody(sphere)->getPosition().y == Approx(1.0).margin(1e-3));
        REQUIRE(std::abs(world.getBody(first)->getVelocity().y) < 1e-4);
        REQUIRE(std::abs(world.getBody(second)->getVelocity().y) < 1e-4);
        REQUIRE(std::abs(world.getBody(sphere)->getVelocity().y) < 1e-4);
    }
}

TEST_CASE("Contained bodies escape along the nearest face and point contacts stay finite", "[core][solver]") {
    World world;
    world.createBody(Body{{}, {}, 0.0, BoxCollider{{2.0, 2.0, 2.0}}});
    const auto sphere = world.createBody(Body{{0.0, 1.5, 0.0}, {}, 1.0, SphereCollider{0.25}});
    world.step(0.01);
    REQUIRE(world.getBody(sphere)->getPosition().y == Approx(2.25).margin(2e-5));
    REQUIRE(world.getBody(sphere)->getVelocity().lengthSquared() == 0.0);
    world.clear();
    const auto point = world.createBody(Body{{}, {}, 1.0, SphereCollider{0.0}});
    world.createBody(Body{{}, {}, 0.0, BoxCollider{{}}});
    world.step(0.01);
    REQUIRE(world.getBody(point)->getPosition().lengthSquared() == 0.0);
    REQUIRE(world.getBody(point)->getVelocity().lengthSquared() == 0.0);
}

TEST_CASE("Zero-duration steps preserve overlapping bodies", "[core][solver]") {
    World world;
    const auto first = world.createBody();
    world.createBody();
    world.step(0.0);
    REQUIRE(world.getBody(first)->getPosition().lengthSquared() == 0.0);
    REQUIRE(world.findContacts()[0].penetration == 2.0);
}

TEST_CASE("Materials are preserved and validated without partial edits", "[core][solver][validation]") {
    World world;
    const Body original{{}, {}, 1.0, SphereCollider{}, Material{0.8, 0.2}};
    const auto id = world.createBody(original);
    const auto copy = world.createBody(*world.getBody(id));
    REQUIRE(world.getBody(copy)->getMaterial().restitution == 0.8);
    REQUIRE(world.getBody(copy)->getMaterial().friction == 0.2);
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const Material material : {Material{-0.1, 0.5}, Material{1.1, 0.5}, Material{inf, 0.5},
        Material{nan, 0.5}, Material{0.5, -0.1}, Material{0.5, inf}, Material{0.5, nan}}) {
        REQUIRE_THROWS_AS(world.setBodyMaterial(id, material), std::invalid_argument);
        REQUIRE_THROWS_AS(world.createBody(Body{{}, {}, 1.0, SphereCollider{}, material}), std::invalid_argument);
        REQUIRE_FALSE(world.setBodyMaterial(copy + 1, material));
        REQUIRE(world.getBody(id)->getMaterial().restitution == 0.8);
        REQUIRE(world.getBody(id)->getMaterial().friction == 0.2);
        REQUIRE(world.getBodyCount() == 2);
    }
    REQUIRE(world.setBodyMaterial(id, {1.0, 0.0}));
    REQUIRE(world.getBody(id)->getMaterial().restitution == 1.0);
    REQUIRE(world.getBody(id)->getMaterial().friction == 0.0);
    REQUIRE(world.createBody() == copy + 1);
}

TEST_CASE("Solver configuration rejects invalid values and accepts valid controls", "[core][solver][validation]") {
    World world;
    SolverSettings settings;
    SECTION("Zero velocity iterations") { settings.velocityIterations = 0; }
    SECTION("Zero position iterations") { settings.positionIterations = 0; }
    SECTION("Negative slop") { settings.penetrationSlop = -1.0; }
    SECTION("NaN slop") { settings.penetrationSlop = std::numeric_limits<double>::quiet_NaN(); }
    SECTION("Infinite slop") { settings.penetrationSlop = std::numeric_limits<double>::infinity(); }
    SECTION("Zero correction") { settings.positionCorrection = 0.0; }
    SECTION("Negative correction") { settings.positionCorrection = -0.1; }
    SECTION("Excess correction") { settings.positionCorrection = 1.1; }
    SECTION("NaN correction") { settings.positionCorrection = std::numeric_limits<double>::quiet_NaN(); }
    SECTION("Infinite correction") { settings.positionCorrection = std::numeric_limits<double>::infinity(); }
    SECTION("Negative threshold") { settings.restitutionThreshold = -1.0; }
    SECTION("NaN threshold") { settings.restitutionThreshold = std::numeric_limits<double>::quiet_NaN(); }
    SECTION("Infinite threshold") { settings.restitutionThreshold = std::numeric_limits<double>::infinity(); }
    REQUIRE_THROWS_AS(world.setSolverSettings(settings), std::invalid_argument);
    REQUIRE(world.getSolverSettings().velocityIterations == 16);
    REQUIRE(world.getSolverSettings().positionIterations == 8);
    REQUIRE(world.getSolverSettings().penetrationSlop == 1e-5);
    REQUIRE(world.getSolverSettings().positionCorrection == 0.8);
    REQUIRE(world.getSolverSettings().restitutionThreshold == 0.5);
    REQUIRE_NOTHROW(world.setSolverSettings({1, 1, 0.0, 1.0, 0.0}));
    REQUIRE(world.getSolverSettings().velocityIterations == 1);
    world.createBody(Body{{}, {}, 0.0});
    const auto body = world.createBody();
    world.step(0.01);
    REQUIRE(world.getBody(body)->getPosition().x == Approx(2.0));
}
