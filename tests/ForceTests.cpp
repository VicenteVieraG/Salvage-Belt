/** @file @brief External force accumulation and impulse behavior. */
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <limits>
#include <stdexcept>
#include "salvageCore.hpp"

using namespace salvage::core;
using Catch::Approx;

TEST_CASE("External forces accumulate scale by inverse mass and expire after integration", "[core][force]") {
    World world{{0.0, -2.0, 0.0}};
    const auto light = world.createBody();
    const auto heavy = world.createBody(Body{{0.0, 0.0, 10.0}, {}, 0.5});
    for (const auto id : {light, heavy}) {
        REQUIRE(world.addBodyForce(id, {2.0, 0.0, 0.0}));
        REQUIRE(world.addBodyForce(id, {4.0, 0.0, 0.0}));
    }
    world.step(0.5);
    REQUIRE(world.getBody(light)->getVelocity().x == 3.0);
    REQUIRE(world.getBody(light)->getPosition().x == 0.75);
    REQUIRE(world.getBody(heavy)->getVelocity().x == 1.5);
    REQUIRE(world.getBody(heavy)->getPosition().x == 0.375);
    REQUIRE(world.getBody(light)->getVelocity().y == -1.0);
    REQUIRE(world.getBody(heavy)->getVelocity().y == -1.0);
    world.step(0.5);
    REQUIRE(world.getBody(light)->getVelocity().x == 3.0);
    REQUIRE(world.getBody(light)->getPosition().x == 2.25);
    REQUIRE(world.getBody(heavy)->getVelocity().x == 1.5);
    REQUIRE(world.getBody(light)->getVelocity().y == -2.0);
}

TEST_CASE("Zero and rejected steps preserve pending forces", "[core][force]") {
    World world;
    const auto id = world.createBody();
    world.addBodyForce(id, {2.0, 0.0, 0.0});
    world.step(0.0);
    REQUIRE_THROWS_AS(world.step(-1.0), std::invalid_argument);
    REQUIRE_THROWS_AS(world.step(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
    REQUIRE(world.getBody(id)->getVelocity().x == 0.0);
    world.step(1.0);
    REQUIRE(world.getBody(id)->getVelocity().x == 2.0);
}

TEST_CASE("External impulses change velocity immediately while preserving pending force", "[core][force]") {
    World world;
    const auto id = world.createBody(Body{{}, {1.0, 0.0, 0.0}, 0.5});
    world.addBodyForce(id, {2.0, 0.0, 0.0});
    REQUIRE(world.applyBodyImpulse(id, {4.0, 0.0, 0.0}));
    REQUIRE(world.getBody(id)->getVelocity().x == 3.0);
    REQUIRE(world.getBody(id)->getPosition().x == 0.0);
    world.step(1.0);
    REQUIRE(world.getBody(id)->getVelocity().x == 4.0);
    REQUIRE(world.getBody(id)->getPosition().x == 3.5);
}

TEST_CASE("Static missing and removed bodies handle external forces consistently", "[core][force]") {
    World world;
    const auto fixed = world.createBody(Body{{}, {5.0, 0.0, 0.0}, 0.0});
    REQUIRE(world.addBodyForce(fixed, {100.0, 0.0, 0.0}));
    REQUIRE(world.applyBodyImpulse(fixed, {100.0, 0.0, 0.0}));
    world.step(1.0);
    REQUIRE(world.getBody(fixed)->getPosition().x == 0.0);
    REQUIRE(world.getBody(fixed)->getVelocity().x == 5.0);
    const auto dynamic = world.createBody(Body{{10.0, 0.0, 0.0}});
    world.addBodyForce(dynamic, {100.0, 0.0, 0.0});
    world.removeBody(dynamic);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_FALSE(world.addBodyForce(dynamic, {nan, 0.0, 0.0}));
    REQUIRE_FALSE(world.applyBodyImpulse(dynamic, {nan, 0.0, 0.0}));
    world.clear();
    const auto fresh = world.createBody();
    world.step(1.0);
    REQUIRE(world.getBody(fresh)->getVelocity().x == 0.0);
}

TEST_CASE("Invalid forces and impulses leave state unchanged", "[core][force][validation]") {
    World world;
    const auto id = world.createBody();
    const double maximum = std::numeric_limits<double>::max();
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    world.addBodyForce(id, {maximum, 0.0, 0.0});
    REQUIRE_THROWS_AS(world.addBodyForce(id, {maximum, 0.0, 0.0}), std::invalid_argument);
    for (double value : {inf, nan}) {
        REQUIRE_THROWS_AS(world.addBodyForce(id, {value, 0.0, 0.0}), std::invalid_argument);
        REQUIRE_THROWS_AS(world.applyBodyImpulse(id, {value, 0.0, 0.0}), std::invalid_argument);
    }
    // Cancel the original force to verify a rejected accumulation did not replace it.
    world.addBodyForce(id, {-maximum, 0.0, 0.0});
    world.step(1.0);
    REQUIRE(world.getBody(id)->getVelocity().x == 0.0);
    world.setBodyVelocity(id, {maximum, 0.0, 0.0});
    REQUIRE_THROWS_AS(world.applyBodyImpulse(id, {maximum, 0.0, 0.0}), std::invalid_argument);
    REQUIRE(world.getBody(id)->getVelocity().x == maximum);
}

TEST_CASE("Force driven impacts participate in contact response", "[core][force][solver]") {
    World world;
    world.createBody(Body{{0.0, -0.5, 0.0}, {}, 0.0, BoxCollider{{10.0, 0.5, 10.0}}});
    const auto id = world.createBody(Body{{0.0, 1.0, 0.0}});
    world.addBodyForce(id, {0.0, -10.0, 0.0});
    world.step(0.1);
    REQUIRE(world.getBody(id)->getVelocity().y == Approx(0.0).margin(1e-12));
    REQUIRE(world.getBody(id)->getPosition().y == Approx(1.0).margin(2e-5));
}
