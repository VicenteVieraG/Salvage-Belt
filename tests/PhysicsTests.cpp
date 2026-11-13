#include <catch2/catch_test_macros.hpp>
#include "math/Physics.hpp"

using namespace salvage::core;

namespace {
    void requireVec3(const Vec3& actual, const Vec3& expected) {
        REQUIRE(actual.x == expected.x);
        REQUIRE(actual.y == expected.y);
        REQUIRE(actual.z == expected.z);
    }
}

TEST_CASE("Velocity Verlet matches constant-acceleration motion", "[core][physics]") {
    constexpr Vec3 position{1.0, -2.0, 3.0};
    constexpr Vec3 velocity{4.0, -6.0, 8.0};
    constexpr Vec3 acceleration{2.0, 4.0, -6.0};
    constexpr Vec3 nextPosition = velocityVerletPosition(position, velocity, acceleration, 0.5);
    constexpr Vec3 nextVelocity = velocityVerletVelocity(velocity, acceleration, acceleration, 0.5);

    STATIC_REQUIRE(nextPosition.x == 3.25);
    STATIC_REQUIRE(nextVelocity.x == 5.0);
    requireVec3(nextPosition, {3.25, -4.5, 6.25});
    requireVec3(nextVelocity, {5.0, -4.0, 5.0});
}

TEST_CASE("Velocity Verlet handles zero acceleration and zero duration", "[core][physics]") {
    const Vec3 position{1.0, -2.0, 3.0};
    const Vec3 velocity{4.0, -6.0, 8.0};

    SECTION("Zero acceleration preserves velocity") {
        requireVec3(velocityVerletPosition(position, velocity, {}, 0.5), {3.0, -5.0, 7.0});
        requireVec3(velocityVerletVelocity(velocity, {}, {}, 0.5), velocity);
    }

    SECTION("Zero duration preserves position and velocity") {
        const Vec3 acceleration{2.0, 4.0, -6.0};
        requireVec3(velocityVerletPosition(position, velocity, acceleration, 0.0), position);
        requireVec3(velocityVerletVelocity(velocity, acceleration, {}, 0.0), velocity);
    }
}

TEST_CASE("Velocity Verlet uses acceleration at the new position", "[core][physics]") {
    // Unit-frequency harmonic oscillator: acceleration = -position.
    const Vec3 position{1.0, 2.0, -4.0};
    const Vec3 velocity{};
    const Vec3 acceleration = position * -1.0;
    const Vec3 nextPosition = velocityVerletPosition(position, velocity, acceleration, 0.5);
    const Vec3 nextAcceleration = nextPosition * -1.0;
    const Vec3 nextVelocity = velocityVerletVelocity(velocity, acceleration, nextAcceleration, 0.5);

    requireVec3(nextPosition, {0.875, 1.75, -3.5});
    requireVec3(nextVelocity, {-0.46875, -0.9375, 1.875});
}
