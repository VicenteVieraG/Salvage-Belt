#include <catch2/catch_test_macros.hpp>
#include <type_traits>
#include <utility>
#include "math/Body.hpp"

using namespace salvage::core;

static_assert(std::is_same_v<decltype(std::declval<Body&>().aabb()), const AABB&>);
static_assert(std::is_same_v<decltype(std::declval<const Body&>().aabb()), const AABB&>);
static_assert(std::is_same_v<decltype(std::declval<Body&>().getPosition()), const Vec3&>);
static_assert(std::is_same_v<decltype(std::declval<Body&>().getCollider()), const Collider&>);

namespace {
    void requireVec3(const Vec3& actual, const Vec3& expected) {
        REQUIRE(actual.x == expected.x);
        REQUIRE(actual.y == expected.y);
        REQUIRE(actual.z == expected.z);
    }

    void requireBounds(const AABB& actual, const Vec3& min, const Vec3& max) {
        requireVec3(actual.min, min);
        requireVec3(actual.max, max);
    }
}

TEST_CASE("Default body bounds enclose its unit sphere", "[core][body][aabb]") {
    constexpr Body body;
    STATIC_REQUIRE(body.aabb().min.x == -1.0);
    STATIC_REQUIRE(body.aabb().max.x == 1.0);
    requireBounds(body.aabb(), {-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0});
}

TEST_CASE("Body bounds are initialized from its position and collider", "[core][body][aabb]") {
    SECTION("Sphere radius extends equally along every axis") {
        constexpr Body body{7, {10.0, -5.0, 2.0}, {1.0, 2.0, 3.0}, 0.5, SphereCollider{3.0}};
        STATIC_REQUIRE(body.aabb().min.x == 7.0);
        requireBounds(body.aabb(), {7.0, -8.0, -1.0}, {13.0, -2.0, 5.0});
        REQUIRE(body.getId() == 7);
        requireVec3(body.getPosition(), {10.0, -5.0, 2.0});
        requireVec3(body.getVelocity(), {1.0, 2.0, 3.0});
        REQUIRE(body.getInverseMass() == 0.5);
        REQUIRE(std::get<SphereCollider>(body.getCollider()).radious == 3.0);
    }

    SECTION("Axis-aligned box uses its distinct half-extents") {
        constexpr Body body{8, {-4.0, 2.0, -8.0}, {}, 1.0, BoxCollider{{1.0, 2.5, 4.0}}};
        STATIC_REQUIRE(body.aabb().max.y == 4.5);
        requireBounds(body.aabb(), {-5.0, -0.5, -12.0}, {-3.0, 4.5, -4.0});
    }
}

TEST_CASE("Moving a body refreshes its cached bounds", "[core][body][aabb]") {
    Body body;
    Vec3 extents{};

    SECTION("Sphere") {
        body = Body{1, {2.0, 3.0, 4.0}, {}, 1.0, SphereCollider{2.0}};
        extents = {2.0, 2.0, 2.0};
    }

    SECTION("Box") {
        body = Body{1, {2.0, 3.0, 4.0}, {}, 1.0, BoxCollider{{1.0, 2.0, 3.0}}};
        extents = {1.0, 2.0, 3.0};
    }

    const AABB& cachedBounds = body.aabb();
    for (const Vec3 position : {Vec3{-7.0, 5.0, -3.0}, Vec3{4.0, -2.0, 6.0}}) {
        body.setPosition(position);
        requireVec3(body.getPosition(), position);
        REQUIRE(&body.aabb() == &cachedBounds);
        requireBounds(cachedBounds, position - extents, position + extents);
    }
}

TEST_CASE("Replacing a collider refreshes bounds at the current position", "[core][body][aabb]") {
    Body body{1, {10.0, -5.0, 2.0}, {}, 1.0, SphereCollider{3.0}};
    const AABB& cachedBounds = body.aabb();

    body.setCollider(SphereCollider{0.5});
    requireBounds(cachedBounds, {9.5, -5.5, 1.5}, {10.5, -4.5, 2.5});

    body.setCollider(BoxCollider{{1.0, 2.0, 4.0}});
    REQUIRE(std::holds_alternative<BoxCollider>(body.getCollider()));
    requireBounds(cachedBounds, {9.0, -7.0, -2.0}, {11.0, -3.0, 6.0});

    body.setPosition({-2.0, 4.0, -6.0});
    body.setCollider(BoxCollider{{0.5, 1.0, 2.0}});
    requireBounds(cachedBounds, {-2.5, 3.0, -8.0}, {-1.5, 5.0, -4.0});

    body.setCollider(SphereCollider{2.0});
    REQUIRE(std::holds_alternative<SphereCollider>(body.getCollider()));
    requireBounds(cachedBounds, {-4.0, 2.0, -8.0}, {0.0, 6.0, -4.0});
    requireVec3(body.getPosition(), {-2.0, 4.0, -6.0});
}

TEST_CASE("Changing velocity leaves current bounds unchanged", "[core][body][aabb]") {
    Body body{1, {1.0, -2.0, 3.0}, {}, 1.0, BoxCollider{{2.0, 3.0, 4.0}}};
    body.setVelocity({5.0, 6.0, -7.0});
    requireVec3(body.getVelocity(), {5.0, 6.0, -7.0});
    requireVec3(body.getPosition(), {1.0, -2.0, 3.0});
    requireBounds(body.aabb(), {-1.0, -5.0, -1.0}, {3.0, 1.0, 7.0});
}

TEST_CASE("Zero-size colliders have point bounds", "[core][body][aabb]") {
    Body body{1, {-2.0, 4.0, 8.0}, {}, 1.0, SphereCollider{0.0}};
    requireBounds(body.aabb(), {-2.0, 4.0, 8.0}, {-2.0, 4.0, 8.0});
    body.setCollider(BoxCollider{{0.0, 0.0, 0.0}});
    requireBounds(body.aabb(), {-2.0, 4.0, 8.0}, {-2.0, 4.0, 8.0});
}
