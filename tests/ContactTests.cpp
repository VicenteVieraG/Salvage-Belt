/** @file @brief Shape-level contact geometry and query regression tests. */
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include "salvage-core/collision/Contact.hpp"

using namespace salvage::core;
using Catch::Approx;

namespace {
    void requireVector(const Vec3& actual, const Vec3& expected) {
        REQUIRE(actual.x == Approx(expected.x).margin(1e-12));
        REQUIRE(actual.y == Approx(expected.y).margin(1e-12));
        REQUIRE(actual.z == Approx(expected.z).margin(1e-12));
    }

    void requireContact(const Body& first, const Body& second, const Vec3& normal, double depth) {
        const auto contact = detectContact(first, second);
        REQUIRE(contact);
        REQUIRE(contact->first == first.getId());
        REQUIRE(contact->second == second.getId());
        REQUIRE(contact->penetration == Approx(depth).margin(1e-12));
        requireVector(contact->normal, normal);
        REQUIRE(contact->normal.lengthSquared() == Approx(1.0));
        const auto reversed = detectContact(second, first);
        REQUIRE(reversed);
        REQUIRE(reversed->penetration == Approx(depth).margin(1e-12));
        requireVector(reversed->normal, normal * -1.0);
        requireVector(reversed->point, contact->point);

        // Translating by the reported depth must remove penetration, including containment.
        Body moved = first;
        moved.setPosition(first.getPosition() - normal * (depth + 1e-8));
        REQUIRE_FALSE(detectContact(moved, second));
    }
}

TEST_CASE("Sphere contacts reject bounding-box false positives and handle degeneracy", "[core][contact]") {
    const Body first{10, {}, {}, 1.0, SphereCollider{1.0}};
    Body second{20, {1.5, 0.0, 0.0}, {}, 1.0, SphereCollider{1.0}};
    requireContact(first, second, {1.0, 0.0, 0.0}, 0.5);
    requireVector(detectContact(first, second)->point, {0.75, 0.0, 0.0});
    second.setPosition({2.0, 0.0, 0.0});
    requireContact(first, second, {1.0, 0.0, 0.0}, 0.0);
    second.setPosition({1.5, 1.5, 0.0});
    REQUIRE(overlaps(first.aabb(), second.aabb()));
    REQUIRE_FALSE(detectContact(first, second));
    second.setPosition({});
    requireContact(first, second, {1.0, 0.0, 0.0}, 2.0);
    second.setCollider(SphereCollider{0.0});
    requireContact(first, second, {1.0, 0.0, 0.0}, 1.0);
}

TEST_CASE("Sphere-box contacts cover faces edges corners and containment", "[core][contact]") {
    const Body box{20, {}, {}, 0.0, BoxCollider{{1.0, 1.0, 1.0}}};
    Body sphere{10, {1.5, 0.0, 0.0}, {}, 1.0, SphereCollider{1.0}};
    SECTION("Face") { requireContact(sphere, box, {-1.0, 0.0, 0.0}, 0.5); }
    SECTION("Touching") {
        sphere.setPosition({-2.0, 0.0, 0.0});
        requireContact(sphere, box, {1.0, 0.0, 0.0}, 0.0);
    }
    SECTION("Edge") {
        sphere.setPosition({1.3, 1.4, 0.0});
        requireContact(sphere, box, {-0.6, -0.8, 0.0}, 0.5);
    }
    SECTION("Corner") {
        sphere.setPosition({1.2, 1.4, 1.4});
        requireContact(sphere, box, {-1.0 / 3.0, -2.0 / 3.0, -2.0 / 3.0}, 0.4);
    }
    SECTION("False positive at corner") {
        sphere.setPosition({1.9, 1.9, 1.9});
        REQUIRE(overlaps(sphere.aabb(), box.aabb()));
        REQUIRE_FALSE(detectContact(sphere, box));
        REQUIRE_FALSE(detectContact(box, sphere));
    }
    SECTION("Contained nearest negative face") {
        sphere.setPosition({0.0, -0.75, 0.0});
        requireContact(sphere, box, {0.0, 1.0, 0.0}, 1.25);
    }
    SECTION("Coincident centers") {
        sphere.setPosition({});
        requireContact(sphere, box, {-1.0, 0.0, 0.0}, 2.0);
    }
    SECTION("Center on surface") {
        sphere.setPosition({0.0, 0.0, 1.0});
        requireContact(sphere, box, {0.0, 0.0, -1.0}, 1.0);
    }
}

TEST_CASE("Box contacts use minimum translation including contained boxes", "[core][contact]") {
    Body first{10, {}, {}, 1.0, BoxCollider{{1.0, 1.0, 1.0}}};
    Body second{20, {0.0, 1.75, 0.0}, {}, 1.0, BoxCollider{{1.0, 1.0, 1.0}}};
    requireContact(first, second, {0.0, 1.0, 0.0}, 0.25);
    second.setPosition({0.0, 0.0, -2.0});
    requireContact(first, second, {0.0, 0.0, -1.0}, 0.0);
    second.setPosition({0.0, 0.0, -2.01});
    REQUIRE_FALSE(detectContact(first, second));
    second.setPosition({});
    requireContact(first, second, {1.0, 0.0, 0.0}, 2.0);
    first.setCollider(BoxCollider{{0.25, 0.25, 0.25}});
    first.setPosition({0.0, 0.5, 0.0});
    requireContact(first, second, {0.0, -1.0, 0.0}, 0.75);
}

TEST_CASE("Zero-size shapes produce finite contacts", "[core][contact]") {
    for (const Collider firstShape : {Collider{SphereCollider{0.0}}, Collider{BoxCollider{{}}}}) {
        for (const Collider secondShape : {Collider{SphereCollider{0.0}}, Collider{BoxCollider{{}}}}) {
            const Body first{10, {}, {}, 1.0, firstShape};
            const Body second{20, {}, {}, 1.0, secondShape};
            const auto contact = detectContact(first, second);
            REQUIRE(contact);
            REQUIRE(contact->penetration == 0.0);
            REQUIRE(contact->normal.lengthSquared() == 1.0);
            requireVector(contact->point, {});
            const auto reversed = detectContact(second, first);
            REQUIRE(reversed);
            requireVector(reversed->normal, contact->normal * -1.0);
        }
    }
}
