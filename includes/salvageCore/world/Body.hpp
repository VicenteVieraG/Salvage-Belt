/**
 * @file
 * @brief Body state, collider shapes, and world-space bounds.
 */
#pragma once

#include <cstdint>
#include <utility>
#include <variant>
#include "../math/AABB.hpp"
#include "../math/Vec3.hpp"

namespace salvage::core {
    /// @brief Identifier assigned locally by a World.
    using BodyId = std::uint32_t;
    /// @brief Pair of independent body values.
    using BodyPair = std::pair<Body, Body>;

    /**
     * @brief Sphere centered on the body position.
     */
    struct SphereCollider {
        double radius{ 1.0 }; ///< Radius; World requires a finite, non-negative value.
    };
    /**
     * @brief Box centered on the body position and aligned with world axes.
     */
    struct BoxCollider {
        Vec3 halfExtents{ 0.5, 0.5, 0.5 }; ///< Half-sizes; World requires finite, non-negative components.
    };

    /// @brief Supported collision shapes.
    using Collider = std::variant<
        SphereCollider,
        BoxCollider
    >;

    /**
     * @brief Get the half-size of a sphere along each axis.
     * @param sphere Sphere whose radius supplies all three components.
     * @return A vector containing the radius on each axis.
     */
    constexpr Vec3 getExtents(const SphereCollider& sphere) noexcept {
        const auto [radius] = sphere;

        return { radius, radius, radius };
    }

    /**
     * @brief Get the half-size of an axis-aligned box.
     * @param box Box to measure.
     * @return The stored half-extents.
     */
    constexpr Vec3 getExtents(const BoxCollider& box) noexcept {
        return box.halfExtents;
    }

    /**
     * @brief Compute world-space bounds for a centered collider.
     * Boxes remain aligned with the world axes.
     * @param position Center of the collider.
     * @param collider Shape to enclose.
     * @pre Position and extents are finite, extents are non-negative, and arithmetic remains in range.
     * @return Bounds from position minus extents to position plus extents.
     */
    constexpr AABB computeAABB(const Vec3& position, const Collider& collider) noexcept {
        const Vec3 extents = std::visit([](const auto& shape) -> Vec3 {
            return getExtents(shape);
        }, collider);

        return { position - extents, position + extents };
    }

    /**
     * @brief Translating body with a collider and cached world-space bounds.
     * Constructors and setters do not validate physical values; World validates inputs
     * when bodies are created or edited through its API. Zero inverse mass denotes
     * a static body during World::step().
     */
    class Body {
        private:
            BodyId id{}; ///< Identifier, local to the owning World.

            Vec3 position{}; ///< Center in world coordinates.
            Vec3 velocity{}; ///< Linear velocity.

            double inverseMass{ 1.0 }; ///< Reciprocal mass; zero denotes a static body.

            Collider collider{ SphereCollider{} }; ///< Centered collision shape.
            AABB bounds{ computeAABB(this->position, this->collider) }; ///< Cached bounds matching position and collider.

            /// @brief Refresh bounds after a position or collider change.
            constexpr void updateAABB() noexcept {
                this->bounds = computeAABB(this->position, this->collider);
            }
        public:
            /// @brief Create ID zero at rest at the origin, with unit inverse mass and a unit sphere.
            constexpr Body() noexcept = default;
            /**
             * @brief Create a body with an explicit identifier and physical state.
             * @param id Identifier; World::createBody() assigns its own ID when copying the body.
             * @param position Initial world-space center.
             * @param velocity Initial linear velocity.
             * @param inverseMass Reciprocal mass; zero denotes a static body.
             * @param collider Centered collision shape.
             */
            constexpr Body(
                BodyId id, const Vec3& position, const Vec3& velocity, double inverseMass, const Collider& collider) noexcept:
                id{id}, position{position}, velocity{velocity}, inverseMass{inverseMass}, collider{collider}
            {};

            /**
             * @brief Create a body with ID zero and optional physical properties.
             * @param position Initial world-space center.
             * @param velocity Initial linear velocity, zero by default.
             * @param inverseMass Reciprocal mass, one by default; zero denotes a static body.
             * @param collider Centered collision shape, a unit sphere by default.
             * @note World assigns IDs independently.
             */
            explicit constexpr Body(
                const Vec3& position, const Vec3& velocity = {}, double inverseMass = 1.0,
                const Collider& collider = SphereCollider{}) noexcept:
                Body{0, position, velocity, inverseMass, collider}
            {};

            /**
             * @brief Read the body identifier.
             * @return The stored ID.
             */
            constexpr BodyId getId() const noexcept { return this->id; }
            /**
             * @brief Read the center position.
             * @return A reference to the stored world-space position.
             */
            constexpr const Vec3& getPosition() const noexcept { return this->position; }
            /**
             * @brief Read the linear velocity.
             * @return A reference to the stored velocity.
             */
            constexpr const Vec3& getVelocity() const noexcept { return this->velocity; }
            /**
             * @brief Read the reciprocal mass.
             * @return The stored inverse mass; zero denotes a static body.
             */
            constexpr double getInverseMass() const noexcept { return this->inverseMass; }
            /**
             * @brief Read the collision shape.
             * @return A reference to the stored collider.
             */
            constexpr const Collider& getCollider() const noexcept { return this->collider; }
            /**
             * @brief Read the cached world-space bounds.
             * @return A reference to bounds kept in sync with position and collider.
             */
            constexpr const AABB& aabb() const noexcept { return this->bounds; }

            /**
             * @brief Set the center and refresh the cached bounds.
             * @param position New world-space center; not validated.
             */
            constexpr void setPosition(const Vec3& position) noexcept {
                this->position = position;
                this->updateAABB();
            }

            /**
             * @brief Set linear velocity without changing the bounds.
             * @param velocity New velocity; not validated.
             */
            constexpr void setVelocity(const Vec3& velocity) noexcept {
                this->velocity = velocity;
            }

            /**
             * @brief Replace the shape and refresh the cached bounds.
             * @param collider New centered shape; not validated.
             */
            constexpr void setCollider(const Collider& collider) noexcept {
                this->collider = collider;
                this->updateAABB();
            }
    };
};
