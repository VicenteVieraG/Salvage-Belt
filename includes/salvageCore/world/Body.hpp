#pragma once

#include <cstdint>
#include <variant>
#include "../math/AABB.hpp"
#include "../math/Vec3.hpp"

namespace salvage::core {
    using BodyId = std::uint32_t;

    struct SphereCollider {
        double radius{ 1.0 };
    };
    struct BoxCollider {
        Vec3 halfExtents{ 0.5, 0.5, 0.5 };
    };

    using Collider = std::variant<
        SphereCollider,
        BoxCollider
    >;

    constexpr Vec3 getExtents(const SphereCollider& sphere) noexcept {
        const auto [radius] = sphere;

        return { radius, radius, radius };
    }

    constexpr Vec3 getExtents(const BoxCollider& box) noexcept {
        return box.halfExtents;
    }

    // Colliders are centered on position; boxes remain aligned with the world axes.
    constexpr AABB computeAABB(const Vec3& position, const Collider& collider) noexcept {
        const Vec3 extents = std::visit([](const auto& shape) -> Vec3 {
            return getExtents(shape);
        }, collider);

        return { position - extents, position + extents };
    }

    class Body {
        private:
            BodyId id{};

            Vec3 position{};
            Vec3 velocity{};

            double inverseMass{ 1.0 };

            Collider collider{ SphereCollider{} };
            AABB bounds{ computeAABB(this->position, this->collider) };

            constexpr void updateAABB() noexcept {
                this->bounds = computeAABB(this->position, this->collider);
            }
        public:
            constexpr Body() noexcept = default;
            constexpr Body(
                BodyId id, const Vec3& position, const Vec3& velocity, double inverseMass, const Collider& collider) noexcept:
                id{id}, position{position}, velocity{velocity}, inverseMass{inverseMass}, collider{collider}
            {};

            constexpr BodyId getId() const noexcept { return this->id; }
            constexpr const Vec3& getPosition() const noexcept { return this->position; }
            constexpr const Vec3& getVelocity() const noexcept { return this->velocity; }
            constexpr double getInverseMass() const noexcept { return this->inverseMass; }
            constexpr const Collider& getCollider() const noexcept { return this->collider; }
            constexpr const AABB& aabb() const noexcept { return this->bounds; }

            constexpr void setPosition(const Vec3& position) noexcept {
                this->position = position;
                this->updateAABB();
            }

            constexpr void setVelocity(const Vec3& velocity) noexcept {
                this->velocity = velocity;
            }

            constexpr void setCollider(const Collider& collider) noexcept {
                this->collider = collider;
                this->updateAABB();
            }
    };
};
