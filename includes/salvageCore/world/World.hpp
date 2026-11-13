#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>
#include "Body.hpp"

namespace salvage::core {
    // Owns bodies and advances translation only. IDs are local to a World.
    class World {
        private:
            // Strictly increasing IDs: creation appends and removal preserves order.
            std::vector<Body> bodies{};
            std::uint64_t nextBodyId{};
            Vec3 gravity{};

            Body* findBody(BodyId id) noexcept;
        public:
            // Zero gravity by default; callers can opt into uniform acceleration.
            explicit World(const Vec3& gravity = {});

            // IDs start at zero and are never reused by creation, removal, or clear().
            // Copies the body's state and assigns a fresh ID, ignoring its input ID.
            // Invalid bodies throw std::invalid_argument; exhausted IDs
            // throw std::overflow_error without modifying the World.
            BodyId createBody(const Body& body = {});
            bool removeBody(BodyId id) noexcept;
            void clear() noexcept;

            std::size_t getBodyCount() const noexcept;
            const Body* getBody(BodyId id) const noexcept;
            // Views/pointers must be reacquired after creation, removal, or clear.
            std::span<const Body> getBodies() const noexcept;

            // Return false for unknown IDs. Invalid values throw
            // std::invalid_argument before changing an existing body.
            bool setBodyPosition(BodyId id, const Vec3& position);
            bool setBodyVelocity(BodyId id, const Vec3& velocity);
            bool setBodyCollider(BodyId id, const Collider& collider);

            const Vec3& getGravity() const noexcept;
            void setGravity(const Vec3& gravity);

            // Semi-implicit Euler: velocity += gravity * dt, position += velocity * dt.
            // Zero inverse mass means static. dt must be finite and non-negative;
            // zero is a no-op. Invalid dt throws std::invalid_argument.
            void step(double deltaTime);

            // Inclusive AABB overlaps, once per pair, in creation order.
            // Static/static pairs are omitted. This is not contact resolution.
            // Returns body copies; editing them does not change the World.
            std::vector<std::pair<Body, Body>> findPotentialCollisions() const;
    };
}
