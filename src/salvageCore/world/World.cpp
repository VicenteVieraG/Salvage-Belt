/**
 * @file
 * @brief World storage, validation, integration, and broad-phase collision queries.
 */
#include "world/World.hpp"
#include "algorithm/BruteForce.hpp"
#include "physics/Physics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace salvage::core {
    namespace {
        /**
         * @brief Check that every vector component is finite.
         * @param value Vector to inspect.
         * @return True if no component is infinite or NaN.
         */
        bool isFinite(const Vec3& value) noexcept {
            const auto [x, y, z] = value;

            return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        }

        /**
         * @brief Validate a world-space vector.
         * @param value Vector to inspect.
         * @throws std::invalid_argument If any component is not finite.
         */
        void validateVector(const Vec3& value) {
            if (!isFinite(value)) throw std::invalid_argument("World vectors must have finite components");
        }

        /**
         * @brief Validate collider dimensions.
         * @param collider Shape to inspect.
         * @throws std::invalid_argument If any extent is non-finite or negative.
         */
        void validateCollider(const Collider& collider) {
            const Vec3 extents = std::visit([](const auto& shape) { return getExtents(shape); }, collider);
            if (const auto [x, y, z] = extents; !isFinite(extents) || x < 0.0 || y < 0.0 || z < 0.0) {
                throw std::invalid_argument("Collider extents must be finite and non-negative");
            }
        }
    }

    World::World(const Vec3& gravity) {
        this->setGravity(gravity);
    }

    BodyId World::createBody(const Body& body) {
        validateVector(body.getPosition());
        validateVector(body.getVelocity());
        validateCollider(body.getCollider());
        if (!std::isfinite(body.getInverseMass()) || body.getInverseMass() < 0.0) {
            throw std::invalid_argument("Inverse mass must be finite and non-negative");
        }
        if (this->nextBodyId > std::numeric_limits<BodyId>::max()) {
            throw std::overflow_error("World body IDs are exhausted");
        }

        const auto id = static_cast<BodyId>(this->nextBodyId);
        this->bodies.push_back(Body{
            id,
            body.getPosition(),
            body.getVelocity(),
            body.getInverseMass(),
            body.getCollider()
        });
        this->nextBodyId++;
        return id;
    }

    bool World::removeBody(BodyId id) noexcept {
        if(const auto it = std::ranges::lower_bound(this->bodies, id, {}, &Body::getId);
            it == this->bodies.end() || it->getId() != id) return false;
        else this->bodies.erase(it);
        return true;
    }

    void World::clear() noexcept {
        this->bodies.clear();
    }

    std::size_t World::getBodyCount() const noexcept {
        return this->bodies.size();
    }

    Body* World::findBody(BodyId id) noexcept {
        if(const auto it = std::ranges::lower_bound(this->bodies, id, {}, &Body::getId);
            it == this->bodies.end() || it->getId() != id) return nullptr;
        else return &*it;
    }

    const Body* World::getBody(BodyId id) const noexcept {
        if(const auto it = std::ranges::lower_bound(this->bodies, id, {}, &Body::getId);
            it == this->bodies.end() || it->getId() != id) return nullptr;
        else return &*it;
    }

    std::span<const Body> World::getBodies() const noexcept {
        return this->bodies;
    }

    bool World::setBodyPosition(BodyId id, const Vec3& position) {
        if(Body* body = this->findBody(id)){
            validateVector(position);
            body->setPosition(position);

            return true;
        } else return false;
    }

    bool World::setBodyVelocity(BodyId id, const Vec3& velocity) {
        if(Body* body = this->findBody(id)){
            validateVector(velocity);
            body->setVelocity(velocity);

            return true;
        } else return false;
    }

    bool World::setBodyCollider(BodyId id, const Collider& collider) {
        if(Body* body = this->findBody(id)){
            validateCollider(collider);
            body->setCollider(collider);

            return true;
        } else return false;
    }

    const Vec3& World::getGravity() const noexcept {
        return this->gravity;
    }

    void World::setGravity(const Vec3& gravity) {
        validateVector(gravity);
        this->gravity = gravity;
    }

    void World::step(double deltaTime) {
        if (!std::isfinite(deltaTime) || deltaTime < 0.0)
            throw std::invalid_argument("Time step must be finite and non-negative");
        if (deltaTime == 0.0) return;

        for (Body& body : this->bodies) {
            if (body.getInverseMass() == 0.0) continue;

            const Vec3 position = velocityVerletPosition(body.getPosition(), body.getVelocity(), this->gravity, deltaTime);
            const Vec3 velocity = velocityVerletVelocity(body.getVelocity(), this->gravity, this->gravity, deltaTime);

            body.setPosition(position);
            body.setVelocity(velocity);
        }
    }

    std::vector<BodyPair> World::findPotentialCollisions() const {
        return BruteForce::broadPhase(this->bodies);
    }
}
