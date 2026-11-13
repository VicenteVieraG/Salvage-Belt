#include "world/World.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace salvage::core {
    namespace {
        bool isFinite(const Vec3& value) noexcept {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        void validateVector(const Vec3& value) {
            if (!isFinite(value)) {
                throw std::invalid_argument("World vectors must have finite components");
            }
        }

        void validateCollider(const Collider& collider) {
            const Vec3 extents = std::visit([](const auto& shape) { return getExtents(shape); }, collider);
            if (!isFinite(extents) || extents.x < 0.0 || extents.y < 0.0 || extents.z < 0.0) {
                throw std::invalid_argument("Collider extents must be finite and non-negative");
            }
        }
    }

    World::World(const Vec3& gravity) {
        this->setGravity(gravity);
    }

    BodyId World::createBody(const BodyDefinition& definition) {
        validateVector(definition.position);
        validateVector(definition.velocity);
        validateCollider(definition.collider);
        if (!std::isfinite(definition.inverseMass) || definition.inverseMass < 0.0) {
            throw std::invalid_argument("Inverse mass must be finite and non-negative");
        }
        if (this->nextBodyId > std::numeric_limits<BodyId>::max()) {
            throw std::overflow_error("World body IDs are exhausted");
        }

        const auto id = static_cast<BodyId>(this->nextBodyId);
        this->bodies.emplace_back(id, definition.position, definition.velocity,
            definition.inverseMass, definition.collider);
        ++this->nextBodyId;
        return id;
    }

    bool World::removeBody(BodyId id) noexcept {
        const auto it = std::find_if(this->bodies.begin(), this->bodies.end(),
            [id](const Body& body) { return body.getId() == id; });
        if (it == this->bodies.end()) {
            return false;
        }
        this->bodies.erase(it);
        return true;
    }

    void World::clear() noexcept {
        this->bodies.clear();
    }

    std::size_t World::getBodyCount() const noexcept {
        return this->bodies.size();
    }

    Body* World::findBody(BodyId id) noexcept {
        for (Body& body : this->bodies) {
            if (body.getId() == id) {
                return &body;
            }
        }
        return nullptr;
    }

    const Body* World::getBody(BodyId id) const noexcept {
        for (const Body& body : this->bodies) {
            if (body.getId() == id) {
                return &body;
            }
        }
        return nullptr;
    }

    std::span<const Body> World::getBodies() const noexcept {
        return this->bodies;
    }

    bool World::setBodyPosition(BodyId id, const Vec3& position) {
        Body* body = this->findBody(id);
        if (body == nullptr) {
            return false;
        }
        validateVector(position);
        body->setPosition(position);
        return true;
    }

    bool World::setBodyVelocity(BodyId id, const Vec3& velocity) {
        Body* body = this->findBody(id);
        if (body == nullptr) {
            return false;
        }
        validateVector(velocity);
        body->setVelocity(velocity);
        return true;
    }

    bool World::setBodyCollider(BodyId id, const Collider& collider) {
        Body* body = this->findBody(id);
        if (body == nullptr) {
            return false;
        }
        validateCollider(collider);
        body->setCollider(collider);
        return true;
    }

    const Vec3& World::getGravity() const noexcept {
        return this->gravity;
    }

    void World::setGravity(const Vec3& gravity) {
        validateVector(gravity);
        this->gravity = gravity;
    }

    void World::step(double deltaTime) {
        if (!std::isfinite(deltaTime) || deltaTime < 0.0) {
            throw std::invalid_argument("Time step must be finite and non-negative");
        }
        if (deltaTime == 0.0) {
            return;
        }

        for (Body& body : this->bodies) {
            if (body.getInverseMass() == 0.0) {
                continue;
            }
            body.setVelocity(body.getVelocity() + this->gravity * deltaTime);
            body.setPosition(body.getPosition() + body.getVelocity() * deltaTime);
        }
    }

    std::vector<BodyPair> World::findPotentialCollisions() const {
        std::vector<BodyPair> pairs;
        for (std::size_t i = 0; i < this->bodies.size(); ++i) {
            const Body& first = this->bodies[i];
            for (std::size_t j = i + 1; j < this->bodies.size(); ++j) {
                const Body& second = this->bodies[j];
                if (first.getInverseMass() == 0.0 && second.getInverseMass() == 0.0) {
                    continue;
                }
                if (overlaps(first.aabb(), second.aabb())) {
                    pairs.push_back({first.getId(), second.getId()});
                }
            }
        }
        return pairs;
    }
}
