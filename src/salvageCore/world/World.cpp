/**
 * @file
 * @brief World storage, validation, integration, and broad-phase collision queries.
 */
#include "world/World.hpp"
#include "physics/AABB.hpp"
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

        void validateMaterial(const Material& material) {
            if (!std::isfinite(material.restitution) || material.restitution < 0.0 || material.restitution > 1.0
                || !std::isfinite(material.friction) || material.friction < 0.0) {
                throw std::invalid_argument("Material requires restitution in [0, 1] and finite non-negative friction");
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
        validateMaterial(body.getMaterial());
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
            body.getCollider(),
            body.getMaterial()
        });
        this->nextBodyId++;
        return id;
    }

    bool World::removeBody(BodyId id) noexcept {
        if(const auto it = std::ranges::lower_bound(this->bodies, id, {}, &Body::getId);
            it == this->bodies.end() || it->getId() != id) return false;
        else this->bodies.erase(it);
        this->forces.erase(id);
        return true;
    }

    void World::clear() noexcept {
        this->bodies.clear();
        this->forces.clear();
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

    bool World::setBodyMaterial(BodyId id, const Material& material) {
        if (Body* body = this->findBody(id)) {
            validateMaterial(material);
            body->setMaterial(material);
            return true;
        }
        return false;
    }

    const SolverSettings& World::getSolverSettings() const noexcept {
        return this->solverSettings;
    }

    bool World::addBodyForce(BodyId id, const Vec3& force) {
        const Body* body = this->getBody(id);
        if (!body) return false;
        validateVector(force);
        if (body->getInverseMass() == 0.0) return true;
        const auto pending = this->forces.find(id);
        const Vec3 sum = pending == this->forces.end() ? force : pending->second + force;
        validateVector(sum);
        this->forces.insert_or_assign(id, sum);
        return true;
    }

    bool World::applyBodyImpulse(BodyId id, const Vec3& impulse) {
        Body* body = this->findBody(id);
        if (!body) return false;
        validateVector(impulse);
        if (body->getInverseMass() == 0.0) return true;
        const Vec3 velocity = body->getVelocity() + impulse * body->getInverseMass();
        validateVector(velocity);
        body->setVelocity(velocity);
        return true;
    }

    void World::setSolverSettings(const SolverSettings& settings) {
        if (settings.velocityIterations == 0 || settings.positionIterations == 0
            || !std::isfinite(settings.penetrationSlop) || settings.penetrationSlop < 0.0
            || !std::isfinite(settings.positionCorrection) || settings.positionCorrection <= 0.0 || settings.positionCorrection > 1.0
            || !std::isfinite(settings.restitutionThreshold) || settings.restitutionThreshold < 0.0) {
            throw std::invalid_argument("Invalid contact solver settings");
        }
        this->solverSettings = settings;
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

            const auto force = this->forces.find(body.getId());
            const Vec3 acceleration = this->gravity + (force == this->forces.end() ? Vec3{} : force->second * body.getInverseMass());
            const Vec3 position = velocityVerletPosition(body.getPosition(), body.getVelocity(), acceleration, deltaTime);
            const Vec3 velocity = velocityVerletVelocity(body.getVelocity(), acceleration, acceleration, deltaTime);

            body.setPosition(position);
            body.setVelocity(velocity);
        }

        this->forces.clear();

        solveContactVelocities(this->bodies, this->findContacts(), this->solverSettings);
        for (std::size_t iteration = 0; iteration < this->solverSettings.positionIterations; ++iteration) {
            const auto contacts = this->findContacts();
            bool corrected = false;
            for (const Contact& contact : contacts) {
                Body* first = this->findBody(contact.first);
                Body* second = this->findBody(contact.second);
                // Earlier corrections may have changed this pair's geometry.
                if (const auto current = detectContact(*first, *second);
                    current && current->penetration > this->solverSettings.penetrationSlop) {
                    correctContactPosition(*first, *second, *current, this->solverSettings);
                    corrected = true;
                }
            }
            if (!corrected) break;
        }
    }

    std::vector<Contact> World::findContacts() const {
        std::vector<Contact> contacts;
        for (std::size_t i = 0; i < this->bodies.size(); ++i) {
            for (std::size_t j = i + 1; j < this->bodies.size(); ++j) {
                const Body& first = this->bodies[i];
                const Body& second = this->bodies[j];
                if (first.getInverseMass() == 0.0 && second.getInverseMass() == 0.0) continue;
                if (!overlaps(first.aabb(), second.aabb())) continue;
                if (const auto contact = detectContact(first, second)) contacts.push_back(*contact);
            }
        }
        return contacts;
    }

    std::vector<BodyPair> World::findPotentialCollisions() const {
        // Preserve the existing query until the strategy interface is integrated.
        std::vector<BodyPair> pairs;

        for (std::size_t i = 0; i < this->bodies.size(); i++) {
            for (std::size_t j = i + 1; j < this->bodies.size(); j++) {
                const Body& first = this->bodies[i];
                const Body& second = this->bodies[j];

                if (first.getInverseMass() == 0.0 && second.getInverseMass() == 0.0) continue;
                if (overlaps(first.aabb(), second.aabb())) pairs.emplace_back(first, second);
            }
        }
        return pairs;
    }
}
