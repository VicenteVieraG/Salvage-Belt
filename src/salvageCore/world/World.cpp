/**
 * @file
 * @brief World storage, body generation, integration, and broad-phase collision queries.
 */
#include "world/World.hpp"
#include "PlacementIndex.hpp"
#include "physics/AABB.hpp"
#include "physics/Physics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
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

        bool componentwiseLessEqual(const Vec3& lhs, const Vec3& rhs) noexcept {
            return lhs.x <= rhs.x && lhs.y <= rhs.y && lhs.z <= rhs.z;
        }

        bool isOrderedFiniteRange(const RandomBodySettings::Range& range) noexcept {
            return std::isfinite(range.min) && std::isfinite(range.max) && range.min <= range.max;
        }

        void validateRandomBodySettings(const RandomBodySettings& settings) {
            const auto& [min, max] = settings.spawnBounds;
            if (!isFinite(min) || !isFinite(max) || !componentwiseLessEqual(min, max)) {
                throw std::invalid_argument("Spawn bounds must be finite and ordered");
            }
            if (!isOrderedFiniteRange(settings.extent) || settings.extent.min <= 0.0
                || !isOrderedFiniteRange(settings.velocity)
                || !isOrderedFiniteRange(settings.inverseMass) || settings.inverseMass.min < 0.0
                || !isOrderedFiniteRange(settings.restitution) || !isOrderedFiniteRange(settings.friction)) {
                throw std::invalid_argument("Invalid random body attribute range");
            }
            validateMaterial({settings.restitution.min, settings.friction.min});
            validateMaterial({settings.restitution.max, settings.friction.max});
            if (settings.maxPlacementAttempts == 0) {
                throw std::invalid_argument("Random body placement requires at least one attempt per body");
            }
        }

        void forEachPotentialCollision(const World& world, const IBroadPhase& broadPhase, auto visit) {
            std::vector<CollisionPair> candidates;
            broadPhase.findCandidates(world.getBodies(), candidates);
            // Sequential impulses and snapshot queries both use creation order.
            for (auto& [first, second] : candidates) {
                if (second < first) {
                    std::swap(first, second);
                }
            }
            if (!std::ranges::is_sorted(candidates)) {
                std::ranges::sort(candidates);
            }
            for (const auto& [firstId, secondId] : candidates) {
                const Body& first = *world.getBody(firstId);
                const Body& second = *world.getBody(secondId);
                if (first.getInverseMass() == 0.0 && second.getInverseMass() == 0.0) {
                    continue;
                }
                visit(first, second);
            }
        }

        std::vector<Contact> collectContacts(const World& world, const IBroadPhase& broadPhase) {
            std::vector<Contact> contacts;
            forEachPotentialCollision(world, broadPhase, [&](const Body& first, const Body& second) {
                if (const auto contact = detectContact(first, second)) {
                    contacts.push_back(*contact);
                }
            });
            return contacts;
        }
    }

    RandomBodySettings RandomBodySettings::forBodyCount(std::size_t count, double spacing) {
        const double halfSize = (0.5 * std::cbrt(static_cast<double>(std::max(count, std::size_t{1})))) * spacing;
        if (!std::isfinite(halfSize) || halfSize <= 0.0) {
            throw std::invalid_argument("Body spacing must produce a positive, finite spawn half-size");
        }
        return {.spawnBounds = {{-halfSize, -halfSize, -halfSize}, {halfSize, halfSize, halfSize}}};
    }

    World::World(const Vec3& gravity) : World(Algorithm::BruteForce, gravity) {}

    World::World(Algorithm algorithm, const Vec3& gravity)
        : World(BroadPhaseOptions{.algorithm = algorithm}, gravity) {}

    World::World(BroadPhaseOptions options, const Vec3& gravity)
        : broadPhaseOptions(options), broadPhase(makeInitializedBroadPhase()) {
        this->setGravity(gravity);
    }

    World::World(const World& other)
        : bodies(other.bodies), nextBodyId(other.nextBodyId), gravity(other.gravity),
          solverSettings(other.solverSettings), forces(other.forces), broadPhaseOptions(other.broadPhaseOptions),
          broadPhase(makeInitializedBroadPhase()) {}

    World& World::operator=(const World& other) {
        if (this != &other) {
            *this = World(other);
        }
        return *this;
    }

    std::unique_ptr<IBroadPhase> World::makeInitializedBroadPhase() const {
        auto strategy = makeBroadPhase(this->broadPhaseOptions);
        strategy->initialize(this->bodies);
        return strategy;
    }

    IBroadPhase& World::updateBroadPhase() {
        if (this->broadPhase) {
            this->broadPhase->update(this->bodies);
        } else {
            // A moved-from world can be populated and stepped again.
            this->broadPhase = this->makeInitializedBroadPhase();
        }
        return *this->broadPhase;
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

    void World::generateRandomBodies(std::size_t count, std::uint64_t seed) {
        const auto settings = count > 1000 ? RandomBodySettings::forBodyCount(count) : RandomBodySettings{};
        this->generateRandomBodies(count, seed, settings);
    }

    // Preserve the public count and seed parameter types.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    void World::generateRandomBodies(std::size_t count, std::uint64_t seed, const RandomBodySettings& settings) {
        if (count == 0) return;
        validateRandomBodySettings(settings);

        constexpr auto idCapacity = static_cast<std::uint64_t>(std::numeric_limits<BodyId>::max()) + 1;
        if (count > idCapacity - this->nextBodyId) {
            throw std::overflow_error("Too few world body IDs for the requested batch");
        }

        // Stage the full batch using the normal body creation path before publishing it.
        World staged;
        staged.nextBodyId = this->nextBodyId;
        staged.bodies.reserve(count);

        detail::PlacementIndex placement{settings.spawnBounds, settings.extent.max, count};
        for (const Body& body : this->bodies) placement.add(body.aabb());

        std::mt19937_64 generator(seed);
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        std::bernoulli_distribution chooseSphere(0.5);
        const auto sample = [&](const RandomBodySettings::Range& range) {
            return std::clamp(std::lerp(range.min, range.max, unit(generator)), range.min, range.max);
        };

        const auto& [spawnMin, spawnMax] = settings.spawnBounds;
        std::size_t attempts = 0;
        while (staged.getBodyCount() < count) {
            if (attempts == settings.maxPlacementAttempts) {
                throw std::runtime_error("Could not place a random body within the attempt limit");
            }
            ++attempts;

            const Collider collider = chooseSphere(generator)
                ? Collider{SphereCollider{sample(settings.extent)}}
                : Collider{BoxCollider{{sample(settings.extent), sample(settings.extent), sample(settings.extent)}}};
            const Vec3 extents = std::visit([](const auto& shape) { return getExtents(shape); }, collider);
            const Vec3 centerMin = spawnMin + extents;
            const Vec3 centerMax = spawnMax - extents;
            if (!componentwiseLessEqual(centerMin, centerMax)) continue;

            const Vec3 position{
                sample({centerMin.x, centerMax.x}),
                sample({centerMin.y, centerMax.y}),
                sample({centerMin.z, centerMax.z})
            };
            const AABB bounds = computeAABB(position, collider);
            // Verify containment after rounding the center and its bounds.
            if (!componentwiseLessEqual(spawnMin, bounds.min) || !componentwiseLessEqual(bounds.max, spawnMax)) continue;

            if (placement.overlapsAny(bounds)) continue;

            const Vec3 velocity{sample(settings.velocity), sample(settings.velocity), sample(settings.velocity)};
            const double inverseMass = sample(settings.inverseMass);
            const Material material{sample(settings.restitution), sample(settings.friction)};
            staged.createBody(Body{position, velocity, inverseMass, collider, material});
            placement.add(staged.bodies.back().aabb());
            attempts = 0;
        }

        if (this->bodies.empty()) {
            this->bodies.swap(staged.bodies);
        } else {
            this->bodies.insert(this->bodies.end(), staged.bodies.begin(), staged.bodies.end());
        }
        this->nextBodyId = staged.nextBodyId;
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

        IBroadPhase& strategy = this->updateBroadPhase();
        auto contacts = collectContacts(*this, strategy);
        solveContactVelocities(this->bodies, contacts, this->solverSettings);
        for (std::size_t iteration = 0; iteration < this->solverSettings.positionIterations; ++iteration) {
            // Velocity impulses preserve bounds. Only a previous correction requires a new scan.
            if (iteration > 0) {
                strategy.update(this->bodies);
                contacts = collectContacts(*this, strategy);
            }
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
        const auto strategy = this->makeInitializedBroadPhase();
        return collectContacts(*this, *strategy);
    }

    std::vector<BodyPair> World::findPotentialCollisions() const {
        const auto strategy = this->makeInitializedBroadPhase();
        std::vector<BodyPair> pairs;
        forEachPotentialCollision(*this, *strategy, [&](const Body& first, const Body& second) {
            if (overlaps(first.aabb(), second.aabb())) {
                pairs.emplace_back(first, second);
            }
        });
        return pairs;
    }
}
