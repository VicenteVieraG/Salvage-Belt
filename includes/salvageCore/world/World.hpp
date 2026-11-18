/**
 * @file
 * @brief World ownership, body management, and simulation API.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <utility>
#include <unordered_map>
#include <vector>
#include "Body.hpp"
#include "algorithm/BroadPhaseFactory.hpp"
#include "physics/ContactSolver.hpp"

namespace salvage::core {
    /** @brief Bounds and placement controls for World::generateRandomBodies(). */
    struct RandomBodySettings {
        /** @brief Inclusive finite range. Equal endpoints fix the attribute's value. */
        struct Range {
            double min{};
            double max{};
        };

        AABB spawnBounds{{-50.0, -50.0, -50.0}, {50.0, 50.0, 50.0}}; ///< Contains the entire collider.
        Range extent{0.5, 2.0}; ///< Positive sphere radius or per-axis box half-extent.
        Range velocity{-5.0, 5.0}; ///< Initial velocity on each axis.
        Range inverseMass{0.25, 2.0}; ///< Non-negative inverse mass. Zero makes the body static.
        Range restitution{0.0, 1.0}; ///< Bounciness in [0, 1].
        Range friction{0.0, 1.0}; ///< Non-negative friction coefficient.
        std::size_t maxPlacementAttempts{128}; ///< Positive retry budget per body.

        /**
         * @brief Set the spawn bounds to a cube centered at the origin.
         * @param count Number of bodies used to size the cube. Zero is treated as one.
         * @param spacing Positive finite length whose cube is the target volume per body.
         * @return Default settings with a cube side length of spacing times the cube root of count.
         * @throws std::invalid_argument If spacing or the resulting half-size is invalid.
         */
        static RandomBodySettings forBodyCount(std::size_t count, double spacing = 10.0);
    };

    /**
     * @brief Own bodies and advance their translation under uniform gravity.
     * IDs are local to each World. Bodies are stored in creation order.
     * The simulation resolves discrete contacts between spheres and axis-aligned boxes.
     */
    class World {
        private:
            /// @brief Bodies in strictly increasing ID order; removal preserves order.
            std::vector<Body> bodies{};
            std::uint64_t nextBodyId{}; ///< Next ID, wide enough to detect BodyId exhaustion.
            Vec3 gravity{}; ///< Uniform acceleration applied to dynamic bodies.
            SolverSettings solverSettings{}; ///< Iteration counts and contact tolerances.
            std::unordered_map<BodyId, Vec3> forces{}; ///< Accumulated external forces for the next positive step.
            Algorithm algorithm; ///< Broad-phase selection for simulation and collision queries.
            std::unique_ptr<IBroadPhase> broadPhase; ///< Strategy reused across simulation steps.

            /// @brief Build an independent index of the current bodies for a read-only query or a world copy.
            std::unique_ptr<IBroadPhase> makeInitializedBroadPhase() const;
            /// @brief Synchronize the simulation strategy with current bodies before querying it.
            IBroadPhase& updateBroadPhase();

            /**
             * @brief Find a mutable body by its world-local ID.
             * @param id Identifier to look up.
             * @return A pointer to the body, or nullptr if absent.
             */
            Body* findBody(BodyId id) noexcept;
        public:
            /**
             * @brief Create an empty world with uniform acceleration and the brute-force broad phase.
             * @param gravity Finite acceleration vector; defaults to zero.
             * @throws std::invalid_argument If any gravity component is not finite.
             */
            explicit World(const Vec3& gravity = {});

            /**
             * @brief Choose the broad phase for this world's steps and collision queries.
             * @param algorithm Broad-phase strategy to use.
             * @param gravity Finite uniform acceleration, defaulting to zero.
             * @throws std::invalid_argument If the algorithm is unknown or gravity is non-finite.
             */
            explicit World(Algorithm algorithm, const Vec3& gravity = {});

            /// @brief Copy simulation state and build an independent broad-phase strategy.
            World(const World& other);
            /// @brief Replace simulation state and broad-phase selection with a copy of another world.
            World& operator=(const World& other);
            /// @brief Transfer simulation state and strategy ownership.
            World(World&&) noexcept = default;
            /// @brief Replace this world with the source's simulation state and strategy.
            World& operator=(World&&) noexcept = default;

            /**
             * @brief Copy a body into this world and assign a fresh ID.
             * IDs start at zero and are never reused by creation, removal, or clear().
             * The input may refer to a body already in this world; its state is copied before
             * storage grows. Validation failures and ID exhaustion leave the world unchanged.
             * @param body State to copy; its input ID is ignored.
             * @return The newly assigned world-local identifier.
             * @throws std::invalid_argument If position or velocity is non-finite, or if
             * inverse mass or collider extents are non-finite or negative, or material is invalid.
             * @throws std::overflow_error If all BodyId values have been used.
             * @note Reacquire body pointers and views after this call.
             */
            BodyId createBody(const Body& body = {});

            /**
             * @brief Generate with default attributes and a cube centered at the origin.
             * The side length is at least 100 and grows with the cube root of count above 1,000 bodies.
             * @see generateRandomBodies(std::size_t, std::uint64_t, const RandomBodySettings&)
             */
            void generateRandomBodies(std::size_t count, std::uint64_t seed);

            /**
             * @brief Append a reproducible batch of random spheres and boxes.
             * Generated bounds must fit inside the spawn region and be disjoint from all
             * existing and newly placed bounds. A failed batch leaves the world and its IDs unchanged.
             * @param count Number of bodies to append. Zero is a no-op.
             * @param seed Seed for the generator. Reuse with the same settings and starting
             * geometry to reproduce the layout on the same standard library implementation.
             * @param settings Finite ordered spawn bounds, attribute ranges, and placement budget.
             * @throws std::invalid_argument If settings are invalid.
             * @throws std::overflow_error If there are too few available body IDs.
             * @throws std::runtime_error If placement runs out of attempts.
             * @note Reacquire body pointers and views after successful generation.
             */
            void generateRandomBodies(std::size_t count, std::uint64_t seed, const RandomBodySettings& settings);

            /**
             * @brief Remove a body while preserving the remaining creation order.
             * @param id Identifier of the body to remove.
             * @return True if removed, or false if the ID is unknown.
             * @note Reacquire body pointers and views after removal.
             */
            bool removeBody(BodyId id) noexcept;
            /**
             * @brief Remove all bodies and pending forces, preserving gravity, solver settings, and the next ID.
             * @note Invalidates all body pointers and views.
             */
            void clear() noexcept;

            /**
             * @brief Count bodies currently owned by the world.
             * @return The number of bodies.
             */
            std::size_t getBodyCount() const noexcept;
            /**
             * @brief Look up a body by its world-local identifier.
             * @param id Identifier to look up.
             * @return A read-only pointer, or nullptr if the ID is unknown.
             * @note Reacquire the pointer after creation, removal, or clear().
             */
            const Body* getBody(BodyId id) const noexcept;
            /**
             * @brief View bodies in creation order without copying them.
             * @return A read-only view of the owned bodies.
             * @note Reacquire the view after creation, removal, or clear().
             */
            std::span<const Body> getBodies() const noexcept;

            /**
             * @brief Set the position of an existing body.
             * The cached bounds are refreshed.
             * @param id Identifier of the body to edit.
             * @param position New position.
             * @return True if updated, or false for an unknown ID (without validating the value).
             * @throws std::invalid_argument If any position component is not finite;
             * the existing body remains unchanged.
             */
            bool setBodyPosition(BodyId id, const Vec3& position);
            /**
             * @brief Set the velocity of an existing body.
             * The cached bounds are unchanged.
             * @param id Identifier of the body to edit.
             * @param velocity New velocity.
             * @return True if updated, or false for an unknown ID (without validating the value).
             * @throws std::invalid_argument If any velocity component is not finite;
             * the existing body remains unchanged.
             */
            bool setBodyVelocity(BodyId id, const Vec3& velocity);
            /**
             * @brief Set the collider of an existing body.
             * The cached bounds are refreshed.
             * @param id Identifier of the body to edit.
             * @param collider New collider.
             * @return True if updated, or false for an unknown ID (without validating the value).
             * @throws std::invalid_argument If any collider extent is non-finite or negative;
             * the existing body remains unchanged.
             */
            bool setBodyCollider(BodyId id, const Collider& collider);

            /**
             * @brief Set a body's bounce and friction properties.
             * @param id Identifier to edit.
             * @param material Restitution in [0, 1] and finite non-negative friction.
             * @return True if updated, false for an unknown ID without validating the value.
             * @throws std::invalid_argument If material is invalid; the body is unchanged.
             */
            bool setBodyMaterial(BodyId id, const Material& material);

            /**
             * @brief Accumulate a constant external force for the next positive-duration step.
             * Acceleration is gravity + total force * inverse mass. Forces are cleared after
             * integration; zero/invalid steps preserve them. Static bodies ignore valid forces.
             * @param id Body receiving the force.
             * @param force Finite force vector, added to any already pending force.
             * @return True if the body exists, false otherwise without validation.
             * @throws std::invalid_argument If force or its accumulated sum is non-finite; state is unchanged.
             */
            bool addBodyForce(BodyId id, const Vec3& force);

            /**
             * @brief Apply an instantaneous external impulse to a dynamic body.
             * Static bodies ignore valid impulses; pending forces are unaffected.
             * @param id Body receiving the impulse.
             * @param impulse Finite impulse; velocity change is impulse * inverse mass.
             * @return True if the body exists, false otherwise without validation.
             * @throws std::invalid_argument If impulse or resulting velocity is non-finite; state is unchanged.
             */
            bool applyBodyImpulse(BodyId id, const Vec3& impulse);

            /** @brief Read contact solver controls. @return Current settings. */
            const SolverSettings& getSolverSettings() const noexcept;
            /**
             * @brief Configure contact solver iterations and tolerances.
             * @param settings Positive iteration counts, finite non-negative slop and bounce
             * threshold, and a finite correction fraction in (0, 1].
             * @throws std::invalid_argument If settings are invalid; existing settings are unchanged.
             */
            void setSolverSettings(const SolverSettings& settings);

            /**
             * @brief Read the uniform acceleration.
             * @return A reference to the stored gravity vector.
             */
            const Vec3& getGravity() const noexcept;
            /**
             * @brief Set uniform acceleration for subsequent steps.
             * @param gravity New finite acceleration vector.
             * @throws std::invalid_argument If any component is not finite; gravity is unchanged.
             */
            void setGravity(const Vec3& gravity);

            /**
             * @brief Integrate movement, detect contacts, solve impulses, and correct overlap.
             * Gravity and accumulated external forces are constant throughout each step.
             * Bodies with zero inverse mass remain
             * unchanged, including their velocity. Updated positions refresh cached bounds.
             * @param deltaTime Finite, non-negative elapsed time; zero is a no-op.
             * @pre Intermediate integration and contact arithmetic remains in range.
             * @throws std::invalid_argument If deltaTime is non-finite or negative; bodies are unchanged.
             * @note Discrete collision detection can miss objects crossed entirely within a
             * step. Use sufficiently small steps. Rotation is not modeled. New contacts created
             * by position correction get velocity response on the next step.
             * @see velocityVerletPosition()
             * @see velocityVerletVelocity()
             */
            void step(double deltaTime);

            /**
             * @brief Query confirmed contacts without changing the world.
             * Uses the selected broad phase, rejects shape-level false positives,
             * omits static/static pairs, and includes touching shapes.
             * @return Independent contact values containing world-local body IDs.
             */
            std::vector<Contact> findContacts() const;

            /**
             * @brief Find potential collisions using inclusive AABB overlap tests.
             * Reports each pair once in creation order and omits static/static pairs.
             * Touching bounds count as overlaps. Candidates can be false positives for the
             * actual shapes; no contact resolution is performed.
             * @return Independent body copies; editing them does not change the world.
             */
            std::vector<BodyPair> findPotentialCollisions() const;
    };
}
