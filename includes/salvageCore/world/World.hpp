/**
 * @file
 * @brief World ownership, body management, and simulation API.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>
#include "Body.hpp"

namespace salvage::core {
    /**
     * @brief Own bodies and advance their translation under uniform gravity.
     * IDs are local to each World. Bodies are stored in creation order.
     * The simulation updates position, velocity, and bounds; it does not resolve contacts.
     */
    class World {
        private:
            /// @brief Bodies in strictly increasing ID order; removal preserves order.
            std::vector<Body> bodies{};
            std::uint64_t nextBodyId{}; ///< Next ID, wide enough to detect BodyId exhaustion.
            Vec3 gravity{}; ///< Uniform acceleration applied to dynamic bodies.

            /**
             * @brief Find a mutable body by its world-local ID.
             * @param id Identifier to look up.
             * @return A pointer to the body, or nullptr if absent.
             */
            Body* findBody(BodyId id) noexcept;
        public:
            /**
             * @brief Create an empty world with uniform acceleration.
             * @param gravity Finite acceleration vector; defaults to zero.
             * @throws std::invalid_argument If any gravity component is not finite.
             */
            explicit World(const Vec3& gravity = {});

            /**
             * @brief Copy a body into this world and assign a fresh ID.
             * IDs start at zero and are never reused by creation, removal, or clear().
             * The input may refer to a body already in this world; its state is copied before
             * storage grows. Validation failures and ID exhaustion leave the world unchanged.
             * @param body State to copy; its input ID is ignored.
             * @return The newly assigned world-local identifier.
             * @throws std::invalid_argument If position or velocity is non-finite, or if
             * inverse mass or collider extents are non-finite or negative.
             * @throws std::overflow_error If all BodyId values have been used.
             * @note Reacquire body pointers and views after this call.
             */
            BodyId createBody(const Body& body = {});
            /**
             * @brief Remove a body while preserving the remaining creation order.
             * @param id Identifier of the body to remove.
             * @return True if removed, or false if the ID is unknown.
             * @note Reacquire body pointers and views after removal.
             */
            bool removeBody(BodyId id) noexcept;
            /**
             * @brief Remove all bodies while preserving gravity and the next ID.
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
             * @brief Advance dynamic bodies using the position and velocity integration helpers.
             * Gravity is constant throughout each step. Bodies with zero inverse mass remain
             * unchanged, including their velocity. Updated positions refresh cached bounds.
             * @param deltaTime Finite, non-negative elapsed time; zero is a no-op.
             * @pre Resulting position and velocity arithmetic remains in range.
             * @throws std::invalid_argument If deltaTime is non-finite or negative; bodies are unchanged.
             * @note Position integration currently has the formula limitation documented in
             * velocityVerletPosition().
             * @see velocityVerletPosition()
             * @see velocityVerletVelocity()
             */
            void step(double deltaTime);

            /**
             * @brief Find potential collisions using inclusive AABB overlap tests.
             * Reports each pair once in creation order and omits static/static pairs.
             * Touching bounds count as overlaps. Candidates can be false positives for the
             * actual shapes; no contact resolution is performed.
             * @return Independent body copies; editing them does not change the world.
             * @note Currently delegates to BruteForce, with quadratic time complexity.
             * @see BruteForce::broadPhase()
             */
            std::vector<BodyPair> findPotentialCollisions() const;
    };
}
