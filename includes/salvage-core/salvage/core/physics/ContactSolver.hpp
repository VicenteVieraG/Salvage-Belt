/**
 * @file
 * @brief Iterative translational contact response.
 */
#pragma once

#include <cstddef>
#include <span>
#include "salvage/core/collision/Contact.hpp"

namespace salvage::core {
    /** @brief Contact solver controls, validated by Simulation at run initialization. */
    struct SolverSettings {
        std::size_t velocityIterations{16}; ///< Positive number of sequential impulse passes.
        std::size_t positionIterations{8}; ///< Positive number of overlap correction passes.
        double penetrationSlop{1e-5}; ///< Allowed overlap in world distance units, finite and non-negative.
        double positionCorrection{0.8}; ///< Fraction of excess overlap corrected per pass, in (0, 1].
        double restitutionThreshold{0.5}; ///< Closing speed below which bounce is suppressed, finite and non-negative.
    };

    /**
     * @brief Resolve contact velocities with accumulated normal and Coulomb friction impulses.
     * Restitution mixes by maximum; friction mixes by geometric mean. Static
     * bodies contribute zero velocity regardless of their stored velocity.
     * @param bodies Mutable world bodies in dense ID order, unchanged in storage during the call.
     * @param contacts Unique unordered pairs with valid normals and IDs in bodies.
     * @param settings Valid solver settings.
     * @pre Valid body masses/materials and finite arithmetic throughout the solve.
     */
    void solveContactVelocities(std::span<Body> bodies, std::span<const Contact> contacts,
        const SolverSettings& settings);

    /**
     * @brief Correct one overlap using inverse-mass weighting, refreshing both bounds.
     * Does not change velocities or move static bodies.
     * @param first Body matching contact.first.
     * @param second Body matching contact.second.
     * @param contact Current geometry; recompute after position changes.
     * @param settings Valid correction fraction and slop.
     * @pre Valid masses/contact and finite arithmetic throughout the correction.
     */
    void correctContactPosition(Body& first, Body& second, const Contact& contact,
        const SolverSettings& settings) noexcept;
}
