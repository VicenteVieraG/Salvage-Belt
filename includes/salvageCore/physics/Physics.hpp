/**
 * @file
 * @brief Position and velocity integration helpers.
 */
#pragma once

#include <cmath>

#include "Vec3.hpp"

namespace salvage::core {
    /**
     * @brief Compute the position update used by the integrator.
     * @param position Position at the start of the step.
     * @param velocity Velocity at the start of the step.
     * @param acceleration Acceleration at the start of the step.
     * @param deltaTime Non-negative elapsed time.
     * @pre Inputs are finite and arithmetic remains in range; no validation is performed.
     * @return position + velocity * deltaTime + acceleration * (0.5 * std::pow(deltaTime, 2)).
     * @see velocityVerletVelocity()
     */
    constexpr Vec3 velocityVerletPosition(
        const Vec3& position, const Vec3& velocity, const Vec3& acceleration, double deltaTime) noexcept {
        return position + velocity * deltaTime + acceleration * (0.5 * std::pow(deltaTime, 2));
    }

    /**
     * @brief Compute the Velocity Verlet velocity update.
     * @param velocity Start-of-step velocity, not a half-step value.
     * @param acceleration Acceleration at the start of the step.
     * @param nextAcceleration Acceleration evaluated at the new position and time.
     * @param deltaTime Non-negative elapsed time.
     * @pre Inputs are finite and arithmetic remains in range; no validation is performed.
     * @return velocity + (acceleration + nextAcceleration) * deltaTime / 2.
     * @note For constant acceleration, pass the same acceleration at both endpoints.
     * @see velocityVerletPosition()
     */
    constexpr Vec3 velocityVerletVelocity(
        const Vec3& velocity, const Vec3& acceleration, const Vec3& nextAcceleration, double deltaTime) noexcept {
        return velocity + (acceleration + nextAcceleration) * (0.5 * deltaTime);
    }
}
