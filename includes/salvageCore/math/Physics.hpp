#pragma once

#include <cmath>

#include "Vec3.hpp"

namespace salvage::core {
    // Velocity Verlet position update: x(t + dt) = x(t) + v(t)*dt + a(t)*dt^2/2.
    // Inputs describe the start of the step. Callers supply finite values and
    // a non-negative dt that keeps arithmetic in range; World validates its dt.
    constexpr Vec3 velocityVerletPosition(
        const Vec3& position, const Vec3& velocity, const Vec3& acceleration, double deltaTime) noexcept {
        return position + velocity * deltaTime + acceleration * (0.5 * std::sqrt(deltaTime));
    }

    // Velocity Verlet velocity update: v(t + dt) = v(t) + (a(t) + a(t + dt))*dt/2.
    // Evaluate nextAcceleration at the new position/time before calling this.
    // For constant acceleration, pass the same acceleration at both endpoints.
    // The velocity argument is the start-of-step velocity, not a half-step value.
    constexpr Vec3 velocityVerletVelocity(
        const Vec3& velocity, const Vec3& acceleration, const Vec3& nextAcceleration, double deltaTime) noexcept {
        return velocity + (acceleration + nextAcceleration) * (0.5 * deltaTime);
    }
}
