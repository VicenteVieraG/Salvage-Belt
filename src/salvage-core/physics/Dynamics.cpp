#include "salvage/core/physics/Dynamics.hpp"
#include "salvage/core/execution/Executor.hpp"

#include <cmath>
#include <stdexcept>

namespace salvage::core {

void validateSolverSettings(const SolverSettings& s) {
    if (s.velocityIterations == 0 || s.positionIterations == 0 || !std::isfinite(s.penetrationSlop)
        || s.penetrationSlop < 0 || !std::isfinite(s.positionCorrection) || s.positionCorrection <= 0
        || s.positionCorrection > 1 || !std::isfinite(s.restitutionThreshold) || s.restitutionThreshold < 0)
        throw std::invalid_argument("Invalid contact solver settings");
}

void integrate(WorldView current, std::span<Body> next, double dt, Vec3 gravity,
               std::span<const BodyInput> inputs, Executor& executor) {
    if (next.size() != current.bodies.size() || !(dt > 0) || !std::isfinite(dt) || !gravity.finite())
        throw std::invalid_argument("Invalid integration parameters");
    std::vector<BodyInput> accumulated;
    if (!inputs.empty()) accumulated.resize(next.size());
    for (const auto& input : inputs) {
        if (input.body >= next.size() || !input.force.finite() || !input.impulse.finite())
            throw std::invalid_argument("Invalid body input");
        auto& sum = accumulated[input.body];
        sum.force = sum.force + input.force;
        sum.impulse = sum.impulse + input.impulse;
    }
    executor.parallelFor(next.size(), [&](auto, auto begin, auto end) {
        for (auto i = begin; i < end; ++i) {
            auto& body = next[i] = current.bodies[i];
            const double invMass = body.getInverseMass();
            if (invMass == 0) continue;
            auto v = body.getVelocity();
            auto a = gravity;
            if (!inputs.empty()) {
                v = v + accumulated[i].impulse * invMass;
                a = a + accumulated[i].force * invMass;
            }
            body.setPosition(body.getPosition() + (v + a * (0.5 * dt)) * dt);
            body.setVelocity(v + a * dt);
            if (!body.aabb().min.finite() || !body.aabb().max.finite() || !body.getVelocity().finite())
                throw std::overflow_error("Body integration exceeded finite world coordinates");
        }
    });
}

void resolve(std::span<Body> bodies, std::span<const Contact> contacts, const SolverSettings& settings) {
    solveContactVelocities(bodies, contacts, settings);
    for (std::size_t pass = 0; pass < settings.positionIterations; ++pass) {
        bool corrected = false;
        for (const auto& contact : contacts) {
            auto& a = bodies[contact.first];
            auto& b = bodies[contact.second];
            if (a.getInverseMass() == 0 && b.getInverseMass() == 0) continue;
            const auto geometry = detectContact(a, b);
            if (geometry && geometry->penetration > settings.penetrationSlop) {
                correctContactPosition(a, b, *geometry, settings);
                corrected = true;
            }
        }
        if (!corrected) break;
    }
}

}
