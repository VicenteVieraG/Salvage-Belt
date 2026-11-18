/** @file @brief Accumulated sequential impulses and positional stabilization. */
#include "physics/ContactSolver.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace salvage::core {
    namespace {
        struct Constraint {
            Body* first;
            Body* second;
            Vec3 normal;
            double inverseMassSum;
            double targetVelocity;
            double friction;
            double normalImpulse{};
            Vec3 tangentImpulse{};
        };

        Vec3 velocity(const Body& body) noexcept {
            return body.getInverseMass() == 0.0 ? Vec3{} : body.getVelocity();
        }

        void applyImpulse(Constraint& constraint, const Vec3& impulse) noexcept {
            if (constraint.first->getInverseMass() > 0.0) {
                constraint.first->setVelocity(constraint.first->getVelocity()
                    - impulse * constraint.first->getInverseMass());
            }
            if (constraint.second->getInverseMass() > 0.0) {
                constraint.second->setVelocity(constraint.second->getVelocity()
                    + impulse * constraint.second->getInverseMass());
            }
        }
    }

    void solveContactVelocities(std::span<Body> bodies, std::span<const Contact> contacts,
        const SolverSettings& settings) {
        if (contacts.empty()) return;
        std::vector<Constraint> constraints;
        constraints.reserve(contacts.size());
        // Avoid a full body scan per contact in dense scenes. Iteration order
        // still comes exclusively from the deterministic contact list.
        std::unordered_map<BodyId, Body*> bodyIndex;
        bodyIndex.reserve(bodies.size());
        for (Body& body : bodies) bodyIndex.emplace(body.getId(), &body);
        for (const Contact& contact : contacts) {
            const auto firstEntry = bodyIndex.find(contact.first);
            const auto secondEntry = bodyIndex.find(contact.second);
            if (firstEntry == bodyIndex.end() || secondEntry == bodyIndex.end() || firstEntry == secondEntry) continue;
            Body* first = firstEntry->second;
            Body* second = secondEntry->second;
            const double inverseMassSum = first->getInverseMass() + second->getInverseMass();
            if (inverseMassSum == 0.0) continue;
            const double closingVelocity = (velocity(*second) - velocity(*first)).dot(contact.normal);
            const double restitution = std::max(first->getMaterial().restitution, second->getMaterial().restitution);
            // Compute the bounce target once; recomputing it during iterations
            // would repeatedly add energy or erase the first bounce impulse.
            const double target = closingVelocity < -settings.restitutionThreshold ? -restitution * closingVelocity : 0.0;
            const double friction = std::sqrt(first->getMaterial().friction) * std::sqrt(second->getMaterial().friction);
            constraints.push_back({first, second, contact.normal, inverseMassSum, target, friction});
        }

        for (std::size_t iteration = 0; iteration < settings.velocityIterations; ++iteration) {
            for (Constraint& constraint : constraints) {
                Vec3 relative = velocity(*constraint.second) - velocity(*constraint.first);
                const double change = (constraint.targetVelocity - relative.dot(constraint.normal)) / constraint.inverseMassSum;
                const double oldNormalImpulse = constraint.normalImpulse;
                constraint.normalImpulse = std::max(0.0, oldNormalImpulse + change);
                applyImpulse(constraint, constraint.normal * (constraint.normalImpulse - oldNormalImpulse));

                relative = velocity(*constraint.second) - velocity(*constraint.first);
                const Vec3 tangent = relative - constraint.normal * relative.dot(constraint.normal);
                Vec3 nextTangentImpulse = constraint.tangentImpulse - tangent * (1.0 / constraint.inverseMassSum);
                const double magnitude = std::sqrt(nextTangentImpulse.lengthSquared());
                const double limit = constraint.friction * constraint.normalImpulse;
                if (magnitude > limit) nextTangentImpulse = nextTangentImpulse * (limit / magnitude);
                applyImpulse(constraint, nextTangentImpulse - constraint.tangentImpulse);
                constraint.tangentImpulse = nextTangentImpulse;
            }
        }
    }

    void correctContactPosition(Body& first, Body& second, const Contact& contact,
        const SolverSettings& settings) noexcept {
        const double inverseMassSum = first.getInverseMass() + second.getInverseMass();
        if (inverseMassSum == 0.0) return;
        const double distance = settings.positionCorrection * std::max(0.0, contact.penetration - settings.penetrationSlop);
        const Vec3 correction = contact.normal * (distance / inverseMassSum);
        if (first.getInverseMass() > 0.0) first.setPosition(first.getPosition() - correction * first.getInverseMass());
        if (second.getInverseMass() > 0.0) second.setPosition(second.getPosition() + correction * second.getInverseMass());
    }
}
