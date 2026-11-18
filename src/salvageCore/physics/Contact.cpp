/** @file @brief Narrow-phase geometry for the supported collider pairs. */
#include "physics/Contact.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace salvage::core {
    namespace {
        constexpr std::array<Vec3, 3> axes{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};

        // Stable directions for configurations that have no unique normal.
        double direction(double offset, BodyId first, BodyId second) noexcept {
            if (offset != 0.0) return offset > 0.0 ? 1.0 : -1.0;
            return first <= second ? 1.0 : -1.0;
        }

        std::optional<Contact> sphereSphere(const Body& first, const Body& second,
            double firstRadius, double secondRadius) noexcept {
            const Vec3 delta = second.getPosition() - first.getPosition();
            const double distance = std::sqrt(delta.lengthSquared());
            const double radius = firstRadius + secondRadius;
            if (distance > radius) return std::nullopt;

            const Vec3 normal = distance > 0.0 ? delta * (1.0 / distance)
                : axes[0] * direction(0.0, first.getId(), second.getId());
            const Vec3 firstPoint = first.getPosition() + normal * firstRadius;
            const Vec3 secondPoint = second.getPosition() - normal * secondRadius;
            return Contact{first.getId(), second.getId(), normal,
                (firstPoint + secondPoint) * 0.5, radius - distance};
        }

        std::optional<Contact> sphereBox(const Body& sphere, const Body& box, double radius) noexcept {
            const Vec3 center = sphere.getPosition();
            const AABB& bounds = box.aabb();
            Vec3 closest{
                std::clamp(center.x, bounds.min.x, bounds.max.x),
                std::clamp(center.y, bounds.min.y, bounds.max.y),
                std::clamp(center.z, bounds.min.z, bounds.max.z)
            };
            const Vec3 delta = closest - center;
            const double distance = std::sqrt(delta.lengthSquared());
            if (distance > radius) return std::nullopt;

            Vec3 normal;
            double penetration;
            if (distance > 0.0) {
                normal = delta * (1.0 / distance);
                penetration = radius - distance;
            } else {
                // Center inside/on the box: use the nearest exit face. The sphere
                // must move opposite the contact normal to leave that face.
                const Vec3 half = std::get<BoxCollider>(box.getCollider()).halfExtents;
                const Vec3 offset = center - box.getPosition();
                const std::array<double, 3> offsets{offset.x, offset.y, offset.z};
                const std::array<double, 3> distances{
                    half.x - std::abs(offset.x), half.y - std::abs(offset.y), half.z - std::abs(offset.z)};
                const auto axis = static_cast<std::size_t>(std::min_element(distances.begin(), distances.end()) - distances.begin());
                const Vec3 outward = axes[axis] * direction(offsets[axis], sphere.getId(), box.getId());
                const double faceDistance = std::max(0.0, distances[axis]);
                normal = outward * -1.0;
                closest = center + outward * faceDistance;
                penetration = radius + faceDistance;
            }
            return Contact{sphere.getId(), box.getId(), normal,
                (center + normal * radius + closest) * 0.5, penetration};
        }

        std::optional<Contact> boxBox(const Body& first, const Body& second) noexcept {
            if (!overlaps(first.aabb(), second.aabb())) return std::nullopt;
            const Vec3 delta = second.getPosition() - first.getPosition();
            const Vec3 extents = std::get<BoxCollider>(first.getCollider()).halfExtents
                + std::get<BoxCollider>(second.getCollider()).halfExtents;
            // Sum of extents minus center distance also gives the correct escape
            // distance for containment, unlike the intersection interval length.
            const std::array<double, 3> depths{
                extents.x - std::abs(delta.x), extents.y - std::abs(delta.y), extents.z - std::abs(delta.z)};
            const std::array<double, 3> offsets{delta.x, delta.y, delta.z};
            const auto axis = static_cast<std::size_t>(std::min_element(depths.begin(), depths.end()) - depths.begin());
            const double sign = direction(offsets[axis], first.getId(), second.getId());
            const Vec3 normal = axes[axis] * sign;
            const AABB& a = first.aabb();
            const AABB& b = second.aabb();
            Vec3 point{
                (std::max(a.min.x, b.min.x) + std::min(a.max.x, b.max.x)) * 0.5,
                (std::max(a.min.y, b.min.y) + std::min(a.max.y, b.max.y)) * 0.5,
                (std::max(a.min.z, b.min.z) + std::min(a.max.z, b.max.z)) * 0.5
            };
            const Vec3 firstSurface = sign > 0.0 ? a.max : a.min;
            const Vec3 secondSurface = sign > 0.0 ? b.min : b.max;
            const double surfaceMidpoint = (firstSurface.dot(axes[axis]) + secondSurface.dot(axes[axis])) * 0.5;
            point = point + axes[axis] * (surfaceMidpoint - point.dot(axes[axis]));
            return Contact{first.getId(), second.getId(), normal, point, std::max(0.0, depths[axis])};
        }
    }

    std::optional<Contact> detectContact(const Body& first, const Body& second) noexcept {
        const auto* firstSphere = std::get_if<SphereCollider>(&first.getCollider());
        const auto* secondSphere = std::get_if<SphereCollider>(&second.getCollider());
        if (firstSphere && secondSphere) return sphereSphere(first, second, firstSphere->radius, secondSphere->radius);
        if (firstSphere) return sphereBox(first, second, firstSphere->radius);
        if (secondSphere) {
            auto contact = sphereBox(second, first, secondSphere->radius);
            if (contact) {
                std::swap(contact->first, contact->second);
                contact->normal = contact->normal * -1.0;
            }
            return contact;
        }
        return boxBox(first, second);
    }
}
