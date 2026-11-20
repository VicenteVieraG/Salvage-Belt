/**
 * @file
 * @brief Shape-level collision detection for translating spheres and boxes.
 */
#pragma once

#include <optional>
#include "salvage/core/world/Body.hpp"

namespace salvage::core {
    /** @brief One representative contact; angular response is not modeled. */
    struct Contact {
        BodyId first{}; ///< First participating body.
        BodyId second{}; ///< Second participating body.
        Vec3 normal{1.0, 0.0, 0.0}; ///< Unit normal; separation moves first along -normal, second along +normal.
        Vec3 point{}; ///< Midpoint of representative opposing surface points, in world coordinates.
        double penetration{}; ///< Minimum translation distance, zero for touching shapes.
    };

    /**
     * @brief Detect sphere/sphere, sphere/box, or axis-aligned box/box contact.
     * Touching counts as contact. Includes containment and zero-size colliders.
     * Ties choose the x, then y, then z axis; coincident centers use ID order
     * to select a direction (positive for first ID <= second ID).
     * Does not filter static bodies or mutate either input.
     * @param first First shape and its world position.
     * @param second Second shape and its world position.
     * @return Contact geometry, or std::nullopt when separated.
     * @pre Valid finite body geometry; intermediate arithmetic remains in range.
     */
    std::optional<Contact> detectContact(const Body& first, const Body& second) noexcept;
}
