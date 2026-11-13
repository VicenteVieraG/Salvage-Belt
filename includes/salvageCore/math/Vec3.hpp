/**
 * @file
 * @brief Vector arithmetic used by the physics core.
 */
#pragma once

#include <cmath>

namespace salvage::core {
    /**
     * @brief Three-dimensional vector with zero-initialized components.
     */
    struct Vec3 {
        double x{}; ///< Component along the x axis.
        double y{}; ///< Component along the y axis.
        double z{}; ///< Component along the z axis.
    
        /**
         * @brief Add vectors component by component.
         * @param rhs Vector to add.
         * @return The component-wise sum.
         */
        constexpr Vec3 operator+(const Vec3& rhs) const noexcept {
            return { this->x + rhs.x, this->y + rhs.y, this->z + rhs.z };
        }

        /**
         * @brief Subtract vectors component by component.
         * @param rhs Vector to subtract.
         * @return The component-wise difference.
         */
        constexpr Vec3 operator-(const Vec3& rhs) const noexcept {
            return { this->x - rhs.x, this->y - rhs.y, this->z - rhs.z };
        }

        /**
         * @brief Multiply vectors component by component.
         * @param rhs Vector of component multipliers.
         * @return The component-wise product; this is not a dot or cross product.
         */
        constexpr Vec3 operator*(const Vec3& rhs) const noexcept {
            return { this->x * rhs.x, this->y * rhs.y, this->z * rhs.z };
        }

        /**
         * @brief Scale every component.
         * @param scalar Scale factor.
         * @return The scaled vector.
         */
        constexpr Vec3 operator*(double scalar) const noexcept {
            return { this->x * scalar, this->y * scalar, this->z * scalar };
        }

        /**
         * @brief Compute the scalar dot product.
         * @param rhs Other vector.
         * @return The sum of corresponding component products.
         */
        constexpr double dot(const Vec3& rhs) const noexcept {
            return this->x * rhs.x + this->y * rhs.y + this->z * rhs.z;
        }

        /**
         * @brief Compute the squared Euclidean magnitude.
         * @return The dot product of this vector with itself.
         */
        constexpr double lengthSquared() const noexcept {
            return this->dot(*this);
        }

        /**
         * @brief Compute the Euclidean magnitude.
         * @return The square root of lengthSquared().
         */
        constexpr double lenght() const noexcept {
            return std::sqrt(this->lengthSquared());
        }
    };
};
