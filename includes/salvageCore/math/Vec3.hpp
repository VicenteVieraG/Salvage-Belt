#pragma once

#include <cmath>

namespace salvage::core {
    struct Vec3 {
        double x{};
        double y{};
        double z{};
    
        constexpr Vec3 operator+(const Vec3& rhs) const noexcept {
            return { this->x + rhs.x, this->y + rhs.y, this->z + rhs.z };
        }

        constexpr Vec3 operator-(const Vec3& rhs) const noexcept {
            return { this->x - rhs.x, this->y - rhs.y, this->z - rhs.z };
        }

        constexpr Vec3 operator*(const Vec3& rhs) const noexcept {
            return { this->x * rhs.x, this->y * rhs.y, this->z * rhs.z };
        }

        constexpr Vec3 operator*(double scalar) const noexcept {
            return { this->x * scalar, this->y * scalar, this->z * scalar };
        }

        constexpr double dot(const Vec3& rhs) const noexcept {
            return this->x * rhs.x + this->y * rhs.y + this->z * rhs.z;
        }

        constexpr double lengthSquared() const noexcept {
            return this->dot(*this);
        }

        constexpr double lenght() const noexcept {
            return std::sqrt(this->lengthSquared());
        }
    };
};
