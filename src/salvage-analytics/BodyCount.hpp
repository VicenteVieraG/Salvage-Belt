#pragma once

#include "salvage-core/world/World.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

namespace salvage::analytics {

inline core::BodyId parseBodyCount(std::string_view text) {
    core::BodyId count{};
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), count);
    if (error != std::errc{} || end != text.data() + text.size())
        throw std::invalid_argument("Enter a whole number of bodies from 0 to " +
                                    std::to_string(std::numeric_limits<core::BodyId>::max()));
    return count;
}

inline core::FieldSettings fieldWithBodyCount(core::BodyId count, core::FieldSettings field = {}) {
    // Keep small fields usable and expand larger fields at the reference population density.
    const auto population = [](std::size_t n) { return static_cast<double>(std::max<std::size_t>(128, n)); };
    const double scale = std::cbrt(population(count) / population(field.asteroids + field.debris));
    if (scale != 1) {
        const auto center = (field.spawnBounds.min + field.spawnBounds.max) * 0.5;
        const auto half = (field.spawnBounds.max - field.spawnBounds.min) * (0.5 * scale);
        field.spawnBounds = {center - half, center + half};
    }
    field.debris = count / 2;
    field.asteroids = count - field.debris;
    return field;
}

}
