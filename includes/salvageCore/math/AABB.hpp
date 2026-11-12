#pragma once

#include "Vec3.hpp"

namespace salvage::core {
    struct AABB {
        Vec3 min{};
        Vec3 max{};
    };
};
