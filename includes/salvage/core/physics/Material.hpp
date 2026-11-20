/**
 * @file
 * @brief Surface properties for translational collision response.
 */
#pragma once

namespace salvage::core {
    /** @brief Surface response for a body. */
    struct Material {
        double restitution{0.0}; ///< Bounciness in [0, 1]; zero is inelastic, one is elastic.
        double friction{0.5}; ///< Finite non-negative Coulomb coefficient for sticking and sliding.
    };
}
