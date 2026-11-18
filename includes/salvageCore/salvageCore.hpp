/**
 * @file
 * @brief Public entry point for Salvage Belt physics and world APIs.
 *
 * @namespace salvage::core
 * @brief Vector math, colliders, bodies, and translational simulation.
 */
#pragma once

#include "physics/Physics.hpp"
#include "physics/Contact.hpp"
#include "physics/ContactSolver.hpp"
#include "algorithm/BroadPhaseFactory.hpp"
#include "algorithm/BruteForceBroadPhase.hpp"
#include "algorithm/KDTreeBroadPhase.hpp"
#include "world/World.hpp"
