#pragma once

#include "salvage-core/simulation/Simulation.hpp"
#include <filesystem>

namespace salvage::analytics {

/// Open a paused live dashboard for the supplied scene. Return when the user quits.
/// Requires an interactive input and output terminal. Throws on setup failure.
void runDashboard(core::SimulationConfig config = {});

/// Open a version 1 salvage-headless JSON or CSV export in the terminal dashboard.
/// Throws if the report cannot be read or the terminal is unavailable.
void runReport(const std::filesystem::path& path);

}
