#pragma once

#include "salvage-core/simulation/Simulation.hpp"
#include <ftxui/dom/elements.hpp>

#include <deque>

namespace salvage::analytics {

inline constexpr std::size_t historyLimit = 120;

ftxui::Element metricsView(const core::SimulationMetrics& metrics, const std::deque<core::Duration>& history,
                           bool wide, bool showGraph);
ftxui::Element inspectorView(const core::Simulation& simulation, std::size_t selected, bool showContacts, bool wide);

}
