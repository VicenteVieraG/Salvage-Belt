#include "salvage-core/simulation/Simulation.hpp"
#include <config.hpp>
#include <iostream>

int main() {
    salvage::core::Simulation simulation;
    simulation.step();
    const auto metrics = simulation.metrics();
    std::cout << project_name << ' ' << project_version << '\n'
              << "Step " << metrics.stepIndex << ", " << simulation.world().bodies.size() << " bodies, "
              << metrics.detection.uniqueCandidates << " candidates, " << metrics.detection.contacts << " contacts\n";
}
