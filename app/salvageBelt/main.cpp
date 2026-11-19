#include "Launcher.hpp"
#include "salvageBelt/Visualizer.hpp"

int main(int argc, char* argv[]) {
    return salvage::app::launch(argc, argv, "salvageBelt", salvage::app::runVisualizer);
}
