#include "Launcher.hpp"
#include "salvageAnalytic/Tui.hpp"

int main(int argc, char* argv[]) {
    return salvage::app::launch(argc, argv, "salvageAnalytic", salvage::app::runTui);
}
