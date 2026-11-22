#include "salvage-analytics/Dashboard.hpp"
#include "BodyCount.hpp"

#include <config.hpp>
#include <exception>
#include <iostream>
#include <string_view>
#include <utility>

int main(int argc, char* argv[]) {
    const std::string_view option = argc > 1 ? argv[1] : "";
    constexpr auto usage = "Usage: salvage-analytics [--bodies N | --report FILE | FILE | --help | --version]\n"
                           "Start the live dashboard with N bodies (default 128), or open a saved JSON or CSV report.\n";
    if (argc == 2 && (option == "--help" || option == "-h")) {
        std::cout << usage;
        return 0;
    }
    if (argc == 2 && option == "--version") {
        std::cout << "salvage-analytics " << project_version << '\n';
        return 0;
    }
    try {
        if (argc == 1) salvage::analytics::runDashboard();
        else if ((argc == 3 && option == "--bodies") || (argc == 2 && option.starts_with("--bodies="))) {
            const auto count = salvage::analytics::parseBodyCount(argc == 3 ? argv[2] : option.substr(9));
            salvage::core::SimulationConfig config;
            config.scene = salvage::analytics::fieldWithBodyCount(count);
            if (count > 128) config.algorithm = salvage::core::Algorithm::kdTree;
            salvage::analytics::runDashboard(std::move(config));
        }
        else if (argc == 3 && option == "--report") salvage::analytics::runReport(argv[2]);
        else if (argc == 2 && !option.starts_with('-')) salvage::analytics::runReport(argv[1]);
        else {
            std::cerr << usage;
            return 2;
        }
    } catch (const std::exception& error) {
        std::cerr << "salvage-analytics: " << error.what() << '\n';
        return 1;
    }
}
