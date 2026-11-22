#include "salvage-analytics/Dashboard.hpp"

#include <config.hpp>
#include <exception>
#include <iostream>
#include <string_view>

int main(int argc, char* argv[]) {
    const std::string_view option = argc > 1 ? argv[1] : "";
    constexpr auto usage = "Usage: salvage-analytics [--report FILE | FILE | --help | --version]\n"
                           "Open the live dashboard or inspect a saved salvage-headless JSON or CSV report.\n";
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
