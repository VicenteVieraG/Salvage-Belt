#include "Launcher.hpp"

#include <exception>
#include <iostream>
#include <config.hpp>

namespace salvage::app {
    int launch(int argc, char* argv[], std::string_view name, int (*run)()) {
        if (argc <= 1) {
            try {
                return run();
            } catch (const std::exception& error) {
                std::cerr << name << ": " << error.what() << '\n';
                return 1;
            }
        }

        const std::string_view option = argv[1];
        if (argc == 2 && option == "--version") {
            std::cout << name << ' ' << project_version << '\n' << "C++ Standard: " << __cplusplus << '\n';
            return 0;
        }

        const bool help = argc == 2 && (option == "--help" || option == "-h");
        auto& output = help ? std::cout : std::cerr;
        output << "Usage: " << name << " [--help | --version]\n";
        return help ? 0 : 2;
    }
}
