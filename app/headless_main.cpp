#include "salvage-headless/Benchmark.hpp"

#include <iostream>

int main(int argc, char* argv[]) {
    using namespace salvage::headless;
    try {
        auto parsed = parseOptions(argc, argv);
        if (!parsed) return parsed.error();
        const auto& options = *parsed;
        std::vector<Case> cases;
        for (const auto& scenario : options.scenarios)
            for (auto counts : options.sizes)
                for (auto seed : options.seeds)
                    cases.push_back(benchmark(options, scenario, counts, seed));
        auto& summary = options.json == "-" || options.csv == "-" ? std::cerr : std::cout;
        return report(options, cases, summary) ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "salvage-headless: " << e.what() << '\n';
        return 1;
    }
}
