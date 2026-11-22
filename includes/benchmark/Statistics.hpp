#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace salvage::benchmark {

struct Stats {
    double mean, median, p95, stddev;
};

// One independent observation per repetition. Requires at least one repetition mean.
inline Stats summarize(std::vector<double> values) {
    std::ranges::sort(values);
    auto quantile = [&](double q) {
        const double index = q * static_cast<double>(values.size() - 1);
        const auto low = static_cast<std::size_t>(index);
        return std::lerp(values[low], values[static_cast<std::size_t>(std::ceil(index))], index - static_cast<double>(low));
    };
    const double mean = std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
    double squares{};
    for (double v : values) squares += (v - mean) * (v - mean);
    return {mean, quantile(0.5), quantile(0.95), std::sqrt(squares / static_cast<double>(values.size()))};
}

}
