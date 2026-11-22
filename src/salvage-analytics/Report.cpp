#include "salvage-analytics/Report.hpp"
#include "benchmark/Run.hpp"

#include <csv.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <format>
#include <fstream>
#include <stdexcept>
#include <type_traits>

namespace salvage::analytics {
namespace {
using Json = nlohmann::json;

void require(bool condition, std::string_view message) {
    if (!condition) throw std::runtime_error(std::string(message));
}

std::string label(const Json& row, const char* key) { return row.at(key).get<std::string>(); }
std::string label(const csv::CSVRow& row, const char* key) { return std::string(row[key].get_sv()); }

template<class T>
T number(const csv::CSVRow& row, const char* key) {
    auto field = row[key].get_sv();
    if (field.empty()) throw std::runtime_error(std::format("Missing number in '{}'", key));
    T value{};
    const auto [end, err] = std::from_chars(field.data(), field.data() + field.size(), value);
    if (err != std::errc{} || end != field.data() + field.size() || value < T{} || !std::isfinite(value))
        throw std::runtime_error(std::format("Invalid number in '{}'", key));
    return value;
}

template<class T>
T number(const Json& row, const char* key) {
    const Json& field = row.at(key);
    if constexpr (std::is_unsigned_v<T>) {
        if (field.is_number_unsigned()) return field.get<T>();
    } else if (field.is_number()) {
        auto value = field.get<T>();
        if (value >= 0 && std::isfinite(value)) return value;
    }
    throw std::runtime_error(std::format("Invalid number in '{}'", key));
}

auto count(const auto& row, const char* key) { return number<std::uint64_t>(row, key); }

ReportProtocol protocol(const auto& row) {
    const auto mode = label(row, "mode");
    require(mode == "simulation" || mode == "replay", "Unknown benchmark mode");
    ReportProtocol p{mode == "replay", label(row, "response"), label(row, "scale"), number<double>(row, "fixed_step_s"),
                     count(row, "warmup"), count(row, "steps"), count(row, "repetitions")};
    require(p.steps && p.repetitions && p.fixedStep > 0, "Invalid benchmark protocol");
    return p;
}

SceneKey sceneKey(const auto& row) {
    return {label(row, "scenario"), count(row, "seed"), count(row, "bodies"), count(row, "asteroids"), count(row, "debris")};
}

ReportVariant variantInfo(const auto& row) {
    ReportVariant v{label(row, "algorithm"), count(row, "workers"), label(row, "status"),
                    label(row, "validation"), label(row, "error"), {}, {}, {}};
    require(v.workers && (v.status == "complete" || v.status == "failed"), "Invalid benchmark variant");
    return v;
}

Sample sample(const auto& row, bool replay) {
    const auto detection = number<double>(row, "detection_s");
    return {count(row, "tick"), replay ? detection : number<double>(row, "total_step_s"), detection,
            count(row, "pairs_examined"), count(row, "unique_candidates"), count(row, "contacts")};
}

const Json::array_t& array(const Json& value, const char* key) {
    return value.at(key).get_ref<const Json::array_t&>();
}

SavedReport readJson(std::istream& in) {
    auto doc = Json::parse(in);
    require(count(doc, "schema_version") == 1, "Unsupported benchmark schema version");
    SavedReport report{{}, protocol(doc.at("protocol")), {}};
    for (const auto& entry : array(doc, "cases")) {
        auto& scene = report.cases.emplace_back(ReportCase{sceneKey(entry), {}});
        for (const auto& item : array(entry, "variants")) {
            auto& v = scene.variants.emplace_back(variantInfo(item));
            if (v.status == "complete") {
                const auto& summary = item.at("summary");
                const auto& t = summary.at(report.protocol.replay ? "detection_s" : "total_step_s");
                v.timing = {number<double>(t, "mean"), number<double>(t, "median"),
                            number<double>(t, "p95"), number<double>(t, "stddev")};
                if (!summary.at("speedup_vs_brute_force").is_null())
                    v.speedupVsBrute = number<double>(summary, "speedup_vs_brute_force");
            }
            for (const auto& trial : array(item, "runs")) {
                auto& run = v.runs.emplace_back(ReportRun{count(trial, "repetition"), {}});
                const auto& samples = array(trial, "samples");
                run.samples.reserve(samples.size());
                for (const auto& tick : samples) run.samples.push_back(sample(tick, report.protocol.replay));
            }
        }
    }
    return report;
}

// Exported rows are grouped by case, variant and repetition. Keep file order and
// also accept interleaved rows without creating additional groups.
template<class T, class Matches, class Create>
T& group(std::vector<T>& groups, Matches matches, Create create) {
    if (!groups.empty() && matches(groups.back())) return groups.back();
    auto it = std::ranges::find_if(groups, matches);
    if (it != groups.end()) return *it;
    return groups.emplace_back(create());
}

SavedReport readCsv(std::istream& in) {
    csv::CSVFormat format;
    format.delimiter(',').header_row(0).variable_columns(csv::VariableColumnPolicy::THROW);
    csv::CSVReader reader(in, format);
    require(std::ranges::contains(reader.get_col_names(), std::string("schema_version")),
            "Expected a salvage-headless JSON or CSV report");
    SavedReport report;
    csv::CSVRow row;
    while (reader.read_row(row)) {
        require(count(row, "schema_version") == 1, "Unsupported benchmark schema version");
        const auto settings = protocol(row);
        if (report.cases.empty()) report.protocol = settings;
        require(settings == report.protocol, "Conflicting protocols in CSV report");
        auto key = sceneKey(row);
        auto& scene = group(report.cases, [&](const auto& c) { return c.scene == key; },
                            [&] { return ReportCase{std::move(key), {}}; });
        auto info = variantInfo(row);
        auto& v = group(scene.variants, [&](const auto& other) {
            return other.workers == info.workers && other.algorithm == info.algorithm;
        }, [&] { return info; });
        require(v.status == info.status && v.validation == info.validation && v.error == info.error,
                "Conflicting variant status in CSV report");
        if (row["repetition"].get_sv().empty()) {
            require(v.status == "failed" && row["tick"].get_sv().empty(), "Missing CSV repetition");
            continue;
        }
        const auto rep = count(row, "repetition");
        auto& run = group(v.runs, [rep](const auto& r) { return r.repetition == rep; },
                          [rep] { return ReportRun{rep, {}}; });
        run.samples.push_back(sample(row, settings.replay));
    }
    return report;
}

void finishCase(ReportCase& scene, const ReportProtocol& p, bool reconstructTiming) {
    require(!scene.variants.empty(), "Benchmark case has no variants");
    for (auto& v : scene.variants) {
        const bool complete = v.status == "complete";
        require(!complete || v.runs.size() == p.repetitions, "Incomplete benchmark repetitions");
        std::ranges::sort(v.runs, {}, &ReportRun::repetition);
        std::vector<double> means;
        for (std::size_t i = 0; i < v.runs.size(); ++i) {
            auto& run = v.runs[i];
            require(run.repetition == i + 1 && run.samples.size() == p.steps, "Incomplete benchmark repetition");
            if (reconstructTiming) std::ranges::sort(run.samples, {}, &Sample::tick);
            double sum{};
            for (std::size_t j = 0; j < run.samples.size(); ++j) {
                const auto& s = run.samples[j];
                require(s.tick > p.warmup && s.tick - p.warmup == j + 1, "Missing or duplicate benchmark tick");
                sum += s.seconds;
            }
            means.push_back(sum / run.samples.size());
        }
        if (reconstructTiming && complete) v.timing = benchmark::summarize(std::move(means));
    }
    if (!reconstructTiming) return;
    const auto baseline = std::ranges::find_if(scene.variants, [](const auto& v) {
        return v.algorithm == "brute-force" && v.workers == 1 && v.validation == "passed" && v.timing;
    });
    if (baseline == scene.variants.end()) return;
    for (auto& v : scene.variants)
        if (v.validation == "passed" && v.timing && v.timing->median > 0)
            v.speedupVsBrute = baseline->timing->median / v.timing->median;
}
}

std::string displayText(std::string text) {
    for (char& ch : text)
        if (static_cast<unsigned char>(ch) < 32 || ch == 127) ch = ' ';
    return text;
}

SavedReport loadReport(const std::filesystem::path& path) {
    require(std::filesystem::is_regular_file(path), "Choose an existing report file");
    std::ifstream in(path, std::ios::binary);
    require(in.good(), "Cannot read the report file");
    // Detect the contents so redirected exports and filenames without suffixes also work.
    char prefix[3]{};
    in.read(prefix, 3);
    in.clear();
    in.seekg(std::string_view(prefix, 3) == "\xef\xbb\xbf" ? 3 : 0);
    in >> std::ws;
    const int first = in.peek();
    require(first != std::char_traits<char>::eof(), "The report file is empty");
    const bool csv = first != '{' && first != '[';
    auto report = csv ? readCsv(in) : readJson(in);
    require(!report.cases.empty(), "The report has no benchmark cases");
    for (auto& c : report.cases) finishCase(c, report.protocol, csv);
    report.filename = displayText(path.filename().string());
    return report;
}

std::optional<SavedReport> generateBenchmark(const core::SimulationConfig& config,
                                             std::span<const core::Algorithm> algorithms, std::stop_token stop) {
    const benchmark::RunSettings settings{.algorithms = algorithms, .workers = std::span(&config.workers, 1),
                                          .validate = false};
    auto result = benchmark::run(settings, [&](std::size_t workers, core::Duration&) {
        auto trialConfig = config;
        trialConfig.workers = workers;
        return trialConfig;
    }, stop);
    if (stop.stop_requested()) return std::nullopt;

    SceneKey key{"Seeded field", config.seed, 0, 0, 0};
    if (const auto* field = std::get_if<core::FieldSettings>(&config.scene)) {
        key.asteroids = field->asteroids;
        key.debris = field->debris;
    } else {
        key.scenario = "Custom scene";
        const auto& bodies = std::get<std::vector<core::Body>>(config.scene);
        key.asteroids = std::ranges::count_if(bodies, [](const auto& body) {
            return std::holds_alternative<core::SphereCollider>(body.getCollider());
        });
        key.debris = bodies.size() - key.asteroids;
    }
    key.bodies = key.asteroids + key.debris;

    SavedReport report{{}, {false, config.response == core::Response::resolveContacts ? "resolve" : "detection-only",
                            {}, config.fixedStep, settings.warmup, settings.steps, settings.repetitions}, {}};
    auto& scene = report.cases.emplace_back(ReportCase{std::move(key), {}});
    for (auto& variant : result.variants) {
        ReportVariant entry{std::string(benchmark::algorithmName(variant.algorithm)), variant.workers,
                            variant.error.empty() ? "complete" : "failed", std::move(variant.validation),
                            std::move(variant.error), {}, {}, {}};
        for (const auto& trial : variant.trials) {
            auto& run = entry.runs.emplace_back(ReportRun{trial.repetition, {}});
            run.samples.reserve(trial.samples.size());
            for (const auto& sample : trial.samples) {
                const auto& d = sample.detection;
                run.samples.push_back({sample.stepIndex, sample.totalStep.count(), d.total.count(), d.pairsExamined,
                                       d.uniqueCandidates, d.contacts});
            }
        }
        scene.variants.push_back(std::move(entry));
    }
    finishCase(scene, report.protocol, true);
    return report;
}

}
