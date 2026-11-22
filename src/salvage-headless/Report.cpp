#include "salvage-headless/Benchmark.hpp"
#include "salvage-headless/BuildInfo.hpp"
#include "../benchmark/Statistics.hpp"

#include <csv.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <thread>
#ifdef __linux__
#include <sys/utsname.h>
#include <unistd.h>
#endif

namespace salvage::headless {
namespace {
using Json = nlohmann::ordered_json;

Json environment() {
    Json host{{"os", build::system}, {"architecture", build::architecture}, {"cpu", nullptr},
              {"ram_bytes", nullptr}, {"hardware_threads", std::thread::hardware_concurrency()}};
#ifdef __linux__
    struct utsname info{};
    if (uname(&info) == 0) {
        host["os"] = std::string(info.sysname) + " " + info.release;
        host["architecture"] = info.machine;
    }
    std::ifstream cpu("/proc/cpuinfo");
    for (std::string line; std::getline(cpu, line);)
        if (line.starts_with("model name") || line.starts_with("Hardware")) {
            auto pos = line.find(':');
            if (pos != std::string::npos) {
                const auto value = line.find_first_not_of(" \t", pos + 1);
                if (value != std::string::npos) host["cpu"] = line.substr(value);
            }
            break;
        }
    const long pages = sysconf(_SC_PHYS_PAGES), bytes = sysconf(_SC_PAGESIZE);
    if (pages > 0 && bytes > 0) host["ram_bytes"] = static_cast<std::uint64_t>(pages) * bytes;
#endif
    return {{"project_version", build::version}, {"revision_at_configure", build::revision},
            {"compiler", build::compiler}, {"configuration", build::configuration}, {"cxx_standard", 26},
            {"cxx_flags", build::flags}, {"ipo", build::ipo}, {"threads_enabled", build::threads}, {"host", host}};
}

Json ratio(double numerator, double denominator) {
    return denominator > 0 ? Json(numerator / denominator) : Json(nullptr);
}

Json sampleRecord(const Options& o, const core::SimulationMetrics& m, std::size_t bodies) {
    const auto& d = m.detection;
    const double possiblePairs = bodies ? 0.5 * static_cast<double>(bodies) * static_cast<double>(bodies - 1) : 0;
    const auto candidates = static_cast<double>(d.uniqueCandidates);
    return {{"tick", m.stepIndex}, {"maintenance_s", d.maintenance.count()}, {"search_s", d.search.count()},
            {"normalization_s", d.normalization.count()}, {"narrow_phase_s", d.narrowPhase.count()},
            {"detection_s", d.total.count()}, {"integration_s", o.replay ? Json(nullptr) : Json(m.integration.count())},
            {"resolution_s", o.replay ? Json(nullptr) : Json(m.resolution.count())},
            {"total_step_s", o.replay ? Json(nullptr) : Json(m.totalStep.count())}, {"pairs_examined", d.pairsExamined},
            {"raw_candidates", d.rawCandidates}, {"unique_candidates", d.uniqueCandidates}, {"contacts", d.contacts},
            {"candidate_reduction", ratio(possiblePairs - candidates, possiblePairs)},
            {"false_positive_fraction", ratio(static_cast<double>(d.uniqueCandidates - d.contacts), candidates)},
            {"index_bytes", d.indexBytes}, {"pair_buffer_bytes", d.pairBufferBytes},
            {"peak_collision_buffer_bytes", m.peakCollisionBufferBytes}};
}

Json sceneRecord(const Case& scene) {
    const auto& f = scene.field;
    const auto width = f.spawnBounds.max - f.spawnBounds.min;
    double volume = width.x * width.y * width.z;
    return {{"scenario", scene.scenario}, {"seed", scene.seed}, {"asteroids", f.asteroids}, {"debris", f.debris},
            {"bodies", f.asteroids + f.debris}, {"generator", "splitmix64-per-body-v1"},
            {"spawn_min", {f.spawnBounds.min.x, f.spawnBounds.min.y, f.spawnBounds.min.z}},
            {"spawn_max", {f.spawnBounds.max.x, f.spawnBounds.max.y, f.spawnBounds.max.z}},
            {"spawn_volume", volume}, {"nominal_density", static_cast<double>(f.asteroids + f.debris) / volume},
            {"extent_range", {f.extent.min, f.extent.max}}, {"velocity_range", {f.velocity.min, f.velocity.max}},
            {"inverse_mass_range", {f.inverseMass.min, f.inverseMass.max}}, {"restitution", f.material.restitution},
            {"friction", f.material.friction}, {"placement_attempts", f.maxPlacementAttempts},
            {"reference_preparation_s", scene.preparation.count()}};
}

Json protocol(const Options& o) {
    const core::SimulationConfig config;
    const auto& s = config.solver;
    return {{"mode", o.replay ? "replay" : "simulation"},
            {"response", o.response == core::Response::resolveContacts ? "resolve" : "detection-only"},
            {"fixed_step_s", o.fixedStep}, {"warmup", o.warmup}, {"steps", o.steps}, {"repetitions", o.repetitions},
            {"first_measured_tick", o.warmup + 1}, {"scale", o.fixedDensity ? "density" : "volume"},
            {"validation_requested", o.validate}, {"speedup_scope", o.replay ? "detection_s" : "total_step_s"},
            {"gravity", {config.gravity.x, config.gravity.y, config.gravity.z}},
            {"solver", {{"velocity_iterations", s.velocityIterations}, {"position_iterations", s.positionIterations},
                {"penetration_slop", s.penetrationSlop}, {"position_correction", s.positionCorrection},
                {"restitution_threshold", s.restitutionThreshold}}}};
}

double meanTime(const Trial& trial, bool detection) {
    double sum{};
    for (const auto& sample : trial.samples)
        sum += (detection ? sample.detection.total : sample.totalStep).count();
    return sum / static_cast<double>(trial.samples.size());
}

using benchmark::Stats;

Stats statistics(const Variant& variant, bool detection) {
    std::vector<double> values;
    values.reserve(variant.trials.size());
    for (const auto& trial : variant.trials) values.push_back(meanTime(trial, detection));
    return benchmark::summarize(std::move(values));
}

Json statsRecord(const Stats& s) {
    return {{"mean", s.mean}, {"median", s.median}, {"p95", s.p95}, {"stddev", s.stddev}};
}

Json speedup(const Case& scene, core::Algorithm baseline, double time, bool detection) {
    for (const auto& v : scene.variants)
        if (v.algorithm == baseline && v.workers == 1 && v.validation == "passed" && v.error.empty())
            return ratio(statistics(v, detection).median, time);
    return nullptr;
}

Json summaryRecord(const Options& o, const Case& scene, const Variant& variant) {
    if (!variant.error.empty()) return nullptr;
    const auto detection = statistics(variant, true);
    const auto scope = o.replay ? detection : statistics(variant, false);
    Json sequential, bruteForce;
    if (variant.validation == "passed") {
        sequential = speedup(scene, variant.algorithm, scope.median, o.replay);
        bruteForce = speedup(scene, core::Algorithm::bruteForce, scope.median, o.replay);
    }
    return {{"detection_s", statsRecord(detection)}, {"total_step_s", o.replay ? Json(nullptr) : statsRecord(scope)},
            {"speedup_vs_brute_force", bruteForce}, {"speedup_vs_one_worker", sequential},
            {"parallel_efficiency", sequential.is_null() ? Json(nullptr) : ratio(sequential.get<double>(), static_cast<double>(variant.workers))}};
}

// Emit nested arrays incrementally. Peak serialization storage is independent of the number of ticks.
void writeRecord(std::ostream& out, const Json& fields, std::string_view arrayName,
                 const auto& children, auto writeChild) {
    out << '{';
    for (const auto& [key, value] : fields.items()) out << Json(key) << ':' << value << ',';
    out << Json(arrayName) << ":[";
    bool first = true;
    for (const auto& child : children) {
        if (!first) out << ',';
        first = false;
        writeChild(child);
    }
    out << "]}";
}

void writeJson(std::ostream& out, const Options& o, const std::vector<Case>& cases) {
    const Json header{{"schema_version", 1}, {"environment", environment()}, {"protocol", protocol(o)}};
    writeRecord(out, header, "cases", cases, [&](const Case& scene) {
        writeRecord(out, sceneRecord(scene), "variants", scene.variants, [&](const Variant& v) {
            Json variant{{"algorithm", name(v.algorithm)}, {"workers", v.workers}, {"validation", v.validation},
                         {"status", v.error.empty() ? "complete" : "failed"}, {"error", v.error},
                         {"summary", summaryRecord(o, scene, v)}};
            writeRecord(out, variant, "runs", v.trials, [&](const Trial& trial) {
                Json run{{"repetition", trial.repetition}, {"order", trial.order},
                         {"generation_s", o.replay ? Json(nullptr) : Json(trial.generation.count())},
                         {"initialization_s", trial.initialization.count()}};
                writeRecord(out, run, "samples", trial.samples, [&](const core::SimulationMetrics& m) {
                    out << sampleRecord(o, m, scene.field.asteroids + scene.field.debris);
                });
            });
        });
    });
    out << '\n';
}

std::vector<std::string> csvFields(const Json& row) {
    std::vector<std::string> values;
    values.reserve(row.size());
    for (const auto& v : row) {
        if (v.is_null()) values.emplace_back();
        else if (v.is_string()) values.push_back(v.get<std::string>());
        else values.push_back(v.dump()); // Preserve full floating-point precision.
    }
    return values;
}

void writeCsv(std::ostream& out, const Options& o, const std::vector<Case>& cases) {
    auto writer = csv::make_csv_writer(out);
    writer.set_auto_flush(false);
    bool headerWritten = false;
    for (const auto& scene : cases) {
        const auto n = scene.field.asteroids + scene.field.debris;
        for (const auto& v : scene.variants) {
            Json row{{"schema_version", 1}, {"mode", o.replay ? "replay" : "simulation"}, {"scenario", scene.scenario},
                     {"seed", scene.seed}, {"bodies", n}, {"asteroids", scene.field.asteroids}, {"debris", scene.field.debris},
                     {"algorithm", name(v.algorithm)}, {"workers", v.workers}, {"fixed_step_s", o.fixedStep},
                     {"warmup", o.warmup}, {"steps", o.steps}, {"repetitions", o.repetitions},
                     {"response", o.response == core::Response::resolveContacts ? "resolve" : "detection-only"},
                     {"scale", o.fixedDensity ? "density" : "volume"}, {"validation", v.validation},
                     {"status", v.error.empty() ? "complete" : "failed"}, {"error", v.error}, {"repetition", nullptr},
                     {"order", nullptr}, {"generation_s", nullptr}, {"initialization_s", nullptr}};
            auto emptySample = sampleRecord(o, {}, n);
            for (const auto& field : emptySample.items()) row[field.key()] = nullptr;
            if (!headerWritten) {
                std::vector<std::string> header;
                header.reserve(row.size());
                for (const auto& field : row.items()) header.push_back(field.key());
                writer << header;
                headerWritten = true;
            }
            if (v.trials.empty()) writer << csvFields(row);
            for (const auto& trial : v.trials) {
                row["repetition"] = trial.repetition;
                row["order"] = trial.order;
                if (!o.replay) row["generation_s"] = trial.generation.count();
                row["initialization_s"] = trial.initialization.count();
                for (const auto& sample : trial.samples) {
                    row.update(sampleRecord(o, sample, n));
                    writer << csvFields(row);
                }
            }
        }
    }
    writer.flush();
}

template<class Write>
void writeOutput(const std::filesystem::path& path, Write write) {
    if (path.empty()) return;
    if (path == "-") {
        write(std::cout);
        if (!std::cout.flush()) throw std::runtime_error("Could not write to standard output");
        return;
    }
    namespace fs = std::filesystem;
    const auto dest = fs::weakly_canonical(fs::absolute(path));
    if (fs::exists(dest) && !fs::is_regular_file(dest))
        throw std::runtime_error("Expected a regular output file: " + dest.string());
    auto temporary = dest;
    temporary += ".tmp." + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    std::ofstream out(temporary, std::ios::out | std::ios::noreplace);
    if (!out) throw std::runtime_error("Cannot create output for " + dest.string());
    try {
        write(out);
        out.close();
        if (!out) throw std::runtime_error("Failed to write " + dest.string());
        fs::rename(temporary, dest);
    } catch (...) {
        out.close();
        std::error_code ignored;
        fs::remove(temporary, ignored);
        throw;
    }
}
}

bool report(const Options& options, const std::vector<Case>& cases, std::ostream& out) {
    bool success = true;
    out << (options.replay ? "Detection replay" : "Full simulation") << ", " << options.steps
        << " measured ticks after " << options.warmup << " warmup, " << options.repetitions << " repetitions\n";
    for (const auto& scene : cases) {
        out << scene.scenario << ", seed " << scene.seed << ", " << scene.field.asteroids + scene.field.debris
            << " bodies\n";
        for (const auto& v : scene.variants) {
            out << "  " << name(v.algorithm) << ", " << v.workers << " worker(s), validation " << v.validation << ", ";
            if (!v.error.empty()) {
                out << "failed: " << v.error << '\n';
                success = false;
                continue;
            }
            const auto stats = summaryRecord(options, scene, v);
            const auto timing = stats.at(options.replay ? "detection_s" : "total_step_s");
            out << std::setprecision(5) << 1e6 * timing.at("median").get<double>() << " us/tick median";
            const auto& speed = stats.at("speedup_vs_brute_force");
            if (!speed.is_null()) out << ", " << speed.get<double>() << "x vs serial brute force";
            out << '\n';
        }
    }
    writeOutput(options.json, [&](auto& stream) { writeJson(stream, options, cases); });
    writeOutput(options.csv, [&](auto& stream) { writeCsv(stream, options, cases); });
    return success && static_cast<bool>(out);
}
}
