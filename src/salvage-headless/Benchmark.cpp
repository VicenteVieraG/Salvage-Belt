#include "salvage-headless/Benchmark.hpp"
#include "salvage-core/execution/Executor.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace salvage::headless {
namespace {
using namespace core;
using Clock = std::chrono::steady_clock;

FieldSettings fieldFor(const Options& options, const std::string& scenario, Counts counts) {
    FieldSettings field;
    field.asteroids = counts.asteroids;
    field.debris = counts.debris;
    if (scenario == "heterogeneous") field.extent = {0.1, 2.5};
    if (scenario == "fast") field.velocity = {-40, 40};
    if (scenario == "slow") field.velocity = {-0.1, 0.1};
    if (options.fixedDensity && counts.asteroids + counts.debris > 0) {
        const double scale = std::cbrt(static_cast<double>(counts.asteroids + counts.debris) / 128.0);
        field.spawnBounds.min = field.spawnBounds.min * scale;
        field.spawnBounds.max = field.spawnBounds.max * scale;
    }
    return field;
}

SimulationConfig configFor(const Case& scene, const Options& options, std::size_t workers,
                           Duration& extraGeneration) {
    SimulationConfig config;
    config.workers = workers;
    config.seed = scene.seed;
    config.scene = scene.field;
    config.fixedStep = options.fixedStep;
    config.response = options.response;
    if (scene.scenario == "clustered" || scene.scenario == "aligned" || scene.scenario == "dense") {
        Executor executor(workers);
        const auto start = Clock::now();
        World source(scene.field, scene.seed, executor);
        const auto initial = source.view().bodies;
        std::vector<Body> bodies(initial.begin(), initial.end());
        for (auto& body : bodies) {
            auto p = body.getPosition();
            if (scene.scenario == "aligned") p = {p.x, 0, 0};
            else if (scene.scenario == "dense") p = p * 0.16;
            else {
                p = p * 0.25;
                p.x += std::copysign(scene.field.spawnBounds.max.x * 0.55, p.x);
            }
            body.setPosition(p);
        }
        config.scene = std::move(bodies);
        extraGeneration = Clock::now() - start;
    }
    return config;
}

// Compare exact same-build states. Hash individual values to avoid padding and object addresses.
std::uint64_t fingerprint(std::span<const Body> bodies) {
    std::uint64_t hash = 14695981039346656037ULL;
    auto mix = [&](std::uint64_t value) { hash = (hash ^ value) * 1099511628211ULL; };
    auto scalar = [&](double v) { mix(std::bit_cast<std::uint64_t>(v)); };
    auto vector = [&](Vec3 v) { scalar(v.x); scalar(v.y); scalar(v.z); };
    for (const auto& b : bodies) {
        mix(b.getId());
        mix(b.getCollider().index());
        vector(getExtents(b.getCollider()));
        vector(b.getPosition());
        vector(b.getVelocity());
        scalar(b.getInverseMass());
        scalar(b.getMaterial().friction);
        scalar(b.getMaterial().restitution);
    }
    return hash;
}

struct Frame {
    std::vector<Body> detectionInput; // Retained only for replay.
    std::vector<CollisionPair> contacts;
    std::uint64_t resolvedState{};
};

struct Reference {
    std::vector<Body> initial;
    std::vector<Frame> frames;
};

std::vector<Contact> exhaustiveContacts(std::span<const Body> bodies) {
    std::vector<Contact> contacts;
    for (std::size_t a = 0; a < bodies.size(); ++a)
        for (auto b = a + 1; b < bodies.size(); ++b)
            if (auto contact = detectContact(bodies[a], bodies[b])) contacts.push_back(*contact);
    return contacts;
}

Reference prepare(const Case& scene, const Options& options) {
    Executor serial;
    Duration extra{};
    auto config = configFor(scene, options, 1, extra);
    World world(config.scene, config.seed, serial);
    Reference ref;
    const auto initial = world.view().bodies;
    ref.initial.assign(initial.begin(), initial.end());
    auto bodies = ref.initial;
    std::vector<Body> next(bodies.size());
    const auto ticks = options.warmup + options.steps;
    ref.frames.reserve(ticks);
    for (std::size_t tick = 0; tick < ticks; ++tick) {
        integrate({bodies}, next, config.fixedStep, config.gravity, {}, serial);
        auto& frame = ref.frames.emplace_back();
        if (options.replay) frame.detectionInput = next;
        if (options.validate || config.response == Response::resolveContacts) {
            const auto contacts = exhaustiveContacts(next);
            if (options.validate) {
                frame.contacts.reserve(contacts.size());
                for (const auto& contact : contacts)
                    frame.contacts.emplace_back(contact.first, contact.second);
            }
            if (config.response == Response::resolveContacts) resolve(next, contacts, config.solver);
        }
        if (options.validate && !options.replay) frame.resolvedState = fingerprint(next);
        bodies.swap(next);
    }
    return ref;
}

void checkPairs(std::size_t tick, const Frame& expected, std::span<const CollisionPair> candidates,
                std::span<const Contact> contacts, std::size_t count) {
    const bool valid = std::ranges::all_of(candidates, [count](auto p) { return p.first < p.second && p.second < count; })
        && std::ranges::is_sorted(candidates) && std::ranges::adjacent_find(candidates) == candidates.end()
        && std::ranges::includes(candidates, expected.contacts)
        && std::ranges::equal(contacts, expected.contacts, {}, [](const Contact& c) {
               return CollisionPair{c.first, c.second};
           });
    if (!valid) throw std::runtime_error("Candidate or contact mismatch at tick " + std::to_string(tick));
}

void simulate(const Options& options, const Case& scene, const Variant& variant, Trial& trial,
              const Reference* validation) {
    Duration extra{};
    auto config = configFor(scene, options, variant.workers, extra);
    config.algorithm = variant.algorithm;
    Simulation sim(std::move(config));
    trial.generation = extra + sim.metrics().generation;
    trial.initialization = sim.metrics().initialization;
    if (validation && fingerprint(sim.world().bodies) != fingerprint(validation->initial))
        throw std::runtime_error("Initial scene differs from the reference");
    for (std::size_t i = 0; i < options.warmup + options.steps; ++i) {
        sim.step();
        if (validation) {
            const auto& frame = validation->frames[i];
            auto snap = sim.snapshot();
            checkPairs(i + 1, frame, snap.candidates, snap.contacts, snap.bodies.size());
            if (fingerprint(snap.bodies) != frame.resolvedState)
                throw std::runtime_error("Body trajectory mismatch at tick " + std::to_string(i + 1));
        } else if (i >= options.warmup) trial.samples.push_back(sim.metrics());
    }
}

void replay(const Options& options, const Variant& variant, Trial& trial, const Reference& ref, bool validate) {
    Executor executor(variant.workers);
    const auto start = Clock::now();
    auto strategy = makeBroadPhase(variant.algorithm);
    strategy->update({ref.initial});
    trial.initialization = Clock::now() - start;
    auto peak = strategy->indexBytes();
    for (std::size_t i = 0; i < ref.frames.size(); ++i) {
        const auto& frame = ref.frames[i];
        const auto detection = detect({frame.detectionInput}, *strategy, executor);
        const auto& metrics = detection.metrics;
        peak = std::max(peak, metrics.indexBytes + metrics.pairBufferBytes);
        if (validate) checkPairs(i + 1, frame, detection.candidates, detection.contacts, ref.initial.size());
        else if (i >= options.warmup) {
            auto& sample = trial.samples.emplace_back();
            sample.stepIndex = i + 1;
            sample.detection = metrics;
            sample.peakCollisionBufferBytes = peak;
        }
    }
}

void run(const Options& options, const Case& scene, const Reference& reference,
         Variant& variant, Trial& trial, bool validate) {
    if (!validate) trial.samples.reserve(options.steps);
    if (options.replay) replay(options, variant, trial, reference, validate);
    else simulate(options, scene, variant, trial, validate ? &reference : nullptr);
}
}

Case benchmark(const Options& options, const std::string& scenario, Counts counts, std::uint64_t seed) {
    Case scene{scenario, fieldFor(options, scenario, counts), seed, {}, {}};
    for (auto a : options.algorithms)
        for (auto workers : options.workers)
            scene.variants.push_back({a, workers, options.validate ? "not-run" : "skipped", {}, {}});

    Reference reference;
    if (options.validate || options.replay) {
        const auto start = Clock::now();
        try {
            reference = prepare(scene, options);
        } catch (const std::exception& e) {
            for (auto& variant : scene.variants) variant.error = e.what();
            return scene;
        }
        scene.preparation = Clock::now() - start;
    }
    if (options.validate) {
        for (auto& variant : scene.variants) {
            try {
                Trial preflight;
                run(options, scene, reference, variant, preflight, true);
                variant.validation = "passed";
            } catch (const std::exception& e) {
                variant.validation = "failed";
                variant.error = e.what();
            }
        }
    }
    // Rotate the cold-start order once per repetition so the same variant does not always run first.
    for (std::size_t rep = 0; rep < options.repetitions; ++rep) {
        for (std::size_t order = 0; order < scene.variants.size(); ++order) {
            auto& variant = scene.variants[(rep % scene.variants.size() + order) % scene.variants.size()];
            if (!variant.error.empty()) continue;
            Trial trial{rep + 1, order + 1, {}, {}, {}};
            try {
                run(options, scene, reference, variant, trial, false);
                variant.trials.push_back(std::move(trial));
            } catch (const std::exception& e) {
                variant.error = e.what();
            }
        }
        if (std::ranges::all_of(scene.variants, [](const auto& v) { return !v.error.empty(); })) break;
    }
    return scene;
}
}
