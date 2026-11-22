#include "benchmark/Run.hpp"
#include "salvage-core/execution/Executor.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <stdexcept>

namespace salvage::benchmark {
namespace {
using namespace core;
using Clock = std::chrono::steady_clock;

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

Reference prepare(const RunSettings& options, const Setup& setup, std::stop_token stop) {
    Executor serial;
    Duration extra{};
    auto config = setup(1, extra);
    World world(config.scene, config.seed, serial);
    Reference ref;
    const auto initial = world.view().bodies;
    ref.initial.assign(initial.begin(), initial.end());
    auto bodies = ref.initial;
    std::vector<Body> next(bodies.size());
    const auto ticks = options.warmup + options.steps;
    ref.frames.reserve(ticks);
    for (std::size_t tick = 0; tick < ticks && !stop.stop_requested(); ++tick) {
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

void simulate(const RunSettings& options, const Setup& setup, const Variant& variant, Trial& trial,
              const Reference* validation, std::stop_token stop) {
    Duration extra{};
    auto config = setup(variant.workers, extra);
    config.algorithm = variant.algorithm;
    Simulation sim(std::move(config));
    trial.generation = extra + sim.metrics().generation;
    trial.initialization = sim.metrics().initialization;
    if (validation && fingerprint(sim.world().bodies) != fingerprint(validation->initial))
        throw std::runtime_error("Initial scene differs from the reference");
    for (std::size_t i = 0; i < options.warmup + options.steps && !stop.stop_requested(); ++i) {
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

void replay(const RunSettings& options, const Variant& variant, Trial& trial, const Reference& ref, bool validate,
            std::stop_token stop) {
    Executor executor(variant.workers);
    const auto start = Clock::now();
    auto strategy = makeBroadPhase(variant.algorithm);
    strategy->update({ref.initial});
    trial.initialization = Clock::now() - start;
    auto peak = strategy->indexBytes();
    for (std::size_t i = 0; i < ref.frames.size() && !stop.stop_requested(); ++i) {
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

void runTrial(const RunSettings& options, const Setup& setup, const Reference& reference,
              const Variant& variant, Trial& trial, bool validate, std::stop_token stop) {
    if (!validate) trial.samples.reserve(options.steps);
    if (options.replay) replay(options, variant, trial, reference, validate, stop);
    else simulate(options, setup, variant, trial, validate ? &reference : nullptr, stop);
}
}

std::string_view algorithmName(core::Algorithm algorithm) {
    switch (algorithm) {
        case core::Algorithm::bruteForce: return "brute-force";
        case core::Algorithm::kdTree: return "kd-tree";
        case core::Algorithm::sweepAndPrune: return "sweep-and-prune";
    }
    throw std::invalid_argument("Unknown broad phase");
}

RunResult run(const RunSettings& options, const Setup& setup, std::stop_token stop) {
    RunResult result;
    for (auto a : options.algorithms)
        for (auto workers : options.workers)
            result.variants.push_back({a, workers, options.validate ? "not-run" : "skipped", {}, {}});

    Reference reference;
    if (options.validate || options.replay) {
        const auto start = Clock::now();
        try {
            reference = prepare(options, setup, stop);
        } catch (const std::exception& e) {
            for (auto& variant : result.variants) variant.error = e.what();
            return result;
        }
        result.preparation = Clock::now() - start;
    }
    if (options.validate) {
        for (auto& variant : result.variants) {
            try {
                if (stop.stop_requested()) return result;
                Trial preflight;
                runTrial(options, setup, reference, variant, preflight, true, stop);
                variant.validation = "passed";
            } catch (const std::exception& e) {
                variant.validation = "failed";
                variant.error = e.what();
            }
        }
    }
    // Rotate the cold-start order once per repetition so the same variant does not always run first.
    for (std::size_t rep = 0; rep < options.repetitions; ++rep) {
        for (std::size_t order = 0; order < result.variants.size(); ++order) {
            if (stop.stop_requested()) return result;
            auto& variant = result.variants[(rep % result.variants.size() + order) % result.variants.size()];
            if (!variant.error.empty()) continue;
            Trial trial{rep + 1, order + 1, {}, {}, {}};
            try {
                runTrial(options, setup, reference, variant, trial, false, stop);
                if (stop.stop_requested()) return result;
                variant.trials.push_back(std::move(trial));
            } catch (const std::exception& e) {
                variant.error = e.what();
            }
        }
        if (std::ranges::all_of(result.variants, [](const auto& v) { return !v.error.empty(); })) break;
    }
    return result;
}
}
