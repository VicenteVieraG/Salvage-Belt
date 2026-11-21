#include "salvage-core/simulation/Simulation.hpp"
#include "salvage-core/execution/Executor.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>
#if SALVAGE_ENABLE_THREADS
#include <thread>
#endif

using namespace salvage::core;
using Catch::Approx;

namespace {
constexpr std::size_t workers = SALVAGE_ENABLE_THREADS ? 4 : 1;

void sameState(const SimulationSnapshot& a, const SimulationSnapshot& b) {
    REQUIRE(a.stepIndex == b.stepIndex);
    REQUIRE(a.simulationTime == b.simulationTime);
    REQUIRE(a.candidates == b.candidates);
    REQUIRE(a.contacts.size() == b.contacts.size());
    for (std::size_t i = 0; i < a.contacts.size(); ++i) {
        const auto& p = a.contacts[i];
        const auto& q = b.contacts[i];
        REQUIRE(p.first == q.first);
        REQUIRE(p.second == q.second);
        REQUIRE(p.normal == q.normal);
        REQUIRE(p.point == q.point);
        REQUIRE(p.penetration == q.penetration);
    }
    REQUIRE(a.bodies.size() == b.bodies.size());
    for (std::size_t i = 0; i < a.bodies.size(); ++i) {
        const auto& p = a.bodies[i];
        const auto& q = b.bodies[i];
        REQUIRE(p.getId() == q.getId());
        REQUIRE(p.getPosition() == q.getPosition());
        REQUIRE(p.getVelocity() == q.getVelocity());
        REQUIRE(p.getCollider().index() == q.getCollider().index());
        REQUIRE(p.aabb().min == q.aabb().min);
        REQUIRE(p.aabb().max == q.aabb().max);
        REQUIRE(p.getInverseMass() == q.getInverseMass());
        REQUIRE(p.getMaterial().friction == q.getMaterial().friction);
        REQUIRE(p.getMaterial().restitution == q.getMaterial().restitution);
    }
}

SimulationConfig collisionCourse() {
    const Material elastic{1, 0};
    SimulationConfig config;
    config.fixedStep = 0.2;
    config.scene = std::vector<Body>{
        Body{{-1.1, 0, 0}, {1, 0, 0}, 1, SphereCollider{}, elastic},
        Body{{1.1, 0, 0}, {-1, 0, 0}, 0.5, SphereCollider{}, elastic}
    };
    return config;
}
}

TEST_CASE("Seeded generation is deterministic and returns disjoint, contained bodies") {
    FieldSettings field;
    field.asteroids = 60;
    field.debris = 40;
    field.spawnBounds = {{-8, -8, -8}, {8, 8, 8}};
    SimulationConfig config;
    config.scene = field;
    Simulation simulation(config);
    const auto initial = simulation.snapshot();
    REQUIRE(initial.bodies.size() == 100);
    for (std::size_t i = 0; i < initial.bodies.size(); ++i) {
        const auto& b = initial.bodies[i];
        REQUIRE(b.getId() == i);
        REQUIRE(std::holds_alternative<SphereCollider>(b.getCollider()) == (i < field.asteroids));
        const auto& box = b.aabb();
        for (const auto axis : {&Vec3::x, &Vec3::y, &Vec3::z}) {
            REQUIRE(box.min.*axis >= field.spawnBounds.min.*axis);
            REQUIRE(box.max.*axis <= field.spawnBounds.max.*axis);
        }
        for (std::size_t j = 0; j < i; ++j) REQUIRE_FALSE(overlaps(box, initial.bodies[j].aabb()));
    }
    simulation.step();
    simulation.restartWith(Algorithm::kdTree, workers);
    sameState(initial, simulation.snapshot());
    REQUIRE(simulation.metrics().stepIndex == 0);
    REQUIRE(simulation.metrics().detection.contacts == 0);
    REQUIRE(simulation.metrics().totalStep == Duration::zero());
    simulation.step();
    simulation.reset();
    sameState(initial, simulation.snapshot());
    ++config.seed;
    simulation.reset(config);
    REQUIRE(simulation.world().bodies[0].getPosition() != initial.bodies[0].getPosition());
}

TEST_CASE("Fixed steps reproduce trajectories and contact order across algorithms and CPU counts") {
    FieldSettings field;
    field.asteroids = 24;
    field.debris = 32;
    field.spawnBounds = {{-6, -6, -6}, {6, 6, 6}};
    field.extent = {0.4, 0.85};
    SimulationConfig config;
    config.scene = field;
    config.fixedStep = 0.04;
    Simulation reference(config);
    std::vector<SimulationSnapshot> expected;
    std::size_t contacts{};
    for (int i = 0; i < 60; ++i) {
        reference.step();
        expected.push_back(reference.snapshot());
        contacts += expected.back().contacts.size();
    }
    REQUIRE(contacts > 0);
    for (auto algorithm : {Algorithm::bruteForce, Algorithm::kdTree, Algorithm::sweepAndPrune}) {
        reference.restartWith(algorithm, workers);
        for (const auto& snapshot : expected) {
            reference.step();
            sameState(reference.snapshot(), snapshot);
        }
    }
}

TEST_CASE("One elastic tick conserves momentum and energy and publishes independently owned results") {
    Simulation simulation(collisionCourse());
    const auto initial = simulation.snapshot();
    simulation.step();
    const auto frame = simulation.snapshot();
    REQUIRE(frame.stepIndex == 1);
    REQUIRE(frame.simulationTime == Approx(0.2));
    REQUIRE(frame.contacts.size() == 1);
    const auto a = frame.bodies[0].getVelocity().x, b = frame.bodies[1].getVelocity().x;
    REQUIRE(a == Approx(-5.0 / 3.0));
    REQUIRE(b == Approx(1.0 / 3.0));
    REQUIRE(a + 2 * b == Approx(-1.0));
    REQUIRE(0.5 * a * a + b * b == Approx(1.5));
    REQUIRE(initial.bodies[0].getVelocity().x == 1);
    const auto m = simulation.metrics();
    REQUIRE(m.detection.pairsExamined == 1);
    REQUIRE(m.detection.contacts == 1);
    REQUIRE(m.totalStep >= m.integration + m.detection.total + m.resolution);
    simulation.reset();
    REQUIRE(frame.bodies[0].getVelocity().x == a);
    sameState(simulation.snapshot(), initial);
}

TEST_CASE("Detection-only ticks integrate and report contacts without applying impulses or corrections") {
    auto config = collisionCourse();
    config.response = Response::detectionOnly;
    Simulation simulation(config);
    simulation.step();
    auto frame = simulation.snapshot();
    REQUIRE(frame.contacts.size() == 1);
    REQUIRE(frame.bodies[0].getPosition().x == Approx(-0.9));
    REQUIRE(frame.bodies[0].getVelocity().x == 1);
    REQUIRE(simulation.metrics().resolution == Duration::zero());
}

TEST_CASE("Static bodies stay fixed and external forces and impulses are consumed for one tick") {
    SimulationConfig config;
    config.fixedStep = 0.5;
    config.gravity = {0, -2, 0};
    config.scene = std::vector<Body>{Body{}, Body{{10, 0, 0}, {5, 0, 0}, 0}};
    Simulation simulation(config);
    const std::array inputs{BodyInput{0, {4, 0, 0}, {1, 0, 0}}, BodyInput{1, {}, {9, 0, 0}}};
    simulation.step(inputs);
    REQUIRE(simulation.world().bodies[0].getPosition() == Vec3{1, -0.25, 0});
    REQUIRE(simulation.world().bodies[0].getVelocity() == Vec3{3, -1, 0});
    simulation.step();
    REQUIRE(simulation.world().bodies[0].getPosition() == Vec3{2.5, -1, 0});
    REQUIRE(simulation.world().bodies[1].getPosition() == Vec3{10, 0, 0});
}

TEST_CASE("Invalid resets, failed placement and rejected tick inputs preserve the published run") {
    auto config = collisionCourse();
    Simulation simulation(config);
    simulation.step();
    const auto saved = simulation.snapshot();
    config.fixedStep = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_THROWS_AS(simulation.reset(config), std::invalid_argument);
    REQUIRE_THROWS_AS(simulation.restartWith(Algorithm::kdTree, 0), std::invalid_argument);
    FieldSettings crowded;
    crowded.spawnBounds = {{-1, -1, -1}, {1, 1, 1}};
    crowded.extent = {1, 1};
    crowded.asteroids = 2;
    crowded.debris = 0;
    crowded.maxPlacementAttempts = 2;
    config = collisionCourse();
    config.scene = crowded;
    REQUIRE_THROWS_AS(simulation.reset(config), std::runtime_error);
    config.scene = std::vector<Body>{Body{{}, {}, -1}};
    REQUIRE_THROWS_AS(simulation.reset(config), std::invalid_argument);
    REQUIRE_THROWS_AS(simulation.step(std::array{BodyInput{9, {}, {}}}), std::invalid_argument);
    sameState(simulation.snapshot(), saved);
}

TEST_CASE("The executor reuses workers and remains usable after a failed range") {
    Executor executor(workers);
    std::vector<int> hits(127);
    const auto visit = [&](auto, auto begin, auto end) {
        for (auto i = begin; i < end; ++i) ++hits[i];
    };
    executor.parallelFor(hits.size(), visit);
    REQUIRE_THROWS_AS(executor.parallelFor(7, [](auto slot, auto, auto) {
        if (slot == workers - 1) throw std::runtime_error("range error");
    }), std::runtime_error);
    executor.parallelFor(hits.size(), visit);
    REQUIRE(std::ranges::all_of(hits, [](int n) { return n == 2; }));
#if SALVAGE_ENABLE_THREADS
    std::vector<std::thread::id> first(workers), second(workers);
    for (auto* ids : {&first, &second})
        executor.parallelFor(workers, [&](auto slot, auto, auto) { (*ids)[slot] = std::this_thread::get_id(); });
    REQUIRE(first == second);
    REQUIRE(first[0] != first[1]);
#else
    REQUIRE_THROWS_AS(Executor(2), std::invalid_argument);
#endif
}

TEST_CASE("Separating overlaps receive positional correction without an attractive impulse") {
    auto config = collisionCourse();
    config.scene = std::vector<Body>{Body{{-0.5, 0, 0}, {-1, 0, 0}},
                                     Body{{0.5, 0, 0}, {1, 0, 0}}};
    Simulation simulation(config);
    simulation.step();
    const auto frame = simulation.snapshot();
    REQUIRE(frame.contacts.size() == 1);
    REQUIRE(frame.bodies[0].getVelocity().x == -1);
    REQUIRE(frame.bodies[1].getVelocity().x == 1);
    REQUIRE(frame.bodies[1].getPosition().x - frame.bodies[0].getPosition().x == Approx(2).margin(2e-5));
}

TEST_CASE("Discrete steps may miss a fast crossing between sampled positions") {
    SimulationConfig config;
    config.fixedStep = 1;
    config.scene = std::vector<Body>{Body{{-4, 0, 0}, {8, 0, 0}}, Body{{}, {}, 0}};
    Simulation simulation(config);
    simulation.step();
    REQUIRE(simulation.metrics().detection.contacts == 0);
    REQUIRE(simulation.world().bodies[0].getPosition().x == 4);
}
