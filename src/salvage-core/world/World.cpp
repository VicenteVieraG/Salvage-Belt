#include "salvage/core/world/World.hpp"
#include "salvage/core/execution/Executor.hpp"
#include "PlacementIndex.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace salvage::core {
namespace {

// SplitMix64 and an explicit 53-bit conversion also fix the random draw order across workers.
std::uint64_t mix(std::uint64_t value) {
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31);
}

struct Stream {
    std::uint64_t state{};
    double draw(Range r) {
        state += 0x9e3779b97f4a7c15ULL;
        const double u = static_cast<double>(mix(state) >> 11) * 0x1p-53;
        return std::lerp(r.min, r.max, u);
    }
};

bool ordered(Range r) {
    return std::isfinite(r.min) && std::isfinite(r.max) && r.min <= r.max;
}

void validate(Material m) {
    if (!std::isfinite(m.restitution) || m.restitution < 0 || m.restitution > 1
        || !std::isfinite(m.friction) || m.friction < 0)
        throw std::invalid_argument("Invalid restitution or friction");
}

void validate(const Body& body) {
    const auto e = getExtents(body.getCollider());
    if (!body.getPosition().finite() || !body.getVelocity().finite()
        || !std::isfinite(body.getInverseMass()) || body.getInverseMass() < 0
        || !e.finite() || e.x < 0 || e.y < 0 || e.z < 0
        || !body.aabb().min.finite() || !body.aabb().max.finite())
        throw std::invalid_argument("Invalid initial body state");
    validate(body.getMaterial());
}

void place(Body& body, Stream& stream, const AABB& region) {
    const auto e = getExtents(body.getCollider());
    const auto lo = region.min + e;
    const auto hi = region.max - e;
    body.setPosition({stream.draw({lo.x, hi.x}), stream.draw({lo.y, hi.y}), stream.draw({lo.z, hi.z})});
}

void generate(std::vector<Body>& bodies, const FieldSettings& field, std::uint64_t seed, Executor& executor) {
    constexpr auto maxBodies = std::numeric_limits<BodyId>::max();
    if (field.asteroids > maxBodies || field.debris > maxBodies - field.asteroids)
        throw std::length_error("Too many bodies for scene identifiers");
    if (!ordered(field.extent) || field.extent.min <= 0 || !ordered(field.velocity)
        || !ordered(field.inverseMass) || field.inverseMass.min < 0 || !field.maxPlacementAttempts)
        throw std::invalid_argument("Invalid field ranges or placement budget");
    validate(field.material);
    const auto& [lo, hi] = field.spawnBounds;
    const auto diameter = 2 * field.extent.max;
    const Vec3 width = hi - lo;
    if (!lo.finite() || !hi.finite() || !width.finite() || width.x < diameter
        || width.y < diameter || width.z < diameter)
        throw std::invalid_argument("Spawn bounds must contain the largest collider");

    bodies.resize(field.asteroids + field.debris);
    std::vector<Stream> streams(bodies.size());
    executor.parallelFor(bodies.size(), [&](std::size_t, std::size_t begin, std::size_t end) {
        for (auto i = begin; i < end; ++i) {
            auto& rng = streams[i];
            rng.state = mix(seed + i);
            Collider shape;
            if (i < field.asteroids) shape = SphereCollider{rng.draw(field.extent)};
            else shape = BoxCollider{{rng.draw(field.extent), rng.draw(field.extent), rng.draw(field.extent)}};
            const Vec3 velocity{rng.draw(field.velocity), rng.draw(field.velocity), rng.draw(field.velocity)};
            bodies[i] = Body{static_cast<BodyId>(i), {}, velocity, rng.draw(field.inverseMass), shape, field.material};
            place(bodies[i], rng, field.spawnBounds);
        }
    });

    detail::PlacementIndex accepted(field.spawnBounds, field.extent.max, bodies.size());
    for (auto& body : bodies) {
        for (std::size_t attempt = 1; accepted.overlapsAny(body); ++attempt) {
            if (attempt == field.maxPlacementAttempts)
                throw std::runtime_error("Field exhausted its placement budget at body " + std::to_string(body.getId()));
            place(body, streams[body.getId()], field.spawnBounds);
        }
        accepted.add(body);
    }
}

}

World::World(const Scene& scene, std::uint64_t seed, Executor& executor) {
    if (const auto* field = std::get_if<FieldSettings>(&scene)) {
        generate(bodies_, *field, seed, executor);
        return;
    }
    const auto& input = std::get<std::vector<Body>>(scene);
    if (input.size() > std::numeric_limits<BodyId>::max()) throw std::length_error("Too many bodies");
    bodies_.reserve(input.size());
    for (const auto& body : input) {
        validate(body);
        bodies_.emplace_back(static_cast<BodyId>(bodies_.size()), body.getPosition(), body.getVelocity(),
                             body.getInverseMass(), body.getCollider(), body.getMaterial());
    }
}

}
