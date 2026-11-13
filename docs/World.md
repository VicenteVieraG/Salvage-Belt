# World implementation proposal

`salvage::core::World` is the first simulation layer over the existing `Body`,
`Vec3`, and `AABB` types. This implementation owns bodies, advances their linear
motion, and supplies potential collision pairs for a future contact solver.
Include `salvageCore.hpp` (or `world/World.hpp`) and link `salvageCore`.

## Ownership and identity

- `createBody(BodyDefinition)` copies the definition and assigns a nonzero
  `BodyId`. Defaults produce a dynamic unit sphere at rest at the origin.
- IDs increase monotonically and are local to a world. Removing bodies or
  clearing the world never recycles IDs; exhaustion throws `std::overflow_error`.
  Failed creation does not consume an ID.
- `getBody(id)` returns a read-only pointer, or `nullptr` for an unknown ID.
  `getBodies()` provides a read-only span in creation order. Use the world's
  setters to edit bodies so callers cannot replace their identity or bypass
  validation. Reacquire pointers and spans after creation, removal, or clear.
- `removeBody(id)` and body setters return `false` for unknown IDs. Removing
  one body preserves the order and IDs of the others. `clear()` retains gravity.
- Worlds have ordinary value semantics: a copy is an independent snapshot with
  the same local IDs. IDs do not identify bodies across worlds or replacement
  of an entire world by assignment.

Contiguous vector storage keeps stepping simple. Lookup and removal are O(n);
creation is amortized O(1). This favors a small initial simulation over an
additional index or a generational handle system.

## Simulation contract

`step(dt)` uses semi-implicit Euler for each body with positive inverse mass:

```text
velocity = velocity + gravity * dt
position = position + velocity * dt
```

Gravity defaults to zero and can be changed with `setGravity`. It is an
acceleration, so all dynamic masses respond equally. A zero inverse mass marks
a static body: stepping preserves its position, velocity, and bounds even if it
has a nonzero stored velocity. Explicit setters can still reposition it.
Position changes go through `Body::setPosition`, keeping cached AABBs current.

`dt` must be finite and non-negative; zero is a no-op. The caller owns timing
and should supply small, fixed steps. There is no internal clock or accumulator.
Creation and edits reject non-finite vectors, negative or non-finite inverse
masses, and negative or non-finite collider sizes with `std::invalid_argument`.
Invalid edits leave existing state unchanged. Zero-size colliders are valid.
Callers must choose magnitudes and time steps that keep floating-point arithmetic
in range; this initial implementation does not detect arithmetic overflow.

## Potential collisions

`findPotentialCollisions()` computes a fresh O(n²) list from current AABBs,
with each unordered pair appearing once in creation order. Touching faces,
edges, and corners count as overlaps. Static/static pairs are omitted.
The returned list owns its data; it is a snapshot and should be queried again
after changes to the world.

These are broad-phase candidates, not confirmed contacts. For example, two
spheres can have intersecting AABBs without intersecting each other. Bodies
currently pass through each other: narrow-phase shape tests, impulses,
penetration correction, rotation, continuous collision detection, and spatial
acceleration are future work.

## Example

```cpp
#include "salvageCore.hpp"

using namespace salvage::core;

World world;
const BodyId debris = world.createBody({
    .position = {0.0, 0.0, 0.0},
    .velocity = {2.0, 0.0, 0.0},
    .collider = SphereCollider{0.5}
});
const BodyId station = world.createBody({
    .position = {3.0, 0.0, 0.0},
    .inverseMass = 0.0,
    .collider = BoxCollider{{1.0, 1.0, 1.0}}
});

world.step(0.5); // Debris is now at x = 1; the station remains fixed.
const auto candidates = world.findPotentialCollisions();
world.setBodyVelocity(debris, {1.0, 0.0, 0.0});
world.removeBody(station);
```

## Validation

The Catch2 suite covers ownership, stable IDs, removal, clearing, read-only
access, edits and cached bounds, input validation, gravity, static bodies,
time-step validation, and candidate generation. The existing body tests are
also included. Build and run with:

```sh
cmake -S . -B build-world -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DBUILD_TESTS=ON -DGLFW_BUILD_WAYLAND=OFF -DBUILD_EXAMPLES=OFF \
  -DENABLE_WARNINGS=ON -DENABLE_WARNINGS_AS_ERRORS=ON
cmake --build build-world --target tests
ctest --test-dir build-world --output-on-failure
```

`BUILD_TESTS_WITH_MAIN=OFF` uses Catch2's supplied main; setting it to `ON`
uses the project's custom `Catch::Session` entry point. Add
`-DENABLE_SANITIZERS=ON` to instrument the core and test executable with the
configured sanitizer preset.
