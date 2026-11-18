# Translational physics step

The core supports translating spheres and axis-aligned boxes, including dynamic
and static bodies. No additional third-party dependency is required.

## Step sequence

`World::step(dt)` validates the duration, then:

1. Integrates each dynamic body with Velocity Verlet and constant acceleration
   `gravity + accumulatedForce * inverseMass`. Position changes refresh AABBs.
2. Clears accumulated external forces.
3. Scans AABB candidates and confirms actual shape contacts with `detectContact`.
4. Resolves velocities with accumulated sequential normal and friction impulses.
5. Repeats mass-weighted position correction, querying candidates again each pass
   and refreshing individual contact geometry after preceding corrections.

Zero duration is a no-op, including preserving pending forces. Invalid durations
throw before changing any state. Bodies with zero inverse mass remain unchanged;
their stored velocity is ignored by contact response. All calculations require
finite inputs and intermediate arithmetic within floating-point range.

The broad-phase strategies remain interface placeholders. The step uses the
existing quadratic AABB scan directly and does not depend on those strategies.
`findPotentialCollisions()` still returns independent body snapshots.
`findContacts()` returns confirmed contact values containing IDs, without mutation.
Responses operate on the world's actual bodies.

## Contacts and materials

`detectContact(first, second)` supports sphere/sphere, sphere/box (either order),
and box/box pairs. It includes touching, contained, coincident, and zero-size
shapes. A contact contains two IDs, a unit normal, a penetration depth, and one
representative point. Separating the first body moves it opposite the normal;
separating the second moves it along the normal. Ties have deterministic normals.
The point represents opposing surfaces; it is not a rotational contact manifold.

Each body has a `Material`:

- `restitution`: bounciness in [0, 1], default 0. Pair mixing uses the maximum.
- `friction`: non-negative Coulomb coefficient, default 0.5. Pair mixing uses
  the geometric mean. One coefficient governs both sticking and sliding.

Normal impulses are non-negative, so separating pairs are not pulled together.
Bounce targets are computed once per solve. Friction impulses are bounded by
the accumulated normal impulse times the mixed coefficient. Position correction
does not change velocities and is weighted by inverse mass.

`SolverSettings` defaults to 16 velocity passes, 8 position passes, a penetration
slop of `1e-5`, a correction fraction of 0.8, and a bounce threshold of 0.5 speed
units. The threshold suppresses tiny bounces at resting contacts. Settings are
validated by `World::setSolverSettings()`; both iteration counts must be positive.

## Example

```cpp
#include "salvageCore.hpp"
using namespace salvage::core;

World world{{0.0, -9.81, 0.0}};
world.createBody(Body{{0.0, -0.5, 0.0}, {}, 0.0,
    BoxCollider{{10.0, 0.5, 10.0}}, Material{0.0, 0.6}});
const auto ball = world.createBody(Body{{0.0, 3.0, 0.0}, {}, 1.0,
    SphereCollider{0.5}, Material{0.7, 0.4}});

world.applyBodyImpulse(ball, {1.0, 0.0, 0.0}); // Immediate velocity change.
world.addBodyForce(ball, {0.0, 2.0, 0.0});     // Constant for the next positive step only.
world.step(1.0 / 120.0);
const auto contacts = world.findContacts();
```

## Remaining scope

- Broad-phase implementations and strategy selection in `World` remain separate
  algorithm work. Narrow phase and response can be reused once candidates use IDs.
- Detection is discrete: a fast object can cross an obstacle entirely within one
  step. Use a small fixed time step appropriate to speed and collider size.
  Continuous detection requires swept candidates and time-of-impact integration.
- Rotation needs orientation, angular velocity, inertia, torque, oriented-box
  geometry, and contact manifolds. Friction here only changes linear motion.
- Position correction can create new contacts. They are corrected geometrically
  during later position passes; their velocity response occurs on the next step.
- Finite iteration counts approximate simultaneous constraints. Tall stacks and
  extreme mass ratios may need more passes; there is no warm starting or sleeping.
- Force accumulation supports constant per-step external forces. Position-dependent
  force generators (such as mutual gravity or springs) need acceleration evaluation
  at both ends of the Verlet update; they are not implemented here.

Tests cover geometry and reversed pair order, momentum and energy for elastic
impacts, inelastic response, static surfaces, separation, friction, containment,
degenerate shapes, resting stacks, force lifetimes, and invalid inputs.
