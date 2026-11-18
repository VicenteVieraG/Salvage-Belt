# Broad-phase interface prototype

**Current physics:** The shared narrow phase and contact solver are now implemented
in `World::step()` using the existing AABB scan. See [PhysicsStep.md](PhysicsStep.md).
The strategies themselves remain no-op placeholders; the prototype notes below
describe that interface boundary.

This iteration introduces the strategy boundary for review. The old standalone
brute-force module has been removed; its existing pair scan now lives inside
`World::findPotentialCollisions()` until strategy integration. The new strategies
contain no collision algorithm and do not change simulation behavior.

## Structure

- Strategies receive `std::span<const Body>`, a read-only view without body copies.
  IDs, bounds, and other properties are available through the `Body` getters.
- `CollisionPair` is an alias for `std::pair<BodyId, BodyId>` declared in
  `IBroadPhase.hpp`. It is separate from the existing `BodyPair`, which holds
  body copies.
- `IBroadPhase` defines `initialize`, `update`, and `findCandidates`, with a virtual
  destructor for ownership through `std::unique_ptr<IBroadPhase>`.
- `BruteForceBroadPhase` and `KDTreeBroadPhase` are concrete, no-op placeholders.
- `Algorithm` and `makeBroadPhase()` select and create a strategy. The factory is
  the only selection switch; consumers call the virtual interface afterward.

## Lifecycle and ownership

Initialize with a view of the complete body collection. Update with a fresh view
after movement, collider changes, additions, or removals. Reinitialization resets
the strategy for a new collection. Empty views are valid. Strategies may own
internal data, but may not retain pointers or references into the supplied spans.
Reacquire views after storage changes; bodies must remain unchanged during each
strategy call. Passing a view does not create a snapshot or extend body lifetimes.

Queries receive a body view matching the state at the latest initialization or update and
append ID pairs to caller-owned output. The caller clears output when it wants a
fresh result. Pair ordering and orientation are unspecified. The completed
algorithms will include every overlapping AABB pair, may include false positives,
and will emit each unordered pair at most once per query. Static/static filtering
and narrow-phase confirmation remain the caller's responsibility in this contract.

All three operations currently do nothing in both strategies, including leaving
existing output entries untouched. Empty output is a placeholder result.

## Standalone use

```cpp
#include "salvageCore.hpp"

int main() {
    using namespace salvage::core;

    World world;
    world.createBody();
    world.createBody(Body{{1.0, 0.0, 0.0}});

    auto strategy = makeBroadPhase(Algorithm::KDTree);
    strategy->initialize(world.getBodies());

    world.step(0.016);
    strategy->update(world.getBodies());

    std::vector<CollisionPair> candidates;
    strategy->findCandidates(world.getBodies(), candidates); // Still empty: placeholder.
}
```

## Deferred integration

`World` does not yet own an `IBroadPhase` or accept an `Algorithm`. A later
iteration can connect strategy updates and candidate generation to the shared
simulation flow. Algorithm selection will happen when creating a new world,
consistent with restarting simulations for comparisons. Integration, narrow-phase
checks, and collision response stay outside the strategy interface.
