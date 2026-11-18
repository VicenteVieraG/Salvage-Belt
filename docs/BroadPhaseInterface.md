# Broad-phase strategies

`BruteForceBroadPhase` compares each unordered pair of bodies once and appends
the IDs of pairs whose AABBs overlap. Touching bounds and static/static pairs
are included. The scan takes O(n²) time and O(1) space beyond the output.

## Structure

- Strategies receive `std::span<const Body>`, a read-only view without body copies.
  IDs, bounds, and other properties are available through the `Body` getters.
- `CollisionPair` is an alias for `std::pair<BodyId, BodyId>` declared in
  `IBroadPhase.hpp`. It is separate from the existing `BodyPair`, which holds
  body copies.
- `IBroadPhase` defines `initialize`, `update`, and `findCandidates`, with a virtual
  destructor for ownership through `std::unique_ptr<IBroadPhase>`.
- `KDTreeBroadPhase` is a placeholder and appends no candidates.
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
fresh result. Pair ordering and orientation are unspecified. Implementations
must include every overlapping AABB pair, may include false positives,
and must emit each unordered pair at most once per query. Static/static filtering
and narrow-phase confirmation remain the caller's responsibility in this contract.

The brute-force strategy reads the current bounds from the query's body view.
Its initialization and update operations require no work.

## Standalone use

```cpp
#include "salvageCore.hpp"

int main() {
    using namespace salvage::core;

    World world;
    world.createBody();
    world.createBody(Body{{1.0, 0.0, 0.0}});

    auto strategy = makeBroadPhase(Algorithm::BruteForce);
    strategy->initialize(world.getBodies());

    std::vector<CollisionPair> candidates;
    strategy->findCandidates(world.getBodies(), candidates); // One overlapping pair.
}
```

## Simulation use

Choose the algorithm when constructing a world. `World{}` and `World{gravity}`
default to brute force. The k-d tree is currently a placeholder. Selecting it
advances bodies with forces and gravity but produces no collision response.

```cpp
World world{Algorithm::BruteForce, {0.0, -9.81, 0.0}};
world.generateRandomBodies(100, 42);
world.step(1.0 / 120.0);
```

Each world owns a strategy and updates it after integration and between position
correction passes. The world puts candidate pairs in body creation order so
the solver processes contacts consistently across algorithms. Read-only world
queries build an independent instance of the selected strategy from the current
bodies. Copying a world preserves its selection and gives the copy its own index.
