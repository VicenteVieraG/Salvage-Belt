/**
 * @file
 * @brief Polymorphic lifecycle for broad-phase candidate generation.
 */
#pragma once

#include <span>
#include <utility>
#include <vector>
#include "world/Body.hpp"

namespace salvage::core {
    /**
     * @brief Potential collision represented by two world-local body IDs.
     * Does not own or copy either body. Pair orientation and result order are
     * unspecified; the IDs do not extend the lifetime of the bodies.
     */
    using CollisionPair = std::pair<BodyId, BodyId>;

    /**
     * @brief Strategy interface for collision candidate generation only.
     * Integration, body ownership, pair filtering, and contact resolution remain
     * outside this interface. Each instance serves one body collection at a time.
     *
     * Input spans are borrowed only for the duration of each call. Implementations
     * may copy data into their own structures, but must not retain input pointers
     * or references. Bodies must have unique IDs and valid finite AABBs.
     * Reacquire views after storage changes and call update() after body changes
     * before querying candidates. The bodies must remain unchanged during each call.
     *
     * @note The supplied strategies are placeholders and generate no candidates.
     * Their empty output must not be interpreted as proof that no bodies overlap.
     */
    class IBroadPhase {
        public:
            /// @brief Destroy a strategy through the interface.
            virtual ~IBroadPhase() = default;

            /**
             * @brief Initialize or reset all internal state from a full body collection.
             * May be called repeatedly, including with an empty collection.
             * @param bodies Read-only view of the complete body collection; no bodies are copied.
             */
            virtual void initialize(std::span<const Body> bodies) = 0;

            /**
             * @brief Synchronize internal state with the current body collection.
             * The full collection includes any additions, removals, and changed
             * bounds since the previous call. An empty collection removes all entries.
             * @param bodies Read-only view of the complete current body collection.
             * @pre initialize() has been called on this instance.
             */
            virtual void update(std::span<const Body> bodies) = 0;

            /**
             * @brief Append potentially colliding ID pairs to caller-owned output.
             * Implementations may produce false positives, but a completed algorithm
             * must include every overlapping AABB pair. Each invocation reports an
             * unordered pair at most once, with distinct IDs from the input bodies.
             * Ordering and pair orientation are unspecified. Static-body filtering
             * and narrow-phase confirmation belong to the caller.
             * @param bodies Read-only body view matching the state last supplied to
             * initialize() or update().
             * @param out Destination to append to; existing entries are preserved.
             * Clear it before the call when a fresh result is required.
             * @pre initialize() has been called; internal state matches bodies.
             * @note Placeholder implementations append nothing.
             */
            virtual void findCandidates(std::span<const Body> bodies, std::vector<CollisionPair>& out) const = 0;
    };
}
