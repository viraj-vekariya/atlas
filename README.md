# atlas — a route-planning and logistics engine in C++17

Dijkstra and A* shortest-path search over a synthetic road network, plus a
nearest-neighbor + 2-opt multi-stop route optimizer — the algorithms behind
"design Uber/Swiggy routing" system-design questions, actually implemented
and benchmarked, not just described.

## Why this exists

Sibling project to [`quorum`](https://github.com/viraj-vekariya/quorum),
[`relay`](https://github.com/viraj-vekariya/relay), and
[`redis-lite`](https://github.com/viraj-vekariya/redis-lite) — same
portfolio, same bar for correctness and honesty — but deliberately
different in kind: no networking, no threading, no concurrency. This one is
about the algorithms themselves: is A*'s heuristic actually saving work, and
does 2-opt actually improve a tour, with real numbers backing both claims
rather than "should be faster in theory."

## Architecture

- **`include/atlas/graph.hpp` / `src/graph.cpp`** — a directed graph of
  2D-coordinate nodes and weighted edges, plus `GenerateSyntheticRoadNetwork`:
  a grid of nodes with randomly jittered positions (so it isn't a perfectly
  uniform lattice), grid-adjacency edges weighted by real Euclidean
  distance, and a sparse set of long "highway" shortcut edges.
- **`include/atlas/dijkstra.hpp` / `src/dijkstra.cpp`** — Dijkstra's
  algorithm with a binary min-heap (`std::priority_queue`), O((V+E) log V).
  Tracks `nodes_expanded` (heap pops of newly-finalized nodes) for the A*
  comparison.
- **`include/atlas/astar.hpp` / `src/astar.cpp`** — A* using straight-line
  Euclidean distance to the goal as the heuristic.
- **`include/atlas/bidirectional_dijkstra.hpp` / `src/bidirectional_dijkstra.cpp`**
  — alternating forward-from-start / backward-from-goal Dijkstra, meeting in
  the middle. Requires an undirected graph (this repo's real graphs always
  are — see Deliberate limitations) since the backward search reuses
  `NeighborsOf` rather than building a separate reverse-adjacency structure.
- **`include/atlas/tour.hpp` / `src/tour.cpp`** — `PairwiseRoadDistances`
  (real road-network Dijkstra distances between a set of stops, not
  straight-line), `NearestNeighborTour` (greedy construction), and
  `TwoOptImprove` (2-opt local search: repeatedly reverse a segment if it
  shortens the tour, until no improving move exists).

## Real results (from `make cli`, 60x60 grid = 3,600 nodes)

```
Dijkstra:      distance=91.615  nodes_expanded=3600
A*:            distance=91.615  nodes_expanded=1477
Bidirectional: distance=91.615  nodes_expanded=2753
A* expanded 41.0% of what Dijkstra expanded
Bidirectional expanded 76.5% of what Dijkstra expanded, 186.4% of what A* expanded

Averaging node-expansion over 30 random start/goal pairs (same graph):
  avg nodes expanded -- Dijkstra: 1768.3  A*: 259.4  Bidirectional: 1128.2
  Bidirectional / Dijkstra: 63.8%   Bidirectional / A*: 434.9%

Multi-stop route optimization (10 random stops + depot):
  nearest-neighbor tour length: 229.812
  after 2-opt:                  215.221  (6.3% shorter)
```

All three algorithms find the **same optimal distance** (91.615) — this
equivalence is directly asserted in the test suite
(`dijkstra_and_astar_agree_on_distance_...`,
`bidirectional_dijkstra_agrees_with_dijkstra_and_astar_...`), not just
observed once in a demo run. Dijkstra expanding essentially the whole graph
(3600/3600) on the single corner-to-corner query isn't a bug — that's close
to Dijkstra's worst case (it explores uniformly outward regardless of where
the goal is), which is exactly why the averaged 30-query numbers are the
more honest comparison.

**Bidirectional Dijkstra genuinely beats plain Dijkstra here (63.8% of its
node count, averaged), but loses badly to A* (434.9% of A*'s node count).**
That's an honest, unglamorous result, not a disappointing implementation:
bidirectional search cuts the *search radius* roughly in half by growing
two smaller frontiers instead of one big one, but it has zero information
about *which direction* the goal actually is — it explores its half-radius
uniformly in every direction, the same blind-uniform-expansion problem
plain Dijkstra has, just twice, from two starting points. A*'s heuristic
tells it which way to go from the first step. The two techniques solve
different problems and are not competitors; the standard real-world
approach (out of scope here) is bidirectional search *with* an A*-style
heuristic on both frontiers, combining both advantages.

## Hardest bugs

Four. The bidirectional Dijkstra one is the most instructive of the set —
it's the textbook subtlety this technique is known for, hit for real
rather than just read about.

**1. A* heuristic admissibility bug in the graph generator (caught while
writing, before ever compiling).** The first version of
`GenerateSyntheticRoadNetwork` weighted highway shortcut edges at a 0.6x
discount below the straight-line (Euclidean) distance between their
endpoints, meant to represent "highways are faster per unit distance than
surface streets." That's wrong for this project specifically: A*'s
heuristic here is Euclidean distance to the goal, which is only
*admissible* (guaranteed to never overestimate true remaining cost) if
every edge costs at least as much as the straight-line distance between its
own endpoints. An edge cheaper than that can make the heuristic
overestimate the true cost through it, which breaks A*'s optimality
guarantee — the exact thing `dijkstra_and_astar_agree_on_distance_...`
exists to catch. Fixed by weighting highway edges at exactly the Euclidean
distance between their endpoints, not a discount below it. They're still
real shortcuts relative to the grid (routing through intermediate jittered
grid nodes costs more than one straight hop), so A* still gets meaningful
pruning without sacrificing correctness.

**2. A hand-computed test expectation was wrong, not the algorithm (caught
by the test itself failing on first run).**
`dijkstra_finds_correct_shortest_path_on_hand_checked_graph` was originally
built around a small graph where I intended `0 -> 3 -> 2` (cost 2+1=3) to be
the unique shortest path — but I'd left edge `1 -> 2` weighted at 1, making
`0 -> 1 -> 2` cost 1+1=2, actually cheaper than the path the test meant to
exercise. Dijkstra correctly returned distance 2 via node 1; the test's own
hand-arithmetic was wrong, not Dijkstra. Fixed by re-weighting `1 -> 2` to 3
so the intended path (via node 3, cost 3) is genuinely the cheapest of all
three routes through that graph — verified by re-checking all three route
costs by hand a second time, not just re-running until it passed.

**3. `TwoOptImprove` mixed up node IDs and array indices (caught while
writing, before ever compiling `tour.cpp`).** The first draft had
`NearestNeighborTour` return a `Tour` whose `.stops` are real graph
`NodeId`s (the sensible external API), but `TwoOptImprove` was written
assuming `.stops` held index-space positions (0..n-1) directly castable to
`NodeId` — which only happens to be correct if node IDs are assigned in the
same order as the `stops` list, true only by coincidence for small examples
and false in general. Fixed by having `TwoOptImprove` take the original
`stops` vector as a parameter and build an explicit `NodeId -> index` map,
so it's correct regardless of what node IDs the caller's stops happen to
have.

**4. Bidirectional Dijkstra's termination check was too narrow, and it
returned longer-than-optimal distances on most queries (caught by testing
against Dijkstra before this was committed, not after).** The first working
version only checked whether the *exact same node* had been finalized on
both the forward and backward search — updating the best-known distance
`mu` only when that happened. That misses the common case where the true
shortest path crosses from forward-settled to backward-settled territory
*via an edge* between two different nodes: a node `u` that just got
forward-finalized, connected to a neighbor `v` that was already
backward-finalized earlier but may never itself become forward-finalized
(the search can legitimately stop before that happens). A quick check
against plain Dijkstra across 30 random queries on the large synthetic
graph caught this immediately: 22 of 30 queries came back with a
longer-than-optimal distance, in one case by over 4.5 units. Fixed by also
checking, during every edge relaxation, whether the neighbor on the other
side of that edge is already finalized on the opposite search — not just
whether the current node itself is. Re-running the same 30-query check
after the fix: 0 mismatches.

## Deliberate limitations

- **Not an optimal TSP solver.** Nearest-neighbor + 2-opt is a well-known
  heuristic that produces *good* tours quickly, not a guarantee of the
  shortest possible tour. No claim of optimality is made anywhere in this
  project.
- **Synthetic graph, not real map data.** The road network is generated
  (jittered grid + random highways), not sourced from OpenStreetMap or any
  real geography. Coordinates are unitless, not lat/lon.
- **Static weights.** No live traffic, no time-of-day variation, no dynamic
  re-routing — every query sees the same fixed graph.
- **Bidirectional Dijkstra assumes an undirected graph.** Its backward
  search reuses `NeighborsOf` directly rather than building a real
  reverse-adjacency structure, which is only correct because
  `GenerateSyntheticRoadNetwork` only ever adds undirected edges. It would
  give wrong answers on a graph with genuinely directed edges (the repo's
  own `SmallHandCheckedGraph` test fixture is directed for exactly this
  reason, and has its own separate undirected fixture just for this
  algorithm's tests). Not combined with a heuristic (see "Real results"
  above) — a real production router would do both.

## Testing

```bash
make test
```

11 correctness tests: graph construction, Dijkstra and A* against
hand-verified small graphs, Dijkstra/A* distance agreement and A*'s
expansion-count advantage on a large synthetic graph, disconnected-graph
handling for both algorithms, and 2-opt's non-worsening + valid-permutation
guarantees.

## Running the demo

```bash
make cli
./build/atlas_cli
```
