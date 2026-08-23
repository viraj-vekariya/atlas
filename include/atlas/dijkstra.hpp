#pragma once

#include "atlas/graph.hpp"

namespace atlas {

// Classic Dijkstra with a binary min-heap (std::priority_queue), O((V+E) log V).
// nodes_expanded counts pops from the heap of nodes not yet finalized --
// this is the number A* is compared against.
PathResult DijkstraShortestPath(const Graph& g, NodeId start, NodeId goal);

}  // namespace atlas
