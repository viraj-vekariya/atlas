#pragma once

#include "atlas/graph.hpp"

namespace atlas {

// Bidirectional Dijkstra: alternates a forward search from `start` with a
// backward search from `goal`, stopping once neither frontier can possibly
// improve on the best complete path found so far. Requires non-negative
// edge weights (same requirement as plain Dijkstra).
//
// This repo's Graph only ever has edges added via AddUndirectedEdge (see
// GenerateSyntheticRoadNetwork in graph.cpp), so the "reverse graph" the
// backward search needs is identical to the forward graph -- NeighborsOf(n)
// already returns edges usable in either direction. If a directed edge were
// ever added via the lower-level AddEdge, this implementation would need a
// real reverse-adjacency structure; it does not build one, since nothing in
// this codebase currently produces directed-only edges. See README.
//
// nodes_expanded counts total settles across BOTH frontiers combined, so
// it's directly comparable to Dijkstra's and A*'s nodes_expanded.
PathResult BidirectionalDijkstraShortestPath(const Graph& g, NodeId start, NodeId goal);

}  // namespace atlas
