#pragma once

#include "atlas/graph.hpp"

namespace atlas {

// A* using straight-line Euclidean distance to the goal as the heuristic.
// This is admissible exactly when every edge weight is >= the Euclidean
// distance between its own endpoints (otherwise the heuristic can
// overestimate true remaining cost through a "too cheap" edge, and A* can
// return a suboptimal path). GenerateSyntheticRoadNetwork's grid edges use
// real Euclidean distance and its highway edges are weighted at EXACTLY
// Euclidean distance (not discounted below it) specifically to preserve
// this. See README "Hardest bugs" for how this was caught.
PathResult AStarShortestPath(const Graph& g, NodeId start, NodeId goal);

}  // namespace atlas
