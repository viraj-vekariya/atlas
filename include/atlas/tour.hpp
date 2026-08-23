#pragma once

#include <vector>

#include "atlas/graph.hpp"

namespace atlas {

struct Tour {
    std::vector<NodeId> stops;  // visiting order, starts and ends at the same depot
    double length = 0.0;
};

// Real shortest-path distances between stops (via Dijkstra), not straight-line --
// the graph's road network is what actually matters for a delivery route.
// Returns a full distance matrix over `stops` (including the depot as stops[0]).
std::vector<std::vector<double>> PairwiseRoadDistances(const Graph& g, const std::vector<NodeId>& stops);

// Greedy nearest-neighbor construction: starting at stops[0] (the depot),
// repeatedly go to the nearest unvisited stop, then return to the depot.
Tour NearestNeighborTour(const std::vector<NodeId>& stops, const std::vector<std::vector<double>>& dist);

// 2-opt local search: repeatedly reverse a segment of the tour if doing so
// shortens it, until no improving move exists or `max_passes` full sweeps
// have run. This is a heuristic improvement over the nearest-neighbor
// starting tour, NOT an optimal TSP solver -- see README.
//
// `stops` must be the SAME vector (same order) originally passed to
// PairwiseRoadDistances to build `dist` -- it's needed here to map each
// NodeId in `tour.stops` back to its row/column index in `dist`.
Tour TwoOptImprove(Tour tour, const std::vector<NodeId>& stops, const std::vector<std::vector<double>>& dist,
                   int max_passes = 100);

}  // namespace atlas
