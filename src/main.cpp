#include <cstdio>
#include <random>

#include "atlas/astar.hpp"
#include "atlas/bidirectional_dijkstra.hpp"
#include "atlas/dijkstra.hpp"
#include "atlas/graph.hpp"
#include "atlas/tour.hpp"

int main() {
    using namespace atlas;

    printf("Generating synthetic road network (60x60 grid)...\n");
    Graph g = GenerateSyntheticRoadNetwork(60, 60, /*seed=*/42);
    printf("  %zu nodes\n\n", g.NodeCount());

    NodeId start = 0;
    NodeId goal = static_cast<NodeId>(g.NodeCount() - 1);

    PathResult dij = DijkstraShortestPath(g, start, goal);
    PathResult astar = AStarShortestPath(g, start, goal);
    PathResult bidir = BidirectionalDijkstraShortestPath(g, start, goal);

    printf("Dijkstra:      distance=%.3f  nodes_expanded=%zu\n", dij.distance, dij.nodes_expanded);
    printf("A*:            distance=%.3f  nodes_expanded=%zu\n", astar.distance, astar.nodes_expanded);
    printf("Bidirectional: distance=%.3f  nodes_expanded=%zu\n", bidir.distance, bidir.nodes_expanded);
    printf("A* expanded %.1f%% of what Dijkstra expanded\n",
           100.0 * static_cast<double>(astar.nodes_expanded) / static_cast<double>(dij.nodes_expanded));
    printf("Bidirectional expanded %.1f%% of what Dijkstra expanded, %.1f%% of what A* expanded\n\n",
           100.0 * static_cast<double>(bidir.nodes_expanded) / static_cast<double>(dij.nodes_expanded),
           100.0 * static_cast<double>(bidir.nodes_expanded) / static_cast<double>(astar.nodes_expanded));

    printf("Averaging node-expansion over 30 random start/goal pairs (same graph)...\n");
    {
        std::mt19937 bench_rng(123);
        std::uniform_int_distribution<int> bench_pick(0, static_cast<int>(g.NodeCount() - 1));
        long total_dij = 0, total_astar = 0, total_bidir = 0;
        int n = 0;
        while (n < 30) {
            NodeId s = bench_pick(bench_rng), t = bench_pick(bench_rng);
            if (s == t) continue;
            PathResult d = DijkstraShortestPath(g, s, t);
            PathResult a = AStarShortestPath(g, s, t);
            PathResult b = BidirectionalDijkstraShortestPath(g, s, t);
            if (!d.found) continue;
            total_dij += static_cast<long>(d.nodes_expanded);
            total_astar += static_cast<long>(a.nodes_expanded);
            total_bidir += static_cast<long>(b.nodes_expanded);
            n++;
        }
        printf("  avg nodes expanded -- Dijkstra: %.1f  A*: %.1f  Bidirectional: %.1f\n",
               static_cast<double>(total_dij) / n, static_cast<double>(total_astar) / n,
               static_cast<double>(total_bidir) / n);
        printf("  Bidirectional / Dijkstra: %.1f%%   Bidirectional / A*: %.1f%%\n\n",
               100.0 * total_bidir / static_cast<double>(total_dij),
               100.0 * total_bidir / static_cast<double>(total_astar));
    }

    printf("Multi-stop route optimization (10 random stops + depot)...\n");
    std::mt19937 rng(7);
    std::uniform_int_distribution<int> pick(0, static_cast<int>(g.NodeCount() - 1));
    std::vector<NodeId> stops = {0};
    for (int i = 0; i < 10; ++i) stops.push_back(pick(rng));

    auto dist = PairwiseRoadDistances(g, stops);
    Tour nn = NearestNeighborTour(stops, dist);
    Tour improved = TwoOptImprove(nn, stops, dist);

    printf("  nearest-neighbor tour length: %.3f\n", nn.length);
    printf("  after 2-opt:                  %.3f  (%.1f%% shorter)\n", improved.length,
           100.0 * (1.0 - improved.length / nn.length));

    return 0;
}
