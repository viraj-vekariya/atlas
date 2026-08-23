#include <cstdio>
#include <random>

#include "atlas/astar.hpp"
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

    printf("Dijkstra: distance=%.3f  nodes_expanded=%zu\n", dij.distance, dij.nodes_expanded);
    printf("A*:       distance=%.3f  nodes_expanded=%zu\n", astar.distance, astar.nodes_expanded);
    printf("A* expanded %.1f%% of what Dijkstra expanded\n\n",
           100.0 * static_cast<double>(astar.nodes_expanded) / static_cast<double>(dij.nodes_expanded));

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
