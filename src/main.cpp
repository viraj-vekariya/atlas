#include <cstdio>

#include "atlas/astar.hpp"
#include "atlas/dijkstra.hpp"
#include "atlas/graph.hpp"

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
    printf("A* expanded %.1f%% of what Dijkstra expanded\n",
           100.0 * static_cast<double>(astar.nodes_expanded) / static_cast<double>(dij.nodes_expanded));

    return 0;
}
