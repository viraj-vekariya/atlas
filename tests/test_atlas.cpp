#include <vector>

#include "atlas/dijkstra.hpp"
#include "atlas/graph.hpp"
#include "test_framework.hpp"

using namespace atlas;

TEST(generated_network_has_expected_node_count) {
    Graph g = GenerateSyntheticRoadNetwork(10, 8, /*seed=*/1);
    ASSERT_EQ(g.NodeCount(), static_cast<size_t>(80));
}

TEST(undirected_edge_is_traversable_both_directions_with_equal_weight) {
    Graph g;
    NodeId a = g.AddNode(Point{0, 0});
    NodeId b = g.AddNode(Point{3, 4});  // distance 5
    g.AddUndirectedEdge(a, b, Graph::EuclideanDistance(g.PointOf(a), g.PointOf(b)));

    ASSERT_EQ(g.NeighborsOf(a).size(), static_cast<size_t>(1));
    ASSERT_EQ(g.NeighborsOf(b).size(), static_cast<size_t>(1));
    ASSERT_EQ(g.NeighborsOf(a)[0].to, b);
    ASSERT_EQ(g.NeighborsOf(b)[0].to, a);
    ASSERT_TRUE(g.NeighborsOf(a)[0].weight == g.NeighborsOf(b)[0].weight);
    ASSERT_EQ(g.NeighborsOf(a)[0].weight, 5.0);
}

TEST(euclidean_distance_is_correct_on_a_known_3_4_5_triangle) {
    Point a{0, 0}, b{3, 4};
    ASSERT_EQ(Graph::EuclideanDistance(a, b), 5.0);
}

namespace {

// A small hand-verifiable graph. All three 0->2 routes:
//   0 --1--> 1 --3--> 2   = 4
//   0 --5--> 2            = 5  (direct, expensive)
//   0 --2--> 3 --1--> 2   = 3  (genuinely the cheapest)
// Correct shortest 0->2 is via node 3, distance 3. (An earlier version of
// this test had 1->2 weighted 1, not 3, which made 0->1->2 cost 2 -- cheaper
// than the path this test meant to exercise. That was a bug in the test's
// own hand-computed expectation, not in Dijkstra: caught by the test
// actually failing on first run. See README "Hardest bugs".)
Graph SmallHandCheckedGraph() {
    Graph g;
    for (int i = 0; i < 4; ++i) g.AddNode(Point{static_cast<double>(i), 0.0});
    g.AddEdge(0, 1, 1.0);
    g.AddEdge(1, 2, 3.0);
    g.AddEdge(0, 2, 5.0);
    g.AddEdge(0, 3, 2.0);
    g.AddEdge(3, 2, 1.0);
    return g;
}

}  // namespace

TEST(dijkstra_finds_correct_shortest_path_on_hand_checked_graph) {
    Graph g = SmallHandCheckedGraph();
    PathResult r = DijkstraShortestPath(g, 0, 2);
    ASSERT_TRUE(r.found);
    ASSERT_EQ(r.distance, 3.0);
    std::vector<NodeId> expected = {0, 3, 2};
    ASSERT_TRUE(r.path == expected);
}

TEST(dijkstra_reports_not_found_on_disconnected_graph_without_crashing) {
    Graph g;
    NodeId x = g.AddNode(Point{0, 0});
    NodeId y = g.AddNode(Point{100, 100});  // no edge at all
    PathResult r = DijkstraShortestPath(g, x, y);
    ASSERT_TRUE(!r.found);
}

int main() { return testing::RunAll(); }
