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

int main() { return testing::RunAll(); }
