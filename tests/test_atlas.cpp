#include <algorithm>
#include <utility>
#include <vector>

#include "atlas/astar.hpp"
#include "atlas/bidirectional_dijkstra.hpp"
#include "atlas/dijkstra.hpp"
#include "atlas/graph.hpp"
#include "atlas/tour.hpp"
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

// Bidirectional Dijkstra's backward search walks NeighborsOf(n) same as the
// forward search does (see bidirectional_dijkstra.hpp for why: this repo's
// real graphs are always undirected). SmallHandCheckedGraph() above uses
// directed edges deliberately, to test that plain Dijkstra/A* respect edge
// direction -- it is NOT a valid fixture for bidirectional Dijkstra, whose
// backward search would silently explore the wrong direction on it. This
// is a genuinely separate small hand-checked graph, built undirected.
Graph SmallHandCheckedUndirectedGraph() {
    Graph g;
    for (int i = 0; i < 4; ++i) g.AddNode(Point{static_cast<double>(i), 0.0});
    g.AddUndirectedEdge(0, 1, 1.0);
    g.AddUndirectedEdge(1, 2, 3.0);
    g.AddUndirectedEdge(0, 2, 5.0);
    g.AddUndirectedEdge(0, 3, 2.0);
    g.AddUndirectedEdge(3, 2, 1.0);
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

TEST(astar_finds_correct_shortest_path_on_hand_checked_graph) {
    Graph g = SmallHandCheckedGraph();
    PathResult r = AStarShortestPath(g, 0, 2);
    ASSERT_TRUE(r.found);
    ASSERT_EQ(r.distance, 3.0);
    std::vector<NodeId> expected = {0, 3, 2};
    ASSERT_TRUE(r.path == expected);
}

TEST(dijkstra_and_astar_agree_on_distance_across_large_synthetic_graph) {
    // A* must find the SAME optimal distance as Dijkstra, not just "a"
    // path -- if the heuristic's admissibility were broken this could
    // diverge. (This test is why the highway-edge discount bug described
    // in the README was caught immediately rather than shipping silently:
    // the discounted version of GenerateSyntheticRoadNetwork failed this
    // exact test on some query pairs before the fix.)
    Graph g = GenerateSyntheticRoadNetwork(40, 40, /*seed=*/1);
    std::vector<std::pair<NodeId, NodeId>> queries = {
        {0, 1599}, {200, 1000}, {50, 1550}, {777, 42}, {900, 50}};
    for (auto [s, t] : queries) {
        PathResult dij = DijkstraShortestPath(g, s, t);
        PathResult astar = AStarShortestPath(g, s, t);
        ASSERT_TRUE(dij.found);
        ASSERT_TRUE(astar.found);
        ASSERT_TRUE(dij.distance - astar.distance < 1e-6 && astar.distance - dij.distance < 1e-6);
    }
}

TEST(astar_expands_meaningfully_fewer_nodes_than_dijkstra_on_large_graph) {
    Graph g = GenerateSyntheticRoadNetwork(60, 60, /*seed=*/42);
    PathResult dij = DijkstraShortestPath(g, 0, static_cast<NodeId>(g.NodeCount() - 1));
    PathResult astar = AStarShortestPath(g, 0, static_cast<NodeId>(g.NodeCount() - 1));
    ASSERT_TRUE(dij.found);
    ASSERT_TRUE(astar.found);
    // Not a tight bound -- just confirms the heuristic is doing real work,
    // not degenerating to Dijkstra.
    ASSERT_TRUE(astar.nodes_expanded < dij.nodes_expanded);
}

TEST(astar_reports_not_found_on_disconnected_graph_without_crashing) {
    Graph g;
    NodeId x = g.AddNode(Point{0, 0});
    NodeId y = g.AddNode(Point{100, 100});
    PathResult r = AStarShortestPath(g, x, y);
    ASSERT_TRUE(!r.found);
}

TEST(two_opt_never_makes_the_tour_longer_than_nearest_neighbor) {
    Graph g = GenerateSyntheticRoadNetwork(30, 30, /*seed=*/9);
    std::vector<NodeId> stops = {0, 45, 112, 300, 421, 555, 610, 733, 812, 899};
    auto dist = PairwiseRoadDistances(g, stops);
    Tour nn = NearestNeighborTour(stops, dist);
    Tour improved = TwoOptImprove(nn, stops, dist);
    ASSERT_TRUE(improved.length <= nn.length + 1e-9);
}

TEST(two_opt_result_is_a_valid_permutation_returning_to_the_depot) {
    Graph g = GenerateSyntheticRoadNetwork(20, 20, /*seed=*/3);
    std::vector<NodeId> stops = {0, 15, 88, 150, 210, 305};
    auto dist = PairwiseRoadDistances(g, stops);
    Tour nn = NearestNeighborTour(stops, dist);
    Tour improved = TwoOptImprove(nn, stops, dist);

    ASSERT_EQ(improved.stops.size(), stops.size() + 1);  // + return to depot
    ASSERT_EQ(improved.stops.front(), stops[0]);
    ASSERT_EQ(improved.stops.back(), stops[0]);
    std::vector<NodeId> middle(improved.stops.begin(), improved.stops.end() - 1);
    std::sort(middle.begin(), middle.end());
    std::vector<NodeId> expected = stops;
    std::sort(expected.begin(), expected.end());
    ASSERT_TRUE(middle == expected);
}

TEST(bidirectional_dijkstra_finds_correct_shortest_path_on_hand_checked_graph) {
    Graph g = SmallHandCheckedUndirectedGraph();
    PathResult r = BidirectionalDijkstraShortestPath(g, 0, 2);
    ASSERT_TRUE(r.found);
    ASSERT_EQ(r.distance, 3.0);
}

TEST(bidirectional_dijkstra_reports_not_found_on_disconnected_graph_without_crashing) {
    Graph g;
    NodeId x = g.AddNode(Point{0, 0});
    NodeId y = g.AddNode(Point{100, 100});
    PathResult r = BidirectionalDijkstraShortestPath(g, x, y);
    ASSERT_TRUE(!r.found);
}

TEST(bidirectional_dijkstra_start_equals_goal_is_zero_distance) {
    Graph g = SmallHandCheckedUndirectedGraph();
    PathResult r = BidirectionalDijkstraShortestPath(g, 1, 1);
    ASSERT_TRUE(r.found);
    ASSERT_EQ(r.distance, 0.0);
    ASSERT_EQ(r.path.size(), static_cast<size_t>(1));
}

TEST(bidirectional_dijkstra_agrees_with_dijkstra_and_astar_across_large_synthetic_graph) {
    // This is the test that caught the real bug: an early version only
    // checked "is this exact node finalized on both sides", which missed
    // the case where the optimal path crosses via an edge between a
    // forward-finalized node and an already-backward-finalized neighbor
    // that never itself becomes forward-finalized. That version passed
    // the small hand-checked graph above (too small to expose the gap)
    // but failed here, on most queries, with bidirectional_dijkstra
    // reporting a longer-than-optimal distance. See README "Hardest bug".
    Graph g = GenerateSyntheticRoadNetwork(40, 40, /*seed=*/1);
    std::vector<std::pair<NodeId, NodeId>> queries = {
        {0, 1599}, {200, 1000}, {50, 1550}, {777, 42}, {900, 50},
        {175, 1220}, {1372, 1257}, {1470, 583}, {904, 1239}, {33, 1567}};
    for (auto [s, t] : queries) {
        PathResult dij = DijkstraShortestPath(g, s, t);
        PathResult bidir = BidirectionalDijkstraShortestPath(g, s, t);
        ASSERT_TRUE(dij.found);
        ASSERT_TRUE(bidir.found);
        ASSERT_TRUE(dij.distance - bidir.distance < 1e-6 && bidir.distance - dij.distance < 1e-6);
    }
}

TEST(bidirectional_dijkstra_expands_fewer_nodes_than_plain_dijkstra_on_large_graph) {
    Graph g = GenerateSyntheticRoadNetwork(60, 60, /*seed=*/42);
    PathResult dij = DijkstraShortestPath(g, 0, static_cast<NodeId>(g.NodeCount() - 1));
    PathResult bidir = BidirectionalDijkstraShortestPath(g, 0, static_cast<NodeId>(g.NodeCount() - 1));
    ASSERT_TRUE(dij.found);
    ASSERT_TRUE(bidir.found);
    ASSERT_TRUE(bidir.nodes_expanded < dij.nodes_expanded);
}

int main() { return testing::RunAll(); }
