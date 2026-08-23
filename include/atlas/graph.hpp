#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace atlas {

using NodeId = int32_t;

struct Point {
    double x = 0.0;
    double y = 0.0;
};

struct Edge {
    NodeId to;
    double weight;  // real-valued distance along this road segment
};

// A road network: nodes carry 2D coordinates (used by A*'s heuristic and by
// the synthetic generator to produce realistic edge weights), edges are
// directed (undirected roads are represented as a pair of directed edges).
class Graph {
public:
    NodeId AddNode(Point p);
    void AddEdge(NodeId from, NodeId to, double weight);
    void AddUndirectedEdge(NodeId a, NodeId b, double weight);

    size_t NodeCount() const { return points_.size(); }
    const Point& PointOf(NodeId n) const { return points_[n]; }
    const std::vector<Edge>& NeighborsOf(NodeId n) const { return adj_[n]; }

    static double EuclideanDistance(const Point& a, const Point& b);

private:
    std::vector<Point> points_;
    std::vector<std::vector<Edge>> adj_;
};

struct PathResult {
    bool found = false;
    double distance = 0.0;
    std::vector<NodeId> path;      // node sequence, start..goal, empty if !found
    size_t nodes_expanded = 0;     // for algorithm comparison, not part of the "answer"
};

// A road network generator: a grid of nodes with randomly perturbed
// positions (so it isn't a perfectly uniform lattice) and edges to nearby
// grid neighbors weighted by real Euclidean distance, plus a sparse set of
// long "highway" edges so shortest paths aren't trivially grid-shaped.
Graph GenerateSyntheticRoadNetwork(int grid_width, int grid_height, uint32_t seed);

}  // namespace atlas
