#include "atlas/graph.hpp"

#include <cmath>
#include <random>

namespace atlas {

NodeId Graph::AddNode(Point p) {
    points_.push_back(p);
    adj_.emplace_back();
    return static_cast<NodeId>(points_.size() - 1);
}

void Graph::AddEdge(NodeId from, NodeId to, double weight) {
    adj_[from].push_back(Edge{to, weight});
}

void Graph::AddUndirectedEdge(NodeId a, NodeId b, double weight) {
    AddEdge(a, b, weight);
    AddEdge(b, a, weight);
}

double Graph::EuclideanDistance(const Point& a, const Point& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

Graph GenerateSyntheticRoadNetwork(int grid_width, int grid_height, uint32_t seed) {
    Graph g;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> jitter(-0.35, 0.35);

    std::vector<std::vector<NodeId>> ids(grid_height, std::vector<NodeId>(grid_width));
    for (int y = 0; y < grid_height; ++y) {
        for (int x = 0; x < grid_width; ++x) {
            Point p{static_cast<double>(x) + jitter(rng), static_cast<double>(y) + jitter(rng)};
            ids[y][x] = g.AddNode(p);
        }
    }

    // Grid roads: each node connects to its right and down neighbor
    // (undirected), weighted by real Euclidean distance so the jitter
    // actually matters.
    for (int y = 0; y < grid_height; ++y) {
        for (int x = 0; x < grid_width; ++x) {
            NodeId here = ids[y][x];
            if (x + 1 < grid_width) {
                NodeId right = ids[y][x + 1];
                g.AddUndirectedEdge(here, right, Graph::EuclideanDistance(g.PointOf(here), g.PointOf(right)));
            }
            if (y + 1 < grid_height) {
                NodeId down = ids[y + 1][x];
                g.AddUndirectedEdge(here, down, Graph::EuclideanDistance(g.PointOf(here), g.PointOf(down)));
            }
        }
    }

    // A sparse set of long "highway" shortcuts between random distant nodes.
    // Weighted at EXACTLY the straight-line distance between endpoints, not
    // a discount below it: A*'s heuristic here is Euclidean distance to the
    // goal, which is only admissible if no edge costs less than the
    // straight-line distance between its own endpoints (otherwise the
    // heuristic can overestimate the true remaining cost through that edge
    // and A* loses its optimality guarantee -- this was caught while
    // writing the generator, see README "Hardest bugs"). A shortcut that
    // costs exactly the straight-line distance is still a real shortcut
    // relative to the grid: routing through intermediate grid nodes costs
    // MORE than that thanks to jitter and the fact that grid paths aren't
    // straight lines, so highways still make A* meaningfully prune search
    // vs. Dijkstra without needing to be artificially cheap.
    int highway_count = (grid_width * grid_height) / 20;
    std::uniform_int_distribution<int> pick(0, grid_width * grid_height - 1);
    for (int i = 0; i < highway_count; ++i) {
        NodeId a = pick(rng);
        NodeId b = pick(rng);
        if (a == b) continue;
        double w = Graph::EuclideanDistance(g.PointOf(a), g.PointOf(b));
        g.AddUndirectedEdge(a, b, w);
    }

    return g;
}

}  // namespace atlas
