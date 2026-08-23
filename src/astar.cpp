#include "atlas/astar.hpp"

#include <limits>
#include <queue>
#include <vector>

namespace atlas {

namespace {
struct QueueEntry {
    double f;  // g + h
    NodeId node;
    bool operator>(const QueueEntry& o) const { return f > o.f; }
};
}  // namespace

PathResult AStarShortestPath(const Graph& g, NodeId start, NodeId goal) {
    const double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> gscore(g.NodeCount(), kInf);
    std::vector<NodeId> prev(g.NodeCount(), -1);
    std::vector<bool> finalized(g.NodeCount(), false);

    Point goal_pt = g.PointOf(goal);
    auto h = [&](NodeId n) { return Graph::EuclideanDistance(g.PointOf(n), goal_pt); };

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> pq;
    gscore[start] = 0.0;
    pq.push({h(start), start});

    PathResult result;

    while (!pq.empty()) {
        QueueEntry top = pq.top();
        pq.pop();
        if (finalized[top.node]) continue;
        finalized[top.node] = true;
        result.nodes_expanded++;

        if (top.node == goal) break;

        for (const Edge& e : g.NeighborsOf(top.node)) {
            if (finalized[e.to]) continue;
            double ng = gscore[top.node] + e.weight;
            if (ng < gscore[e.to]) {
                gscore[e.to] = ng;
                prev[e.to] = top.node;
                pq.push({ng + h(e.to), e.to});
            }
        }
    }

    if (gscore[goal] == kInf) {
        result.found = false;
        return result;
    }

    result.found = true;
    result.distance = gscore[goal];
    std::vector<NodeId> rev;
    for (NodeId at = goal; at != -1; at = prev[at]) rev.push_back(at);
    result.path.assign(rev.rbegin(), rev.rend());
    return result;
}

}  // namespace atlas
