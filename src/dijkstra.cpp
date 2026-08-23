#include "atlas/dijkstra.hpp"

#include <limits>
#include <queue>
#include <vector>

namespace atlas {

namespace {
struct QueueEntry {
    double dist;
    NodeId node;
    bool operator>(const QueueEntry& o) const { return dist > o.dist; }
};
}  // namespace

PathResult DijkstraShortestPath(const Graph& g, NodeId start, NodeId goal) {
    const double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> dist(g.NodeCount(), kInf);
    std::vector<NodeId> prev(g.NodeCount(), -1);
    std::vector<bool> finalized(g.NodeCount(), false);

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> pq;
    dist[start] = 0.0;
    pq.push({0.0, start});

    PathResult result;

    while (!pq.empty()) {
        QueueEntry top = pq.top();
        pq.pop();
        if (finalized[top.node]) continue;  // stale heap entry from an earlier relaxation
        finalized[top.node] = true;
        result.nodes_expanded++;

        if (top.node == goal) break;

        for (const Edge& e : g.NeighborsOf(top.node)) {
            if (finalized[e.to]) continue;
            double nd = dist[top.node] + e.weight;
            if (nd < dist[e.to]) {
                dist[e.to] = nd;
                prev[e.to] = top.node;
                pq.push({nd, e.to});
            }
        }
    }

    if (dist[goal] == kInf) {
        result.found = false;
        return result;
    }

    result.found = true;
    result.distance = dist[goal];
    std::vector<NodeId> rev;
    for (NodeId at = goal; at != -1; at = prev[at]) rev.push_back(at);
    result.path.assign(rev.rbegin(), rev.rend());
    return result;
}

}  // namespace atlas
