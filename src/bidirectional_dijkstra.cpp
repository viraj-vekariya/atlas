#include "atlas/bidirectional_dijkstra.hpp"

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

using Heap = std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>>;

// Pops stale entries (nodes already finalized) off the top of `heap` and
// returns the current minimum tentative distance still in it, or +inf if
// the heap has nothing live left.
double PeekMinLive(Heap& heap, const std::vector<bool>& finalized) {
    while (!heap.empty() && finalized[heap.top().node]) heap.pop();
    return heap.empty() ? std::numeric_limits<double>::infinity() : heap.top().dist;
}
}  // namespace

PathResult BidirectionalDijkstraShortestPath(const Graph& g, NodeId start, NodeId goal) {
    const double kInf = std::numeric_limits<double>::infinity();
    const size_t n = g.NodeCount();

    std::vector<double> dist_f(n, kInf), dist_b(n, kInf);
    std::vector<NodeId> prev_f(n, -1), prev_b(n, -1);
    std::vector<bool> final_f(n, false), final_b(n, false);

    Heap pq_f, pq_b;
    dist_f[start] = 0.0;
    pq_f.push({0.0, start});
    dist_b[goal] = 0.0;
    pq_b.push({0.0, goal});

    PathResult result;
    double mu = kInf;         // best complete s..t distance found so far
    NodeId meet_f = -1;       // forward-side endpoint of the best crossing found
    NodeId meet_b = -1;       // backward-side endpoint (== meet_f unless the
                               // best path crosses via an edge between a
                               // forward-settled node and a *different*,
                               // already backward-settled node)

    if (start == goal) {
        result.found = true;
        result.distance = 0.0;
        result.path = {start};
        result.nodes_expanded = 0;
        return result;
    }

    // Settle `top.node` on the side given by (dist/prev/final), then check
    // every outgoing edge for a possible improvement to `mu` -- NOT just
    // whether `top.node` itself happens to already be finalized on the
    // other side. This second, easy-to-miss check is required: the true
    // optimal path can cross from forward- to backward-settled territory
    // via an edge (u, v) where u just got forward-finalized and v was
    // ALREADY backward-finalized earlier, without v ever becoming
    // forward-finalized itself (the search can legitimately stop before
    // that happens). Checking only "is this exact node settled on both
    // sides" misses that crossing and can report a distance that's too
    // long -- this was a real bug caught by testing against Dijkstra on
    // the large synthetic graph before this ever got committed; see
    // README "Hardest bug".
    auto settle_and_relax = [&](Heap& own_heap, std::vector<double>& own_dist,
                                 std::vector<NodeId>& own_prev, std::vector<bool>& own_final,
                                 const std::vector<double>& other_dist,
                                 const std::vector<bool>& other_final, bool is_forward_side) {
        QueueEntry top = own_heap.top();
        own_heap.pop();
        if (own_final[top.node]) return;  // stale entry
        own_final[top.node] = true;
        result.nodes_expanded++;

        // Same-node meeting: top.node has now been finalized on both sides.
        if (other_final[top.node]) {
            double candidate = own_dist[top.node] + other_dist[top.node];
            if (candidate < mu) {
                mu = candidate;
                meet_f = top.node;
                meet_b = top.node;
            }
        }

        for (const Edge& e : g.NeighborsOf(top.node)) {
            // Edge-crossing meeting: top.node (this side) connects directly
            // to a neighbor already finalized on the OTHER side. Check this
            // regardless of whether `e.to` is finalized on THIS side yet.
            if (other_final[e.to]) {
                double candidate = own_dist[top.node] + e.weight + other_dist[e.to];
                if (candidate < mu) {
                    mu = candidate;
                    if (is_forward_side) {
                        meet_f = top.node;
                        meet_b = e.to;
                    } else {
                        meet_f = e.to;
                        meet_b = top.node;
                    }
                }
            }

            if (own_final[e.to]) continue;
            double nd = own_dist[top.node] + e.weight;
            if (nd < own_dist[e.to]) {
                own_dist[e.to] = nd;
                own_prev[e.to] = top.node;
                own_heap.push({nd, e.to});
            }
        }
    };

    while (!pq_f.empty() || !pq_b.empty()) {
        double top_f = PeekMinLive(pq_f, final_f);
        double top_b = PeekMinLive(pq_b, final_b);

        // Standard bidirectional-Dijkstra stopping rule: once the sum of the
        // two frontiers' current minimum tentative distances is no smaller
        // than the best complete path already found, no future settlement
        // on either side can beat `mu`. (Settling one more node on the
        // cheaper side FIRST, THEN checking this, is the classic mistake --
        // it can settle one node past the correct stopping point and still
        // happens to work on many graphs, but isn't provably correct. This
        // checks the bound BEFORE deciding to expand again.)
        if (top_f + top_b >= mu) break;
        if (top_f == kInf && top_b == kInf) break;  // both sides exhausted, disconnected

        // Alternate toward whichever frontier is cheaper to advance -- pure
        // round-robin also works, but this tends to expand fewer nodes in
        // practice since it always grows the frontier most likely to be
        // useful next.
        if (top_f <= top_b) {
            if (pq_f.empty()) break;
            settle_and_relax(pq_f, dist_f, prev_f, final_f, dist_b, final_b, /*is_forward_side=*/true);
        } else {
            if (pq_b.empty()) break;
            settle_and_relax(pq_b, dist_b, prev_b, final_b, dist_f, final_f, /*is_forward_side=*/false);
        }
    }

    if (meet_f == -1) {
        result.found = false;
        return result;
    }

    result.found = true;
    result.distance = mu;

    // Forward half of the path: start .. meet_f (inclusive).
    std::vector<NodeId> forward_half;
    for (NodeId at = meet_f; at != -1; at = prev_f[at]) forward_half.push_back(at);
    std::vector<NodeId> path(forward_half.rbegin(), forward_half.rend());

    // Backward half: meet_b .. goal. If the crossing was a same-node
    // meeting (meet_f == meet_b), meet_b is already the last element of
    // path above, so start from its backward predecessor instead of
    // pushing it twice.
    NodeId b_start = (meet_f == meet_b) ? prev_b[meet_b] : meet_b;
    for (NodeId at = b_start; at != -1; at = prev_b[at]) path.push_back(at);

    result.path = std::move(path);
    return result;
}

}  // namespace atlas
