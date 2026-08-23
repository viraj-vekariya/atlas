#include "atlas/tour.hpp"

#include <algorithm>
#include <limits>
#include <numeric>
#include <unordered_map>

#include "atlas/dijkstra.hpp"

namespace atlas {

std::vector<std::vector<double>> PairwiseRoadDistances(const Graph& g, const std::vector<NodeId>& stops) {
    size_t n = stops.size();
    std::vector<std::vector<double>> dist(n, std::vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            PathResult r = DijkstraShortestPath(g, stops[i], stops[j]);
            double d = r.found ? r.distance : std::numeric_limits<double>::infinity();
            dist[i][j] = d;
            dist[j][i] = d;
        }
    }
    return dist;
}

namespace {
double TourLength(const std::vector<size_t>& order, const std::vector<std::vector<double>>& dist) {
    double total = 0.0;
    for (size_t i = 0; i + 1 < order.size(); ++i) total += dist[order[i]][order[i + 1]];
    return total;
}
}  // namespace

Tour NearestNeighborTour(const std::vector<NodeId>& stops, const std::vector<std::vector<double>>& dist) {
    size_t n = stops.size();
    std::vector<bool> visited(n, false);
    std::vector<size_t> order;
    order.push_back(0);
    visited[0] = true;

    for (size_t step = 1; step < n; ++step) {
        size_t current = order.back();
        size_t best = SIZE_MAX;
        double best_dist = std::numeric_limits<double>::infinity();
        for (size_t cand = 0; cand < n; ++cand) {
            if (visited[cand]) continue;
            if (dist[current][cand] < best_dist) {
                best_dist = dist[current][cand];
                best = cand;
            }
        }
        order.push_back(best);
        visited[best] = true;
    }
    order.push_back(0);  // return to depot

    Tour t;
    t.length = TourLength(order, dist);
    for (size_t idx : order) t.stops.push_back(stops[idx]);
    return t;
}

Tour TwoOptImprove(Tour tour, const std::vector<NodeId>& stops, const std::vector<std::vector<double>>& dist,
                   int max_passes) {
    // Map NodeId -> index into `stops`/`dist` (distinct node ids, so this is
    // unambiguous), then reconstruct the tour's visiting order in index
    // space so segment reversals index directly into `dist`.
    std::unordered_map<NodeId, size_t> index_of;
    for (size_t i = 0; i < stops.size(); ++i) index_of[stops[i]] = i;

    std::vector<size_t> order(tour.stops.size());
    for (size_t i = 0; i < tour.stops.size(); ++i) order[i] = index_of.at(tour.stops[i]);

    size_t n = order.size();
    bool improved = true;
    int pass = 0;
    while (improved && pass < max_passes) {
        improved = false;
        pass++;
        // Classic 2-opt: for each pair of edges (order[i-1],order[i]) and
        // (order[j],order[j+1]), check whether reversing the segment
        // order[i..j] shortens the tour. Skip the fixed depot at position 0
        // and the fixed return-to-depot at the last position.
        for (size_t i = 1; i + 1 < n; ++i) {
            for (size_t j = i + 1; j + 1 < n; ++j) {
                double before = dist[order[i - 1]][order[i]] + dist[order[j]][order[j + 1]];
                double after = dist[order[i - 1]][order[j]] + dist[order[i]][order[j + 1]];
                if (after + 1e-9 < before) {
                    std::reverse(order.begin() + i, order.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }

    Tour result;
    result.length = TourLength(order, dist);
    result.stops.reserve(order.size());
    for (size_t idx : order) result.stops.push_back(stops[idx]);
    return result;
}

}  // namespace atlas
