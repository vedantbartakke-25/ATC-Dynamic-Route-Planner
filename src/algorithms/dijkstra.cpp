#include "dijkstra.hpp"
#include "core/high_res_timer.hpp"
#include <queue>
#include <vector>
#include <algorithm>

struct SearchNode {
    NodeID id;
    EdgeWeight cost;

    bool operator>(const SearchNode& other) const {
        return cost > other.cost;
    }
};

PathResult Dijkstra::solve(const Graph& graph, NodeID start, NodeID goal) {
    PathResult result;
    HighResTimer timer;
    timer.start();

    size_t num_nodes = graph.get_num_nodes();
    if (start >= num_nodes || goal >= num_nodes) {
        return result; // Invalid start or goal
    }

    std::vector<EdgeWeight> dist(num_nodes, INF_WEIGHT);
    std::vector<NodeID> parent(num_nodes, INVALID_NODE);
    std::vector<bool> visited(num_nodes, false);

    std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> pq;

    size_t peak_pq_size = 0;
    dist[start] = 0.0;
    pq.push({start, 0.0});
    result.nodes_generated++;
    peak_pq_size = std::max(peak_pq_size, pq.size());

    while (!pq.empty()) {
        SearchNode current = pq.top();
        pq.pop();

        if (visited[current.id]) continue;
        visited[current.id] = true;
        result.nodes_expanded++;

        if (current.id == goal) {
            break; // Goal reached
        }

        for (const auto& edge : graph.get_neighbors(current.id)) {
            if (!edge.is_open) continue;

            NodeID next_id = edge.target;
            EdgeWeight new_cost = current.cost + edge.weight;

            if (new_cost < dist[next_id]) {
                dist[next_id] = new_cost;
                parent[next_id] = current.id;
                pq.push({next_id, new_cost});
                result.nodes_generated++;
                peak_pq_size = std::max(peak_pq_size, pq.size());
            }
        }
    }

    result.search_time_ms = timer.elapsed_ms();

    // Compute estimated peak search memory:
    // dist array + parent array + visited array + peak priority queue elements
    size_t search_bytes = num_nodes * sizeof(EdgeWeight) +
                          num_nodes * sizeof(NodeID) +
                          num_nodes * sizeof(bool) +
                          peak_pq_size * sizeof(SearchNode);
    result.peak_memory_kb = static_cast<double>(search_bytes) / 1024.0;

    // Reconstruct path
    if (dist[goal] != INF_WEIGHT) {
        result.total_cost = dist[goal];
        NodeID current = goal;
        while (current != INVALID_NODE) {
            result.path.push_back(current);
            current = parent[current];
        }
        std::reverse(result.path.begin(), result.path.end());
    }

    return result;
}
