#include "dijkstra.hpp"
#include <queue>
#include <vector>
#include <chrono>
#include <algorithm>

using namespace std::chrono;

struct SearchNode {
    NodeID id;
    EdgeWeight cost;

    bool operator>(const SearchNode& other) const {
        return cost > other.cost;
    }
};

PathResult Dijkstra::solve(const Graph& graph, NodeID start, NodeID goal) {
    PathResult result;
    auto start_time = steady_clock::now();

    size_t num_nodes = graph.get_num_nodes();
    if (start >= num_nodes || goal >= num_nodes) {
        return result; // Invalid start or goal
    }

    std::vector<EdgeWeight> dist(num_nodes, INF_WEIGHT);
    std::vector<NodeID> parent(num_nodes, INVALID_NODE);
    std::vector<bool> visited(num_nodes, false);

    std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> pq;

    dist[start] = 0.0;
    pq.push({start, 0.0});
    result.nodes_generated++;

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
            }
        }
    }

    auto end_time = steady_clock::now();
    result.search_time_ms = duration<double, std::milli>(end_time - start_time).count();

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
