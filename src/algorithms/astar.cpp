#include "astar.hpp"
#include <queue>
#include <vector>
#include <chrono>
#include <algorithm>

using namespace std::chrono;

struct AStarNode {
    NodeID id;
    EdgeWeight g_score;
    EdgeWeight f_score;

    bool operator>(const AStarNode& other) const {
        return f_score > other.f_score;
    }
};

PathResult AStar::solve(const Graph& graph, NodeID start, NodeID goal) {
    PathResult result;
    auto start_time = steady_clock::now();
    double heuristic_duration_ms = 0.0;

    size_t num_nodes = graph.get_num_nodes();
    if (start >= num_nodes || goal >= num_nodes) {
        return result;
    }

    std::vector<EdgeWeight> g_dist(num_nodes, INF_WEIGHT);
    std::vector<NodeID> parent(num_nodes, INVALID_NODE);
    std::vector<bool> visited(num_nodes, false);

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> pq;

    g_dist[start] = 0.0;
    
    auto h_start = steady_clock::now();
    EdgeWeight start_h = heuristic->compute(start, goal, graph);
    auto h_end = steady_clock::now();
    heuristic_duration_ms += duration<double, std::milli>(h_end - h_start).count();

    pq.push({start, 0.0, start_h});
    result.nodes_generated++;

    while (!pq.empty()) {
        AStarNode current = pq.top();
        pq.pop();

        // Standard A* visited check (for consistency with Dijkstra and monotonic heuristics)
        if (visited[current.id]) continue;
        visited[current.id] = true;
        result.nodes_expanded++;

        if (current.id == goal) {
            break;
        }

        for (const auto& edge : graph.get_neighbors(current.id)) {
            if (!edge.is_open) continue;

            NodeID next_id = edge.target;
            EdgeWeight tentative_g = g_dist[current.id] + edge.weight;

            if (tentative_g < g_dist[next_id]) {
                g_dist[next_id] = tentative_g;
                parent[next_id] = current.id;
                
                h_start = steady_clock::now();
                EdgeWeight h_val = heuristic->compute(next_id, goal, graph);
                h_end = steady_clock::now();
                heuristic_duration_ms += duration<double, std::milli>(h_end - h_start).count();

                EdgeWeight f_val = tentative_g + h_val;
                pq.push({next_id, tentative_g, f_val});
                result.nodes_generated++;
            }
        }
    }

    auto end_time = steady_clock::now();
    result.search_time_ms = duration<double, std::milli>(end_time - start_time).count();
    result.heuristic_time_ms = heuristic_duration_ms;

    if (g_dist[goal] != INF_WEIGHT) {
        result.total_cost = g_dist[goal];
        NodeID current = goal;
        while (current != INVALID_NODE) {
            result.path.push_back(current);
            current = parent[current];
        }
        std::reverse(result.path.begin(), result.path.end());
    }

    return result;
}
