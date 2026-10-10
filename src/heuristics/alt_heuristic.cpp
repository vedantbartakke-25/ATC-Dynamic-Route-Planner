#include "alt_heuristic.hpp"
#include <queue>
#include <cmath>
#include <chrono>
#include <random>
#include <algorithm>

using namespace std::chrono;

ALTHeuristic::ALTHeuristic(const Graph& graph, const std::vector<NodeID>& chosen_landmarks) 
    : landmarks(chosen_landmarks), preprocessing_time_ms(0.0) {
    
    auto start_time = steady_clock::now();

    size_t num_nodes = graph.get_num_nodes();
    dist_table.resize(landmarks.size(), std::vector<EdgeWeight>(num_nodes, INF_WEIGHT));

    for (size_t i = 0; i < landmarks.size(); ++i) {
        compute_landmark_distances(graph, landmarks[i], dist_table[i]);
    }

    auto end_time = steady_clock::now();
    preprocessing_time_ms = duration<double, std::milli>(end_time - start_time).count();
}

struct ALTSearchNode {
    NodeID id;
    EdgeWeight cost;
    bool operator>(const ALTSearchNode& other) const { return cost > other.cost; }
};

void ALTHeuristic::compute_landmark_distances(const Graph& graph, NodeID landmark_id, std::vector<EdgeWeight>& distances) {
    std::priority_queue<ALTSearchNode, std::vector<ALTSearchNode>, std::greater<ALTSearchNode>> pq;
    
    distances[landmark_id] = 0.0;
    pq.push({landmark_id, 0.0});

    while (!pq.empty()) {
        auto current = pq.top();
        pq.pop();

        if (current.cost > distances[current.id]) continue;

        for (const auto& edge : graph.get_neighbors(current.id)) {
            if (!edge.is_open) continue;

            EdgeWeight new_cost = current.cost + edge.weight;
            if (new_cost < distances[edge.target]) {
                distances[edge.target] = new_cost;
                pq.push({edge.target, new_cost});
            }
        }
    }
}

EdgeWeight ALTHeuristic::compute(NodeID u, NodeID goal, const Graph& graph) {
    EdgeWeight max_h = 0.0;
    for (size_t i = 0; i < landmarks.size(); ++i) {
        EdgeWeight d_u = dist_table[i][u];
        EdgeWeight d_goal = dist_table[i][goal];
        
        if (d_u != INF_WEIGHT && d_goal != INF_WEIGHT) {
            EdgeWeight diff = std::abs(d_u - d_goal);
            if (diff > max_h) {
                max_h = diff;
            }
        }
    }
    return max_h;
}

std::vector<NodeID> ALTHeuristic::select_landmarks(const Graph& graph, size_t count) {
    std::vector<NodeID> chosen;
    size_t num_nodes = graph.get_num_nodes();
    if (num_nodes == 0) return chosen;

    count = std::min(count, num_nodes);

    // Simple random selection for now, could be improved (e.g. farthest-first traversal)
    std::mt19937 rng(42); // Fixed seed for reproducibility
    std::vector<NodeID> all_nodes(num_nodes);
    for (NodeID i = 0; i < num_nodes; ++i) all_nodes[i] = i;
    
    std::shuffle(all_nodes.begin(), all_nodes.end(), rng);
    for (size_t i = 0; i < count; ++i) {
        chosen.push_back(all_nodes[i]);
    }

    return chosen;
}
