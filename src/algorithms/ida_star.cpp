#include "ida_star.hpp"
#include <chrono>

using namespace std::chrono;

EdgeWeight IDAStar::search(const Graph& graph, NodeID current, NodeID goal, 
                           EdgeWeight g, EdgeWeight threshold, 
                           std::vector<NodeID>& path, std::vector<EdgeWeight>& min_g,
                           std::vector<bool>& in_path, PathResult& res) {
    
    auto h_start = steady_clock::now();
    EdgeWeight h = heuristic->compute(current, goal, graph);
    auto h_end = steady_clock::now();
    res.heuristic_time_ms += duration<double, std::milli>(h_end - h_start).count();

    EdgeWeight f = g + h;
    if (f > threshold) {
        return f;
    }
    if (current == goal) {
        res.total_cost = g;
        return 0.0; // Found
    }

    res.nodes_expanded++;
    EdgeWeight min_val = INF_WEIGHT;

    for (const auto& edge : graph.get_neighbors(current)) {
        if (!edge.is_open) continue;
        NodeID next_id = edge.target;
        
        EdgeWeight next_g = g + edge.weight;
        
        // Cycle / redundancy check for graph search:
        // Do not visit if it's already in the current path.
        // Also prune if we've already reached this node with a lower or equal g in this iteration.
        if (in_path[next_id] || next_g >= min_g[next_id]) {
            continue;
        }

        in_path[next_id] = true;
        EdgeWeight prev_min_g = min_g[next_id];
        min_g[next_id] = next_g;
        path.push_back(next_id);
        res.nodes_generated++;
        
        EdgeWeight t = search(graph, next_id, goal, next_g, threshold, path, min_g, in_path, res);
        
        if (t == 0.0) return 0.0; // Found
        if (t < min_val) min_val = t;
        
        // Backtrack
        path.pop_back();
        in_path[next_id] = false;
        // In IDA* graph search, retaining min_g across branches within the same iteration
        // can prune redundant states. Some implementations reset it, but keeping it
        // helps avoid exponential blowup on grids. We keep it as is without reverting.
    }

    return min_val;
}

PathResult IDAStar::solve(const Graph& graph, NodeID start, NodeID goal) {
    PathResult res;
    res.preprocessing_time_ms = heuristic->get_preprocessing_time_ms();
    auto start_time = steady_clock::now();

    size_t num_nodes = graph.get_num_nodes();
    if (start >= num_nodes || goal >= num_nodes) return res;

    auto h_start = steady_clock::now();
    EdgeWeight threshold = heuristic->compute(start, goal, graph);
    auto h_end = steady_clock::now();
    res.heuristic_time_ms += duration<double, std::milli>(h_end - h_start).count();

    std::vector<NodeID> path;
    path.push_back(start);
    
    std::vector<bool> in_path(num_nodes, false);
    in_path[start] = true;

    while (true) {
        res.iterations++;
        // Initialize min_g cache for this iteration to avoid redundant expansions
        std::vector<EdgeWeight> min_g(num_nodes, INF_WEIGHT);
        min_g[start] = 0.0;

        EdgeWeight t = search(graph, start, goal, 0.0, threshold, path, min_g, in_path, res);
        
        if (t == 0.0) {
            res.path = path;
            break;
        }
        if (t == INF_WEIGHT) {
            break; // No path
        }
        threshold = t;
    }

    auto end_time = steady_clock::now();
    res.search_time_ms = duration<double, std::milli>(end_time - start_time).count();

    return res;
}
