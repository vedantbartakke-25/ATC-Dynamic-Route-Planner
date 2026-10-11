#include "ida_star.hpp"
#include <chrono>

using namespace std::chrono;

EdgeWeight IDAStar::search(const Graph& graph, NodeID current, NodeID goal, 
                           EdgeWeight g, EdgeWeight threshold, 
                           std::vector<NodeID>& path, std::vector<EdgeWeight>& min_g,
                           std::vector<bool>& in_path, PathResult& res,
                           const std::chrono::steady_clock::time_point& deadline,
                           size_t& max_depth) {
    
    // Check deadline safeguard
    if ((res.nodes_expanded & 0x3FF) == 0) { // Check every 1024 expansions for low overhead
        if (steady_clock::now() >= deadline) {
            res.status = ExecutionStatus::TIMEOUT;
            return INF_WEIGHT;
        }
    }
    // Check expansion limit safeguard
    if (res.nodes_expanded >= max_node_expansions) {
        res.status = ExecutionStatus::RESOURCE_LIMIT;
        return INF_WEIGHT;
    }

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
    max_depth = std::max(max_depth, path.size());
    EdgeWeight min_val = INF_WEIGHT;

    for (const auto& edge : graph.get_neighbors(current)) {
        if (!edge.is_open) continue;
        NodeID next_id = edge.target;
        
        EdgeWeight next_g = g + edge.weight;
        
        // Cycle / redundancy check for graph search:
        if (in_path[next_id] || next_g >= min_g[next_id]) {
            continue;
        }

        in_path[next_id] = true;
        EdgeWeight prev_min_g = min_g[next_id];
        min_g[next_id] = next_g;
        path.push_back(next_id);
        res.nodes_generated++;
        
        EdgeWeight t = search(graph, next_id, goal, next_g, threshold, path, min_g, in_path, res, deadline, max_depth);
        
        if (res.status == ExecutionStatus::TIMEOUT || res.status == ExecutionStatus::RESOURCE_LIMIT) {
            return INF_WEIGHT;
        }
        if (t == 0.0) return 0.0; // Found
        if (t < min_val) min_val = t;
        
        // Backtrack
        path.pop_back();
        in_path[next_id] = false;
    }

    return min_val;
}

#include "core/high_res_timer.hpp"

PathResult IDAStar::solve(const Graph& graph, NodeID start, NodeID goal) {
    PathResult res;
    res.status = ExecutionStatus::SUCCESS;
    res.preprocessing_time_ms = heuristic->get_preprocessing_time_ms();
    HighResTimer search_timer;
    search_timer.start();
    auto start_time = steady_clock::now();
    auto deadline = start_time + std::chrono::milliseconds(static_cast<long long>(time_limit_ms));

    size_t num_nodes = graph.get_num_nodes();
    if (start >= num_nodes || goal >= num_nodes) {
        res.status = ExecutionStatus::ERROR;
        return res;
    }

    auto h_start = steady_clock::now();
    EdgeWeight threshold = heuristic->compute(start, goal, graph);
    auto h_end = steady_clock::now();
    res.heuristic_time_ms += duration<double, std::milli>(h_end - h_start).count();

    std::vector<NodeID> path;
    path.push_back(start);
    
    std::vector<bool> in_path(num_nodes, false);
    in_path[start] = true;
    size_t max_depth = 1;

    while (true) {
        if (steady_clock::now() >= deadline) {
            res.status = ExecutionStatus::TIMEOUT;
            break;
        }
        if (res.nodes_expanded >= max_node_expansions) {
            res.status = ExecutionStatus::RESOURCE_LIMIT;
            break;
        }

        res.iterations++;
        std::vector<EdgeWeight> min_g(num_nodes, INF_WEIGHT);
        min_g[start] = 0.0;

        EdgeWeight t = search(graph, start, goal, 0.0, threshold, path, min_g, in_path, res, deadline, max_depth);
        
        if (res.status == ExecutionStatus::TIMEOUT || res.status == ExecutionStatus::RESOURCE_LIMIT) {
            break;
        }
        if (t == 0.0) {
            res.path = path;
            break;
        }
        if (t == INF_WEIGHT) {
            res.status = ExecutionStatus::NO_PATH;
            break; // No path
        }
        threshold = t;
    }

    res.search_time_ms = search_timer.elapsed_ms();

    // Compute estimated peak search memory for IDA*:
    // min_g array (num_nodes * sizeof(EdgeWeight)) + in_path (num_nodes * sizeof(bool))
    // + path stack (max_depth * sizeof(NodeID)) + recursion call stack (~128 bytes per stack frame)
    size_t search_bytes = num_nodes * sizeof(EdgeWeight) +
                          num_nodes * sizeof(bool) +
                          max_depth * sizeof(NodeID) +
                          max_depth * 128;
    res.peak_memory_kb = static_cast<double>(search_bytes) / 1024.0;

    return res;
}
