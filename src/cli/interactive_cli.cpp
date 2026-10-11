#include "interactive_cli.hpp"
#include "generator/graph_generator.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/ida_star.hpp"
#include <iomanip>
#include <sstream>
#include <limits>
#include <algorithm>

InteractiveCLI::InteractiveCLI(std::istream& input_stream, std::ostream& output_stream)
    : graph_loaded(false), graph_source_desc("None"), is_undirected(true),
      selected_algo(AlgorithmChoice::DIJKSTRA), alt_landmarks_count(4),
      has_active_query(false), current_start(INVALID_NODE), current_goal(INVALID_NODE),
      last_algo_name("None"), route_graph_version(0), route_is_stale(false),
      route_might_be_suboptimal(false), in(input_stream), out(output_stream) {
    
    // Automatically load default demonstration graph: 15 vertices, Sparse, seed 42
    current_graph = GraphGenerator::generate_random_geometric(15, DensityCategory::SPARSE, 42, true, true);
    graph_loaded = true;
    graph_source_desc = "Generated Geometric Graph (15 nodes, Sparse, Seed: 42)";
    is_undirected = true;
    init_heuristics_if_needed();
}

std::string InteractiveCLI::get_algo_display_name(AlgorithmChoice choice) const {
    switch (choice) {
        case AlgorithmChoice::DIJKSTRA: return "Dijkstra";
        case AlgorithmChoice::ASTAR_EUCLIDEAN: return "A* Euclidean";
        case AlgorithmChoice::ASTAR_ALT: return "A* ALT (" + std::to_string(alt_landmarks_count) + " LMs)";
        case AlgorithmChoice::IDASTAR_ALT: return "IDA* ALT (" + std::to_string(alt_landmarks_count) + " LMs)";
        default: return "Unknown";
    }
}

void InteractiveCLI::init_heuristics_if_needed() {
    if (!graph_loaded || current_graph.get_num_nodes() == 0) return;
    
    euclidean_heuristic = std::make_shared<EuclideanHeuristic>();
    
    size_t lms = std::min<size_t>(alt_landmarks_count, current_graph.get_num_nodes());
    if (lms == 0) lms = 1;
    auto chosen = ALTHeuristic::select_landmarks(current_graph, lms);
    alt_heuristic = std::make_shared<ALTHeuristic>(current_graph, chosen);
}

bool InteractiveCLI::get_int_input(int& val, const std::string& prompt) {
    while (true) {
        out << prompt;
        std::string line;
        if (!std::getline(in, line)) return false;
        if (line.empty()) continue;
        std::istringstream iss(line);
        if (iss >> val) {
            std::string remaining;
            if (!(iss >> remaining)) {
                return true;
            }
        }
        out << "Error: Invalid numeric input. Please try again.\n";
    }
}

bool InteractiveCLI::get_node_input(NodeID& node, const std::string& prompt) {
    while (true) {
        out << prompt;
        std::string line;
        if (!std::getline(in, line)) return false;
        if (line.empty()) continue;
        std::istringstream iss(line);
        long long val;
        if (iss >> val) {
            std::string remaining;
            if (!(iss >> remaining)) {
                if (val >= 0 && static_cast<size_t>(val) < current_graph.get_num_nodes()) {
                    node = static_cast<NodeID>(val);
                    return true;
                }
                out << "Error: Vertex ID " << val << " out of range (Valid range: 0 to " 
                    << (current_graph.get_num_nodes() > 0 ? current_graph.get_num_nodes() - 1 : 0) << ").\n";
                continue;
            }
        }
        out << "Error: Invalid vertex input. Please enter a valid integer.\n";
    }
}

bool InteractiveCLI::get_double_input(double& val, const std::string& prompt) {
    while (true) {
        out << prompt;
        std::string line;
        if (!std::getline(in, line)) return false;
        if (line.empty()) continue;
        std::istringstream iss(line);
        if (iss >> val) {
            std::string remaining;
            if (!(iss >> remaining)) {
                return true;
            }
        }
        out << "Error: Invalid floating-point value. Please try again.\n";
    }
}

void InteractiveCLI::print_main_menu() {
    out << "\n==========================================\n"
        << "   DYNAMIC SHORTEST PATH PLANNER\n"
        << "          Interactive Demo\n"
        << "==========================================\n\n"
        << "1. Load / Generate Graph\n"
        << "2. Display Graph Information\n"
        << "3. Select Algorithm\n"
        << "4. Find Shortest Path\n"
        << "5. Close an Edge\n"
        << "6. Reopen an Edge\n"
        << "7. Update Edge Weight\n"
        << "8. Replan Current Route\n"
        << "9. Compare All Algorithms\n"
        << "10. View Current Route\n"
        << "0. Exit\n\n";
}

void InteractiveCLI::handle_load_or_generate() {
    out << "\n--- Load or Generate Graph ---\n"
        << "1. Load existing graph from file\n"
        << "2. Generate geometric graph (Custom size & density)\n"
        << "3. Reset to default demo graph (15 nodes, Sparse, Seed: 42)\n"
        << "0. Cancel\n";
    
    int subchoice = 0;
    if (!get_int_input(subchoice, "Enter choice: ")) return;

    if (subchoice == 1) {
        out << "Enter graph filepath: ";
        std::string path;
        if (!std::getline(in, path)) return;
        while (path.empty()) {
            out << "Enter graph filepath: ";
            if (!std::getline(in, path)) return;
        }

        Graph g;
        if (g.load_from_file(path)) {
            current_graph = g;
            graph_loaded = true;
            graph_source_desc = "File: " + path;
            is_undirected = false; // files typically list directed entries
            has_active_query = false;
            init_heuristics_if_needed();
            out << "Graph successfully loaded from file! (" << current_graph.get_num_nodes() 
                << " nodes, " << current_graph.get_num_edges() << " edges)\n";
        } else {
            out << "Error: Failed to load graph from \"" << path << "\".\n";
        }
    } else if (subchoice == 2) {
        int n_val = 15;
        if (!get_int_input(n_val, "Enter number of vertices (e.g. 15, 50, 100): ")) return;
        if (n_val < 2) {
            out << "Error: Number of vertices must be at least 2.\n";
            return;
        }

        out << "Select density:\n"
            << "1. Sparse (avg degree ~4)\n"
            << "2. Medium (avg degree ~10)\n"
            << "3. Dense  (avg degree ~25)\n";
        int d_val = 1;
        if (!get_int_input(d_val, "Enter density choice (1-3): ")) return;
        DensityCategory cat = DensityCategory::SPARSE;
        std::string d_str = "Sparse";
        if (d_val == 2) { cat = DensityCategory::MEDIUM; d_str = "Medium"; }
        else if (d_val == 3) { cat = DensityCategory::DENSE; d_str = "Dense"; }

        int seed = 42;
        if (!get_int_input(seed, "Enter random seed (default 42): ")) return;

        current_graph = GraphGenerator::generate_random_geometric(static_cast<size_t>(n_val), cat, static_cast<unsigned int>(seed), true, true);
        graph_loaded = true;
        graph_source_desc = "Generated Geometric (" + std::to_string(n_val) + " nodes, " + d_str + ", Seed: " + std::to_string(seed) + ")";
        is_undirected = true;
        has_active_query = false;
        init_heuristics_if_needed();
        out << "Geometric graph successfully generated!\n";
    } else if (subchoice == 3) {
        current_graph = GraphGenerator::generate_random_geometric(15, DensityCategory::SPARSE, 42, true, true);
        graph_loaded = true;
        graph_source_desc = "Generated Geometric Graph (15 nodes, Sparse, Seed: 42)";
        is_undirected = true;
        has_active_query = false;
        init_heuristics_if_needed();
        out << "Reset to default demo graph (15 nodes, Sparse, Seed: 42).\n";
    }
}

void InteractiveCLI::handle_display_info() {
    out << "\n--- Graph Information ---\n";
    if (!graph_loaded) {
        out << "Status: No graph is currently loaded.\n";
        return;
    }

    size_t num_nodes = current_graph.get_num_nodes();
    size_t total_edges = 0;
    size_t open_edges = 0;
    size_t closed_edges = 0;

    for (NodeID u = 0; u < num_nodes; ++u) {
        for (const auto& e : current_graph.get_neighbors(u)) {
            total_edges++;
            if (e.is_open) open_edges++;
            else closed_edges++;
        }
    }

    out << "  Graph Status: Loaded\n"
        << "  Description: " << graph_source_desc << "\n"
        << "  Topology: " << (is_undirected ? "Undirected (symmetric entries)" : "Directed") << "\n"
        << "  Number of Vertices: " << num_nodes << "\n"
        << "  Total Adjacency Edges: " << total_edges << "\n"
        << "  Open Edges: " << open_edges << "\n"
        << "  Closed Edges: " << closed_edges << "\n"
        << "  Graph Mutation Version: " << current_graph.get_version() << "\n"
        << "  Selected Algorithm: " << get_algo_display_name(selected_algo) << "\n";
    
    if (alt_heuristic) {
        out << "  ALT Landmark Table Memory: " << std::fixed << std::setprecision(2) 
            << alt_heuristic->get_memory_kb() << " KB\n"
            << "  ALT Preprocessing Time: " << std::setprecision(4) 
            << alt_heuristic->get_preprocessing_time_ms() << " ms\n";
    }
}

void InteractiveCLI::handle_select_algorithm() {
    out << "\n--- Select Algorithm ---\n"
        << "1. Dijkstra (Optimal Baseline)\n"
        << "2. A* with Euclidean Heuristic\n"
        << "3. A* with ALT Landmark Heuristic\n"
        << "4. IDA* with ALT Heuristic\n";
    
    int choice = 1;
    if (!get_int_input(choice, "Enter choice (1-4): ")) return;

    if (choice == 1) {
        selected_algo = AlgorithmChoice::DIJKSTRA;
        out << "Selected algorithm: Dijkstra\n";
    } else if (choice == 2) {
        selected_algo = AlgorithmChoice::ASTAR_EUCLIDEAN;
        out << "Selected algorithm: A* Euclidean\n";
    } else if (choice == 3 || choice == 4) {
        int lms = 4;
        if (!get_int_input(lms, "Enter landmark count (default 4): ")) return;
        if (lms <= 0) lms = 1;
        alt_landmarks_count = static_cast<size_t>(lms);

        if (graph_loaded) {
            init_heuristics_if_needed();
        }

        if (choice == 3) {
            selected_algo = AlgorithmChoice::ASTAR_ALT;
            out << "Selected algorithm: A* ALT (" << alt_landmarks_count << " landmarks)\n";
        } else {
            selected_algo = AlgorithmChoice::IDASTAR_ALT;
            out << "Selected algorithm: IDA* ALT (" << alt_landmarks_count << " landmarks)\n";
        }
    } else {
        out << "Error: Invalid algorithm choice.\n";
    }
}

void InteractiveCLI::print_path_result(const std::string& algo_name, const PathResult& res, NodeID start, NodeID goal) {
    out << "\n==========================================\n"
        << "Algorithm: " << algo_name << "\n"
        << "Source: " << start << " -> Destination: " << goal << "\n";

    if (res.status == ExecutionStatus::RESOURCE_LIMIT) {
        out << "Status: RESOURCE_LIMIT (Exceeded max node expansions / search bound)\n";
    } else if (res.status == ExecutionStatus::TIMEOUT) {
        out << "Status: TIMEOUT (Exceeded configured search deadline)\n";
    } else if (res.status == ExecutionStatus::NO_PATH || res.total_cost == INF_WEIGHT || res.path.empty()) {
        out << "Status: NO_PATH (Destination unreachable from source)\n";
    } else {
        out << "Status: SUCCESS\n"
            << "Total Path Cost: " << std::fixed << std::setprecision(4) << res.total_cost << "\n"
            << "Path Sequence (" << res.path.size() << " nodes):\n  ";
        for (size_t i = 0; i < res.path.size(); ++i) {
            out << res.path[i];
            if (i + 1 < res.path.size()) out << " -> ";
        }
        out << "\n";
    }

    out << "Search Time: " << std::fixed << std::setprecision(4) << res.search_time_ms << " ms\n";
    if (res.preprocessing_time_ms > 0.0) {
        out << "Preprocessing/Refresh Time: " << res.preprocessing_time_ms << " ms\n";
    }
    if (res.heuristic_time_ms > 0.0) {
        out << "Heuristic Evaluation Time: " << res.heuristic_time_ms << " ms\n";
    }
    out << "Nodes Generated: " << res.nodes_generated << "\n"
        << "Nodes Expanded: " << res.nodes_expanded << "\n";
    if (res.iterations > 0) {
        out << "IDA* Iterations: " << res.iterations << "\n";
    }
    out << "Estimated Peak Search Memory: " << std::setprecision(2) << res.peak_memory_kb << " KB\n"
        << "==========================================\n";
}

void InteractiveCLI::handle_find_shortest_path() {
    if (!graph_loaded) {
        out << "Error: No graph loaded. Please load or generate a graph first.\n";
        return;
    }

    NodeID start = INVALID_NODE;
    NodeID goal = INVALID_NODE;

    if (!get_node_input(start, "Enter source vertex: ")) return;
    if (!get_node_input(goal, "Enter destination vertex: ")) return;

    if (start == goal) {
        out << "Source and destination are identical (Cost = 0.0, Path = [" << start << "]).\n";
        return;
    }

    out << "\nExecuting " << get_algo_display_name(selected_algo) << "...\n";

    std::shared_ptr<Solver> solver;
    if (selected_algo == AlgorithmChoice::DIJKSTRA) {
        solver = std::make_shared<Dijkstra>();
    } else if (selected_algo == AlgorithmChoice::ASTAR_EUCLIDEAN) {
        if (!euclidean_heuristic) euclidean_heuristic = std::make_shared<EuclideanHeuristic>();
        solver = std::make_shared<AStar>(euclidean_heuristic);
    } else if (selected_algo == AlgorithmChoice::ASTAR_ALT) {
        init_heuristics_if_needed();
        solver = std::make_shared<AStar>(alt_heuristic);
    } else if (selected_algo == AlgorithmChoice::IDASTAR_ALT) {
        init_heuristics_if_needed();
        solver = std::make_shared<IDAStar>(alt_heuristic, 500000, 5000.0);
    }

    solver->refresh(current_graph);
    PathResult res = solver->solve(current_graph, start, goal);

    // Save as current active route
    current_start = start;
    current_goal = goal;
    last_result = res;
    last_algo_name = get_algo_display_name(selected_algo);
    route_graph_version = current_graph.get_version();
    route_is_stale = false;
    route_might_be_suboptimal = false;
    has_active_query = true;

    print_path_result(last_algo_name, res, start, goal);
}

void InteractiveCLI::handle_close_edge() {
    if (!graph_loaded) {
        out << "Error: No graph loaded.\n";
        return;
    }

    NodeID u = INVALID_NODE;
    NodeID v = INVALID_NODE;
    if (!get_node_input(u, "Enter edge source vertex u: ")) return;
    if (!get_node_input(v, "Enter edge destination vertex v: ")) return;

    if (!current_graph.has_edge(u, v)) {
        out << "Error: Edge (" << u << " -> " << v << ") does not exist in the graph.\n";
        return;
    }

    try {
        current_graph.set_edge_status(u, v, false);
        if (is_undirected && current_graph.has_edge(v, u)) {
            current_graph.set_edge_status(v, u, false);
        }

        out << "\n--- Edge Modification Applied ---\n"
            << "Event: CLOSE_EDGE (" << u << " <-> " << v << ")\n"
            << "New Graph Version: " << current_graph.get_version() << "\n";

        // Check impact on active route
        if (has_active_query) {
            bool edge_on_route = false;
            for (size_t i = 0; i + 1 < last_result.path.size(); ++i) {
                NodeID r_u = last_result.path[i];
                NodeID r_v = last_result.path[i + 1];
                if ((r_u == u && r_v == v) || (is_undirected && r_u == v && r_v == u)) {
                    edge_on_route = true;
                    break;
                }
            }

            if (edge_on_route) {
                route_is_stale = true;
                out << "ALERT: The modified edge was part of your active route!\n"
                    << "Previous Route Status: BROKEN / INVALID (Must replan)\n";
            } else {
                out << "Previous Route Status: Still valid (edge was off-route)\n";
            }
        }
    } catch (const std::exception& ex) {
        out << "Error closing edge: " << ex.what() << "\n";
    }
}

void InteractiveCLI::handle_reopen_edge() {
    if (!graph_loaded) {
        out << "Error: No graph loaded.\n";
        return;
    }

    NodeID u = INVALID_NODE;
    NodeID v = INVALID_NODE;
    if (!get_node_input(u, "Enter edge source vertex u: ")) return;
    if (!get_node_input(v, "Enter edge destination vertex v: ")) return;

    if (!current_graph.has_edge(u, v)) {
        out << "Error: Edge (" << u << " -> " << v << ") does not exist in the graph.\n";
        return;
    }

    try {
        current_graph.set_edge_status(u, v, true);
        if (is_undirected && current_graph.has_edge(v, u)) {
            current_graph.set_edge_status(v, u, true);
        }

        out << "\n--- Edge Modification Applied ---\n"
            << "Event: REOPEN_EDGE (" << u << " <-> " << v << ")\n"
            << "New Graph Version: " << current_graph.get_version() << "\n";

        if (has_active_query) {
            route_might_be_suboptimal = true;
            out << "Previous Route Status: Path remains physically valid.\n"
                << "Optimality Status: Reopened edge might offer a shorter alternative route (Replan recommended).\n";
        }
    } catch (const std::exception& ex) {
        out << "Error reopening edge: " << ex.what() << "\n";
    }
}

void InteractiveCLI::handle_update_edge_weight() {
    if (!graph_loaded) {
        out << "Error: No graph loaded.\n";
        return;
    }

    NodeID u = INVALID_NODE;
    NodeID v = INVALID_NODE;
    if (!get_node_input(u, "Enter edge source vertex u: ")) return;
    if (!get_node_input(v, "Enter edge destination vertex v: ")) return;

    if (!current_graph.has_edge(u, v)) {
        out << "Error: Edge (" << u << " -> " << v << ") does not exist in the graph.\n";
        return;
    }

    // Inspect old weight and Euclidean distance
    double geom_dist = current_graph.euclidean_distance(u, v);
    EdgeWeight old_weight = 0.0;
    for (const auto& e : current_graph.get_neighbors(u)) {
        if (e.target == v) {
            old_weight = e.weight;
            break;
        }
    }

    out << "Current edge weight: " << old_weight << "\n"
        << "Geometric Euclidean lower bound: " << geom_dist << "\n";

    double new_w = 0.0;
    if (!get_double_input(new_w, "Enter new positive edge weight: ")) return;

    if (new_w <= 0.0) {
        out << "Error: Edge weight must be strictly positive.\n";
        return;
    }

    // Validate Euclidean admissibility constraint
    if (new_w < geom_dist - 1e-9) {
        out << "Error: New weight (" << new_w << ") is strictly less than Euclidean distance (" 
            << geom_dist << ").\n"
            << "Update rejected to preserve admissibility of the Euclidean heuristic.\n";
        return;
    }

    try {
        current_graph.update_edge_weight(u, v, new_w);
        if (is_undirected && current_graph.has_edge(v, u)) {
            current_graph.update_edge_weight(v, u, new_w);
        }

        out << "\n--- Edge Weight Update Applied ---\n"
            << "Edge: (" << u << " <-> " << v << ")\n"
            << "Old Weight: " << old_weight << " -> New Weight: " << new_w << "\n"
            << "New Graph Version: " << current_graph.get_version() << "\n";

        if (has_active_query) {
            bool on_route = false;
            for (size_t i = 0; i + 1 < last_result.path.size(); ++i) {
                NodeID r_u = last_result.path[i];
                NodeID r_v = last_result.path[i + 1];
                if ((r_u == u && r_v == v) || (is_undirected && r_u == v && r_v == u)) {
                    on_route = true;
                    break;
                }
            }

            if (on_route) {
                if (new_w > old_weight) {
                    route_might_be_suboptimal = true;
                    out << "ALERT: Weight increased on route edge. The current path's cost increased and may no longer be optimal.\n";
                } else {
                    out << "Weight decreased on active route edge. Current path remains optimal or improved.\n";
                }
            } else {
                if (new_w < old_weight) {
                    route_might_be_suboptimal = true;
                    out << "NOTE: Weight decreased on off-route edge. A shorter route detour might now exist.\n";
                } else {
                    out << "Weight increased on off-route edge. Active route is unaffected.\n";
                }
            }
        }
    } catch (const std::exception& ex) {
        out << "Error updating edge weight: " << ex.what() << "\n";
    }
}

void InteractiveCLI::handle_replan_route() {
    if (!graph_loaded) {
        out << "Error: No graph loaded.\n";
        return;
    }
    if (!has_active_query) {
        out << "Error: No active route to replan. Please run \"Find Shortest Path\" (Option 4) first.\n";
        return;
    }

    out << "\n==========================================\n"
        << "PREVIOUS ROUTE (" << last_algo_name << ")\n  ";
    if (last_result.path.empty()) {
        out << "[No Path]\n";
    } else {
        for (size_t i = 0; i < last_result.path.size(); ++i) {
            out << last_result.path[i];
            if (i + 1 < last_result.path.size()) out << " -> ";
        }
        out << "\nCost: " << std::fixed << std::setprecision(4) << last_result.total_cost << "\n";
    }

    out << "\nREPLANNING WITH " << get_algo_display_name(selected_algo) << "...\n";

    std::shared_ptr<Solver> solver;
    if (selected_algo == AlgorithmChoice::DIJKSTRA) {
        solver = std::make_shared<Dijkstra>();
    } else if (selected_algo == AlgorithmChoice::ASTAR_EUCLIDEAN) {
        if (!euclidean_heuristic) euclidean_heuristic = std::make_shared<EuclideanHeuristic>();
        solver = std::make_shared<AStar>(euclidean_heuristic);
    } else if (selected_algo == AlgorithmChoice::ASTAR_ALT) {
        init_heuristics_if_needed();
        solver = std::make_shared<AStar>(alt_heuristic);
    } else if (selected_algo == AlgorithmChoice::IDASTAR_ALT) {
        init_heuristics_if_needed();
        solver = std::make_shared<IDAStar>(alt_heuristic, 500000, 5000.0);
    }

    solver->refresh(current_graph);
    PathResult new_res = solver->solve(current_graph, current_start, current_goal);

    out << "\nUPDATED ROUTE\n  ";
    if (new_res.status == ExecutionStatus::SUCCESS && !new_res.path.empty()) {
        for (size_t i = 0; i < new_res.path.size(); ++i) {
            out << new_res.path[i];
            if (i + 1 < new_res.path.size()) out << " -> ";
        }
        out << "\nCost: " << std::fixed << std::setprecision(4) << new_res.total_cost << "\n";
    } else {
        out << "[No Valid Route Available]\n";
    }

    out << "\nSearch Time: " << std::fixed << std::setprecision(4) << new_res.search_time_ms << " ms\n";
    if (new_res.preprocessing_time_ms > 0.0) {
        out << "ALT Refresh Time: " << new_res.preprocessing_time_ms << " ms\n";
    }
    out << "Nodes Expanded: " << new_res.nodes_expanded << "\n"
        << "==========================================\n";

    // Update active route
    last_result = new_res;
    last_algo_name = get_algo_display_name(selected_algo);
    route_graph_version = current_graph.get_version();
    route_is_stale = false;
    route_might_be_suboptimal = false;
}

void InteractiveCLI::handle_compare_all() {
    if (!graph_loaded) {
        out << "Error: No graph loaded. Please load or generate a graph first.\n";
        return;
    }

    NodeID start = current_start;
    NodeID goal = current_goal;

    if (!has_active_query) {
        out << "\nNo active route found. Please enter query endpoints:\n";
        if (!get_node_input(start, "Enter source vertex: ")) return;
        if (!get_node_input(goal, "Enter destination vertex: ")) return;
    } else {
        out << "\nComparing algorithms for active endpoints: " << start << " -> " << goal << "\n";
    }

    if (start == goal) {
        out << "Source and destination are identical (Cost = 0.0 for all algorithms).\n";
        return;
    }

    init_heuristics_if_needed();

    // Prepare solvers
    Dijkstra dijkstra;
    auto eucl_h = std::make_shared<EuclideanHeuristic>();
    AStar astar_eucl(eucl_h);
    AStar astar_alt(alt_heuristic);
    IDAStar ida_alt(alt_heuristic, 500000, 5000.0);

    // Refresh ALT on graph state without mutating
    alt_heuristic->refresh(current_graph);
    double alt_refresh_time = alt_heuristic->get_preprocessing_time_ms();

    // Solve
    PathResult d_res = dijkstra.solve(current_graph, start, goal);
    PathResult ae_res = astar_eucl.solve(current_graph, start, goal);
    PathResult aalt_res = astar_alt.solve(current_graph, start, goal);
    PathResult ida_res = ida_alt.solve(current_graph, start, goal);

    out << "\n========================================================================================\n"
        << "                         ALGORITHM PERFORMANCE COMPARISON                               \n"
        << "========================================================================================\n"
        << std::left << std::setw(16) << "Algorithm"
        << std::right << std::setw(12) << "Cost"
        << std::setw(14) << "Search(ms)"
        << std::setw(12) << "Expanded"
        << std::setw(14) << "Memory(KB)"
        << std::setw(18) << "Status/Oracle\n"
        << "----------------------------------------------------------------------------------------\n";

    auto print_row = [&](const std::string& name, const PathResult& r, double oracle_cost) {
        out << std::left << std::setw(16) << name
            << std::right;
        
        if (r.status == ExecutionStatus::SUCCESS && r.total_cost != INF_WEIGHT) {
            out << std::setw(12) << std::fixed << std::setprecision(2) << r.total_cost;
        } else {
            out << std::setw(12) << "N/A";
        }

        out << std::setw(14) << std::fixed << std::setprecision(4) << r.search_time_ms
            << std::setw(12) << r.nodes_expanded
            << std::setw(14) << std::fixed << std::setprecision(2) << r.peak_memory_kb;

        if (r.status == ExecutionStatus::RESOURCE_LIMIT) {
            out << std::setw(18) << "RESOURCE_LIMIT";
        } else if (r.status == ExecutionStatus::TIMEOUT) {
            out << std::setw(18) << "TIMEOUT";
        } else if (r.status == ExecutionStatus::NO_PATH || r.total_cost == INF_WEIGHT) {
            out << std::setw(18) << "NO_PATH";
        } else {
            bool matches = std::abs(r.total_cost - oracle_cost) <= 1e-4;
            out << std::setw(18) << (matches ? "PASS (Optimal)" : "FAIL (Suboptimal)");
        }
        out << "\n";
    };

    print_row("Dijkstra", d_res, d_res.total_cost);
    print_row("A* Euclidean", ae_res, d_res.total_cost);
    print_row("A* ALT", aalt_res, d_res.total_cost);
    print_row("IDA* ALT", ida_res, d_res.total_cost);

    out << "----------------------------------------------------------------------------------------\n";
    out << "ALT Preprocessing / Refresh Time: " << std::fixed << std::setprecision(4) << alt_refresh_time << " ms\n";
    out << "ALT Landmark Table Memory: " << std::fixed << std::setprecision(2) << alt_heuristic->get_memory_kb() << " KB\n";
    out << "========================================================================================\n";
}

void InteractiveCLI::handle_view_current_route() {
    out << "\n--- Current Route Display ---\n";
    if (!has_active_query) {
        out << "No route has been calculated yet. Use Option 4 to find a shortest path.\n";
        return;
    }

    out << "Algorithm Used: " << last_algo_name << "\n"
        << "Endpoints: " << current_start << " -> " << current_goal << "\n"
        << "Route Graph Version: " << route_graph_version << " (Current Graph Version: " << current_graph.get_version() << ")\n";

    if (current_graph.get_version() != route_graph_version) {
        out << "\n[!] STATUS: STALE (Graph modified since route was computed)\n";
        if (route_is_stale) {
            out << "    Reason: An edge on this route was CLOSED. Path is no longer feasible!\n";
        } else if (route_might_be_suboptimal) {
            out << "    Reason: Edge weights decreased or reopened. A shorter detour may exist!\n";
        }
        out << "    Please select Option 8 to Replan Current Route.\n";
    } else {
        out << "\nSTATUS: ACTIVE & OPTIMAL (Synchronized with graph state)\n";
    }

    if (last_result.path.empty()) {
        out << "Path: [No Route Exists]\n";
    } else {
        out << "Total Cost: " << std::fixed << std::setprecision(4) << last_result.total_cost << "\n"
            << "Vertex Sequence (" << last_result.path.size() << " nodes):\n  ";
        for (size_t i = 0; i < last_result.path.size(); ++i) {
            out << last_result.path[i];
            if (i + 1 < last_result.path.size()) out << " -> ";
        }
        out << "\n";
    }

    out << "Search Time: " << std::fixed << std::setprecision(4) << last_result.search_time_ms << " ms\n"
        << "Nodes Expanded: " << last_result.nodes_expanded << "\n"
        << "Peak Search Memory: " << std::setprecision(2) << last_result.peak_memory_kb << " KB\n";
}

void InteractiveCLI::run() {
    while (true) {
        print_main_menu();
        int choice = -1;
        if (!get_int_input(choice, "Enter choice: ")) {
            out << "\nExiting Dynamic Shortest Path Planner. Goodbye!\n";
            break;
        }

        switch (choice) {
            case 1: handle_load_or_generate(); break;
            case 2: handle_display_info(); break;
            case 3: handle_select_algorithm(); break;
            case 4: handle_find_shortest_path(); break;
            case 5: handle_close_edge(); break;
            case 6: handle_reopen_edge(); break;
            case 7: handle_update_edge_weight(); break;
            case 8: handle_replan_route(); break;
            case 9: handle_compare_all(); break;
            case 10: handle_view_current_route(); break;
            case 0:
                out << "\nExiting Dynamic Shortest Path Planner. Goodbye!\n";
                return;
            default:
                out << "Error: Unrecognized option (" << choice << "). Please enter a number between 0 and 10.\n";
                break;
        }
    }
}
