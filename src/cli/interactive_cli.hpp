#ifndef INTERACTIVE_CLI_HPP
#define INTERACTIVE_CLI_HPP

#include "core/graph.hpp"
#include "algorithms/solver.hpp"
#include "heuristics/alt_heuristic.hpp"
#include "heuristics/euclidean_heuristic.hpp"
#include <memory>
#include <string>
#include <vector>
#include <iostream>

enum class AlgorithmChoice {
    DIJKSTRA,
    ASTAR_EUCLIDEAN,
    ASTAR_ALT,
    IDASTAR_ALT
};

class InteractiveCLI {
private:
    Graph current_graph;
    bool graph_loaded;
    std::string graph_source_desc;
    bool is_undirected;

    // Algorithm configuration
    AlgorithmChoice selected_algo;
    size_t alt_landmarks_count;
    std::shared_ptr<ALTHeuristic> alt_heuristic;
    std::shared_ptr<EuclideanHeuristic> euclidean_heuristic;

    // Route tracking
    bool has_active_query;
    NodeID current_start;
    NodeID current_goal;
    PathResult last_result;
    std::string last_algo_name;
    uint64_t route_graph_version;
    bool route_is_stale;
    bool route_might_be_suboptimal;

    // Streams (configurable for testing)
    std::istream& in;
    std::ostream& out;

    void print_main_menu();
    void handle_load_or_generate();
    void handle_display_info();
    void handle_select_algorithm();
    void handle_find_shortest_path();
    void handle_close_edge();
    void handle_reopen_edge();
    void handle_update_edge_weight();
    void handle_replan_route();
    void handle_compare_all();
    void handle_view_current_route();

    void init_heuristics_if_needed();
    std::string get_algo_display_name(AlgorithmChoice choice) const;
    void print_path_result(const std::string& algo_name, const PathResult& res, NodeID start, NodeID goal);

    // Input helpers
    bool get_int_input(int& val, const std::string& prompt);
    bool get_node_input(NodeID& node, const std::string& prompt);
    bool get_double_input(double& val, const std::string& prompt);

public:
    InteractiveCLI(std::istream& input_stream = std::cin, std::ostream& output_stream = std::cout);

    // Runs the interactive REPL loop
    void run();
};

#endif // INTERACTIVE_CLI_HPP
