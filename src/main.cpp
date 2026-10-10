#include <iostream>
#include <string>
#include <memory>
#include "core/graph.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/ida_star.hpp"
#include "heuristics/euclidean_heuristic.hpp"
#include "heuristics/alt_heuristic.hpp"

void print_help() {
    std::cout << "Dynamic Shortest Path Replanning\n"
              << "Usage: planner [options]\n"
              << "Options:\n"
              << "  --help          Show this help message\n"
              << "  --graph <file>  Specify the graph input file\n"
              << "  --start <id>    Start node ID\n"
              << "  --goal <id>     Goal node ID\n";
}

void print_result(const std::string& name, const PathResult& res, NodeID start_node, NodeID goal_node) {
    std::cout << "\n--- " << name << " ---\n";
    if (res.path.empty() && start_node != goal_node) {
        std::cout << "No path found.\n";
    } else {
        std::cout << "Path found!\n"
                  << "  Cost: " << res.total_cost << "\n"
                  << "  Nodes Generated: " << res.nodes_generated << "\n"
                  << "  Nodes Expanded: " << res.nodes_expanded << "\n";
        if (res.iterations > 0) {
            std::cout << "  Iterations (IDA*): " << res.iterations << "\n";
        }
        std::cout << "  Total Time (ms): " << res.search_time_ms << "\n"
                  << "  Heuristic Time (ms): " << res.heuristic_time_ms << "\n"
                  << "  Path Length: " << res.path.size() << " nodes\n";
    }
}

int main(int argc, char** argv) {
    if (argc == 1) {
        print_help();
        return 0;
    }

    std::string graph_file = "";
    NodeID start_node = INVALID_NODE;
    NodeID goal_node = INVALID_NODE;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") {
            print_help();
            return 0;
        } else if (arg == "--graph" && i + 1 < argc) {
            graph_file = argv[++i];
        } else if (arg == "--start" && i + 1 < argc) {
            start_node = std::stoul(argv[++i]);
        } else if (arg == "--goal" && i + 1 < argc) {
            goal_node = std::stoul(argv[++i]);
        } else {
            std::cerr << "Unknown or incomplete argument: " << arg << "\n";
            print_help();
            return 1;
        }
    }

    if (!graph_file.empty()) {
        Graph g;
        if (!g.load_from_file(graph_file)) {
            std::cerr << "Failed to load graph from " << graph_file << "\n";
            return 1;
        }
        
        g.print_info();
        
        if (start_node != INVALID_NODE && goal_node != INVALID_NODE) {
            std::cout << "\nSolving from " << start_node << " to " << goal_node << "...\n";
            
            Dijkstra dijkstra;
            PathResult d_res = dijkstra.solve(g, start_node, goal_node);
            print_result("Dijkstra", d_res, start_node, goal_node);
            
            auto heuristic_e = std::make_shared<EuclideanHeuristic>();
            AStar astar_e(heuristic_e);
            PathResult a_res_e = astar_e.solve(g, start_node, goal_node);
            print_result("A* (Euclidean)", a_res_e, start_node, goal_node);

            // Using 4 landmarks for quick CLI demo
            auto landmarks = ALTHeuristic::select_landmarks(g, 4);
            auto heuristic_alt = std::make_shared<ALTHeuristic>(g, landmarks);
            AStar astar_alt(heuristic_alt);
            PathResult a_res_alt = astar_alt.solve(g, start_node, goal_node);
            print_result("A* (ALT - 4 LMs)", a_res_alt, start_node, goal_node);

            IDAStar ida_alt(heuristic_alt);
            PathResult ida_res_alt = ida_alt.solve(g, start_node, goal_node);
            print_result("IDA* (ALT - 4 LMs)", ida_res_alt, start_node, goal_node);
        }
    }
    
    return 0;
}
