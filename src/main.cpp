#include <iostream>
#include <string>
#include "core/graph.hpp"
#include "algorithms/dijkstra.hpp"

void print_help() {
    std::cout << "Dynamic Shortest Path Replanning\n"
              << "Usage: planner [options]\n"
              << "Options:\n"
              << "  --help          Show this help message\n"
              << "  --graph <file>  Specify the graph input file\n"
              << "  --start <id>    Start node ID\n"
              << "  --goal <id>     Goal node ID\n";
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
            Dijkstra solver;
            std::cout << "\nRunning Dijkstra from " << start_node << " to " << goal_node << "...\n";
            PathResult res = solver.solve(g, start_node, goal_node);
            
            if (res.path.empty() && start_node != goal_node) {
                std::cout << "No path found.\n";
            } else {
                std::cout << "Path found!\n"
                          << "  Cost: " << res.total_cost << "\n"
                          << "  Nodes Generated: " << res.nodes_generated << "\n"
                          << "  Nodes Expanded: " << res.nodes_expanded << "\n"
                          << "  Time (ms): " << res.search_time_ms << "\n"
                          << "  Path Length: " << res.path.size() << " nodes\n";
                std::cout << "  Path: ";
                for (NodeID n : res.path) std::cout << n << " ";
                std::cout << "\n";
            }
        }
    }
    
    return 0;
}
