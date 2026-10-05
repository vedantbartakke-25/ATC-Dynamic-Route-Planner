#include <iostream>
#include <string>
#include "core/graph.hpp"

void print_help() {
    std::cout << "Dynamic Shortest Path Replanning\n"
              << "Usage: planner [options]\n"
              << "Options:\n"
              << "  --help          Show this help message\n"
              << "  --graph <file>  Specify the graph input file (not implemented yet)\n"
              << "  --start <id>    Start node ID\n"
              << "  --goal <id>     Goal node ID\n"
              << "\nPhase 0 Complete. Algorithms not yet implemented.\n";
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

    std::cout << "Initialization successful.\n";
    if (!graph_file.empty()) {
        std::cout << "Graph file: " << graph_file << "\n";
    }
    
    return 0;
}
