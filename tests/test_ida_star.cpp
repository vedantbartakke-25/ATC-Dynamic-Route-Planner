#include <iostream>
#include <cassert>
#include <cmath>
#include <memory>
#include <vector>
#include <iomanip>
#include "core/graph.hpp"
#include "generator/graph_generator.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/ida_star.hpp"
#include "heuristics/alt_heuristic.hpp"

bool double_equals(double a, double b) {
    return std::abs(a - b) < 1e-9;
}

void print_result(const std::string& name, const PathResult& res) {
    std::cout << std::left << std::setw(20) << name 
              << " | Cost: " << std::setw(8) << res.total_cost 
              << " | Expanded: " << std::setw(6) << res.nodes_expanded
              << " | Gen: " << std::setw(6) << res.nodes_generated
              << " | Iters: " << std::setw(4) << res.iterations
              << " | Search Time: " << std::setw(8) << res.search_time_ms << "ms"
              << " | Preprocess Time: " << std::setw(8) << res.preprocessing_time_ms << "ms\n";
}

void test_small_graphs() {
    std::cout << "Testing small graphs for IDA* correctness...\n";
    Graph g;
    g.add_node(0, 0); // 0
    g.add_node(1, 1); // 1
    g.add_node(2, 0); // 2
    g.add_edge(0, 1, 1.0);
    g.add_edge(1, 2, 1.0);
    g.add_edge(0, 2, 3.0); // Suboptimal

    auto landmarks = ALTHeuristic::select_landmarks(g, 2);
    auto alt_h = std::make_shared<ALTHeuristic>(g, landmarks);

    Dijkstra dijkstra;
    PathResult d_res = dijkstra.solve(g, 0, 2);

    IDAStar ida(alt_h);
    PathResult ida_res = ida.solve(g, 0, 2);

    assert(double_equals(d_res.total_cost, ida_res.total_cost));
    std::cout << "Small graph correct.\n";
}

void test_comparative() {
    std::cout << "\n=== IDA* ALT vs A* ALT vs Dijkstra ===\n";
    // 15x15 grid (225 nodes). IDA* can be slower so we keep it relatively small for the test
    Graph g = GraphGenerator::generate_grid(15, 15);
    NodeID start = 0;
    NodeID goal = 224;

    // 1. Dijkstra
    Dijkstra dijkstra;
    PathResult d_res = dijkstra.solve(g, start, goal);
    print_result("Dijkstra", d_res);

    auto landmarks = ALTHeuristic::select_landmarks(g, 4);
    auto alt_h = std::make_shared<ALTHeuristic>(g, landmarks);

    // 2. A* ALT
    AStar astar_alt(alt_h);
    PathResult a_res = astar_alt.solve(g, start, goal);
    assert(double_equals(d_res.total_cost, a_res.total_cost));
    print_result("A* ALT (4 LMs)", a_res);

    // 3. IDA* ALT
    IDAStar ida_alt(alt_h);
    PathResult ida_res = ida_alt.solve(g, start, goal);
    assert(double_equals(d_res.total_cost, ida_res.total_cost));
    print_result("IDA* ALT (4 LMs)", ida_res);

    std::cout << "\nComparison complete. Notice IDA* expands more nodes due to iterations but uses less peak memory (O(depth)).\n";
}

int main() {
    test_small_graphs();
    test_comparative();
    
    std::cout << "\nAll Phase 5 IDA* tests passed!\n";
    return 0;
}
