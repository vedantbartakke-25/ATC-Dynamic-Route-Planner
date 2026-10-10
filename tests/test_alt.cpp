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
#include "heuristics/euclidean_heuristic.hpp"
#include "heuristics/alt_heuristic.hpp"

bool double_equals(double a, double b) {
    return std::abs(a - b) < 1e-9;
}

void print_result(const std::string& name, const PathResult& res) {
    std::cout << std::left << std::setw(20) << name 
              << " | Cost: " << std::setw(8) << res.total_cost 
              << " | Expanded: " << std::setw(6) << res.nodes_expanded
              << " | Gen: " << std::setw(6) << res.nodes_generated
              << " | Search Time: " << std::setw(8) << res.search_time_ms << "ms"
              << " | Heuristic Time: " << std::setw(8) << res.heuristic_time_ms << "ms"
              << " | Preprocess Time: " << std::setw(8) << res.preprocessing_time_ms << "ms\n";
}

void test_alt_correctness_and_experiments() {
    std::cout << "\n=== ALT Correctness & Landmark Experiments ===\n";
    // 30x30 grid = 900 nodes
    Graph g = GraphGenerator::generate_grid(30, 30);
    NodeID start = 0;
    NodeID goal = 899;

    // 1. Dijkstra
    Dijkstra dijkstra;
    PathResult d_res = dijkstra.solve(g, start, goal);
    print_result("Dijkstra", d_res);

    // 2. A* Euclidean
    auto eucl_h = std::make_shared<EuclideanHeuristic>();
    AStar astar_eucl(eucl_h);
    PathResult eucl_res = astar_eucl.solve(g, start, goal);
    assert(double_equals(d_res.total_cost, eucl_res.total_cost));
    print_result("A* Euclidean", eucl_res);

    // 3. A* ALT with different landmark counts
    std::vector<size_t> counts = {2, 4, 8, 16};
    for (size_t k : counts) {
        auto landmarks = ALTHeuristic::select_landmarks(g, k);
        auto alt_h = std::make_shared<ALTHeuristic>(g, landmarks);
        AStar astar_alt(alt_h);
        PathResult alt_res = astar_alt.solve(g, start, goal);
        
        // Correctness check
        assert(double_equals(d_res.total_cost, alt_res.total_cost));
        
        print_result("A* ALT (" + std::to_string(k) + " LMs)", alt_res);
    }
    std::cout << "All correctness assertions passed.\n\n";
}

int main() {
    test_alt_correctness_and_experiments();
    std::cout << "All Phase 4 ALT tests passed!\n";
    return 0;
}
