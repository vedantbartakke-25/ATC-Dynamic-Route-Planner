#include <iostream>
#include <cassert>
#include <cmath>
#include <memory>
#include "core/graph.hpp"
#include "generator/graph_generator.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/astar.hpp"
#include "heuristics/euclidean_heuristic.hpp"

// Utility to compare float values safely
bool double_equals(double a, double b) {
    return std::abs(a - b) < 1e-9;
}

void test_simple_graph() {
    Graph g;
    g.add_node(0, 0); // 0
    g.add_node(1, 0); // 1
    g.add_node(2, 0); // 2
    g.add_edge(0, 1, 1.0);
    g.add_edge(1, 2, 2.0); // Cost is 2.0, euclidean is 1.0 (admissible since 2.0 >= 1.0)

    auto heuristic = std::make_shared<EuclideanHeuristic>();
    AStar solver(heuristic);
    PathResult res = solver.solve(g, 0, 2);
    
    assert(res.path.size() == 3);
    assert(double_equals(res.total_cost, 3.0));
    std::cout << "test_simple_graph passed.\n";
}

void test_direct_vs_indirect_path() {
    Graph g;
    g.add_node(0, 0); // 0
    g.add_node(0, 2); // 1
    g.add_node(2, 2); // 2
    g.add_node(2, 0); // 3

    // Direct diagonal edge
    g.add_edge(0, 2, 5.0); // Non-optimal cost (euclidean is ~2.828, admissible)
    
    // Indirect path
    g.add_edge(0, 1, 2.0);
    g.add_edge(1, 2, 2.0); // Cost 4.0

    auto heuristic = std::make_shared<EuclideanHeuristic>();
    AStar solver(heuristic);
    PathResult res = solver.solve(g, 0, 2);
    
    assert(res.path.size() == 3); // 0 -> 1 -> 2
    assert(double_equals(res.total_cost, 4.0));
    std::cout << "test_direct_vs_indirect_path passed.\n";
}

void test_disconnected_graph() {
    Graph g;
    g.add_node(0, 0);
    g.add_node(1, 0);
    g.add_node(2, 0);
    g.add_edge(0, 1, 1.0);

    auto heuristic = std::make_shared<EuclideanHeuristic>();
    AStar solver(heuristic);
    PathResult res = solver.solve(g, 0, 2);
    
    assert(res.path.empty());
    assert(res.total_cost == INF_WEIGHT);
    std::cout << "test_disconnected_graph passed.\n";
}

void test_source_equals_target() {
    Graph g;
    g.add_node(0, 0);

    auto heuristic = std::make_shared<EuclideanHeuristic>();
    AStar solver(heuristic);
    PathResult res = solver.solve(g, 0, 0);
    
    assert(res.path.size() == 1);
    assert(double_equals(res.total_cost, 0.0));
    std::cout << "test_source_equals_target passed.\n";
}

void test_comparative_dijkstra_vs_astar() {
    // Generate a 20x20 grid (400 nodes)
    Graph g = GraphGenerator::generate_grid(20, 20);
    
    Dijkstra dijkstra;
    PathResult d_res = dijkstra.solve(g, 0, 399);
    
    auto heuristic = std::make_shared<EuclideanHeuristic>();
    AStar astar(heuristic);
    PathResult a_res = astar.solve(g, 0, 399);
    
    // Verify Dijkstra vs A* paths cost the same
    assert(!d_res.path.empty());
    assert(double_equals(d_res.total_cost, a_res.total_cost));
    
    std::cout << "test_comparative_dijkstra_vs_astar passed.\n";
    std::cout << "  Dijkstra Cost: " << d_res.total_cost 
              << ", Expanded: " << d_res.nodes_expanded 
              << ", Search Time: " << d_res.search_time_ms << "ms\n";
    std::cout << "  A* Cost:       " << a_res.total_cost 
              << ", Expanded: " << a_res.nodes_expanded 
              << ", Search Time: " << a_res.search_time_ms << "ms"
              << " (Heuristic: " << a_res.heuristic_time_ms << "ms)\n";
}

int main() {
    test_simple_graph();
    test_direct_vs_indirect_path();
    test_disconnected_graph();
    test_source_equals_target();
    test_comparative_dijkstra_vs_astar();
    
    std::cout << "All Phase 3 A* tests passed!\n";
    return 0;
}
