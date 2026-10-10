#include <iostream>
#include <cassert>
#include <cmath>
#include "core/graph.hpp"
#include "generator/graph_generator.hpp"
#include "algorithms/dijkstra.hpp"

void test_simple_graph() {
    Graph g;
    g.add_node(0, 0); // 0
    g.add_node(1, 0); // 1
    g.add_node(2, 0); // 2
    g.add_edge(0, 1, 1.0);
    g.add_edge(1, 2, 2.0);

    Dijkstra solver;
    PathResult res = solver.solve(g, 0, 2);
    
    assert(res.path.size() == 3);
    assert(res.path[0] == 0 && res.path[1] == 1 && res.path[2] == 2);
    assert(std::abs(res.total_cost - 3.0) < 1e-9);
    
    std::cout << "test_simple_graph passed.\n";
}

void test_multiple_paths() {
    Graph g;
    g.add_node(0, 0); // 0 (Start)
    g.add_node(1, 1); // 1 (Top path)
    g.add_node(1, -1); // 2 (Bottom path)
    g.add_node(2, 0); // 3 (Goal)

    g.add_edge(0, 1, 1.0);
    g.add_edge(1, 3, 5.0); // Cost: 6.0
    
    g.add_edge(0, 2, 2.0);
    g.add_edge(2, 3, 2.0); // Cost: 4.0

    Dijkstra solver;
    PathResult res = solver.solve(g, 0, 3);
    
    assert(res.path.size() == 3);
    assert(res.path[1] == 2); // Should take the bottom path
    assert(std::abs(res.total_cost - 4.0) < 1e-9);

    std::cout << "test_multiple_paths passed.\n";
}

void test_disconnected_graph() {
    Graph g;
    g.add_node(0, 0);
    g.add_node(1, 0);
    g.add_node(2, 0);
    g.add_node(3, 0);

    g.add_edge(0, 1, 1.0);
    g.add_edge(2, 3, 1.0);

    Dijkstra solver;
    PathResult res = solver.solve(g, 0, 3);
    
    assert(res.path.empty());
    assert(res.total_cost == INF_WEIGHT);

    std::cout << "test_disconnected_graph passed.\n";
}

void test_source_equals_target() {
    Graph g;
    g.add_node(0, 0);

    Dijkstra solver;
    PathResult res = solver.solve(g, 0, 0);
    
    assert(res.path.size() == 1);
    assert(res.path[0] == 0);
    assert(res.total_cost == 0.0);

    std::cout << "test_source_equals_target passed.\n";
}

void test_multiple_equal_cost_paths() {
    Graph g;
    g.add_node(0, 0); // 0
    g.add_node(1, 1); // 1
    g.add_node(1, -1); // 2
    g.add_node(2, 0); // 3
    
    g.add_edge(0, 1, 1.0);
    g.add_edge(1, 3, 1.0);
    
    g.add_edge(0, 2, 1.0);
    g.add_edge(2, 3, 1.0);
    
    Dijkstra solver;
    PathResult res = solver.solve(g, 0, 3);
    
    assert(res.path.size() == 3);
    assert(std::abs(res.total_cost - 2.0) < 1e-9);
    // Path could go through 1 or 2, both are valid and cost is 2.0
    assert(res.path[1] == 1 || res.path[1] == 2);

    std::cout << "test_multiple_equal_cost_paths passed.\n";
}

void test_large_generated_graph() {
    // Generate a 10x10 grid (100 nodes)
    Graph g = GraphGenerator::generate_grid(10, 10);
    
    Dijkstra solver;
    // Solve from top-left (0) to bottom-right (99)
    PathResult res = solver.solve(g, 0, 99);
    
    assert(!res.path.empty());
    assert(res.total_cost > 0.0);
    assert(res.nodes_expanded > 0);
    
    std::cout << "test_large_generated_graph passed.\n";
}

int main() {
    test_simple_graph();
    test_multiple_paths();
    test_disconnected_graph();
    test_source_equals_target();
    test_multiple_equal_cost_paths();
    test_large_generated_graph();

    std::cout << "All Phase 2 Dijkstra tests passed!\n";
    return 0;
}
