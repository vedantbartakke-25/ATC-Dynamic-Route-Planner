#include <iostream>
#include <cassert>
#include <cmath>
#include "core/graph.hpp"
#include "generator/graph_generator.hpp"

void test_graph_creation() {
    Graph g;
    NodeID n0 = g.add_node(0.0, 0.0);
    NodeID n1 = g.add_node(3.0, 4.0);
    
    assert(g.get_num_nodes() == 2);
    
    g.add_edge(n0, n1, 5.0);
    const auto& neighbors = g.get_neighbors(n0);
    assert(neighbors.size() == 1);
    assert(neighbors[0].target == n1);
    assert(neighbors[0].weight == 5.0);
    assert(neighbors[0].is_open == true);
    
    // Test euclidean distance
    double dist = g.euclidean_distance(n0, n1);
    assert(std::abs(dist - 5.0) < 1e-9);
    
    std::cout << "test_graph_creation passed." << std::endl;
}

void test_graph_generator() {
    Graph g = GraphGenerator::generate_grid(3, 3);
    assert(g.get_num_nodes() == 9);
    
    // Node 0 should have 3 neighbors in a 3x3 grid (right, bottom, bottom-right diag)
    assert(g.get_neighbors(0).size() == 3);
    // Node 4 (center) should have 8 neighbors
    assert(g.get_neighbors(4).size() == 8);
    
    std::cout << "test_graph_generator passed." << std::endl;
}

int main() {
    test_graph_creation();
    test_graph_generator();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
