#include <iostream>
#include <cassert>
#include <cmath>
#include "core/graph.hpp"

void test_valid_graph() {
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
    assert(neighbors[0].source == n0);
    
    double dist = g.euclidean_distance(n0, n1);
    assert(std::abs(dist - 5.0) < 1e-9);
    
    std::cout << "test_valid_graph passed.\n";
}

void test_disconnected_graph() {
    Graph g;
    g.add_node(0.0, 0.0);
    g.add_node(1.0, 1.0);
    
    assert(g.get_neighbors(0).empty());
    assert(g.get_neighbors(1).empty());
    std::cout << "test_disconnected_graph passed.\n";
}

void test_weight_update() {
    Graph g;
    g.add_node(0.0, 0.0);
    g.add_node(1.0, 1.0);
    g.add_edge(0, 1, 10.0);
    
    g.update_edge_weight(0, 1, 15.0);
    assert(g.get_neighbors(0)[0].weight == 15.0);
    std::cout << "test_weight_update passed.\n";
}

void test_edge_closure_and_reopening() {
    Graph g;
    g.add_node(0.0, 0.0);
    g.add_node(1.0, 1.0);
    g.add_edge(0, 1, 10.0);
    
    g.set_edge_status(0, 1, false);
    assert(g.get_neighbors(0)[0].is_open == false);
    
    g.set_edge_status(0, 1, true);
    assert(g.get_neighbors(0)[0].is_open == true);
    std::cout << "test_edge_closure_and_reopening passed.\n";
}

void test_invalid_node_access() {
    Graph g;
    g.add_node(0.0, 0.0);
    
    bool caught = false;
    try {
        g.add_edge(0, 5, 10.0); // 5 is invalid
    } catch (const std::out_of_range&) {
        caught = true;
    }
    assert(caught);
    
    caught = false;
    try {
        g.get_neighbors(5);
    } catch (const std::out_of_range&) {
        caught = true;
    }
    assert(caught);
    
    std::cout << "test_invalid_node_access passed.\n";
}

void test_remove_edge() {
    Graph g;
    g.add_node(0.0, 0.0);
    g.add_node(1.0, 1.0);
    g.add_edge(0, 1, 10.0);
    
    bool removed = g.remove_edge(0, 1);
    assert(removed);
    assert(g.get_neighbors(0).empty());
    
    removed = g.remove_edge(0, 1); // Already removed
    assert(!removed);
    
    std::cout << "test_remove_edge passed.\n";
}

void test_load_graph() {
    Graph g;
    bool success = g.load_from_file("data/sample_graph.txt");
    assert(success);
    assert(g.get_num_nodes() == 4);
    assert(g.get_neighbors(0).size() == 1);
    
    g.print_info();
    std::cout << "test_load_graph passed.\n";
}

int main() {
    test_valid_graph();
    test_disconnected_graph();
    test_weight_update();
    test_edge_closure_and_reopening();
    test_invalid_node_access();
    test_remove_edge();
    test_load_graph();
    
    std::cout << "All Phase 1 tests passed!\n";
    return 0;
}
