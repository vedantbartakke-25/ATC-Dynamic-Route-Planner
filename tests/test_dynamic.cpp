#include <iostream>
#include <cassert>
#include <cmath>
#include <memory>
#include "core/graph.hpp"
#include "generator/graph_generator.hpp"
#include "algorithms/dijkstra.hpp"
#include "dynamic/dynamic_engine.hpp"

bool double_equals(double a, double b) { return std::abs(a - b) < 1e-9; }

void test_dynamic_engine() {
    std::cout << "=== Testing Dynamic Replanning Engine ===\n";
    Graph g;
    g.add_node(0, 0); // 0
    g.add_node(1, 0); // 1
    g.add_node(2, 0); // 2
    g.add_node(1, 1); // 3
    
    // Path 1 (direct but slightly longer conceptually in weights)
    g.add_undirected_edge(0, 1, 1.0);
    g.add_undirected_edge(1, 2, 1.0);
    
    // Path 2 (indirect)
    g.add_undirected_edge(0, 3, 2.0);
    g.add_undirected_edge(3, 2, 2.0);

    auto solver = std::make_shared<Dijkstra>();
    DynamicEngine engine(g, solver);

    // Initial solve
    PathResult res = solver->solve(g, 0, 2);
    assert(double_equals(res.total_cost, 2.0)); // 0 -> 1 -> 2
    
    // 1. Irrelevant edge change
    GraphEvent ev1{10.0, EventType::UPDATE_WEIGHT, 0, 3, 5.0};
    ReplanningResult rep1 = engine.handle_event(ev1, res, 0, 2);
    // Not in path, weight increased -> no replan needed
    assert(!rep1.replanned);

    // 2. Route weight increase
    GraphEvent ev2{20.0, EventType::UPDATE_WEIGHT, 1, 2, 5.0};
    ReplanningResult rep2 = engine.handle_event(ev2, res, 0, 2);
    assert(rep2.replanned);
    assert(double_equals(rep2.new_result.total_cost, 4.0)); // 0 -> 3 -> 2
    res = rep2.new_result; // Update current path

    // 3. Route edge closure (close 0 -> 3)
    GraphEvent ev3{30.0, EventType::CLOSE_EDGE, 0, 3, INF_WEIGHT};
    ReplanningResult rep3 = engine.handle_event(ev3, res, 0, 2);
    assert(rep3.replanned);
    assert(double_equals(rep3.new_result.total_cost, 6.0)); // 0 -> 1 -> 2 (remember 1->2 is now 5.0, so 1+5=6)
    res = rep3.new_result;

    // 4. Route reopening (open an edge / decrease weight that makes a better route)
    GraphEvent ev4{40.0, EventType::UPDATE_WEIGHT, 1, 2, 1.0};
    ReplanningResult rep4 = engine.handle_event(ev4, res, 0, 2);
    // Weight decreased, should replan
    assert(rep4.replanned);
    assert(double_equals(rep4.new_result.total_cost, 2.0));

    std::cout << "All Dynamic Engine tests passed!\n";
}

int main() {
    test_dynamic_engine();
    return 0;
}
