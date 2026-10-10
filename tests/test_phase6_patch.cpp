#include <iostream>
#include <cassert>
#include <cmath>
#include <memory>
#include <vector>
#include "core/graph.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/ida_star.hpp"
#include "heuristics/alt_heuristic.hpp"
#include "heuristics/euclidean_heuristic.hpp"
#include "dynamic/dynamic_engine.hpp"

bool double_equals(double a, double b) { return std::abs(a - b) < 1e-9; }

void run_cache_tests() {
    std::cout << "=== Testing Phase 6 Correctness Patch ===\n";

    Graph g;
    g.add_node(0, 0);
    g.add_node(10, 0);
    g.add_node(20, 0);
    g.add_undirected_edge(0, 1, 10.0);
    g.add_undirected_edge(1, 2, 10.0);
    g.add_undirected_edge(0, 2, 30.0);

    auto landmarks = ALTHeuristic::select_landmarks(g, 1);
    auto alt = std::make_shared<ALTHeuristic>(g, landmarks);
    auto astar = std::make_shared<AStar>(alt);
    auto ida = std::make_shared<IDAStar>(alt);
    auto dijkstra = std::make_shared<Dijkstra>();

    DynamicEngine dyn_astar(g, astar);
    DynamicEngine dyn_ida(g, ida);
    DynamicEngine dyn_dijk(g, dijkstra);

    // Initial Path
    PathResult res_d = dijkstra->solve(g, 0, 2);
    assert(double_equals(res_d.total_cost, 20.0));
    
    // Test 1: Multiple queries without graph updates (should use 0 ms for preprocessing)
    astar->refresh(g);
    PathResult res_a1 = astar->solve(g, 0, 2);
    assert(res_a1.preprocessing_time_ms == 0.0);
    assert(double_equals(res_a1.total_cost, 20.0));

    // Test 2: Edge weight increase
    GraphEvent ev1{1.0, EventType::UPDATE_WEIGHT, 0, 1, 15.0};
    ReplanningResult rep_d1 = dyn_dijk.handle_event(ev1, res_d, 0, 2);
    ReplanningResult rep_a1 = dyn_astar.handle_event(ev1, res_a1, 0, 2);
    assert(rep_a1.replanned);
    assert(double_equals(rep_a1.new_result.total_cost, 25.0));

    // Test 3: Multiple updates before one query
    GraphEvent ev2{2.0, EventType::CLOSE_EDGE, 1, 2, INF_WEIGHT};
    dyn_dijk.handle_event(ev2, rep_d1.new_result, 0, 2);
    // don't query astar yet, apply another
    GraphEvent ev3{3.0, EventType::UPDATE_WEIGHT, 0, 2, 25.0};
    ReplanningResult rep_a3 = dyn_astar.handle_event(ev3, rep_a1.new_result, 0, 2);
    assert(double_equals(rep_a3.new_result.total_cost, 25.0)); // Direct path is now 25

    // Test 4: Unreachable landmarks handling
    // If we disconnect the graph, ALT should gracefully handle INF
    GraphEvent ev4{4.0, EventType::CLOSE_EDGE, 0, 2, INF_WEIGHT};
    ReplanningResult rep_a4 = dyn_astar.handle_event(ev4, rep_a3.new_result, 0, 2);
    assert(rep_a4.new_result.total_cost == INF_WEIGHT);

    // Test 5: Graph replacement (Different object, same version)
    Graph g2;
    g2.add_node(0, 0); g2.add_node(5, 5); g2.add_undirected_edge(0, 1, 10.0);
    astar->refresh(g2); // Should trigger refresh because g2 has different ID
    PathResult res_g2 = astar->solve(g2, 0, 1);
    assert(double_equals(res_g2.total_cost, 10.0));

    std::cout << "All Phase 6 Correctness Patch tests passed!\n";
}

int main() {
    run_cache_tests();
    return 0;
}
