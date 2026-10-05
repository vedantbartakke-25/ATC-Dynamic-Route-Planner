#ifndef SOLVER_HPP
#define SOLVER_HPP

#include "core/graph.hpp"
#include "core/path_result.hpp"

class Solver {
public:
    virtual ~Solver() = default;
    
    // Solves the shortest path from start to goal
    virtual PathResult solve(const Graph& graph, NodeID start, NodeID goal) = 0;
};

#endif // SOLVER_HPP
