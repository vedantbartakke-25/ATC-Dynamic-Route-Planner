#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include "algorithms/solver.hpp"

class Dijkstra : public Solver {
public:
    PathResult solve(const Graph& graph, NodeID start, NodeID goal) override;
};

#endif // DIJKSTRA_HPP
