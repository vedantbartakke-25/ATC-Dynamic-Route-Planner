#ifndef ASTAR_HPP
#define ASTAR_HPP

#include "solver.hpp"
#include "heuristics/heuristic.hpp"
#include <memory>

class AStar : public Solver {
private:
    std::shared_ptr<Heuristic> heuristic;

public:
    AStar(std::shared_ptr<Heuristic> h) : heuristic(h) {}

    PathResult solve(const Graph& graph, NodeID start, NodeID goal) override;
};

#endif // ASTAR_HPP
