#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP

#include "core/types.hpp"
#include "core/graph.hpp"

class Heuristic {
public:
    virtual ~Heuristic() = default;

    // Computes the heuristic value from 'u' to 'goal'
    virtual EdgeWeight compute(NodeID u, NodeID goal, const Graph& graph) = 0;
};

#endif // HEURISTIC_HPP
