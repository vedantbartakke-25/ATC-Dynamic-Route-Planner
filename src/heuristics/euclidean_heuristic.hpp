#ifndef EUCLIDEAN_HEURISTIC_HPP
#define EUCLIDEAN_HEURISTIC_HPP

#include "heuristic.hpp"

class EuclideanHeuristic : public Heuristic {
public:
    EdgeWeight compute(NodeID u, NodeID goal, const Graph& graph) override {
        return graph.euclidean_distance(u, goal);
    }
};

#endif // EUCLIDEAN_HEURISTIC_HPP
