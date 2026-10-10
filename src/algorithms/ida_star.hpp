#ifndef IDA_STAR_HPP
#define IDA_STAR_HPP

#include "solver.hpp"
#include "heuristics/heuristic.hpp"
#include <memory>
#include <vector>

class IDAStar : public Solver {
private:
    std::shared_ptr<Heuristic> heuristic;
    
    // Internal recursive search function for a single iteration
    EdgeWeight search(const Graph& graph, NodeID current, NodeID goal, 
                      EdgeWeight g, EdgeWeight threshold, 
                      std::vector<NodeID>& path, std::vector<EdgeWeight>& min_g,
                      std::vector<bool>& in_path, PathResult& res);

public:
    IDAStar(std::shared_ptr<Heuristic> h) : heuristic(h) {}

    PathResult solve(const Graph& graph, NodeID start, NodeID goal) override;
};

#endif // IDA_STAR_HPP
