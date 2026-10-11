#ifndef IDA_STAR_HPP
#define IDA_STAR_HPP

#include "solver.hpp"
#include "heuristics/heuristic.hpp"
#include <memory>
#include <vector>
#include <chrono>

class IDAStar : public Solver {
private:
    std::shared_ptr<Heuristic> heuristic;
    size_t max_node_expansions;
    double time_limit_ms;
    
    // Internal recursive search function for a single iteration
    EdgeWeight search(const Graph& graph, NodeID current, NodeID goal, 
                      EdgeWeight g, EdgeWeight threshold, 
                      std::vector<NodeID>& path, std::vector<EdgeWeight>& min_g,
                      std::vector<bool>& in_path, PathResult& res,
                      const std::chrono::steady_clock::time_point& deadline,
                      size_t& max_depth);

public:
    IDAStar(std::shared_ptr<Heuristic> h, size_t max_expansions = 1000000, double timeout_ms = 10000.0) 
        : heuristic(h), max_node_expansions(max_expansions), time_limit_ms(timeout_ms) {}

    void set_limits(size_t max_expansions, double timeout_ms) {
        max_node_expansions = max_expansions;
        time_limit_ms = timeout_ms;
    }

    void refresh(const Graph& graph) override {
        if (heuristic) heuristic->refresh(graph);
    }

    PathResult solve(const Graph& graph, NodeID start, NodeID goal) override;
};

#endif // IDA_STAR_HPP
