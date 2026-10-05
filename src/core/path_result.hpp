#ifndef PATH_RESULT_HPP
#define PATH_RESULT_HPP

#include <vector>
#include "types.hpp"

struct PathResult {
    std::vector<NodeID> path;
    EdgeWeight total_cost;
    size_t nodes_expanded;
    size_t nodes_generated;
    double search_time_ms;
    
    PathResult() 
        : total_cost(INF_WEIGHT), nodes_expanded(0), nodes_generated(0), search_time_ms(0.0) {}
};

#endif // PATH_RESULT_HPP
