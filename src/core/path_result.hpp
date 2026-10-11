#ifndef PATH_RESULT_HPP
#define PATH_RESULT_HPP

#include <vector>
#include "types.hpp"

enum class ExecutionStatus {
    SUCCESS,
    NO_PATH,
    TIMEOUT,
    RESOURCE_LIMIT,
    INCORRECT_RESULT,
    ERROR
};

struct PathResult {
    std::vector<NodeID> path;
    EdgeWeight total_cost;
    size_t nodes_expanded;
    size_t nodes_generated;
    size_t iterations;
    double search_time_ms;
    double heuristic_time_ms;
    double preprocessing_time_ms;
    double peak_memory_kb;        // Estimated peak search data structure memory in KB
    ExecutionStatus status;
    
    PathResult() 
        : total_cost(INF_WEIGHT), nodes_expanded(0), nodes_generated(0), iterations(0),
          search_time_ms(0.0), heuristic_time_ms(0.0), preprocessing_time_ms(0.0),
          peak_memory_kb(0.0), status(ExecutionStatus::SUCCESS) {}
};

#endif // PATH_RESULT_HPP
