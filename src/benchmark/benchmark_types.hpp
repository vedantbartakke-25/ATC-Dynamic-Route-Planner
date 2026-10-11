#ifndef BENCHMARK_TYPES_HPP
#define BENCHMARK_TYPES_HPP

#include <string>
#include <vector>
#include "core/types.hpp"
#include "core/path_result.hpp"

enum class BenchmarkStatus {
    SUCCESS,
    NO_PATH,
    TIMEOUT,
    RESOURCE_LIMIT,
    INCORRECT_RESULT,
    ERROR
};

inline std::string status_to_string(BenchmarkStatus s) {
    switch (s) {
        case BenchmarkStatus::SUCCESS: return "SUCCESS";
        case BenchmarkStatus::NO_PATH: return "NO_PATH";
        case BenchmarkStatus::TIMEOUT: return "TIMEOUT";
        case BenchmarkStatus::RESOURCE_LIMIT: return "RESOURCE_LIMIT";
        case BenchmarkStatus::INCORRECT_RESULT: return "INCORRECT_RESULT";
        case BenchmarkStatus::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

inline BenchmarkStatus execution_status_to_benchmark(ExecutionStatus s) {
    switch (s) {
        case ExecutionStatus::SUCCESS: return BenchmarkStatus::SUCCESS;
        case ExecutionStatus::NO_PATH: return BenchmarkStatus::NO_PATH;
        case ExecutionStatus::TIMEOUT: return BenchmarkStatus::TIMEOUT;
        case ExecutionStatus::RESOURCE_LIMIT: return BenchmarkStatus::RESOURCE_LIMIT;
        case ExecutionStatus::INCORRECT_RESULT: return BenchmarkStatus::INCORRECT_RESULT;
        case ExecutionStatus::ERROR: return BenchmarkStatus::ERROR;
        default: return BenchmarkStatus::ERROR;
    }
}

struct ExperimentMetadata {
    std::string experiment_id;
    size_t query_id;
    std::string algorithm;
    size_t graph_size;
    size_t actual_edges;
    std::string density;
    std::string graph_type;
    NodeID source;
    NodeID destination;
    unsigned int random_seed;
    size_t landmark_count;
    size_t repetition;
    std::string dynamic_event_type;
};

struct PerformanceMetrics {
    double initial_preprocessing_ms;
    double alt_refresh_ms;
    double heuristic_eval_ms;
    double search_time_ms;
    double total_execution_ms;
    double replanning_time_ms;
    size_t nodes_generated;
    size_t nodes_expanded;
    double peak_search_memory_kb;     // Estimated search data structures
    double landmark_table_memory_kb;  // Measured distance table memory
    EdgeWeight path_cost;
    size_t alt_refresh_count;
    size_t ida_iterations;
    BenchmarkStatus status;
    std::string error_message;
};

struct BenchmarkRecord {
    ExperimentMetadata metadata;
    PerformanceMetrics metrics;
};

struct SummaryRecord {
    std::string experiment_id;
    std::string algorithm;
    size_t graph_size;
    std::string density;
    size_t landmark_count;
    std::string dynamic_event_type;
    size_t sample_count;
    
    // Median values
    double median_search_time_ms;
    double median_total_time_ms;
    double median_nodes_expanded;
    double median_nodes_generated;
    double median_search_memory_kb;
    double median_landmark_memory_kb;
    
    // Mean & standard deviation
    double mean_search_time_ms;
    double stddev_search_time_ms;
    double mean_total_time_ms;
    double stddev_total_time_ms;
    double mean_nodes_expanded;
    double stddev_nodes_expanded;
    
    // Verification
    size_t success_count;
    size_t failure_count;
};

struct CorrectnessRecord {
    std::string experiment_id;
    size_t query_id;
    std::string algorithm;
    size_t graph_size;
    NodeID source;
    NodeID destination;
    EdgeWeight reference_cost; // From Dijkstra oracle
    EdgeWeight algorithm_cost;
    double cost_difference;
    bool passed;
    std::string notes;
};

struct BenchmarkConfig {
    std::string experiment_selection = "all"; // "1", "2", ..., "7", or "all"
    bool smoke_test = false;
    std::string output_dir = "results/benchmarks";
    size_t repetitions = 5;
    unsigned int random_seed = 42;
    size_t query_count = 5;
    double ida_timeout_ms = 5000.0;
    size_t ida_max_expansions = 500000;
};

#endif // BENCHMARK_TYPES_HPP
