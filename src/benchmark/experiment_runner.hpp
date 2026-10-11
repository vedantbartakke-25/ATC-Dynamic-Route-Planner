#ifndef EXPERIMENT_RUNNER_HPP
#define EXPERIMENT_RUNNER_HPP

#include "benchmark_types.hpp"
#include <vector>

class ExperimentRunner {
public:
    // Experiment 1: Graph Size Scaling (100, 500, 1000, 5000, 10000)
    static void run_experiment_1(const BenchmarkConfig& config, 
                                 std::vector<BenchmarkRecord>& raw_records,
                                 std::vector<CorrectnessRecord>& correctness_records);

    // Experiment 2: Graph Density (Sparse, Medium, Dense)
    static void run_experiment_2(const BenchmarkConfig& config, 
                                 std::vector<BenchmarkRecord>& raw_records,
                                 std::vector<CorrectnessRecord>& correctness_records);

    // Experiment 3: Heuristic Quality (Dijkstra vs A* Euclidean vs A* ALT)
    static void run_experiment_3(const BenchmarkConfig& config, 
                                 std::vector<BenchmarkRecord>& raw_records,
                                 std::vector<CorrectnessRecord>& correctness_records);

    // Experiment 4: Landmark Count (2, 4, 8, 16)
    static void run_experiment_4(const BenchmarkConfig& config, 
                                 std::vector<BenchmarkRecord>& raw_records,
                                 std::vector<CorrectnessRecord>& correctness_records);

    // Experiment 5: A* ALT vs IDA* ALT (Memory-Time Tradeoff)
    static void run_experiment_5(const BenchmarkConfig& config, 
                                 std::vector<BenchmarkRecord>& raw_records,
                                 std::vector<CorrectnessRecord>& correctness_records);

    // Experiment 6: Dynamic Replanning (Batch & Sequential updates; 10%, 25%, 50% perturbations)
    static void run_experiment_6(const BenchmarkConfig& config, 
                                 std::vector<BenchmarkRecord>& raw_records,
                                 std::vector<CorrectnessRecord>& correctness_records);

    // Experiment 7: ALT Break-Even Analysis (1, 10, 50, 100, 500, 1000 queries)
    static void run_experiment_7(const BenchmarkConfig& config, 
                                 std::vector<BenchmarkRecord>& raw_records,
                                 std::vector<CorrectnessRecord>& correctness_records);
};

#endif // EXPERIMENT_RUNNER_HPP
