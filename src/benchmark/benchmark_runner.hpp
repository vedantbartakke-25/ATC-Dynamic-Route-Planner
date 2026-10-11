#ifndef BENCHMARK_RUNNER_HPP
#define BENCHMARK_RUNNER_HPP

#include "benchmark_types.hpp"
#include <vector>

class BenchmarkRunner {
private:
    BenchmarkConfig config;
    std::vector<BenchmarkRecord> raw_records;
    std::vector<SummaryRecord> summary_records;
    std::vector<CorrectnessRecord> correctness_records;

    void compute_summaries();

public:
    BenchmarkRunner(const BenchmarkConfig& cfg) : config(cfg) {}

    void run_benchmarks();

    const std::vector<BenchmarkRecord>& get_raw_records() const { return raw_records; }
    const std::vector<SummaryRecord>& get_summaries() const { return summary_records; }
    const std::vector<CorrectnessRecord>& get_correctness_records() const { return correctness_records; }

    bool export_csv_results() const;
};

#endif // BENCHMARK_RUNNER_HPP
