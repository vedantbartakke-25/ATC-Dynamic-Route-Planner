#include "benchmark_runner.hpp"
#include "experiment_runner.hpp"
#include "csv_exporter.hpp"
#include <iostream>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <map>

namespace {
    double calculate_median(std::vector<double> vals) {
        if (vals.empty()) return 0.0;
        std::sort(vals.begin(), vals.end());
        size_t n = vals.size();
        if (n % 2 == 1) return vals[n / 2];
        return (vals[n / 2 - 1] + vals[n / 2]) / 2.0;
    }

    double calculate_mean(const std::vector<double>& vals) {
        if (vals.empty()) return 0.0;
        double sum = std::accumulate(vals.begin(), vals.end(), 0.0);
        return sum / vals.size();
    }

    double calculate_stddev(const std::vector<double>& vals, double mean) {
        if (vals.size() <= 1) return 0.0;
        double sum_sq = 0.0;
        for (double v : vals) {
            sum_sq += (v - mean) * (v - mean);
        }
        return std::sqrt(sum_sq / (vals.size() - 1));
    }
}

void BenchmarkRunner::run_benchmarks() {
    std::cout << "\n========================================\n";
    std::cout << "      STARTING DAA BENCHMARK SUITE       \n";
    std::cout << "========================================\n";
    std::cout << "Mode: " << (config.smoke_test ? "SMOKE TEST (Fast Verification - 1 Repetition)" : "FULL BENCHMARK (Standard - >=5 Repetitions)") << "\n";
    std::cout << "Random Seed: " << config.random_seed << "\n";
    std::cout << "Repetitions Configured: " << (config.smoke_test ? 1 : config.repetitions) << "\n";
    std::cout << "Selected Experiments: " << config.experiment_selection << "\n\n";

    raw_records.clear();
    correctness_records.clear();
    summary_records.clear();

    std::string sel = config.experiment_selection;

    // If running a subset of experiments, preserve existing valid experiment data
    if (sel != "all") {
        std::string raw_path = config.output_dir + "/raw_results.csv";
        std::string correctness_path = config.output_dir + "/correctness_results.csv";
        
        std::vector<BenchmarkRecord> existing_raw;
        std::vector<CorrectnessRecord> existing_correctness;
        
        if (CSVExporter::import_raw_results(raw_path, existing_raw)) {
            for (const auto& r : existing_raw) {
                // If running "6" or "6,7", keep records that don't belong to the selected experiment
                bool is_selected = false;
                if ((sel == "6" || sel == "6,7") && r.metadata.experiment_id == "EXP6_DYNAMIC") is_selected = true;
                if ((sel == "7" || sel == "6,7") && r.metadata.experiment_id == "EXP7_BREAKEVEN") is_selected = true;
                if (sel == "1" && r.metadata.experiment_id == "EXP1_SIZE") is_selected = true;
                if (sel == "2" && r.metadata.experiment_id == "EXP2_DENSITY") is_selected = true;
                if (sel == "3" && r.metadata.experiment_id == "EXP3_HEURISTICS") is_selected = true;
                if (sel == "4" && r.metadata.experiment_id == "EXP4_LANDMARKS") is_selected = true;
                if (sel == "5" && r.metadata.experiment_id == "EXP5_ASTAR_VS_IDASTAR") is_selected = true;

                if (!is_selected) {
                    raw_records.push_back(r);
                }
            }
            std::cout << "Preserved " << raw_records.size() << " existing benchmark records from previous runs.\n";
        }

        if (CSVExporter::import_correctness_results(correctness_path, existing_correctness)) {
            for (const auto& c : existing_correctness) {
                bool is_selected = false;
                if ((sel == "6" || sel == "6,7") && c.experiment_id == "EXP6_DYNAMIC") is_selected = true;
                if ((sel == "7" || sel == "6,7") && c.experiment_id == "EXP7_BREAKEVEN") is_selected = true;
                if (sel == "1" && c.experiment_id == "EXP1_SIZE") is_selected = true;
                if (sel == "2" && c.experiment_id == "EXP2_DENSITY") is_selected = true;
                if (sel == "3" && c.experiment_id == "EXP3_HEURISTICS") is_selected = true;
                if (sel == "4" && c.experiment_id == "EXP4_LANDMARKS") is_selected = true;
                if (sel == "5" && c.experiment_id == "EXP5_ASTAR_VS_IDASTAR") is_selected = true;

                if (!is_selected) {
                    correctness_records.push_back(c);
                }
            }
        }
    }

    if (sel == "all" || sel == "1") {
        ExperimentRunner::run_experiment_1(config, raw_records, correctness_records);
    }
    if (sel == "all" || sel == "2") {
        ExperimentRunner::run_experiment_2(config, raw_records, correctness_records);
    }
    if (sel == "all" || sel == "3") {
        ExperimentRunner::run_experiment_3(config, raw_records, correctness_records);
    }
    if (sel == "all" || sel == "4") {
        ExperimentRunner::run_experiment_4(config, raw_records, correctness_records);
    }
    if (sel == "all" || sel == "5") {
        ExperimentRunner::run_experiment_5(config, raw_records, correctness_records);
    }
    if (sel == "all" || sel == "6" || sel == "6,7") {
        ExperimentRunner::run_experiment_6(config, raw_records, correctness_records);
    }
    if (sel == "all" || sel == "7" || sel == "6,7") {
        ExperimentRunner::run_experiment_7(config, raw_records, correctness_records);
    }

    std::cout << "\nExperiments completed! Total measured executions: " << raw_records.size() << "\n";
    std::cout << "Aggregating summary statistics...\n";
    compute_summaries();

    export_csv_results();
}

void BenchmarkRunner::compute_summaries() {
    // Group by (exp_id, algorithm, graph_size, density, landmark_count, dynamic_event_type)
    struct GroupKey {
        std::string exp_id;
        std::string algo;
        size_t size;
        std::string density;
        size_t landmarks;
        std::string event_type;

        bool operator<(const GroupKey& o) const {
            if (exp_id != o.exp_id) return exp_id < o.exp_id;
            if (algo != o.algo) return algo < o.algo;
            if (size != o.size) return size < o.size;
            if (density != o.density) return density < o.density;
            if (landmarks != o.landmarks) return landmarks < o.landmarks;
            return event_type < o.event_type;
        }
    };

    std::map<GroupKey, std::vector<BenchmarkRecord>> groups;
    for (const auto& r : raw_records) {
        GroupKey key{
            r.metadata.experiment_id,
            r.metadata.algorithm,
            r.metadata.graph_size,
            r.metadata.density,
            r.metadata.landmark_count,
            r.metadata.dynamic_event_type
        };
        groups[key].push_back(r);
    }

    for (const auto& item : groups) {
        const GroupKey& key = item.first;
        const std::vector<BenchmarkRecord>& list = item.second;
        SummaryRecord s;
        s.experiment_id = key.exp_id;
        s.algorithm = key.algo;
        s.graph_size = key.size;
        s.density = key.density;
        s.landmark_count = key.landmarks;
        s.dynamic_event_type = key.event_type;
        s.sample_count = list.size();

        std::vector<double> search_times;
        std::vector<double> total_times;
        std::vector<double> expanded;
        std::vector<double> generated;
        std::vector<double> search_mem;
        std::vector<double> landmark_mem;

        s.success_count = 0;
        s.failure_count = 0;

        for (const auto& r : list) {
            if (r.metrics.status == BenchmarkStatus::SUCCESS) s.success_count++;
            else s.failure_count++;

            search_times.push_back(r.metrics.search_time_ms);
            total_times.push_back(r.metrics.total_execution_ms);
            expanded.push_back(static_cast<double>(r.metrics.nodes_expanded));
            generated.push_back(static_cast<double>(r.metrics.nodes_generated));
            search_mem.push_back(r.metrics.peak_search_memory_kb);
            landmark_mem.push_back(r.metrics.landmark_table_memory_kb);
        }

        s.median_search_time_ms = calculate_median(search_times);
        s.median_total_time_ms = calculate_median(total_times);
        s.median_nodes_expanded = calculate_median(expanded);
        s.median_nodes_generated = calculate_median(generated);
        s.median_search_memory_kb = calculate_median(search_mem);
        s.median_landmark_memory_kb = calculate_median(landmark_mem);

        s.mean_search_time_ms = calculate_mean(search_times);
        s.stddev_search_time_ms = calculate_stddev(search_times, s.mean_search_time_ms);

        s.mean_total_time_ms = calculate_mean(total_times);
        s.stddev_total_time_ms = calculate_stddev(total_times, s.mean_total_time_ms);

        s.mean_nodes_expanded = calculate_mean(expanded);
        s.stddev_nodes_expanded = calculate_stddev(expanded, s.mean_nodes_expanded);

        summary_records.push_back(s);
    }
}

bool BenchmarkRunner::export_csv_results() const {
    CSVExporter::ensure_directory_exists(config.output_dir);

    std::string raw_path = config.output_dir + "/raw_results.csv";
    std::string summary_path = config.output_dir + "/summary_results.csv";
    std::string correctness_path = config.output_dir + "/correctness_results.csv";

    bool ok1 = CSVExporter::export_raw_results(raw_path, raw_records);
    bool ok2 = CSVExporter::export_summary_results(summary_path, summary_records);
    bool ok3 = CSVExporter::export_correctness_results(correctness_path, correctness_records);

    if (ok1 && ok2 && ok3) {
        std::cout << "\nSuccessfully exported benchmark CSV results to:\n"
                  << "  " << raw_path << "\n"
                  << "  " << summary_path << "\n"
                  << "  " << correctness_path << "\n";
        return true;
    } else {
        std::cerr << "Error writing one or more CSV export files!\n";
        return false;
    }
}
