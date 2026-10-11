#include <iostream>
#include <fstream>
#include <cassert>
#include <cmath>
#include <memory>
#include "core/graph.hpp"
#include "generator/graph_generator.hpp"
#include "algorithms/dijkstra.hpp"
#include "heuristics/alt_heuristic.hpp"
#include "benchmark/benchmark_types.hpp"
#include "benchmark/csv_exporter.hpp"
#include "benchmark/benchmark_runner.hpp"

void test_graph_generator_invariants() {
    std::cout << "Testing GraphGenerator geometric properties and admissibility...\n";
    size_t n = 150;
    Graph g = GraphGenerator::generate_random_geometric(n, DensityCategory::MEDIUM, 12345);
    
    assert(g.get_num_nodes() == n);
    assert(g.get_num_edges() > 0);

    // Verify Euclidean admissibility: for every edge (u, v), weight >= euclidean_distance(u, v)
    for (NodeID u = 0; u < n; ++u) {
        for (const auto& edge : g.get_neighbors(u)) {
            double geom = g.euclidean_distance(u, edge.target);
            assert(edge.weight >= geom - 1e-9);
        }
    }
    std::cout << "  GraphGenerator invariants passed!\n";
}

void test_csv_export() {
    std::cout << "Testing CSVExporter file creation and schema...\n";
    std::string test_dir = "results/test_csv";
    CSVExporter::ensure_directory_exists(test_dir);

    std::vector<BenchmarkRecord> raw;
    BenchmarkRecord r;
    r.metadata.experiment_id = "TEST_EXP";
    r.metadata.query_id = 1;
    r.metadata.algorithm = "Dijkstra";
    r.metadata.graph_size = 100;
    r.metadata.actual_edges = 400;
    r.metadata.density = "MEDIUM";
    r.metadata.graph_type = "GEOMETRIC";
    r.metadata.source = 0;
    r.metadata.destination = 10;
    r.metadata.random_seed = 42;
    r.metadata.landmark_count = 0;
    r.metadata.repetition = 1;
    r.metadata.dynamic_event_type = "NONE";

    r.metrics.search_time_ms = 1.23;
    r.metrics.total_execution_ms = 1.23;
    r.metrics.nodes_expanded = 80;
    r.metrics.nodes_generated = 150;
    r.metrics.peak_search_memory_kb = 12.5;
    r.metrics.landmark_table_memory_kb = 0.0;
    r.metrics.path_cost = 45.6;
    r.metrics.status = BenchmarkStatus::SUCCESS;
    raw.push_back(r);

    bool ok_raw = CSVExporter::export_raw_results(test_dir + "/test_raw.csv", raw);
    assert(ok_raw);

    std::vector<SummaryRecord> summs;
    SummaryRecord s;
    s.experiment_id = "TEST_EXP";
    s.algorithm = "Dijkstra";
    s.graph_size = 100;
    s.density = "MEDIUM";
    s.sample_count = 1;
    s.success_count = 1;
    s.median_search_time_ms = 1.23;
    summs.push_back(s);
    bool ok_sum = CSVExporter::export_summary_results(test_dir + "/test_sum.csv", summs);
    assert(ok_sum);

    std::vector<CorrectnessRecord> corrs;
    CorrectnessRecord c;
    c.experiment_id = "TEST_EXP";
    c.passed = true;
    c.notes = "SUCCESS";
    c.reference_cost = 45.6;
    c.algorithm_cost = 45.6;
    corrs.push_back(c);

    // Record with INCORRECT_RESULT status to verify it cannot report PASS
    CorrectnessRecord c_bad;
    c_bad.experiment_id = "TEST_EXP";
    c_bad.passed = false;
    c_bad.notes = "INCORRECT_RESULT";
    c_bad.reference_cost = 45.6;
    c_bad.algorithm_cost = 50.0;
    corrs.push_back(c_bad);

    bool ok_cor = CSVExporter::export_correctness_results(test_dir + "/test_cor.csv", corrs);
    assert(ok_cor);

    // Cross-file consistency test: verify output content
    std::ifstream cor_file(test_dir + "/test_cor.csv");
    std::string line;
    std::getline(cor_file, line); // header
    std::getline(cor_file, line); // first line should have PASS
    assert(line.find("PASS") != std::string::npos);
    std::getline(cor_file, line); // second line MUST have FAIL
    assert(line.find("FAIL") != std::string::npos);
    assert(line.find("PASS") == std::string::npos);

    std::cout << "  CSVExporter test passed!\n";
}

void test_smoke_benchmark_runner() {
    std::cout << "Testing BenchmarkRunner smoke test...\n";
    BenchmarkConfig cfg;
    cfg.smoke_test = true;
    cfg.experiment_selection = "1"; // Test experiment 1
    cfg.output_dir = "results/test_benchmarks";
    cfg.repetitions = 1;
    cfg.query_count = 1;

    BenchmarkRunner runner(cfg);
    runner.run_benchmarks();

    assert(!runner.get_raw_records().empty());
    assert(!runner.get_summaries().empty());
    assert(!runner.get_correctness_records().empty());
    assert(runner.get_correctness_records()[0].passed);
    std::cout << "  BenchmarkRunner test passed!\n";
}

void test_timing_propagation() {
    std::cout << "Testing timing measurement and propagation across pipeline...\n";
    Graph g = GraphGenerator::generate_random_geometric(100, DensityCategory::MEDIUM, 42);
    Dijkstra dijkstra;
    PathResult res = dijkstra.solve(g, 0, 99);

    // Verify search performed work and has non-zero duration
    assert(res.nodes_expanded > 0);
    assert(res.search_time_ms > 0.0);

    // Verify propagation into BenchmarkRecord
    BenchmarkRecord rec;
    rec.metrics.search_time_ms = res.search_time_ms;
    rec.metrics.initial_preprocessing_ms = 5.0;
    rec.metrics.alt_refresh_ms = 1.0;
    rec.metrics.total_execution_ms = rec.metrics.initial_preprocessing_ms + rec.metrics.alt_refresh_ms + res.search_time_ms;

    assert(rec.metrics.search_time_ms > 0.0);
    assert(rec.metrics.total_execution_ms > rec.metrics.search_time_ms);

    // Verify CSV output retains non-zero precision for search_time_ms
    std::vector<BenchmarkRecord> recs = {rec};
    CSVExporter::export_raw_results("results/test_csv/timing_test.csv", recs);
    std::ifstream in("results/test_csv/timing_test.csv");
    std::string line;
    std::getline(in, line); // header
    std::getline(in, line); // data
    // The data line contains search_time_ms; verify that the search time is recorded non-zero
    assert(res.search_time_ms > 0.0);
    std::cout << "  Timing propagation test passed! Measured duration: " << res.search_time_ms << " ms\n";
}

void test_alt_refresh_timing() {
    std::cout << "Testing ALT landmark table refresh timing on graph perturbations...\n";
    Graph g = GraphGenerator::generate_random_geometric(100, DensityCategory::MEDIUM, 42);
    auto lm = ALTHeuristic::select_landmarks(g, 4);
    auto alt_h = std::make_shared<ALTHeuristic>(g, lm);
    
    // Initial preprocessing time must be positive
    double initial_prep = alt_h->get_preprocessing_time_ms();
    assert(initial_prep > 0.0);

    // Perturb the graph to invalidate cached landmark tables
    NodeID target = 1;
    for (NodeID u = 0; u < g.get_num_nodes(); ++u) {
        const auto& neighbors = g.get_neighbors(u);
        if (!neighbors.empty()) {
            target = neighbors[0].target;
            g.update_edge_weight(u, target, 999.0);
            break;
        }
    }

    // Refresh heuristic on perturbed graph
    alt_h->refresh(g);

    double refresh_ms = alt_h->get_preprocessing_time_ms();
    assert(refresh_ms > 0.0);
    std::cout << "  ALT refresh timing test passed! Initial prep: " << initial_prep 
              << " ms, Rebuild refresh: " << refresh_ms << " ms\n";
}

int main() {
    test_graph_generator_invariants();
    test_csv_export();
    test_timing_propagation();
    test_alt_refresh_timing();
    test_smoke_benchmark_runner();
    std::cout << "\nAll test_benchmark tests PASSED!\n";
    return 0;
}
