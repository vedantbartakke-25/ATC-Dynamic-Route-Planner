#include "experiment_runner.hpp"
#include "generator/graph_generator.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/ida_star.hpp"
#include "heuristics/euclidean_heuristic.hpp"
#include "heuristics/alt_heuristic.hpp"
#include "dynamic/dynamic_engine.hpp"

#include <iostream>
#include <random>
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

namespace {
    bool double_equals(double a, double b, double tol = 1e-4) {
        if (a == INF_WEIGHT && b == INF_WEIGHT) return true;
        if (a == INF_WEIGHT || b == INF_WEIGHT) return false;
        return std::abs(a - b) <= tol * (std::max(1.0, std::max(std::abs(a), std::abs(b))));
    }

    struct QueryPair {
        NodeID src;
        NodeID dst;
    };

    std::vector<QueryPair> generate_query_pairs(size_t num_nodes, size_t count, unsigned int seed) {
        std::vector<QueryPair> pairs;
        if (num_nodes == 0) return pairs;
        std::mt19937 rng(seed);
        std::uniform_int_distribution<NodeID> dist(0, static_cast<NodeID>(num_nodes - 1));

        for (size_t i = 0; i < count; ++i) {
            NodeID s = dist(rng);
            NodeID d = dist(rng);
            pairs.push_back({s, d});
        }
        return pairs;
    }

    void record_execution(const std::string& exp_id,
                          size_t query_id,
                          const std::string& algo_name,
                          const Graph& graph,
                          const std::string& density_name,
                          const std::string& graph_type,
                          NodeID src,
                          NodeID dst,
                          unsigned int seed,
                          size_t landmark_count,
                          size_t rep,
                          const std::string& dynamic_event_type,
                          const PathResult& res,
                          double initial_prep_ms,
                          double alt_refresh_ms,
                          size_t refresh_count,
                          double landmark_mem_kb,
                          EdgeWeight ref_cost,
                          std::vector<BenchmarkRecord>& raw_records,
                          std::vector<CorrectnessRecord>& correctness_records) {
        
        BenchmarkRecord rec;
        rec.metadata.experiment_id = exp_id;
        rec.metadata.query_id = query_id;
        rec.metadata.algorithm = algo_name;
        rec.metadata.graph_size = graph.get_num_nodes();
        rec.metadata.actual_edges = graph.get_num_edges();
        rec.metadata.density = density_name;
        rec.metadata.graph_type = graph_type;
        rec.metadata.source = src;
        rec.metadata.destination = dst;
        rec.metadata.random_seed = seed;
        rec.metadata.landmark_count = landmark_count;
        rec.metadata.repetition = rep;
        rec.metadata.dynamic_event_type = dynamic_event_type;

        rec.metrics.initial_preprocessing_ms = initial_prep_ms;
        rec.metrics.alt_refresh_ms = alt_refresh_ms;
        rec.metrics.heuristic_eval_ms = res.heuristic_time_ms;
        rec.metrics.search_time_ms = res.search_time_ms;
        rec.metrics.total_execution_ms = initial_prep_ms + alt_refresh_ms + res.search_time_ms;
        rec.metrics.replanning_time_ms = alt_refresh_ms + res.search_time_ms;
        rec.metrics.nodes_generated = res.nodes_generated;
        rec.metrics.nodes_expanded = res.nodes_expanded;
        rec.metrics.peak_search_memory_kb = res.peak_memory_kb;
        rec.metrics.landmark_table_memory_kb = landmark_mem_kb;
        rec.metrics.path_cost = res.total_cost;
        rec.metrics.alt_refresh_count = refresh_count;
        rec.metrics.ida_iterations = res.iterations;
        rec.metrics.status = execution_status_to_benchmark(res.status);

        // Verification check: Only completed runs (SUCCESS) are expected to match the oracle cost
        if (rec.metrics.status == BenchmarkStatus::SUCCESS) {
            bool cost_matches = double_equals(ref_cost, res.total_cost);
            if (ref_cost != INF_WEIGHT && !cost_matches) {
                rec.metrics.status = BenchmarkStatus::INCORRECT_RESULT;
                rec.metrics.error_message = "Path cost mismatch with Dijkstra oracle";
            }
        } else if (rec.metrics.status == BenchmarkStatus::TIMEOUT) {
            rec.metrics.error_message = "Algorithm exceeded timeout threshold";
        } else if (rec.metrics.status == BenchmarkStatus::RESOURCE_LIMIT) {
            rec.metrics.error_message = "Algorithm exceeded node expansion limit";
        } else if (rec.metrics.status == BenchmarkStatus::NO_PATH) {
            rec.metrics.error_message = "No path exists between source and destination";
        }

        raw_records.push_back(rec);

        if (rep == 1) { // Record correctness once per unique query
            CorrectnessRecord cr;
            cr.experiment_id = exp_id;
            cr.query_id = query_id;
            cr.algorithm = algo_name;
            cr.graph_size = graph.get_num_nodes();
            cr.source = src;
            cr.destination = dst;
            cr.reference_cost = ref_cost;
            cr.algorithm_cost = res.total_cost;
            cr.cost_difference = (ref_cost == INF_WEIGHT || res.total_cost == INF_WEIGHT) ? 0.0 : std::abs(ref_cost - res.total_cost);
            // PASSED only if cost matches Dijkstra AND algorithm completed with SUCCESS
            cr.passed = (rec.metrics.status == BenchmarkStatus::SUCCESS);
            cr.notes = status_to_string(rec.metrics.status);
            correctness_records.push_back(cr);
        }
    }
}

// -------------------------------------------------------------
// EXPERIMENT 1: Graph Size Scaling
// -------------------------------------------------------------
void ExperimentRunner::run_experiment_1(const BenchmarkConfig& config, 
                                        std::vector<BenchmarkRecord>& raw_records,
                                        std::vector<CorrectnessRecord>& correctness_records) {
    std::cout << "Running Experiment 1: Graph Size Scaling...\n";
    
    std::vector<size_t> sizes = config.smoke_test 
        ? std::vector<size_t>{50, 100} 
        : std::vector<size_t>{100, 500, 1000, 5000, 10000};

    for (size_t n : sizes) {
        std::cout << "  Graph size: " << n << " nodes...\n";
        Graph g = GraphGenerator::generate_random_geometric(n, DensityCategory::MEDIUM, config.random_seed);
        size_t q_count = config.smoke_test ? 2 : config.query_count;
        auto queries = generate_query_pairs(n, q_count, config.random_seed + 1);

        // Prepare algorithms
        Dijkstra dijkstra;
        auto eucl_h = std::make_shared<EuclideanHeuristic>();
        AStar astar_eucl(eucl_h);

        // Preprocess ALT once for the graph
        size_t lm_count = 4;
        auto landmarks = ALTHeuristic::select_landmarks(g, lm_count);
        auto alt_h = std::make_shared<ALTHeuristic>(g, landmarks);
        double alt_initial_prep_ms = alt_h->get_preprocessing_time_ms();
        double alt_mem_kb = alt_h->get_memory_kb();

        AStar astar_alt(alt_h);
        IDAStar ida_alt(alt_h, config.ida_max_expansions, config.ida_timeout_ms);

        for (size_t q = 0; q < queries.size(); ++q) {
            NodeID src = queries[q].src;
            NodeID dst = queries[q].dst;

            // Oracle
            PathResult ref_res = dijkstra.solve(g, src, dst);
            EdgeWeight ref_cost = ref_res.total_cost;

            size_t reps = config.smoke_test ? 1 : config.repetitions;
            for (size_t r = 1; r <= reps; ++r) {
                // Dijkstra
                PathResult d_res = dijkstra.solve(g, src, dst);
                record_execution("EXP1_SIZE", q, "Dijkstra", g, "MEDIUM", "GEOMETRIC",
                                 src, dst, config.random_seed, 0, r, "NONE",
                                 d_res, 0.0, 0.0, 0, 0.0, ref_cost, raw_records, correctness_records);

                // A* Euclidean
                PathResult ae_res = astar_eucl.solve(g, src, dst);
                record_execution("EXP1_SIZE", q, "A* Euclidean", g, "MEDIUM", "GEOMETRIC",
                                 src, dst, config.random_seed, 0, r, "NONE",
                                 ae_res, 0.0, 0.0, 0, 0.0, ref_cost, raw_records, correctness_records);

                // A* ALT (reusing precomputed landmark table without re-preprocessing)
                PathResult aalt_res = astar_alt.solve(g, src, dst);
                record_execution("EXP1_SIZE", q, "A* ALT", g, "MEDIUM", "GEOMETRIC",
                                 src, dst, config.random_seed, lm_count, r, "NONE",
                                 aalt_res, alt_initial_prep_ms, 0.0, 0, alt_mem_kb, ref_cost, raw_records, correctness_records);

                // IDA* ALT (only on feasible graph sizes to avoid excessive timeouts)
                if (n <= (config.smoke_test ? 100 : 500)) {
                    PathResult ida_res = ida_alt.solve(g, src, dst);
                    record_execution("EXP1_SIZE", q, "IDA* ALT", g, "MEDIUM", "GEOMETRIC",
                                     src, dst, config.random_seed, lm_count, r, "NONE",
                                     ida_res, alt_initial_prep_ms, 0.0, 0, alt_mem_kb, ref_cost, raw_records, correctness_records);
                }
            }
        }
    }
}

// -------------------------------------------------------------
// EXPERIMENT 2: Graph Density
// -------------------------------------------------------------
void ExperimentRunner::run_experiment_2(const BenchmarkConfig& config, 
                                        std::vector<BenchmarkRecord>& raw_records,
                                        std::vector<CorrectnessRecord>& correctness_records) {
    std::cout << "Running Experiment 2: Graph Density Scaling...\n";
    
    size_t n = config.smoke_test ? 80 : 500;
    std::vector<std::pair<std::string, DensityCategory>> densities = {
        {"SPARSE", DensityCategory::SPARSE},
        {"MEDIUM", DensityCategory::MEDIUM},
        {"DENSE", DensityCategory::DENSE}
    };

    for (const auto& item : densities) {
        const std::string& d_name = item.first;
        DensityCategory d_cat = item.second;
        std::cout << "  Density: " << d_name << "...\n";
        Graph g = GraphGenerator::generate_random_geometric(n, d_cat, config.random_seed);
        size_t q_count = config.smoke_test ? 2 : config.query_count;
        auto queries = generate_query_pairs(n, q_count, config.random_seed + 2);

        Dijkstra dijkstra;
        auto eucl_h = std::make_shared<EuclideanHeuristic>();
        AStar astar_eucl(eucl_h);

        size_t lm_count = 4;
        auto landmarks = ALTHeuristic::select_landmarks(g, lm_count);
        auto alt_h = std::make_shared<ALTHeuristic>(g, landmarks);
        double prep_ms = alt_h->get_preprocessing_time_ms();
        double mem_kb = alt_h->get_memory_kb();

        AStar astar_alt(alt_h);
        IDAStar ida_alt(alt_h, config.ida_max_expansions, config.ida_timeout_ms);

        for (size_t q = 0; q < queries.size(); ++q) {
            NodeID src = queries[q].src;
            NodeID dst = queries[q].dst;
            PathResult ref_res = dijkstra.solve(g, src, dst);
            EdgeWeight ref_cost = ref_res.total_cost;

            size_t reps = config.smoke_test ? 1 : config.repetitions;
            for (size_t r = 1; r <= reps; ++r) {
                auto res_d = dijkstra.solve(g, src, dst);
                record_execution("EXP2_DENSITY", q, "Dijkstra", g, d_name, "GEOMETRIC",
                                 src, dst, config.random_seed, 0, r, "NONE",
                                 res_d, 0.0, 0.0, 0, 0.0, ref_cost, raw_records, correctness_records);

                auto res_ae = astar_eucl.solve(g, src, dst);
                record_execution("EXP2_DENSITY", q, "A* Euclidean", g, d_name, "GEOMETRIC",
                                 src, dst, config.random_seed, 0, r, "NONE",
                                 res_ae, 0.0, 0.0, 0, 0.0, ref_cost, raw_records, correctness_records);

                auto res_aalt = astar_alt.solve(g, src, dst);
                record_execution("EXP2_DENSITY", q, "A* ALT", g, d_name, "GEOMETRIC",
                                 src, dst, config.random_seed, lm_count, r, "NONE",
                                 res_aalt, prep_ms, 0.0, 0, mem_kb, ref_cost, raw_records, correctness_records);

                if (d_cat != DensityCategory::DENSE || config.smoke_test) {
                    auto res_ida = ida_alt.solve(g, src, dst);
                    record_execution("EXP2_DENSITY", q, "IDA* ALT", g, d_name, "GEOMETRIC",
                                     src, dst, config.random_seed, lm_count, r, "NONE",
                                     res_ida, prep_ms, 0.0, 0, mem_kb, ref_cost, raw_records, correctness_records);
                }
            }
        }
    }
}

// -------------------------------------------------------------
// EXPERIMENT 3: Heuristic Quality Comparison
// -------------------------------------------------------------
void ExperimentRunner::run_experiment_3(const BenchmarkConfig& config, 
                                        std::vector<BenchmarkRecord>& raw_records,
                                        std::vector<CorrectnessRecord>& correctness_records) {
    std::cout << "Running Experiment 3: Heuristic Quality Comparison...\n";
    
    size_t n = config.smoke_test ? 100 : 1000;
    Graph g = GraphGenerator::generate_random_geometric(n, DensityCategory::MEDIUM, config.random_seed);
    size_t q_count = config.smoke_test ? 3 : config.query_count;
    auto queries = generate_query_pairs(n, q_count, config.random_seed + 3);

    Dijkstra dijkstra;
    auto eucl_h = std::make_shared<EuclideanHeuristic>();
    AStar astar_eucl(eucl_h);

    size_t lm_count = 4;
    auto landmarks = ALTHeuristic::select_landmarks(g, lm_count);
    auto alt_h = std::make_shared<ALTHeuristic>(g, landmarks);
    double prep_ms = alt_h->get_preprocessing_time_ms();
    double mem_kb = alt_h->get_memory_kb();

    AStar astar_alt(alt_h);

    for (size_t q = 0; q < queries.size(); ++q) {
        NodeID src = queries[q].src;
        NodeID dst = queries[q].dst;
        PathResult ref_res = dijkstra.solve(g, src, dst);
        EdgeWeight ref_cost = ref_res.total_cost;

        size_t reps = config.smoke_test ? 1 : config.repetitions;
        for (size_t r = 1; r <= reps; ++r) {
            auto res_d = dijkstra.solve(g, src, dst);
            record_execution("EXP3_HEURISTICS", q, "Dijkstra", g, "MEDIUM", "GEOMETRIC",
                             src, dst, config.random_seed, 0, r, "NONE",
                             res_d, 0.0, 0.0, 0, 0.0, ref_cost, raw_records, correctness_records);

            auto res_ae = astar_eucl.solve(g, src, dst);
            record_execution("EXP3_HEURISTICS", q, "A* Euclidean", g, "MEDIUM", "GEOMETRIC",
                             src, dst, config.random_seed, 0, r, "NONE",
                             res_ae, 0.0, 0.0, 0, 0.0, ref_cost, raw_records, correctness_records);

            auto res_aalt = astar_alt.solve(g, src, dst);
            record_execution("EXP3_HEURISTICS", q, "A* ALT", g, "MEDIUM", "GEOMETRIC",
                             src, dst, config.random_seed, lm_count, r, "NONE",
                             res_aalt, prep_ms, 0.0, 0, mem_kb, ref_cost, raw_records, correctness_records);
        }
    }
}

// -------------------------------------------------------------
// EXPERIMENT 4: Landmark Count Scaling
// -------------------------------------------------------------
void ExperimentRunner::run_experiment_4(const BenchmarkConfig& config, 
                                        std::vector<BenchmarkRecord>& raw_records,
                                        std::vector<CorrectnessRecord>& correctness_records) {
    std::cout << "Running Experiment 4: Landmark Count Scaling...\n";
    
    size_t n = config.smoke_test ? 100 : 800;
    Graph g = GraphGenerator::generate_random_geometric(n, DensityCategory::MEDIUM, config.random_seed);
    size_t q_count = config.smoke_test ? 2 : config.query_count;
    auto queries = generate_query_pairs(n, q_count, config.random_seed + 4);

    Dijkstra dijkstra;
    std::vector<size_t> lm_counts = config.smoke_test 
        ? std::vector<size_t>{2, 4} 
        : std::vector<size_t>{2, 4, 8, 16};

    for (size_t k : lm_counts) {
        std::cout << "  Landmarks: " << k << "...\n";
        auto landmarks = ALTHeuristic::select_landmarks(g, k);
        auto alt_h = std::make_shared<ALTHeuristic>(g, landmarks);
        double prep_ms = alt_h->get_preprocessing_time_ms();
        double mem_kb = alt_h->get_memory_kb();

        AStar astar_alt(alt_h);

        for (size_t q = 0; q < queries.size(); ++q) {
            NodeID src = queries[q].src;
            NodeID dst = queries[q].dst;
            PathResult ref_res = dijkstra.solve(g, src, dst);
            EdgeWeight ref_cost = ref_res.total_cost;

            size_t reps = config.smoke_test ? 1 : config.repetitions;
            for (size_t r = 1; r <= reps; ++r) {
                auto res_aalt = astar_alt.solve(g, src, dst);
                record_execution("EXP4_LANDMARKS", q, "A* ALT (" + std::to_string(k) + " LMs)", 
                                 g, "MEDIUM", "GEOMETRIC", src, dst, config.random_seed, 
                                 k, r, "NONE", res_aalt, prep_ms, 0.0, 0, mem_kb, ref_cost, 
                                 raw_records, correctness_records);
            }
        }
    }
}

// -------------------------------------------------------------
// EXPERIMENT 5: A* ALT vs IDA* ALT
// -------------------------------------------------------------
void ExperimentRunner::run_experiment_5(const BenchmarkConfig& config, 
                                        std::vector<BenchmarkRecord>& raw_records,
                                        std::vector<CorrectnessRecord>& correctness_records) {
    std::cout << "Running Experiment 5: A* ALT vs IDA* ALT Tradeoff...\n";
    
    // Grid provides natural depth paths for memory comparison
    int side = config.smoke_test ? 10 : 20; // 100 or 400 nodes
    Graph g = GraphGenerator::generate_grid(side, side);
    size_t num_nodes = g.get_num_nodes();

    Dijkstra dijkstra;
    size_t lm_count = 4;
    auto landmarks = ALTHeuristic::select_landmarks(g, lm_count);
    auto alt_h = std::make_shared<ALTHeuristic>(g, landmarks);
    double prep_ms = alt_h->get_preprocessing_time_ms();
    double mem_kb = alt_h->get_memory_kb();

    AStar astar_alt(alt_h);
    IDAStar ida_alt(alt_h, config.ida_max_expansions, config.ida_timeout_ms);

    size_t q_count = config.smoke_test ? 2 : 4;
    auto queries = generate_query_pairs(num_nodes, q_count, config.random_seed + 5);

    for (size_t q = 0; q < queries.size(); ++q) {
        NodeID src = queries[q].src;
        NodeID dst = queries[q].dst;
        PathResult ref_res = dijkstra.solve(g, src, dst);
        EdgeWeight ref_cost = ref_res.total_cost;

        size_t reps = config.smoke_test ? 1 : config.repetitions;
        for (size_t r = 1; r <= reps; ++r) {
            auto a_res = astar_alt.solve(g, src, dst);
            record_execution("EXP5_ASTAR_VS_IDASTAR", q, "A* ALT", g, "GRID", "GRID",
                             src, dst, config.random_seed, lm_count, r, "NONE",
                             a_res, prep_ms, 0.0, 0, mem_kb, ref_cost, raw_records, correctness_records);

            auto ida_res = ida_alt.solve(g, src, dst);
            record_execution("EXP5_ASTAR_VS_IDASTAR", q, "IDA* ALT", g, "GRID", "GRID",
                             src, dst, config.random_seed, lm_count, r, "NONE",
                             ida_res, prep_ms, 0.0, 0, mem_kb, ref_cost, raw_records, correctness_records);
        }
    }
}

// -------------------------------------------------------------
// EXPERIMENT 6: Dynamic Replanning
// -------------------------------------------------------------
void ExperimentRunner::run_experiment_6(const BenchmarkConfig& config, 
                                        std::vector<BenchmarkRecord>& raw_records,
                                        std::vector<CorrectnessRecord>& correctness_records) {
    std::cout << "Running Experiment 6: Dynamic Replanning...\n";
    
    size_t n = config.smoke_test ? 60 : 300;
    Graph base_g = GraphGenerator::generate_random_geometric(n, DensityCategory::MEDIUM, config.random_seed);

    NodeID src = 0;
    NodeID dst = static_cast<NodeID>(n - 1);

    // Perturbation rates
    std::vector<double> rates = config.smoke_test ? std::vector<double>{0.10} : std::vector<double>{0.10, 0.25, 0.50};
    std::vector<std::string> event_types = {"BATCH_UPDATE", "SEQUENTIAL_UPDATE"};

    for (double rate : rates) {
        for (const std::string& ev_mode : event_types) {
            std::cout << "  Rate: " << (rate * 100) << "% (" << ev_mode << ")...\n";

            // Generate deterministic events
            std::mt19937 rng(config.random_seed + 6);
            std::vector<GraphEvent> events;
            size_t num_events = std::max<size_t>(1, static_cast<size_t>(base_g.get_num_edges() * rate / 2));

            for (size_t i = 0; i < num_events; ++i) {
                NodeID u = rng() % n;
                const auto& neighbors = base_g.get_neighbors(u);
                if (neighbors.empty()) continue;
                NodeID v = neighbors[rng() % neighbors.size()].target;

                GraphEvent ev;
                ev.time = static_cast<double>(i * 10);
                ev.u = u;
                ev.v = v;
                int type_choice = rng() % 4;
                if (type_choice == 0) {
                    ev.type = EventType::UPDATE_WEIGHT;
                    ev.new_weight = base_g.euclidean_distance(u, v) * 2.0; // increase
                } else if (type_choice == 1) {
                    ev.type = EventType::UPDATE_WEIGHT;
                    ev.new_weight = base_g.euclidean_distance(u, v) * 1.05; // decrease
                } else if (type_choice == 2) {
                    ev.type = EventType::CLOSE_EDGE;
                } else {
                    ev.type = EventType::OPEN_EDGE;
                }
                events.push_back(ev);
            }

            size_t reps = config.smoke_test ? 1 : config.repetitions;
            for (size_t r = 1; r <= reps; ++r) {
                // Test on cloned graphs to ensure perfect fairness
                std::vector<std::pair<std::string, std::shared_ptr<Solver>>> solvers;
                solvers.push_back({"Dijkstra", std::make_shared<Dijkstra>()});
                
                auto eucl_h = std::make_shared<EuclideanHeuristic>();
                solvers.push_back({"A* Euclidean", std::make_shared<AStar>(eucl_h)});

                auto lm = ALTHeuristic::select_landmarks(base_g, 4);
                auto alt_h = std::make_shared<ALTHeuristic>(base_g, lm);
                solvers.push_back({"A* ALT", std::make_shared<AStar>(alt_h)});

                for (auto& item : solvers) {
                    std::string algo_name = item.first;
                    std::shared_ptr<Solver> solver = item.second;
                    Graph test_g = base_g; // Identical clone
                    solver->refresh(test_g); // Initial refresh

                    DynamicEngine engine(test_g, solver);
                    PathResult initial_res = solver->solve(test_g, src, dst);
                    PathResult current_res = initial_res;

                    double total_search_time = 0.0;
                    double total_refresh_time = 0.0;
                    size_t total_replan_nodes = 0;
                    size_t replan_count = 0;
                    size_t refresh_count = 0;

                    for (const auto& ev : events) {
                        ReplanningResult rep = engine.handle_event(ev, current_res, src, dst);
                        if (rep.replanned) {
                            replan_count++;
                            if (rep.new_result.preprocessing_time_ms > 0.0) {
                                refresh_count++;
                                total_refresh_time += rep.new_result.preprocessing_time_ms;
                            }
                            total_search_time += rep.new_result.search_time_ms;
                            total_replan_nodes += rep.new_result.nodes_expanded;
                            current_res = rep.new_result;
                        }
                    }

                    // Reference from Dijkstra on final state
                    Dijkstra oracle;
                    PathResult ref_final = oracle.solve(test_g, src, dst);

                    PathResult final_metric = current_res;
                    final_metric.search_time_ms = total_search_time;
                    final_metric.nodes_expanded = total_replan_nodes;

                    std::string tag = ev_mode + "_" + std::to_string(static_cast<int>(rate * 100)) + "PCT";
                    record_execution("EXP6_DYNAMIC", 0, algo_name, test_g, "MEDIUM", "DYNAMIC",
                                     src, dst, config.random_seed, (algo_name == "A* ALT" ? 4 : 0), r, 
                                     tag, final_metric, 0.0, total_refresh_time, refresh_count, 
                                     (algo_name == "A* ALT" ? alt_h->get_memory_kb() : 0.0), 
                                     ref_final.total_cost, raw_records, correctness_records);
                }
            }
        }
    }
}

// -------------------------------------------------------------
// EXPERIMENT 7: ALT Break-Even Analysis
// -------------------------------------------------------------
void ExperimentRunner::run_experiment_7(const BenchmarkConfig& config, 
                                        std::vector<BenchmarkRecord>& raw_records,
                                        std::vector<CorrectnessRecord>& correctness_records) {
    std::cout << "Running Experiment 7: ALT Break-Even Analysis...\n";
    
    size_t n = config.smoke_test ? 100 : 1000;
    Graph g = GraphGenerator::generate_random_geometric(n, DensityCategory::MEDIUM, config.random_seed);

    std::vector<size_t> query_counts = config.smoke_test 
        ? std::vector<size_t>{1, 10, 50} 
        : std::vector<size_t>{1, 10, 50, 100, 500, 1000};

    // Preprocessing ALT once
    size_t lm_count = 4;
    auto landmarks = ALTHeuristic::select_landmarks(g, lm_count);
    auto alt_h = std::make_shared<ALTHeuristic>(g, landmarks);
    double alt_initial_prep_ms = alt_h->get_preprocessing_time_ms();
    double alt_mem_kb = alt_h->get_memory_kb();

    Dijkstra dijkstra;
    auto eucl_h = std::make_shared<EuclideanHeuristic>();
    AStar astar_eucl(eucl_h);
    AStar astar_alt(alt_h);

    size_t max_q = query_counts.back();
    auto queries = generate_query_pairs(n, max_q, config.random_seed + 7);

    size_t reps = config.smoke_test ? 1 : config.repetitions;

    for (size_t q_target : query_counts) {
        std::cout << "  Break-Even Workload: " << q_target << " queries...\n";

        for (size_t r = 1; r <= reps; ++r) {
            for (size_t i = 0; i < q_target; ++i) {
                NodeID s = queries[i].src;
                NodeID d = queries[i].dst;

                PathResult ref_res = dijkstra.solve(g, s, d);
                EdgeWeight ref_cost = ref_res.total_cost;

                std::string q_tag = "WORKLOAD_" + std::to_string(q_target);

                // Dijkstra
                record_execution("EXP7_BREAKEVEN", i, "Dijkstra", g, "MEDIUM", "GEOMETRIC",
                                 s, d, config.random_seed, 0, r, q_tag,
                                 ref_res, 0.0, 0.0, 0, 0.0, ref_cost, raw_records, correctness_records);

                // A* Euclidean
                auto ae_res = astar_eucl.solve(g, s, d);
                record_execution("EXP7_BREAKEVEN", i, "A* Euclidean", g, "MEDIUM", "GEOMETRIC",
                                 s, d, config.random_seed, 0, r, q_tag,
                                 ae_res, 0.0, 0.0, 0, 0.0, ref_cost, raw_records, correctness_records);

                // A* ALT (amortized preprocessing: recorded with initial prep on query 0, 0 on subsequent)
                auto alt_res = astar_alt.solve(g, s, d);
                double prep_charge = (i == 0 ? alt_initial_prep_ms : 0.0);
                record_execution("EXP7_BREAKEVEN", i, "A* ALT", g, "MEDIUM", "GEOMETRIC",
                                 s, d, config.random_seed, lm_count, r, q_tag,
                                 alt_res, prep_charge, 0.0, 0, alt_mem_kb, ref_cost, raw_records, correctness_records);
            }
        }
    }
}
