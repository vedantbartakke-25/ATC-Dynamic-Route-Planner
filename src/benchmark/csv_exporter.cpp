#include "csv_exporter.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#define MKDIR(dir) _mkdir(dir)
#else
#define MKDIR(dir) mkdir(dir, 0755)
#endif

bool CSVExporter::ensure_directory_exists(const std::string& dir_path) {
    if (dir_path.empty()) return true;

    // Handle nested directories like "results/benchmarks"
    std::string current = "";
    for (size_t i = 0; i < dir_path.size(); ++i) {
        char c = dir_path[i];
        if (c == '/' || c == '\\') {
            if (!current.empty()) {
                MKDIR(current.c_str());
            }
        }
        current += c;
    }
    MKDIR(current.c_str());
    return true;
}

bool CSVExporter::export_raw_results(const std::string& filepath, 
                                     const std::vector<BenchmarkRecord>& records) {
    std::ofstream out(filepath);
    if (!out.is_open()) return false;

    // Header compatible with pandas and excel
    out << "experiment_id,query_id,algorithm,graph_size,actual_edges,density,graph_type,"
        << "source,destination,random_seed,landmark_count,repetition,dynamic_event_type,"
        << "status,initial_preprocessing_ms,alt_refresh_ms,heuristic_eval_ms,search_time_ms,"
        << "total_execution_ms,replanning_time_ms,nodes_generated,nodes_expanded,"
        << "peak_search_memory_kb,landmark_table_memory_kb,path_cost,alt_refresh_count,"
        << "ida_iterations,error_message\n";

    for (const auto& r : records) {
        const auto& m = r.metadata;
        const auto& p = r.metrics;

        out << m.experiment_id << ","
            << m.query_id << ","
            << "\"" << m.algorithm << "\","
            << m.graph_size << ","
            << m.actual_edges << ","
            << m.density << ","
            << m.graph_type << ","
            << m.source << ","
            << m.destination << ","
            << m.random_seed << ","
            << m.landmark_count << ","
            << m.repetition << ","
            << "\"" << m.dynamic_event_type << "\","
            << status_to_string(p.status) << ","
            << std::fixed << std::setprecision(6) << p.initial_preprocessing_ms << ","
            << p.alt_refresh_ms << ","
            << p.heuristic_eval_ms << ","
            << p.search_time_ms << ","
            << p.total_execution_ms << ","
            << p.replanning_time_ms << ","
            << p.nodes_generated << ","
            << p.nodes_expanded << ","
            << std::setprecision(4) << p.peak_search_memory_kb << ","
            << p.landmark_table_memory_kb << ","
            << (p.path_cost == INF_WEIGHT ? -1.0 : p.path_cost) << ","
            << p.alt_refresh_count << ","
            << p.ida_iterations << ","
            << "\"" << p.error_message << "\"\n";
    }
    return true;
}

bool CSVExporter::export_summary_results(const std::string& filepath, 
                                         const std::vector<SummaryRecord>& summaries) {
    std::ofstream out(filepath);
    if (!out.is_open()) return false;

    out << "experiment_id,algorithm,graph_size,density,landmark_count,dynamic_event_type,"
        << "sample_count,success_count,failure_count,"
        << "median_search_time_ms,median_total_time_ms,median_nodes_expanded,median_nodes_generated,"
        << "median_search_memory_kb,median_landmark_memory_kb,"
        << "mean_search_time_ms,stddev_search_time_ms,"
        << "mean_total_time_ms,stddev_total_time_ms,"
        << "mean_nodes_expanded,stddev_nodes_expanded\n";

    for (const auto& s : summaries) {
        out << s.experiment_id << ","
            << "\"" << s.algorithm << "\","
            << s.graph_size << ","
            << s.density << ","
            << s.landmark_count << ","
            << "\"" << s.dynamic_event_type << "\","
            << s.sample_count << ","
            << s.success_count << ","
            << s.failure_count << ","
            << std::fixed << std::setprecision(6) << s.median_search_time_ms << ","
            << s.median_total_time_ms << ","
            << std::setprecision(4) << s.median_nodes_expanded << ","
            << s.median_nodes_generated << ","
            << s.median_search_memory_kb << ","
            << s.median_landmark_memory_kb << ","
            << std::setprecision(6) << s.mean_search_time_ms << ","
            << s.stddev_search_time_ms << ","
            << s.mean_total_time_ms << ","
            << s.stddev_total_time_ms << ","
            << std::setprecision(4) << s.mean_nodes_expanded << ","
            << s.stddev_nodes_expanded << "\n";
    }
    return true;
}

bool CSVExporter::export_correctness_results(const std::string& filepath, 
                                             const std::vector<CorrectnessRecord>& checks) {
    std::ofstream out(filepath);
    if (!out.is_open()) return false;

    out << "experiment_id,query_id,algorithm,graph_size,source,destination,"
        << "reference_cost,algorithm_cost,cost_difference,passed,notes\n";

    for (const auto& c : checks) {
        bool actually_passed = c.passed && (c.notes == "SUCCESS");
        out << c.experiment_id << ","
            << c.query_id << ","
            << "\"" << c.algorithm << "\","
            << c.graph_size << ","
            << c.source << ","
            << c.destination << ","
            << std::fixed << std::setprecision(4) << (c.reference_cost == INF_WEIGHT ? -1.0 : c.reference_cost) << ","
            << (c.algorithm_cost == INF_WEIGHT ? -1.0 : c.algorithm_cost) << ","
            << c.cost_difference << ","
            << (actually_passed ? "PASS" : "FAIL") << ","
            << "\"" << c.notes << "\"\n";
    }
    return true;
}

namespace {
    std::vector<std::string> parse_csv_line(const std::string& line) {
        std::vector<std::string> fields;
        std::string current;
        bool in_quotes = false;
        for (size_t i = 0; i < line.size(); ++i) {
            char c = line[i];
            if (c == '"') {
                in_quotes = !in_quotes;
            } else if (c == ',' && !in_quotes) {
                fields.push_back(current);
                current.clear();
            } else {
                current += c;
            }
        }
        fields.push_back(current);
        return fields;
    }

    BenchmarkStatus parse_status(const std::string& s) {
        if (s == "SUCCESS") return BenchmarkStatus::SUCCESS;
        if (s == "NO_PATH") return BenchmarkStatus::NO_PATH;
        if (s == "TIMEOUT") return BenchmarkStatus::TIMEOUT;
        if (s == "RESOURCE_LIMIT") return BenchmarkStatus::RESOURCE_LIMIT;
        if (s == "INCORRECT_RESULT") return BenchmarkStatus::INCORRECT_RESULT;
        return BenchmarkStatus::ERROR;
    }
}

bool CSVExporter::import_raw_results(const std::string& filepath,
                                     std::vector<BenchmarkRecord>& records) {
    std::ifstream in(filepath);
    if (!in.is_open()) return false;

    std::string header;
    if (!std::getline(in, header)) return false;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        auto fields = parse_csv_line(line);
        if (fields.size() < 28) continue;

        BenchmarkRecord r;
        r.metadata.experiment_id = fields[0];
        r.metadata.query_id = std::stoull(fields[1]);
        r.metadata.algorithm = fields[2];
        r.metadata.graph_size = std::stoull(fields[3]);
        r.metadata.actual_edges = std::stoull(fields[4]);
        r.metadata.density = fields[5];
        r.metadata.graph_type = fields[6];
        r.metadata.source = static_cast<NodeID>(std::stoull(fields[7]));
        r.metadata.destination = static_cast<NodeID>(std::stoull(fields[8]));
        r.metadata.random_seed = static_cast<unsigned int>(std::stoul(fields[9]));
        r.metadata.landmark_count = std::stoull(fields[10]);
        r.metadata.repetition = std::stoull(fields[11]);
        r.metadata.dynamic_event_type = fields[12];

        r.metrics.status = parse_status(fields[13]);
        r.metrics.initial_preprocessing_ms = std::stod(fields[14]);
        r.metrics.alt_refresh_ms = std::stod(fields[15]);
        r.metrics.heuristic_eval_ms = std::stod(fields[16]);
        r.metrics.search_time_ms = std::stod(fields[17]);
        r.metrics.total_execution_ms = std::stod(fields[18]);
        r.metrics.replanning_time_ms = std::stod(fields[19]);
        r.metrics.nodes_generated = std::stoull(fields[20]);
        r.metrics.nodes_expanded = std::stoull(fields[21]);
        r.metrics.peak_search_memory_kb = std::stod(fields[22]);
        r.metrics.landmark_table_memory_kb = std::stod(fields[23]);
        r.metrics.path_cost = std::stod(fields[24]);
        r.metrics.alt_refresh_count = std::stoull(fields[25]);
        r.metrics.ida_iterations = std::stoull(fields[26]);
        r.metrics.error_message = fields[27];

        records.push_back(r);
    }
    return true;
}

bool CSVExporter::import_correctness_results(const std::string& filepath,
                                            std::vector<CorrectnessRecord>& checks) {
    std::ifstream in(filepath);
    if (!in.is_open()) return false;

    std::string header;
    if (!std::getline(in, header)) return false;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        auto fields = parse_csv_line(line);
        if (fields.size() < 11) continue;

        CorrectnessRecord c;
        c.experiment_id = fields[0];
        c.query_id = std::stoull(fields[1]);
        c.algorithm = fields[2];
        c.graph_size = std::stoull(fields[3]);
        c.source = static_cast<NodeID>(std::stoull(fields[4]));
        c.destination = static_cast<NodeID>(std::stoull(fields[5]));
        c.reference_cost = std::stod(fields[6]);
        c.algorithm_cost = std::stod(fields[7]);
        c.cost_difference = std::stod(fields[8]);
        c.passed = (fields[9] == "PASS");
        c.notes = fields[10];

        checks.push_back(c);
    }
    return true;
}

