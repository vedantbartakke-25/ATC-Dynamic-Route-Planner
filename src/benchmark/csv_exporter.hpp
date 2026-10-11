#ifndef CSV_EXPORTER_HPP
#define CSV_EXPORTER_HPP

#include <string>
#include <vector>
#include "benchmark_types.hpp"

class CSVExporter {
public:
    static bool ensure_directory_exists(const std::string& dir_path);

    static bool export_raw_results(const std::string& filepath, 
                                   const std::vector<BenchmarkRecord>& records);

    static bool export_summary_results(const std::string& filepath, 
                                       const std::vector<SummaryRecord>& summaries);

    static bool export_correctness_results(const std::string& filepath, 
                                           const std::vector<CorrectnessRecord>& correctness_checks);

    static bool import_raw_results(const std::string& filepath,
                                   std::vector<BenchmarkRecord>& records);

    static bool import_correctness_results(const std::string& filepath,
                                           std::vector<CorrectnessRecord>& checks);
};

#endif // CSV_EXPORTER_HPP
