#ifndef ALT_HEURISTIC_HPP
#define ALT_HEURISTIC_HPP

#include "heuristic.hpp"
#include <vector>

class ALTHeuristic : public Heuristic {
private:
    std::vector<NodeID> landmarks;
    std::vector<std::vector<EdgeWeight>> dist_table;
    double preprocessing_time_ms;
    
    // Cache tracking
    uint64_t cached_graph_id;
    uint64_t cached_graph_version;

    // Helper to run Dijkstra from a landmark
    void compute_landmark_distances(const Graph& graph, NodeID landmark_id, std::vector<EdgeWeight>& distances);

    void preprocess(const Graph& graph);

public:
    // Initialize the ALT heuristic with given landmarks
    ALTHeuristic(const Graph& graph, const std::vector<NodeID>& chosen_landmarks);

    // Refresh preprocessing if graph has changed
    void refresh(const Graph& graph) override;

    // Compute the max lower bound using the triangle inequality
    EdgeWeight compute(NodeID u, NodeID goal, const Graph& graph) override;

    double get_preprocessing_time_ms() const override { return preprocessing_time_ms; }

    // Helper to automatically pick landmarks (random/spread out)
    static std::vector<NodeID> select_landmarks(const Graph& graph, size_t count);
};

#endif // ALT_HEURISTIC_HPP
