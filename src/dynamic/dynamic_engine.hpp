#ifndef DYNAMIC_ENGINE_HPP
#define DYNAMIC_ENGINE_HPP

#include "core/graph.hpp"
#include "algorithms/solver.hpp"
#include <string>
#include <vector>
#include <memory>

enum class EventType {
    UPDATE_WEIGHT,
    CLOSE_EDGE,
    OPEN_EDGE
};

struct GraphEvent {
    double time;
    EventType type;
    NodeID u;
    NodeID v;
    EdgeWeight new_weight; // Used only for UPDATE_WEIGHT
};

struct ReplanningResult {
    bool replanned;
    PathResult new_result;
    
    // Tracking old values for output
    EdgeWeight old_cost;
    std::vector<NodeID> old_path;
};

class DynamicEngine {
private:
    Graph& graph;
    std::shared_ptr<Solver> solver;

public:
    DynamicEngine(Graph& g, std::shared_ptr<Solver> s) : graph(g), solver(s) {}

    static std::vector<GraphEvent> load_events(const std::string& filename);

    // Applies the event to the graph and checks if the path is affected.
    // Recomputes the route if necessary (Full Recomputation).
    ReplanningResult handle_event(const GraphEvent& event, const PathResult& current_path, NodeID start, NodeID goal);
};

#endif // DYNAMIC_ENGINE_HPP
