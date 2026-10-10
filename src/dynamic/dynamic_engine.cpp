#include "dynamic_engine.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

std::vector<GraphEvent> DynamicEngine::load_events(const std::string& filename) {
    std::vector<GraphEvent> events;
    std::ifstream file(filename);
    if (!file.is_open()) return events;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        
        if (token == "TIME") {
            GraphEvent ev;
            iss >> ev.time;
            
            std::string type_str;
            iss >> type_str;
            
            if (type_str == "UPDATE_WEIGHT") {
                ev.type = EventType::UPDATE_WEIGHT;
                iss >> ev.u >> ev.v >> ev.new_weight;
            } else if (type_str == "CLOSE_EDGE") {
                ev.type = EventType::CLOSE_EDGE;
                iss >> ev.u >> ev.v;
                ev.new_weight = INF_WEIGHT;
            } else if (type_str == "OPEN_EDGE") {
                ev.type = EventType::OPEN_EDGE;
                iss >> ev.u >> ev.v;
            }
            events.push_back(ev);
        }
    }
    return events;
}

ReplanningResult DynamicEngine::handle_event(const GraphEvent& event, const PathResult& current_path, NodeID start, NodeID goal) {
    ReplanningResult res;
    res.old_path = current_path.path;
    res.old_cost = current_path.total_cost;
    res.new_result = current_path; // Default to old result
    res.replanned = false;

    // Apply the event and capture previous state
    bool edge_was_open = false;
    EdgeWeight old_weight = 0.0;
    
    // Check if edge is in current path
    bool edge_in_path = false;
    for (size_t i = 0; i + 1 < current_path.path.size(); ++i) {
        if ((current_path.path[i] == event.u && current_path.path[i+1] == event.v) ||
            (current_path.path[i] == event.v && current_path.path[i+1] == event.u)) {
            edge_in_path = true;
            break;
        }
    }

    bool needs_replan = false;

    if (event.type == EventType::UPDATE_WEIGHT) {
        old_weight = graph.update_edge_weight(event.u, event.v, event.new_weight);
        try { graph.update_edge_weight(event.v, event.u, event.new_weight); } catch (...) {}
        
        if (edge_in_path) {
            // Path cost changed. We definitely must replan.
            needs_replan = true;
        } else if (event.new_weight < old_weight) {
            // Edge weight decreased. Might provide a better alternative route.
            needs_replan = true;
        }
    } else if (event.type == EventType::CLOSE_EDGE) {
        edge_was_open = graph.set_edge_status(event.u, event.v, false);
        try { graph.set_edge_status(event.v, event.u, false); } catch (...) {}
        
        if (edge_in_path) {
            // Path is broken. Must replan.
            needs_replan = true;
        }
    } else if (event.type == EventType::OPEN_EDGE) {
        edge_was_open = graph.set_edge_status(event.u, event.v, true);
        try { graph.set_edge_status(event.v, event.u, true); } catch (...) {}
        
        if (!edge_was_open) {
            // A new path opened up. Might be better than current.
            needs_replan = true;
        }
    }

    if (needs_replan) {
        res.replanned = true;
        res.new_result = solver->solve(graph, start, goal);
    }

    return res;
}
