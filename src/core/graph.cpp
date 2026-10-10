#include "graph.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

NodeID Graph::add_node(double x, double y) {
    version++;
    NodeID id = static_cast<NodeID>(nodes.size());
    nodes.push_back({x, y});
    adj_list.emplace_back();
    return id;
}

void Graph::add_edge(NodeID u, NodeID v, EdgeWeight weight) {
    if (u >= nodes.size() || v >= nodes.size()) {
        throw std::out_of_range("Node ID out of range");
    }
    EdgeWeight min_bound = euclidean_distance(u, v);
    if (weight < min_bound) weight = min_bound;
    
    version++;
    adj_list[u].emplace_back(u, v, weight);
}

void Graph::add_undirected_edge(NodeID u, NodeID v, EdgeWeight weight) {
    add_edge(u, v, weight);
    add_edge(v, u, weight);
}

bool Graph::remove_edge(NodeID u, NodeID v) {
    if (u >= nodes.size()) return false;
    auto& edges = adj_list[u];
    auto it = std::remove_if(edges.begin(), edges.end(), [v](const Edge& e) { return e.target == v; });
    if (it != edges.end()) {
        edges.erase(it, edges.end());
        version++;
        return true;
    }
    return false;
}

bool Graph::set_edge_status(NodeID u, NodeID v, bool is_open) {
    if (u >= nodes.size()) throw std::out_of_range("Node ID out of range");
    for (auto& edge : adj_list[u]) {
        if (edge.target == v) {
            bool old = edge.is_open;
            if (old != is_open) {
                edge.is_open = is_open;
                version++;
            }
            return old;
        }
    }
    throw std::invalid_argument("Edge not found");
}

EdgeWeight Graph::update_edge_weight(NodeID u, NodeID v, EdgeWeight new_weight) {
    if (u >= nodes.size() || v >= nodes.size()) throw std::out_of_range("Node ID out of range");
    
    // Ensure weight does not drop below physical Euclidean distance to preserve heuristic admissibility
    EdgeWeight min_bound = euclidean_distance(u, v);
    if (new_weight < min_bound) {
        new_weight = min_bound;
    }

    for (auto& edge : adj_list[u]) {
        if (edge.target == v) {
            EdgeWeight old = edge.weight;
            if (old != new_weight) {
                edge.weight = new_weight;
                version++;
            }
            return old;
        }
    }
    throw std::invalid_argument("Edge not found");
}

bool Graph::load_from_file(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;

    version++; // File loading is a structural change
    nodes.clear();
    adj_list.clear();


    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "NODES") {
            size_t count;
            iss >> count;
            for (size_t i = 0; i < count; ++i) {
                if (!std::getline(file, line)) return false;
                std::istringstream niss(line);
                NodeID id;
                double x, y;
                niss >> id >> x >> y;
                add_node(x, y); // Assumes IDs in file are sequential starting from 0
            }
        } else if (type == "EDGES") {
            size_t count;
            iss >> count;
            for (size_t i = 0; i < count; ++i) {
                if (!std::getline(file, line)) return false;
                std::istringstream eiss(line);
                NodeID u, v;
                EdgeWeight w;
                eiss >> u >> v >> w;
                add_edge(u, v, w);
            }
        }
    }
    return true;
}

void Graph::print_info() const {
    size_t num_edges = 0;
    size_t num_closed = 0;
    for (const auto& edges : adj_list) {
        num_edges += edges.size();
        for (const auto& e : edges) {
            if (!e.is_open) num_closed++;
        }
    }
    std::cout << "Graph Information:\n"
              << "  Nodes: " << nodes.size() << "\n"
              << "  Edges: " << num_edges << " (Open: " << (num_edges - num_closed) << ", Closed: " << num_closed << ")\n";
}
