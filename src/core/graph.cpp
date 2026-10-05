#include "graph.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

NodeID Graph::add_node(double x, double y) {
    NodeID id = static_cast<NodeID>(nodes.size());
    nodes.push_back({x, y});
    adj_list.emplace_back();
    return id;
}

void Graph::add_edge(NodeID u, NodeID v, EdgeWeight weight) {
    if (u >= nodes.size() || v >= nodes.size()) {
        throw std::out_of_range("Node ID out of range");
    }
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
        return true;
    }
    return false;
}

void Graph::set_edge_status(NodeID u, NodeID v, bool is_open) {
    if (u >= nodes.size()) throw std::out_of_range("Node ID out of range");
    for (auto& edge : adj_list[u]) {
        if (edge.target == v) {
            edge.is_open = is_open;
            return;
        }
    }
    throw std::invalid_argument("Edge not found");
}

void Graph::update_edge_weight(NodeID u, NodeID v, EdgeWeight new_weight) {
    if (u >= nodes.size()) throw std::out_of_range("Node ID out of range");
    for (auto& edge : adj_list[u]) {
        if (edge.target == v) {
            edge.weight = new_weight;
            return;
        }
    }
    throw std::invalid_argument("Edge not found");
}

bool Graph::load_from_file(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;

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
