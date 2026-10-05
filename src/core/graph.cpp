#include "graph.hpp"

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
    adj_list[u].emplace_back(v, weight);
}

void Graph::add_undirected_edge(NodeID u, NodeID v, EdgeWeight weight) {
    add_edge(u, v, weight);
    add_edge(v, u, weight);
}

void Graph::set_edge_status(NodeID u, NodeID v, bool is_open) {
    for (auto& edge : adj_list[u]) {
        if (edge.target == v) {
            edge.is_open = is_open;
            return;
        }
    }
    throw std::invalid_argument("Edge not found");
}

void Graph::update_edge_weight(NodeID u, NodeID v, EdgeWeight new_weight) {
    for (auto& edge : adj_list[u]) {
        if (edge.target == v) {
            edge.weight = new_weight;
            return;
        }
    }
    throw std::invalid_argument("Edge not found");
}
