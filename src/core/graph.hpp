#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "types.hpp"
#include <vector>
#include <cmath>
#include <stdexcept>

struct Node {
    double x, y;
};

struct Edge {
    NodeID target;
    EdgeWeight weight;
    bool is_open;
    
    Edge(NodeID t, EdgeWeight w, bool open = true) : target(t), weight(w), is_open(open) {}
};

class Graph {
private:
    std::vector<Node> nodes;
    std::vector<std::vector<Edge>> adj_list;

public:
    Graph() = default;

    NodeID add_node(double x, double y);
    void add_edge(NodeID u, NodeID v, EdgeWeight weight);
    void add_undirected_edge(NodeID u, NodeID v, EdgeWeight weight);
    
    void set_edge_status(NodeID u, NodeID v, bool is_open);
    void update_edge_weight(NodeID u, NodeID v, EdgeWeight new_weight);

    size_t get_num_nodes() const { return nodes.size(); }
    const Node& get_node(NodeID u) const { return nodes[u]; }
    const std::vector<Edge>& get_neighbors(NodeID u) const { return adj_list[u]; }
    
    double euclidean_distance(NodeID u, NodeID v) const {
        double dx = nodes[u].x - nodes[v].x;
        double dy = nodes[u].y - nodes[v].y;
        return std::sqrt(dx*dx + dy*dy);
    }
};

#endif // GRAPH_HPP
