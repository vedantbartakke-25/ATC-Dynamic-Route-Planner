#include "graph_generator.hpp"
#include <random>
#include <algorithm>

Graph GraphGenerator::generate_grid(int width, int height, double spacing) {
    Graph g;
    // Add nodes
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            g.add_node(x * spacing, y * spacing);
        }
    }

    auto get_id = [&](int x, int y) { return y * width + x; };

    // Add edges
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            NodeID u = get_id(x, y);
            
            // Right neighbor
            if (x + 1 < width) {
                NodeID v = get_id(x + 1, y);
                g.add_undirected_edge(u, v, spacing);
            }
            // Bottom neighbor
            if (y + 1 < height) {
                NodeID v = get_id(x, y + 1);
                g.add_undirected_edge(u, v, spacing);
            }
            // Diagonal (bottom-right)
            if (x + 1 < width && y + 1 < height) {
                NodeID v = get_id(x + 1, y + 1);
                g.add_undirected_edge(u, v, spacing * std::sqrt(2.0));
            }
            // Diagonal (bottom-left)
            if (x - 1 >= 0 && y + 1 < height) {
                NodeID v = get_id(x - 1, y + 1);
                g.add_undirected_edge(u, v, spacing * std::sqrt(2.0));
            }
        }
    }
    return g;
}

Graph GraphGenerator::generate_random_geometric(size_t num_nodes, 
                                                DensityCategory density, 
                                                unsigned int seed, 
                                                bool undirected, 
                                                bool ensure_connected) {
    Graph g;
    if (num_nodes == 0) return g;

    std::mt19937 rng(seed);
    // Area scaling: sqrt(N) x sqrt(N) to keep spatial density reasonable
    double side = std::sqrt(static_cast<double>(num_nodes)) * 10.0;
    std::uniform_real_distribution<double> coord_dist(0.0, side);
    std::uniform_real_distribution<double> weight_stretch(1.0, 1.25); // >= 1.0 preserves admissibility!

    for (size_t i = 0; i < num_nodes; ++i) {
        g.add_node(coord_dist(rng), coord_dist(rng));
    }

    // Target average degree: target_degree directed entries per node
    // in an undirected graph, average degree k means each node has on average k neighbors,
    // so total directed entries in adjacency lists = k * num_nodes.
    size_t target_degree = 4;
    if (density == DensityCategory::SPARSE) target_degree = 4;
    else if (density == DensityCategory::MEDIUM) target_degree = 10;
    else if (density == DensityCategory::DENSE) target_degree = std::min<size_t>(25, num_nodes > 1 ? num_nodes - 1 : 1);

    // If connected required, first build a spanning tree connecting nodes sequentially
    if (ensure_connected && num_nodes > 1) {
        std::vector<size_t> perm(num_nodes);
        for (size_t i = 0; i < num_nodes; ++i) perm[i] = i;
        std::shuffle(perm.begin(), perm.end(), rng);

        for (size_t i = 1; i < num_nodes; ++i) {
            NodeID u = perm[i - 1];
            NodeID v = perm[i];
            double geom_d = g.euclidean_distance(u, v);
            double w = geom_d * weight_stretch(rng);
            if (undirected) g.add_undirected_edge(u, v, w);
            else g.add_edge(u, v, w);
        }
    }

    // Now add additional random edges until total directed entries reach target_degree * num_nodes
    size_t target_total_directed_edges = target_degree * num_nodes;
    size_t max_attempts = num_nodes * target_degree * 10;
    size_t attempts = 0;
    std::uniform_int_distribution<size_t> node_dist(0, num_nodes - 1);

    while (g.get_num_edges() < target_total_directed_edges && attempts++ < max_attempts) {
        NodeID u = static_cast<NodeID>(node_dist(rng));
        NodeID v = static_cast<NodeID>(node_dist(rng));
        if (u == v) continue;
        if (g.has_edge(u, v)) continue;

        double geom_d = g.euclidean_distance(u, v);
        double w = geom_d * weight_stretch(rng);

        if (undirected) {
            g.add_undirected_edge(u, v, w);
        } else {
            g.add_edge(u, v, w);
        }
    }

    return g;
}
