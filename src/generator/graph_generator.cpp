#include "graph_generator.hpp"
#include <random>

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
