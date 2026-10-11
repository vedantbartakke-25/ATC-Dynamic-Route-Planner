#ifndef GRAPH_GENERATOR_HPP
#define GRAPH_GENERATOR_HPP

#include "core/graph.hpp"

enum class DensityCategory {
    SPARSE,
    MEDIUM,
    DENSE
};

class GraphGenerator {
public:
    // Generates a grid graph of width x height. 
    // Edge weights are at least the Euclidean distance to remain admissible for A*.
    static Graph generate_grid(int width, int height, double spacing = 1.0);

    // Generates a random geometric graph with Euclidean admissibility guarantees
    // Density category controls average degree:
    // SPARSE: ~4, MEDIUM: ~10, DENSE: ~25 (capped safely for large N)
    static Graph generate_random_geometric(size_t num_nodes, 
                                           DensityCategory density, 
                                           unsigned int seed = 42, 
                                           bool undirected = true, 
                                           bool ensure_connected = true);
};

#endif // GRAPH_GENERATOR_HPP
