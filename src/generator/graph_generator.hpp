#ifndef GRAPH_GENERATOR_HPP
#define GRAPH_GENERATOR_HPP

#include "core/graph.hpp"

class GraphGenerator {
public:
    // Generates a grid graph of width x height. 
    // Edge weights are at least the Euclidean distance to remain admissible for A*.
    static Graph generate_grid(int width, int height, double spacing = 1.0);
};

#endif // GRAPH_GENERATOR_HPP
