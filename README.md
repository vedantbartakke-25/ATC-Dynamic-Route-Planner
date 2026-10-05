# Dynamic Shortest Path Replanning

A C++17 command-line application to experimentally compare shortest-path algorithms under dynamic graph changes.

## Phase 0: Project Setup

This project uses standard C++17 and CMake.

### Build Instructions

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

### Run Instructions

```bash
# Run the main planner CLI
./planner --help

# Example usage (algorithms not yet implemented)
./planner --graph ../data/sample_graph.txt --start 0 --goal 3
```

### Test Instructions

```bash
# Run the unit tests
./test_graph
```
