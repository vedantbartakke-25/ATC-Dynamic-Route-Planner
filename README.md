# Dynamic Shortest Path Replanning

A C++17 command-line application to experimentally compare shortest-path algorithms under dynamic graph changes.

## Phase 0: Project Setup

This project uses standard C++17 and CMake.

## Build Instructions

### Option 1: Direct compilation with `g++` (Recommended on Windows if CMake is not in PATH)

Compile the main executable:
```powershell
g++ -std=c++17 src/core/graph.cpp src/generator/graph_generator.cpp src/algorithms/dijkstra.cpp src/algorithms/astar.cpp src/heuristics/alt_heuristic.cpp src/algorithms/ida_star.cpp src/dynamic/dynamic_engine.cpp src/main.cpp -I src -o planner.exe
```

### Option 2: Using CMake (if installed)

```powershell
mkdir build
cd build
cmake ..
cmake --build .
```

---

## Run Instructions

### 1. View Help
```powershell
.\planner.exe --help
```

### 2. Static Shortest Path Search (Dijkstra vs A* Euclidean vs A* ALT vs IDA* ALT)
Solve between two nodes on a graph:
```powershell
.\planner.exe --graph data/sample_graph.txt --start 0 --goal 2
```

### 3. Dynamic Replanning Under Perturbations
Pass an events file to simulate edge closures, openings, and weight changes dynamically:
```powershell
.\planner.exe --graph data/sample_graph.txt --events data/events.txt --start 0 --goal 2
```

---

## Running Test Suites

You can compile and run any of the test suites individually:

- **Graph Engine Tests:**
  ```powershell
  g++ -std=c++17 src/core/graph.cpp src/generator/graph_generator.cpp tests/test_graph.cpp -I src -o test_graph.exe; .\test_graph.exe
  ```

- **Dijkstra Tests:**
  ```powershell
  g++ -std=c++17 src/core/graph.cpp src/generator/graph_generator.cpp src/algorithms/dijkstra.cpp tests/test_dijkstra.cpp -I src -o test_dijkstra.exe; .\test_dijkstra.exe
  ```

- **A* Euclidean Tests:**
  ```powershell
  g++ -std=c++17 src/core/graph.cpp src/generator/graph_generator.cpp src/algorithms/dijkstra.cpp src/algorithms/astar.cpp tests/test_astar.cpp -I src -o test_astar.exe; .\test_astar.exe
  ```

- **ALT Landmark Heuristic Tests:**
  ```powershell
  g++ -std=c++17 src/core/graph.cpp src/generator/graph_generator.cpp src/algorithms/dijkstra.cpp src/algorithms/astar.cpp src/heuristics/alt_heuristic.cpp tests/test_alt.cpp -I src -o test_alt.exe; .\test_alt.exe
  ```

- **IDA* Tests:**
  ```powershell
  g++ -std=c++17 src/core/graph.cpp src/generator/graph_generator.cpp src/algorithms/dijkstra.cpp src/algorithms/astar.cpp src/heuristics/alt_heuristic.cpp src/algorithms/ida_star.cpp tests/test_ida_star.cpp -I src -o test_ida_star.exe; .\test_ida_star.exe
  ```

- **Dynamic Replanning Tests:**
  ```powershell
  g++ -std=c++17 src/core/graph.cpp src/generator/graph_generator.cpp src/algorithms/dijkstra.cpp src/algorithms/astar.cpp src/heuristics/alt_heuristic.cpp src/algorithms/ida_star.cpp src/dynamic/dynamic_engine.cpp tests/test_dynamic.cpp -I src -o test_dynamic.exe; .\test_dynamic.exe
  ```

- **ALT Cache Invalidation & Correctness Patch Tests:**
  ```powershell
  g++ -std=c++17 src/core/graph.cpp src/generator/graph_generator.cpp src/algorithms/dijkstra.cpp src/algorithms/astar.cpp src/heuristics/alt_heuristic.cpp src/algorithms/ida_star.cpp src/dynamic/dynamic_engine.cpp tests/test_phase6_patch.cpp -I src -o run_cache.exe; .\run_cache.exe
  ```
