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

- **Benchmark Engine Tests:**
  ```powershell
  g++ -std=c++17 src/core/graph.cpp src/generator/graph_generator.cpp src/algorithms/dijkstra.cpp src/algorithms/astar.cpp src/heuristics/alt_heuristic.cpp src/algorithms/ida_star.cpp src/dynamic/dynamic_engine.cpp src/benchmark/csv_exporter.cpp src/benchmark/experiment_runner.cpp src/benchmark/benchmark_runner.cpp tests/test_benchmark.cpp -I src -o test_benchmark.exe; .\test_benchmark.exe
  ```

---

## Benchmarking Engine (Phase 7)

### 1. Run Benchmark Smoke-Test (Fast verification across all 7 experiments):
```powershell
.\planner.exe --benchmark --smoke-test --export results/benchmarks
```

### 2. Run Specific Experiments:
```powershell
# Run Experiment 1 (Graph Size Scaling)
.\planner.exe --benchmark --experiment 1 --export results/benchmarks

# Run Experiment 6 (Dynamic Replanning)
.\planner.exe --benchmark --experiment 6 --export results/benchmarks

# Run all experiments with custom repetitions and seed
.\planner.exe --benchmark --experiment all --repetitions 5 --seed 123 --export results/benchmarks
```

### 3. Output CSV Reports:
The benchmark suite generates three files in `results/benchmarks/`:
- `raw_results.csv`: Every individual measured run with 28+ metrics.
- `summary_results.csv`: Aggregated median, mean, and standard deviation per configuration.
- `correctness_results.csv`: Cost verification checks against the Dijkstra oracle.

