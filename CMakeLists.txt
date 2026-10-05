cmake_minimum_required(VERSION 3.10)
project(DAA_Route_Planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add include directories
include_directories(src)

# Core library
add_library(core src/core/graph.cpp)

# Generator library
add_library(generator src/generator/graph_generator.cpp)
target_link_libraries(generator core)

# Main executable
add_executable(planner src/main.cpp)
target_link_libraries(planner core generator)

# Tests executable
add_executable(test_graph tests/test_graph.cpp)
target_link_libraries(test_graph core generator)
