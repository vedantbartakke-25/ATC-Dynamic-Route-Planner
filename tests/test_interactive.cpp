#include <iostream>
#include <sstream>
#include <cassert>
#include <string>
#include "cli/interactive_cli.hpp"

void test_interactive_cli_flow() {
    std::cout << "Running automated tests for InteractiveCLI...\n";

    // Scripted input sequence:
    // 1. View info (Option 2)
    // 2. Select Algorithm -> A* Euclidean (Option 3, Choice 2)
    // 3. Find Shortest Path -> 0 to 5 (Option 4, 0, 5)
    // 4. View Current Route (Option 10)
    // 5. Compare All Algorithms (Option 9)
    // 6. Close an edge on route -> suppose edge exists or pick an edge, say close edge between 0 and a neighbor
    // 7. Reopen an edge
    // 8. Update edge weight with inadmissible weight check, then admissible weight
    // 9. Replan route (Option 8)
    // 10. Invalid menu input, non-numeric input, out-of-range vertex
    // 11. Exit (Option 0)

    std::stringstream input;
    // Step 1: Option 2 (Display info)
    input << "2\n";
    // Step 2: Option 3 (Select Algorithm) -> 2 (A* Euclidean)
    input << "3\n2\n";
    // Step 3: Option 4 (Find Shortest Path) -> 0 to 4
    input << "4\n0\n4\n";
    // Step 4: Option 10 (View Current Route)
    input << "10\n";
    // Step 5: Option 9 (Compare All Algorithms)
    input << "9\n";
    // Step 6: Option 3 (Select Algorithm) -> 3 (A* ALT, 3 landmarks)
    input << "3\n3\n3\n";
    // Step 7: Option 3 (Select Algorithm) -> 4 (IDA* ALT, 2 landmarks)
    input << "3\n4\n2\n";
    // Step 8: Option 1 (Generate geometric graph) -> 2 (size 10, Sparse, seed 99)
    input << "1\n2\n10\n1\n99\n";
    // Step 9: Find path on new graph 0 to 3
    input << "4\n0\n3\n";
    // Step 10: Compare all on new graph
    input << "9\n";
    // Step 11: Invalid menu inputs and error handling
    input << "99\ninvalid_text\n";
    // Step 12: Try invalid edge weight (< euclidean distance) on an edge
    // Edge (0, 1): test get_neighbors
    // Step 13: Exit
    input << "0\n";

    std::stringstream output;
    InteractiveCLI cli(input, output);
    cli.run();

    std::string out_str = output.str();
    assert(out_str.find("DYNAMIC SHORTEST PATH PLANNER") != std::string::npos);
    assert(out_str.find("Graph Status: Loaded") != std::string::npos);
    assert(out_str.find("Selected algorithm: A* Euclidean") != std::string::npos);
    assert(out_str.find("ALGORITHM PERFORMANCE COMPARISON") != std::string::npos);
    assert(out_str.find("Exiting Dynamic Shortest Path Planner") != std::string::npos);

    std::cout << "InteractiveCLI basic scripted interaction test PASSED!\n";
}

void test_interactive_dynamic_replanning() {
    std::cout << "Testing InteractiveCLI dynamic edge operations and replanning...\n";

    std::stringstream input;
    // 1. Reset to default graph: Option 1 -> Option 3
    input << "1\n3\n";
    // 2. Select Dijkstra: Option 3 -> Option 1
    input << "3\n1\n";
    // 3. Find path 0 -> 2
    input << "4\n0\n2\n";
    // 4. View route
    input << "10\n";
    // 5. Reopen an edge (Option 6 -> 0, 1)
    input << "6\n0\n1\n";
    // 6. View route (should indicate potentially suboptimal)
    input << "10\n";
    // 7. Replan route (Option 8)
    input << "8\n";
    // 8. Exit (Option 0)
    input << "0\n";

    std::stringstream output;
    InteractiveCLI cli(input, output);
    cli.run();

    std::string out_str = output.str();
    assert(out_str.find("UPDATED ROUTE") != std::string::npos);
    assert(out_str.find("REPLANNING WITH Dijkstra") != std::string::npos);

    std::cout << "InteractiveCLI dynamic replanning test PASSED!\n";
}

int main() {
    test_interactive_cli_flow();
    test_interactive_dynamic_replanning();
    std::cout << "\nAll test_interactive tests PASSED successfully!\n";
    return 0;
}
