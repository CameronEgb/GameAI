#include <iostream>
#include <iomanip>
#include "Graph.h"
#include "Pathfinder.h"
#include <chrono>
#include <cmath>

static void printResult(const std::string &title, const PathResult &r) {
    std::cout << "=== " << title << " ===\n";
    std::cout << "Found: " << (r.found ? "yes" : "no") << "\n";
    if (r.found) {
        std::cout << "Path length (nodes): " << r.path.size() << "\n";
        std::cout << "Cost: " << r.cost << "\n";
        std::cout << "Path: ";
        for (size_t i=0;i<r.path.size();++i){
            std::cout << r.path[i] << (i+1<r.path.size() ? " -> " : "\n");
        }
    }
    std::cout << "Explored (popped) nodes: " << r.exploredCount << "\n";
    std::cout << "Max fringe size: " << r.maxFringeSize << "\n";
    std::cout << "Runtime (ms): " << std::fixed << std::setprecision(3) << r.runtimeMs << "\n";
    std::cout << "============================\n\n";
}

int main(int argc, char** argv) {
    std::cout << "Pathfinding demo: Dijkstra and A*\n";

    // Create small meaningful graph
    Graph small = Graph::makeSampleGraph();
    Pathfinder pfSmall(small);

    int start = 0;
    int goal = small.numNodes()-1;

    // Heuristic: Euclidean distance (admissible for positive edge weights equal to Euclidean distances)
    auto euclid = [&small](int a, int b) -> double {
        const auto &na = small.nodes()[a];
        const auto &nb = small.nodes()[b];
        return std::hypot(na.x - nb.x, na.y - nb.y);
    };

    // Dijkstra (A* with zero heuristic)
    auto resD = pfSmall.dijkstra(start, goal);
    printResult("Small graph - Dijkstra", resD);

    // A* with Euclidean heuristic
    auto resA = pfSmall.astar(start, goal, euclid);
    printResult("Small graph - A* (Euclidean)", resA);

    // Inadmissible heuristic example: Euclidean * 1.5 (overestimates)
    auto inadmissible = [&small](int a,int b)->double {
        const auto &na = small.nodes()[a];
        const auto &nb = small.nodes()[b];
        return 1.5 * std::hypot(na.x - nb.x, na.y - nb.y);
    };
    auto resA_bad = pfSmall.astar(start, goal, inadmissible);
    printResult("Small graph - A* (inadmissible x1.5)", resA_bad);

    // Large graph test (no path print), defaults: N=20000, k=4
    int N = 20000;
    int k = 4;
    if (argc >= 3) {
        N = std::stoi(argv[1]);
        k = std::stoi(argv[2]);
    }

    std::cout << "Building large graph: N=" << N << " k=" << k << " (this may take a while)\n";
    Graph large = Graph::makeRandomLargeGraph(N, k, /*seed=*/42);
    Pathfinder pfLarge(large);

    // pick two nodes near center-ish
    int sLarge = 0;
    int gLarge = N-1;

    // simple heuristic: Euclidean on coords
    auto euclidLarge = [&large](int a,int b)->double {
        const auto &na = large.nodes()[a];
        const auto &nb = large.nodes()[b];
        return std::hypot(na.x - nb.x, na.y - nb.y);
    };

    // measure Dijkstra on large graph
    std::cout << "Running Dijkstra on large graph...\n";
    auto resLD = pfLarge.dijkstra(sLarge, gLarge);
    printResult("Large graph - Dijkstra", resLD);

    std::cout << "Running A* (Euclidean) on large graph...\n";
    auto resLA = pfLarge.astar(sLarge, gLarge, euclidLarge);
    printResult("Large graph - A* (Euclidean)", resLA);

    return 0;
}
