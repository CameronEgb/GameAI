#ifndef PATHFINDING_H
#define PATHFINDING_H

#include "graph.h"
#include <vector>
#include <chrono>
#include <functional> // Added for std::function

// Metrics struct
struct Metrics {
    float runtime_ms;
    int max_fringe;
    int fill; // visited
};

// Path as vector of nodes
std::vector<int> dijkstra(const Graph& g, int start, int goal, Metrics& m);

// Heuristic func type
using Heuristic = std::function<float(int, int, const Graph&)>;

// A* with heuristic
std::vector<int> aStar(const Graph& g, int start, int goal, Heuristic h, Metrics& m);

// Heuristics for small (spatial)
float euclideanHeur(int u, int v, const Graph& g); // admissible
float overEstHeur(int u, int v, const Graph& g); // inadmissible 2x eucl

// For large (cluster)
void initClusters(const Graph& g, int numClusters); // global or something
float clusterHeur(int u, int v, const Graph& g);

// Analysis funcs
void analyzeHeur(const Graph& g, Heuristic h, bool admissible); // prints freq overest etc

#endif