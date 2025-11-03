#pragma once
#include "Graph.h"
#include <vector>
#include <functional>
#include <optional>

struct PathResult {
    bool found;
    std::vector<int> path; // node ids from start to goal
    double cost;
    long exploredCount; // number of nodes popped from frontier
    long maxFringeSize;
    double runtimeMs;
};

class Pathfinder {
public:
    using HeuristicFn = std::function<double(int,int)>;

    Pathfinder(const Graph &g);

    // Dijkstra wrapper (heuristic = zero)
    PathResult dijkstra(int start, int goal);

    // A* with heuristic function (heuristic(start, goal) should be admissible for correct A*)
    PathResult astar(int start, int goal, HeuristicFn heuristic);

private:
    const Graph &m_g;
};
