#include "Pathfinder.h"
#include <queue>
#include <limits>
#include <chrono>
#include <iostream>
#include <cmath>
#include <unordered_set>

using clk = std::chrono::high_resolution_clock;

struct PQItem {
    int node;
    double priority; // f = g + h
    double g;
    bool operator<(PQItem const& o) const { // reversed for std::priority_queue (max-heap)
        return priority > o.priority;
    }
};

Pathfinder::Pathfinder(const Graph &g): m_g(g) {}

PathResult Pathfinder::dijkstra(int start, int goal) {
    auto zero = [](int a,int b){ return 0.0; };
    return astar(start, goal, zero);
}

PathResult Pathfinder::astar(int start, int goal, HeuristicFn heuristic) {
    PathResult res;
    res.found = false;
    res.cost = std::numeric_limits<double>::infinity();
    res.exploredCount = 0;
    res.maxFringeSize = 0;
    res.runtimeMs = 0.0;

    if (start < 0 || start >= m_g.numNodes() || goal < 0 || goal >= m_g.numNodes()) return res;

    auto t0 = clk::now();

    const int n = m_g.numNodes();
    std::vector<double> gscore(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    std::vector<char> closed(n, 0);

    std::priority_queue<PQItem> open;
    gscore[start] = 0.0;
    open.push(PQItem{start, heuristic(start, goal), 0.0});

    while (!open.empty()) {
        res.maxFringeSize = std::max(res.maxFringeSize, (long)open.size());
        PQItem cur = open.top(); open.pop();
        int u = cur.node;

        if (closed[u]) continue; // stale entry
        closed[u] = 1;
        res.exploredCount++;

        if (u == goal) {
            // reconstruct path
            std::vector<int> path;
            int curNode = goal;
            while (curNode != -1) {
                path.push_back(curNode);
                curNode = parent[curNode];
            }
            std::reverse(path.begin(), path.end());
            res.found = true;
            res.path = std::move(path);
            res.cost = gscore[goal];
            auto t1 = clk::now();
            res.runtimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
            return res;
        }

        // relax edges
        const auto &edges = m_g.adj()[u];
        for (const Edge &e : edges) {
            int v = e.to;
            double tentative_g = gscore[u] + e.weight;
            if (tentative_g < gscore[v]) {
                gscore[v] = tentative_g;
                parent[v] = u;
                double f = tentative_g + heuristic(v, goal);
                open.push(PQItem{v, f, tentative_g});
            }
        }
    }

    // no path
    auto t1 = clk::now();
    res.runtimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    res.found = false;
    return res;
}
