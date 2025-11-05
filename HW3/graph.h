#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <utility> // pair
#include <SFML/Graphics.hpp> // for positions

class Graph {
public:
    int numVertices;
    std::vector<std::vector<std::pair<int, float>>> adj; // adj[u] = {v, weight}
    std::vector<sf::Vector2f> positions; // optional for spatial

    Graph(int n, bool spatial = false);
    void addEdge(int u, int v, float w);
    void generateRandomLarge(int n, int avgDegree); // for large
};

Graph createSmallCampusGraph(); // hardcoded

#endif