#pragma once
#include <vector>
#include <unordered_map>
#include <string>
#include <utility>

struct Edge {
    int to;
    double weight;
    Edge(int t=0, double w=1.0): to(t), weight(w) {}
};

struct Node {
    int id;
    double x, y; // optional coordinates for heuristics / visualization
    std::string label;
    Node(int id_=0, double x_=0.0, double y_=0.0, std::string label_=""):
        id(id_), x(x_), y(y_), label(label_) {}
};

class Graph {
public:
    Graph() = default;

    // add node; returns node id (should match provided id if consistent)
    int addNode(double x=0.0, double y=0.0, const std::string &label="");

    // add directed edge from u -> v with positive weight
    void addEdge(int u, int v, double weight);

    // accessors
    const std::vector<Node>& nodes() const { return m_nodes; }
    const std::vector<std::vector<Edge>>& adj() const { return m_adj; }

    int numNodes() const { return (int)m_nodes.size(); }
    void clear();

    // convenience: build a small meaningful sample graph (road-like)
    static Graph makeSampleGraph();

    // convenience: build a large random geometric k-nearest neighbor graph
    // N = number of nodes; k = neighbors per node (approx)
    static Graph makeRandomLargeGraph(int N, int k, unsigned seed=0);

private:
    std::vector<Node> m_nodes;
    std::vector<std::vector<Edge>> m_adj;
};
