#include "graph.h"
#include "pathfinding.h"
#include "steering.h"
#include <iostream>
#include <SFML/Graphics.hpp>

int main() {
    // Part 1: Graphs
    Graph small = createSmallCentennialGraph();
    std::cout << "Small graph: NCSU Centennial, 40 verts" << std::endl;
    Graph large(0);
    large.generateRandomLarge(50000, 4); // avg degree 4, ~200k edges
    std::cout << "Large graph: Random, 50k verts" << std::endl;

    // Init clusters for large
    initClusters(large, 100);

    // Part 2: Dijkstra/A* compare
    for (int test = 0; test < 5; ++test) {
        int s = rand() % small.numVertices, g = rand() % small.numVertices;
        Metrics md, ma;
        dijkstra(small, s, g, md);
        aStar(small, s, g, euclideanHeur, ma);
        std::cout << "Small Test " << test << ": Dijk rt=" << md.runtime_ms << " fringe=" << md.max_fringe << " fill=" << md.fill << std::endl;
        std::cout << "A* rt=" << ma.runtime_ms << " fringe=" << ma.max_fringe << " fill=" << ma.fill << std::endl;
    }
    // Similar for large, using clusterHeur for A*

    // Part 3: Heuristics on small
    analyzeHeur(small, euclideanHeur, true);
    analyzeHeur(small, overEstHeur, false);

    // Part 4: Integration SFML
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "HW3 Integration");
    // Create grid graph for indoor
    const int GRID_SIZE = 20;
    Graph indoor(GRID_SIZE * GRID_SIZE, true);
    // Set positions: row-major, scale to window
    for (int y = 0; y < GRID_SIZE; ++y) for (int x = 0; x < GRID_SIZE; ++x) {
        int id = y * GRID_SIZE + x;
        indoor.positions[id] = {x * (WINDOW_WIDTH / GRID_SIZE), y * (WINDOW_HEIGHT / GRID_SIZE)};
    }
    // Add edges: 4-dir, skip obstacles
    // Simple 2x2 rooms: divide at 10, openings at mid
    // Obstacles: say 3 blocks
    std::vector<int> obstacles = {5*GRID_SIZE+5, 10*GRID_SIZE+10, 15*GRID_SIZE+15}; // example
    for (int id = 0; id < indoor.numVertices; ++id) {
        if (std::find(obstacles.begin(), obstacles.end(), id) != obstacles.end()) continue;
        int x = id % GRID_SIZE, y = id / GRID_SIZE;
        // Right, down, etc., with weights
        if (x < GRID_SIZE-1 && std::find(obstacles.begin(), obstacles.end(), id+1) == obstacles.end())
            indoor.addEdge(id, id+1, 1.f);
        // Add other dirs, bidirectional for undirected feel, but directed possible
    }
    // Rooms: assume connected via openings (no walls block all)

    Character chara({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}, sf::Color::Red);
    chara.setBehavior(new ArriveAndAlign()); // as specified

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
            if (event.type == sf::Event::MouseButtonPressed) {
                sf::Vector2f click(event.mouseButton.x, event.mouseButton.y);
                // Quantize to grid
                int gx = click.x / (WINDOW_WIDTH / GRID_SIZE), gy = click.y / (WINDOW_HEIGHT / GRID_SIZE);
                int goal = gy * GRID_SIZE + gx;
                int start = /* compute from chara.position similarly */;
                Metrics m;
                auto pathIds = aStar(indoor, start, goal, euclideanHeur, m);
                std::vector<sf::Vector2f> pathPos;
                for (int id : pathIds) pathPos.push_back(indoor.positions[id]);
                chara.followPath(pathPos, 0.f); // init
            }
        }
        // Update chara with dt, draw breadcrumbs, etc.
        // Draw rooms/obstacles as rects
        window.clear();
        // ... draw code
        window.display();
    }

    return 0;
}