#include "graph.h"
#include "pathfinding.h"
#include "steering.h"
#include <iostream>
#include <SFML/Graphics.hpp>
#include <optional> // For std::optional
#include <cmath> // For std::hypot

int main() {
    // Part 1: Graphs
    Graph small = createSmallCentennialGraph();
    std::cout << "Small graph: NCSU Centennial, 40 verts" << std::endl;
    Graph large(0);
    large.generateRandomLarge(50000, 4); // avg degree 4, ~200k edges
    std::cout << "Large graph: Random, 50k verts" << std::endl;

    // Init clusters for large
    initClusters(large, 100);

    // Part 2: Dijkstra/A* compare on small
    for (int test = 0; test < 5; ++test) {
        int s = rand() % small.numVertices, g = rand() % small.numVertices;
        Metrics md, ma;
        dijkstra(small, s, g, md);
        aStar(small, s, g, euclideanHeur, ma);
        std::cout << "Small Test " << test << ": Dijk rt=" << md.runtime_ms << " fringe=" << md.max_fringe << " fill=" << md.fill << std::endl;
        std::cout << "A* rt=" << ma.runtime_ms << " fringe=" << ma.max_fringe << " fill=" << ma.fill << std::endl;
    }

    // On large
    for (int test = 0; test < 5; ++test) {
        int s = rand() % large.numVertices, g = rand() % large.numVertices;
        Metrics md, ma;
        dijkstra(large, s, g, md);
        aStar(large, s, g, clusterHeur, ma);
        std::cout << "Large Test " << test << ": Dijk rt=" << md.runtime_ms << " fringe=" << md.max_fringe << " fill=" << md.fill << std::endl;
        std::cout << "A* rt=" << ma.runtime_ms << " fringe=" << ma.max_fringe << " fill=" << ma.fill << std::endl;
    }

    // Part 3: Heuristics on small
    analyzeHeur(small, euclideanHeur, true);
    analyzeHeur(small, overEstHeur, false);

    // Part 4: Integration SFML
    sf::RenderWindow window(sf::VideoMode({static_cast<unsigned>(WINDOW_WIDTH), static_cast<unsigned>(WINDOW_HEIGHT)}), "HW3 Integration");
    window.setFramerateLimit(60);

    // Create grid graph for indoor
    const int GRID_SIZE = 20;
    Graph indoor(GRID_SIZE * GRID_SIZE, true);
    // Set positions
    for (int y = 0; y < GRID_SIZE; ++y) {
        for (int x = 0; x < GRID_SIZE; ++x) {
            int id = y * GRID_SIZE + x;
            indoor.positions[id] = {static_cast<float>(x) * (WINDOW_WIDTH / static_cast<float>(GRID_SIZE)),
                                    static_cast<float>(y) * (WINDOW_HEIGHT / static_cast<float>(GRID_SIZE))};
        }
    }
    // Add edges: 4-dir, skip obstacles
    std::vector<int> obstacles = {5*GRID_SIZE+5, 10*GRID_SIZE+10, 15*GRID_SIZE+15}; // example 3 obstacles
    for (int id = 0; id < indoor.numVertices; ++id) {
        if (std::find(obstacles.begin(), obstacles.end(), id) != obstacles.end()) continue;
        int x = id % GRID_SIZE;
        int y = id / GRID_SIZE; // Used to fix warning
        // Right
        if (x < GRID_SIZE-1 && std::find(obstacles.begin(), obstacles.end(), id+1) == obstacles.end())
            indoor.addEdge(id, id+1, 1.f);
        // Left
        if (x > 0 && std::find(obstacles.begin(), obstacles.end(), id-1) == obstacles.end())
            indoor.addEdge(id, id-1, 1.f);
        // Down
        if (y < GRID_SIZE-1 && std::find(obstacles.begin(), obstacles.end(), id+GRID_SIZE) == obstacles.end())
            indoor.addEdge(id, id+GRID_SIZE, 1.f);
        // Up
        if (y > 0 && std::find(obstacles.begin(), obstacles.end(), id-GRID_SIZE) == obstacles.end())
            indoor.addEdge(id, id-GRID_SIZE, 1.f);
    }

    Character chara({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}, sf::Color::Red);
    chara.setBehavior(new ArriveAndAlign()); // as specified
    chara.getKinematic().orientation = 0.f; // Init

    sf::Clock clock;
    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();

        // Event loop (SFML 3.0 style)
        while (true) {
            std::optional<sf::Event> opt = window.pollEvent();
            if (!opt) break;
            const sf::Event &event = *opt;

            if (event.is<sf::Event::Closed>()) {
                window.close();
            } else if (event.is<sf::Event::MouseButtonPressed>()) {
                const auto *mb = event.getIf<sf::Event::MouseButtonPressed>();
                if (mb) {
                    sf::Vector2f click(static_cast<float>(mb->position.x), static_cast<float>(mb->position.y));
                    // Quantize target
                    int gx = static_cast<int>(click.x / (WINDOW_WIDTH / static_cast<float>(GRID_SIZE)));
                    int gy = static_cast<int>(click.y / (WINDOW_HEIGHT / static_cast<float>(GRID_SIZE)));
                    int goal = gy * GRID_SIZE + gx;
                    // Quantize start from chara
                    int sx = static_cast<int>(chara.getKinematic().position.x / (WINDOW_WIDTH / static_cast<float>(GRID_SIZE)));
                    int sy = static_cast<int>(chara.getKinematic().position.y / (WINDOW_HEIGHT / static_cast<float>(GRID_SIZE)));
                    int start_id = sy * GRID_SIZE + sx;
                    Metrics m;
                    auto pathIds = aStar(indoor, start_id, goal, euclideanHeur, m);
                    std::vector<sf::Vector2f> pathPos;
                    for (int id : pathIds) pathPos.push_back(indoor.positions[id]);
                    chara.followPath(pathPos, dt); // Update with path
                }
            }
        }

        // Update character (even without path, but followPath handles)
        Kinematic dummyTarget; // If no path, perhaps wander or nothing
        chara.update(dt, dummyTarget);

        window.clear(sf::Color(30, 30, 40));
        chara.draw(window);
        // TODO: Draw grid/rooms/obstacles for visualization (optional rectangles)
        window.display();
    }

    return 0;
}