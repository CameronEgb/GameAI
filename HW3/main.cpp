#include "graph.h"
#include "pathfinding.h"
#include "steering.h"
#include <iostream>
#include <SFML/Graphics.hpp>
#include <optional>
#include <cmath>

int main() {
    // Part 1: Graphs
    Graph small = createSmallCampusGraph();
    std::cout << "Small graph: UKy Campus, 40 verts" << std::endl;
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
    const float CELL_W = WINDOW_WIDTH / static_cast<float>(GRID_SIZE);
    const float CELL_H = WINDOW_HEIGHT / static_cast<float>(GRID_SIZE);
    Graph indoor(GRID_SIZE * GRID_SIZE, true);
    // Set positions
    for (int y = 0; y < GRID_SIZE; ++y) {
        for (int x = 0; x < GRID_SIZE; ++x) {
            int id = y * GRID_SIZE + x;
            indoor.positions[id] = {static_cast<float>(x) * CELL_W + CELL_W / 2.f,
                                    static_cast<float>(y) * CELL_H + CELL_H / 2.f}; // Center of cell
        }
    }

    // Obstacles/walls
    std::vector<int> obstacles;

    // Perimeter walls
    for (int x = 0; x < GRID_SIZE; ++x) {
        obstacles.push_back(0 * GRID_SIZE + x); // top
        obstacles.push_back((GRID_SIZE-1) * GRID_SIZE + x); // bottom
    }
    for (int y = 0; y < GRID_SIZE; ++y) {
        obstacles.push_back(y * GRID_SIZE + 0); // left
        obstacles.push_back(y * GRID_SIZE + (GRID_SIZE-1)); // right
    }

    // Dividers with openings
    int mid_x = GRID_SIZE / 2;
    int mid_y = GRID_SIZE / 2;
    int opening_size = 3; // Open 3 cells in middle
    int opening_start = (GRID_SIZE / 2) - (opening_size / 2);

    // Vertical divider
    for (int y = 0; y < GRID_SIZE; ++y) {
        if (y < opening_start || y >= opening_start + opening_size) {
            obstacles.push_back(y * GRID_SIZE + mid_x);
        }
    }

    // Horizontal divider
    for (int x = 0; x < GRID_SIZE; ++x) {
        if (x < opening_start || x >= opening_start + opening_size) {
            obstacles.push_back(mid_y * GRID_SIZE + x);
        }
    }

    // 3 arbitrary obstacles (squares: single cells for simplicity)
    obstacles.push_back(5 * GRID_SIZE + 5); // Top-left room
    obstacles.push_back(5 * GRID_SIZE + 6);
    obstacles.push_back(6 * GRID_SIZE + 5);
    obstacles.push_back(6 * GRID_SIZE + 6); // Make 2x2 square

    obstacles.push_back(14 * GRID_SIZE + 5); // Bottom-left
    obstacles.push_back(14 * GRID_SIZE + 6);
    obstacles.push_back(15 * GRID_SIZE + 5);
    obstacles.push_back(15 * GRID_SIZE + 6);

    obstacles.push_back(5 * GRID_SIZE + 14); // Top-right
    obstacles.push_back(5 * GRID_SIZE + 15);
    obstacles.push_back(6 * GRID_SIZE + 14);
    obstacles.push_back(6 * GRID_SIZE + 15);

    // Unique
    std::sort(obstacles.begin(), obstacles.end());
    auto last = std::unique(obstacles.begin(), obstacles.end());
    obstacles.erase(last, obstacles.end());

    // Add edges: 8-dir for smoother paths, skip if source or dest is obstacle
    for (int id = 0; id < indoor.numVertices; ++id) {
        if (std::find(obstacles.begin(), obstacles.end(), id) != obstacles.end()) continue;
        int x = id % GRID_SIZE;
        int y = id / GRID_SIZE;
        // 8 directions
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;
                int nx = x + dx, ny = y + dy;
                if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE) {
                    int nid = ny * GRID_SIZE + nx;
                    if (std::find(obstacles.begin(), obstacles.end(), nid) == obstacles.end()) {
                        float w = (dx != 0 && dy != 0) ? std::sqrt(2.f) : 1.f; // Diagonal cost
                        indoor.addEdge(id, nid, w);
                    }
                }
            }
        }
    }

    // Prepare wall shapes for drawing
    std::vector<sf::RectangleShape> wallShapes;
    for (int obs : obstacles) {
        int x = obs % GRID_SIZE;
        int y = obs / GRID_SIZE;
        sf::RectangleShape rect({CELL_W, CELL_H});
        rect.setPosition({static_cast<float>(x) * CELL_W, static_cast<float>(y) * CELL_H});
        rect.setFillColor(sf::Color(100, 100, 100)); // Gray walls
        wallShapes.push_back(rect);
    }

    Character chara({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}, sf::Color::Red); // Center start
    chara.setBehavior(new ArriveAndAlign());
    chara.getKinematic().orientation = 0.f;
    chara.getKinematic().velocity = {0.f, 0.f}; // Start stationary

    sf::Clock clock;
    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        if (dt <= 0.f) dt = 0.016f;

        // Event loop
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
                    int gx = static_cast<int>(click.x / CELL_W);
                    int gy = static_cast<int>(click.y / CELL_H);
                    int goal = gy * GRID_SIZE + gx;
                    if (std::find(obstacles.begin(), obstacles.end(), goal) != obstacles.end()) continue; // Ignore click on wall

                    // Quantize start from chara
                    int sx = static_cast<int>(chara.getKinematic().position.x / CELL_W);
                    int sy = static_cast<int>(chara.getKinematic().position.y / CELL_H);
                    int start_id = sy * GRID_SIZE + sx;

                    Metrics m;
                    auto pathIds = aStar(indoor, start_id, goal, euclideanHeur, m);
                    if (pathIds.empty()) continue; // No path, ignore

                    std::vector<sf::Vector2f> pathPos;
                    for (int id : pathIds) pathPos.push_back(indoor.positions[id]);
                    chara.setPath(pathPos);
                }
            }
        }

        // Update character (passes dummy, but handles path inside)
        Kinematic dummy;
        chara.update(dt, dummy);

        window.clear(sf::Color(30, 30, 40));
        // Draw walls
        for (auto& rect : wallShapes) {
            window.draw(rect);
        }
        chara.draw(window);
        window.display();
    }

    return 0;
}