#include "steering.h"

// Implement all from HW2 main.cpp (copy the implementations: mapToRange, random..., Breadcrumb update/draw, all calculateSteering...)

// For Character::followPath (new)
void Character::followPath(const std::vector<sf::Vector2f>& path, float dt) {
    if (path.empty()) return;
    // Sequential: set target to next waypoint
    static size_t idx = 0;
    Kinematic target;
    target.position = path[idx];
    update(dt, target); // using ArriveAndAlign behavior
    if (std::hypot(kinematic.position.x - path[idx].x, kinematic.position.y - path[idx].y) < 10.f) { // arrived
        idx++;
        if (idx >= path.size()) idx = 0; // reset or stop
    }
}