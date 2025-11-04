// steering.h - From HW2, minimal changes (added path following)

#ifndef STEERING_H
#define STEERING_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <queue>
#include <cmath>
#include <memory>
#include <random>

constexpr float PI = 3.14159265f;
constexpr int WINDOW_WIDTH = 800;
constexpr int WINDOW_HEIGHT = 600;

// Utilities from HW2...
float mapToRange(float rotation);
float randomBinomial();
float randomFloat(float a, float b);

// Kinematic and SteeringOutput from HW2...
struct Kinematic {
    sf::Vector2f position{0.f,0.f};
    float orientation{0.f};
    sf::Vector2f velocity{0.f,0.f};
    float rotation{0.f};
};

struct SteeringOutput {
    sf::Vector2f linear{0.f,0.f};
    float angular{0.f};
};

// Breadcrumb from HW2...
class Breadcrumb {
    // ... full code from provided main.cpp
    // (copy-paste the Breadcrumb class here)
};

// SteeringBehavior interface from HW2...
class SteeringBehavior {
public:
    virtual ~SteeringBehavior() = default;
    virtual SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) = 0;
};

// All classes from HW2: PositionMatching, VelocityMatching, etc.
// (copy-paste all from provided main.cpp: PositionMatching to WanderKinematic, Separation, Cohesion, etc.)
// BlendedSteering, etc.

// ArriveAndAlign as specified
class ArriveAndAlign : public SteeringBehavior {
    // full code from HW2
};

// Character class from HW2, added followPath
class Character {
    // full from HW2
public:
    // ... 
    void followPath(const std::vector<sf::Vector2f>& path, float dt); // new: sequential arrive/align
};

// Boid from HW2 (if needed, but not for this)

#endif