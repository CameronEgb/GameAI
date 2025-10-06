#pragma once
#include <SFML/System.hpp>

// Basic kinematic data structure
struct Kinematic {
    sf::Vector2f position;
    float orientation;    // in radians
    sf::Vector2f velocity;
    float rotation;       // angular velocity
};

// Acceleration output
struct SteeringOutput {
    sf::Vector2f linear;
    float angular;
};

// Base class
class SteeringBehavior {
public:
    virtual ~SteeringBehavior() = default;
    virtual SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) = 0;
};
