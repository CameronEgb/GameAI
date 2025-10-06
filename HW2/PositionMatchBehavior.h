#pragma once
#include "SteeringBehavior.h"

class PositionMatchBehavior : public SteeringBehavior {
public:
    float maxAcceleration = 100.0f;

    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        SteeringOutput result;
        result.linear = target.position - character.position;
        result.angular = 0.0f;

        // Normalize and scale
        float len = std::sqrt(result.linear.x * result.linear.x + result.linear.y * result.linear.y);
        if (len > 0) result.linear = (result.linear / len) * maxAcceleration;
        return result;
    }
};
