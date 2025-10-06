#pragma once
#include "SteeringBehavior.h"

class VelocityMatchBehavior : public SteeringBehavior {
public:
    float timeToTarget = 0.1f;
    float maxAccel = 50.0f;

    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        SteeringOutput result;
        result.linear = (target.velocity - character.velocity) / timeToTarget;

        // Limit acceleration
        float len = std::sqrt(result.linear.x * result.linear.x + result.linear.y * result.linear.y);
        if (len > maxAccel)
            result.linear = (result.linear / len) * maxAccel;

        result.angular = 0;
        return result;
    }
};
