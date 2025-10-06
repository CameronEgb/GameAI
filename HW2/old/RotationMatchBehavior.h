#pragma once
#include "SteeringBehavior.h"

class RotationMatchBehavior : public SteeringBehavior {
public:
    float timeToTarget = 0.1f;
    float maxAngularAccel = 1.0f;

    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        SteeringOutput result;
        result.linear = {0, 0};

        result.angular = (target.rotation - character.rotation) / timeToTarget;

        // Clamp
        if (std::abs(result.angular) > maxAngularAccel)
            result.angular = (result.angular > 0 ? 1 : -1) * maxAngularAccel;

        return result;
    }
};
