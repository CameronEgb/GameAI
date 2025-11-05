#ifndef STEERING_H
#define STEERING_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <queue>
#include <cmath>
#include <memory>
#include <random>
#include <algorithm>
#include <cstdint>
#include <optional>

constexpr float PI = 3.14159265f;
constexpr int WINDOW_WIDTH = 800;
constexpr int WINDOW_HEIGHT = 600;

// ---------------------------------------------------------------------
// Utilities
float mapToRange(float rotation);
float randomBinomial();
float randomFloat(float a, float b);

// ---------------------------------------------------------------------
// Data
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

// ---------------------------------------------------------------------
// Breadcrumbs
class Breadcrumb {
    std::queue<sf::Vector2f> q;
    int maxCrumbs;
    int dropInterval;
    int counter;
    sf::Color color;
public:
    Breadcrumb(int maxCrumbs_=30, int dropInterval_=5, sf::Color c=sf::Color::Blue);
    void update(const sf::Vector2f &pos);
    void draw(sf::RenderWindow &win);
    void clear();
};

// ---------------------------------------------------------------------
// Steering interface
class SteeringBehavior {
public:
    virtual ~SteeringBehavior() = default;
    virtual SteeringOutput calculateSteering(const Kinematic &character,
                                             const Kinematic &target) = 0;
};

// ---------------------------------------------------------------------
// Basic steering algorithms
class PositionMatching : public SteeringBehavior {
    float maxAccel;
public:
    explicit PositionMatching(float m = 100.f);
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override;
};

class VelocityMatching : public SteeringBehavior {
protected:
    float maxAccel;
    float timeToTarget;
public:
    VelocityMatching(float maxA=100.f, float time=0.1f);
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override;
};

class FastVelocityMatching : public VelocityMatching {
public:
    FastVelocityMatching();
};

class OrientationMatching : public SteeringBehavior {
    float maxAngAccel;
    float maxRotation;
    float targetRadius;
    float slowRadius;
    float timeToTarget;
public:
    OrientationMatching(float maxAA=5.f, float maxR=2.f);
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override;
};

class RotationMatching : public SteeringBehavior {
    float maxAngAccel;
    float timeToTarget;
public:
    RotationMatching(float maxA=5.f, float time=0.1f);
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override;
};

class Arrive : public SteeringBehavior {
    float maxAcceleration;
    float maxSpeed;
    float targetRadius;
    float slowRadius;
    float timeToTarget;
public:
    Arrive(float maxAccel=200.f,float maxSpd=100.f,float tRad=5.f,float sRad=100.f,float time=0.1f);
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override;

    // **Declaration only** – definition lives in steering.cpp
    void setSlowRadius(float r);
};

class Align : public SteeringBehavior {
    float maxAngularAcceleration;
    float maxRotation;
    float targetRadius;
    float slowRadius;
    float timeToTarget;
public:
    Align(float maxAngAccel=5.f, float maxRot=2.f, float tRad=0.01f,
          float sRad=0.5f, float time=0.1f);
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override;
};

class SmoothAlign : public Align {
public:
    SmoothAlign();
};

class Face : public SteeringBehavior {
    Align align;
public:
    Face();
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override;
};

class ArriveAndAlign : public SteeringBehavior {
private:
    Arrive arrive;
    Align  align;
public:
    ArriveAndAlign(float arrive_maxAccel = 200.f,
                   float arrive_maxSpeed = 100.f,
                   float arrive_targetRadius = 5.f,
                   float arrive_slowRadius = 100.f,
                   float arrive_timeToTarget = 0.1f,
                   float align_maxAngularAccel = 5.f,
                   float align_maxRotation = 2.f,
                   float align_targetRadius = 0.01f,
                   float align_slowRadius = 0.5f,
                   float align_timeToTarget = 0.1f);
    SteeringOutput calculateSteering(const Kinematic &character,
                                     const Kinematic &target) override;
    void setSlowRadius(float r);          // declaration only
};

class LookWhereYoureGoing : public SteeringBehavior {
    Align align;
public:
    LookWhereYoureGoing();
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &/*t*/) override;
};

class WallAvoidance : public SteeringBehavior {
    float wallMargin;
    float maxAcceleration;
    float detectionDistance;
public:
    WallAvoidance(float margin=20.f, float maxAcc=300.f, float detectDist=120.f);
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &/*t*/) override;
};

// ---------------------------------------------------------------------
// Character
class Character {
    Kinematic kinematic;
    Breadcrumb breadcrumbs;
    sf::CircleShape shape;
    SteeringBehavior *currentBehavior;
    float maxSpeed;
    float maxRotation;
    std::vector<sf::Vector2f> currentPath;
    size_t currentWaypoint = 0;
public:
    Character(sf::Vector2f start, sf::Color color = sf::Color::Blue);
    void setBehavior(SteeringBehavior *b);
    Kinematic &getKinematic();
    void clearBreadcrumbs();
    void setMaxSpeed(float s);
    void setPosition(sf::Vector2f p);
    void update(float dt, const Kinematic &target);
    void updateWithBoundaryHandling(float dt, const Kinematic &target);
    void draw(sf::RenderWindow &win);
    void setPath(const std::vector<sf::Vector2f>& path);
};

#endif