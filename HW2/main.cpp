// main.cpp - SFML 3.0.0 compatible full steering behaviors demo
// Ported/patched for SFML 3 API changes.

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <memory>
#include <random>
#include <algorithm>
#include <cstdint>
#include <optional>

const float PI = 3.14159265f;
const int WINDOW_WIDTH = 600;
const int WINDOW_HEIGHT = 600;

// Forward declarations
struct Kinematic;
struct SteeringOutput;

// Util
float mapToRange(float rotation)
{
    while (rotation > PI) rotation -= 2 * PI;
    while (rotation < -PI) rotation += 2 * PI;
    return rotation;
}

float randomBinomial()
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0.0, 1.0);
    return dis(gen) - dis(gen);
}

float randomFloat(float min, float max)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen);
}

// Data
struct Kinematic
{
    sf::Vector2f position;
    float orientation;
    sf::Vector2f velocity;
    float rotation;

    Kinematic() : position(0.f, 0.f), orientation(0.f), velocity(0.f, 0.f), rotation(0.f) {}
};

struct SteeringOutput
{
    sf::Vector2f linear;
    float angular;
    SteeringOutput() : linear(0.f, 0.f), angular(0.f) {}
};

// Breadcrumbs
class Breadcrumb
{
private:
    std::queue<sf::Vector2f> positions;
    int maxCrumbs;
    int dropInterval;
    int counter;
    sf::Color color;

public:
    Breadcrumb(int max = 30, int interval = 5, sf::Color c = sf::Color::Blue)
        : maxCrumbs(max), dropInterval(interval), counter(0), color(c) {}

    void update(const sf::Vector2f &pos)
    {
        if (++counter >= dropInterval)
        {
            counter = 0;
            positions.push(pos);
            if ((int)positions.size() > maxCrumbs) positions.pop();
        }
    }

    void draw(sf::RenderWindow &window)
    {
        std::queue<sf::Vector2f> temp = positions;
        float alpha = 50.0f;
        float alphaInc = 200.0f / std::max(1, maxCrumbs);

        while (!temp.empty())
        {
            sf::CircleShape crumb(3.f);
            crumb.setPosition(temp.front() - sf::Vector2f(3.f, 3.f));
            sf::Color c = color;
            c.a = static_cast<std::uint8_t>(std::min(255.0f, alpha));
            crumb.setFillColor(c);
            window.draw(crumb);
            temp.pop();
            alpha += alphaInc;
        }
    }

    void clear()
    {
        while (!positions.empty()) positions.pop();
        counter = 0;
    }
};

// Steering interface
class SteeringBehavior
{
public:
    virtual ~SteeringBehavior() {}
    virtual SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) = 0;
};

// PositionMatching
class PositionMatching : public SteeringBehavior
{
private:
    float maxAcceleration;

public:
    PositionMatching(float maxAccel = 100.0f) : maxAcceleration(maxAccel) {}
    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) override
    {
        SteeringOutput result;
        sf::Vector2f direction = target.position - character.position;
        float distance = std::sqrt(direction.x*direction.x + direction.y*direction.y);
        if (distance > 0.001f)
        {
            direction /= distance;
            result.linear = direction * maxAcceleration;
        }
        return result;
    }
};

// VelocityMatching
class VelocityMatching : public SteeringBehavior
{
private:
    float maxAcceleration;
    float timeToTarget;

public:
    VelocityMatching(float maxAccel = 100.0f, float time = 0.1f)
        : maxAcceleration(maxAccel), timeToTarget(time) {}

    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) override
    {
        SteeringOutput result;
        result.linear = (target.velocity - character.velocity) / timeToTarget;
        float length = std::sqrt(result.linear.x*result.linear.x + result.linear.y*result.linear.y);
        if (length > maxAcceleration)
            result.linear = (result.linear / length) * maxAcceleration;
        return result;
    }
};

class FastVelocityMatching : public VelocityMatching
{
public:
    FastVelocityMatching() : VelocityMatching(400.0f, 0.05f) {}
};

// OrientationMatching
class OrientationMatching : public SteeringBehavior
{
private:
    float maxAngularAcceleration;
    float maxRotation;
    float targetRadius;
    float slowRadius;
    float timeToTarget;

public:
    OrientationMatching(float maxAngAccel = 5.0f, float maxRot = 2.0f)
        : maxAngularAcceleration(maxAngAccel), maxRotation(maxRot),
          targetRadius(0.01f), slowRadius(0.5f), timeToTarget(0.1f) {}

    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) override
    {
        SteeringOutput result;
        float rotation = target.orientation - character.orientation;
        rotation = mapToRange(rotation);
        float rotationSize = std::abs(rotation);
        if (rotationSize < targetRadius) return result;
        float targetRotation;
        if (rotationSize > slowRadius) targetRotation = maxRotation;
        else targetRotation = maxRotation * rotationSize / slowRadius;
        targetRotation *= rotation / rotationSize;
        result.angular = (targetRotation - character.rotation) / timeToTarget;
        float angularAcceleration = std::abs(result.angular);
        if (angularAcceleration > maxAngularAcceleration)
        {
            result.angular /= angularAcceleration;
            result.angular *= maxAngularAcceleration;
        }
        return result;
    }
};

// RotationMatching
class RotationMatching : public SteeringBehavior
{
private:
    float maxAngularAcceleration;
    float timeToTarget;
public:
    RotationMatching(float maxAngAccel = 5.0f, float time = 0.1f)
        : maxAngularAcceleration(maxAngAccel), timeToTarget(time) {}
    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) override
    {
        SteeringOutput result;
        result.angular = (target.rotation - character.rotation) / timeToTarget;
        if (std::abs(result.angular) > maxAngularAcceleration)
            result.angular = (result.angular / std::abs(result.angular)) * maxAngularAcceleration;
        return result;
    }
};

// Arrive
class Arrive : public SteeringBehavior
{
private:
    float maxAcceleration;
    float maxSpeed;
    float targetRadius;
    float slowRadius;
    float timeToTarget;

public:
    Arrive(float maxAccel = 200.0f, float maxSpd = 100.0f, float targetRad = 5.0f,
           float slowRad = 100.0f, float time = 0.1f)
        : maxAcceleration(maxAccel), maxSpeed(maxSpd), targetRadius(targetRad), slowRadius(slowRad), timeToTarget(time) {}

    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) override
    {
        SteeringOutput result;
        sf::Vector2f direction = target.position - character.position;
        float distance = std::sqrt(direction.x*direction.x + direction.y*direction.y);
        if (distance < targetRadius) return result;
        float targetSpeed = (distance > slowRadius) ? maxSpeed : maxSpeed * distance / slowRadius;
        sf::Vector2f targetVelocity = direction;
        if (distance > 0.0001f) targetVelocity = (targetVelocity / distance) * targetSpeed;
        result.linear = targetVelocity - character.velocity;
        result.linear /= timeToTarget;
        float length = std::sqrt(result.linear.x*result.linear.x + result.linear.y*result.linear.y);
        if (length > maxAcceleration) result.linear = (result.linear / length) * maxAcceleration;
        return result;
    }
};

// Align
class Align : public SteeringBehavior
{
private:
    float maxAngularAcceleration;
    float maxRotation;
    float targetRadius;
    float slowRadius;
    float timeToTarget;

public:
    Align(float maxAngAccel = 5.0f, float maxRot = 2.0f, float targetRad = 0.01f,
          float slowRad = 0.5f, float time = 0.1f)
        : maxAngularAcceleration(maxAngAccel), maxRotation(maxRot),
          targetRadius(targetRad), slowRadius(slowRad), timeToTarget(time) {}

    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) override
    {
        SteeringOutput result;
        float rotation = target.orientation - character.orientation;
        rotation = mapToRange(rotation);
        float rotationSize = std::abs(rotation);
        if (rotationSize < targetRadius) return result;
        float targetRotation = (rotationSize > slowRadius) ? maxRotation : maxRotation * rotationSize / slowRadius;
        targetRotation *= rotation / rotationSize;
        result.angular = (targetRotation - character.rotation) / timeToTarget;
        float angularAcceleration = std::abs(result.angular);
        if (angularAcceleration > maxAngularAcceleration)
        {
            result.angular /= angularAcceleration;
            result.angular *= maxAngularAcceleration;
        }
        return result;
    }
};

class SmoothAlign : public Align
{
public:
    SmoothAlign() : Align(2.5f, 1.5f, 0.01f, 0.6f, 0.1f) {}
};

// ArriveAndAlign
class ArriveAndAlign : public SteeringBehavior
{
private:
    Arrive arrive;
    Align align;

public:
    ArriveAndAlign(
        // Arrive params
        float arrive_maxSpeed, float arrive_radius, float arrive_timeToTarget, float arrive_slowRadius, float arrive_maxAcceleration,
        // Align params
        float align_maxRotation, float align_radius, float align_timeToTarget, float align_slowRadius, float align_maxAngularAcceleration)
        // Note: Align ctor order is (maxAngAccel, maxRot, targetRadius, slowRadius, timeToTarget)
        : arrive(arrive_maxAcceleration, arrive_maxSpeed, arrive_radius, arrive_slowRadius, arrive_timeToTarget),
          align(align_maxAngularAcceleration, align_maxRotation, align_radius, align_slowRadius, align_timeToTarget)
    {
    }

    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) override
    {
        SteeringOutput arriveResult = arrive.calculateSteering(character, target);
        SteeringOutput alignResult = align.calculateSteering(character, target);
        SteeringOutput result;
        result.linear = arriveResult.linear;
        result.angular = alignResult.angular;
        return result;
    }
};

// Face
class Face : public SteeringBehavior
{
private:
    Align align;
public:
    Face() : align() {}
    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) override
    {
        sf::Vector2f direction = target.position - character.position;
        if (direction.x == 0.f && direction.y == 0.f) return SteeringOutput();
        Kinematic faceTarget;
        faceTarget.orientation = std::atan2(direction.y, direction.x);
        return align.calculateSteering(character, faceTarget);
    }
};

// LookWhereYoureGoing
class LookWhereYoureGoing : public SteeringBehavior
{
private:
    Align align;
public:
    LookWhereYoureGoing() : align() {}
    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic & /*target*/) override
    {
        if (character.velocity.x == 0.f && character.velocity.y == 0.f) return SteeringOutput();
        Kinematic alignTarget;
        alignTarget.orientation = std::atan2(character.velocity.y, character.velocity.x);
        return align.calculateSteering(character, alignTarget);
    }
};

// WallAvoidance
class WallAvoidance : public SteeringBehavior
{
private:
    float wallMargin;
    float maxAcceleration;
    float detectionDistance;
public:
    WallAvoidance(float margin = 20.0f, float maxAccel = 300.0f, float detectDist = 120.0f)
        : wallMargin(margin), maxAcceleration(maxAccel), detectionDistance(detectDist) {}
    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic & /*target*/) override
    {
        SteeringOutput result;
        sf::Vector2f avoidanceForce(0.f, 0.f);
        float distToLeft = character.position.x;
        float distToRight = WINDOW_WIDTH - character.position.x;
        float distToTop = character.position.y;
        float distToBottom = WINDOW_HEIGHT - character.position.y;

        if (distToLeft < detectionDistance)
        {
            float strength = (detectionDistance - distToLeft) / detectionDistance;
            avoidanceForce.x += strength * maxAcceleration;
        }
        if (distToRight < detectionDistance)
        {
            float strength = (detectionDistance - distToRight) / detectionDistance;
            avoidanceForce.x -= strength * maxAcceleration;
        }
        if (distToTop < detectionDistance)
        {
            float strength = (detectionDistance - distToTop) / detectionDistance;
            avoidanceForce.y += strength * maxAcceleration;
        }
        if (distToBottom < detectionDistance)
        {
            float strength = (detectionDistance - distToBottom) / detectionDistance;
            avoidanceForce.y -= strength * maxAcceleration;
        }
        result.linear = avoidanceForce;
        return result;
    }
};

// Wander
class Wander : public SteeringBehavior
{
private:
    float wanderOffset;
    float wanderRadius;
    float wanderRate;
    float wanderOrientation;
    float maxAcceleration;
    LookWhereYoureGoing lookWhereGoing;

public:
    Wander(float offset = 60.0f, float radius = 40.0f, float rate = 0.5f, float maxAccel = 80.0f)
        : wanderOffset(offset), wanderRadius(radius), wanderRate(rate), wanderOrientation(0.f), maxAcceleration(maxAccel), lookWhereGoing() {}

    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) override
    {
        wanderOrientation += randomBinomial() * wanderRate;
        float targetOrientation = wanderOrientation + character.orientation;

        sf::Vector2f characterOrientationVec(std::cos(character.orientation), std::sin(character.orientation));
        sf::Vector2f wanderCircleCenter = character.position + characterOrientationVec * wanderOffset;

        sf::Vector2f wanderTarget;
        wanderTarget.x = wanderCircleCenter.x + wanderRadius * std::cos(targetOrientation);
        wanderTarget.y = wanderCircleCenter.y + wanderRadius * std::sin(targetOrientation);

        SteeringOutput result;
        result.linear = wanderTarget - character.position;
        float length = std::sqrt(result.linear.x*result.linear.x + result.linear.y*result.linear.y);
        if (length > 0.f) result.linear = (result.linear / length) * maxAcceleration;

        SteeringOutput lookSteering = lookWhereGoing.calculateSteering(character, target);
        result.angular = lookSteering.angular;
        return result;
    }
};

// WanderKinematic
class WanderKinematic : public SteeringBehavior
{
private:
    float wanderOffset;
    float wanderRadius;
    float wanderRate;
    float wanderOrientation;
    float maxAcceleration;
    float maxRotation;

public:
    WanderKinematic(float offset = 60.0f, float radius = 40.0f, float rate = 0.5f, float maxAccel = 80.0f, float maxRot = 2.0f)
        : wanderOffset(offset), wanderRadius(radius), wanderRate(rate), wanderOrientation(0.f), maxAcceleration(maxAccel), maxRotation(maxRot) {}

    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic & /*target*/) override
    {
        wanderOrientation += randomBinomial() * wanderRate;
        float targetOrientation = wanderOrientation + character.orientation;

        sf::Vector2f characterOrientationVec(std::cos(character.orientation), std::sin(character.orientation));
        sf::Vector2f wanderCircleCenter = character.position + characterOrientationVec * wanderOffset;

        sf::Vector2f wanderTarget;
        wanderTarget.x = wanderCircleCenter.x + wanderRadius * std::cos(targetOrientation);
        wanderTarget.y = wanderCircleCenter.y + wanderRadius * std::sin(targetOrientation);

        SteeringOutput result;
        result.linear = wanderTarget - character.position;
        float length = std::sqrt(result.linear.x*result.linear.x + result.linear.y*result.linear.y);
        if (length > 0.f) result.linear = (result.linear / length) * maxAcceleration;

        sf::Vector2f toTarget = wanderTarget - character.position;
        float desiredOrientation = std::atan2(toTarget.y, toTarget.x);
        float rotationDiff = desiredOrientation - character.orientation;
        rotationDiff = mapToRange(rotationDiff);

        result.angular = rotationDiff * 3.0f;
        if (std::abs(result.angular) > maxRotation)
            result.angular = (result.angular / std::abs(result.angular)) * maxRotation;

        return result;
    }
};

// Separation
class Separation : public SteeringBehavior
{
private:
    float threshold;
    float decayCoefficient;
    float maxAcceleration;
    std::vector<Kinematic *> *boids;

public:
    Separation(std::vector<Kinematic *> *b, float thresh = 100.0f, float decay = 5000.0f, float maxAccel = 100.0f)
        : threshold(thresh), decayCoefficient(decay), maxAcceleration(maxAccel), boids(b) {}

    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic & /*target*/) override
    {
        SteeringOutput result;
        for (auto &boid : *boids)
        {
            if (boid->position == character.position) continue;
            sf::Vector2f direction = character.position - boid->position;
            float distance = std::sqrt(direction.x*direction.x + direction.y*direction.y);
            if (distance < threshold && distance > 0.f)
            {
                float strength = std::min(decayCoefficient / (distance * distance), maxAcceleration);
                direction = (direction / distance) * strength;
                result.linear += direction;
            }
        }
        float length = std::sqrt(result.linear.x*result.linear.x + result.linear.y*result.linear.y);
        if (length > maxAcceleration) result.linear = (result.linear / length) * maxAcceleration;
        return result;
    }
};

// Cohesion
class Cohesion : public SteeringBehavior
{
private:
    float neighborhoodRadius;
    float maxAcceleration;
    std::vector<Kinematic *> *boids;
    Arrive arrive;

public:
    Cohesion(std::vector<Kinematic *> *b, float radius = 150.0f, float maxAccel = 10.0f)
        : neighborhoodRadius(radius), maxAcceleration(maxAccel), boids(b),
          arrive(maxAccel, 100.0f, 10.0f, 50.0f) {}

    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic & /*target*/) override
    {
        sf::Vector2f centerOfMass(0.f, 0.f);
        int count = 0;
        for (auto &boid : *boids)
        {
            if (boid->position == character.position) continue;
            sf::Vector2f direction = boid->position - character.position;
            float distance = std::sqrt(direction.x*direction.x + direction.y*direction.y);
            if (distance < neighborhoodRadius) { centerOfMass += boid->position; ++count; }
        }
        if (count == 0) return SteeringOutput();
        centerOfMass /= static_cast<float>(count);
        Kinematic cohesionTarget;
        cohesionTarget.position = centerOfMass;
        return arrive.calculateSteering(character, cohesionTarget);
    }
};

// Alignment
class Alignment : public SteeringBehavior
{
private:
    float neighborhoodRadius;
    float maxAcceleration;
    std::vector<Kinematic *> *boids;
    VelocityMatching velocityMatch;

public:
    Alignment(std::vector<Kinematic *> *b, float radius = 100.0f, float maxAccel = 50.0f)
        : neighborhoodRadius(radius), maxAcceleration(maxAccel), boids(b), velocityMatch(maxAccel) {}

    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic & /*target*/) override
    {
        sf::Vector2f averageVelocity(0.f, 0.f);
        int count = 0;
        for (auto &boid : *boids)
        {
            if (boid->position == character.position) continue;
            sf::Vector2f direction = boid->position - character.position;
            float distance = std::sqrt(direction.x*direction.x + direction.y*direction.y);
            if (distance < neighborhoodRadius) { averageVelocity += boid->velocity; ++count; }
        }
        if (count == 0) return SteeringOutput();
        averageVelocity /= static_cast<float>(count);
        Kinematic alignTarget;
        alignTarget.velocity = averageVelocity;
        return velocityMatch.calculateSteering(character, alignTarget);
    }
};

// BlendedSteering
class BlendedSteering : public SteeringBehavior
{
private:
    struct BehaviorAndWeight { SteeringBehavior *behavior; float weight; };
    std::vector<BehaviorAndWeight> behaviors;
    float maxAcceleration;
    float maxAngular;

public:
    BlendedSteering(float maxAccel = 200.0f, float maxAng = 5.0f) : maxAcceleration(maxAccel), maxAngular(maxAng) {}
    void addBehavior(SteeringBehavior *behavior, float weight) { behaviors.push_back({behavior, weight}); }
    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) override
    {
        SteeringOutput result;
        for (auto &bw : behaviors)
        {
            SteeringOutput steering = bw.behavior->calculateSteering(character, target);
            result.linear += steering.linear * bw.weight;
            result.angular += steering.angular * bw.weight;
        }
        float length = std::sqrt(result.linear.x*result.linear.x + result.linear.y*result.linear.y);
        if (length > maxAcceleration) result.linear = (result.linear / length) * maxAcceleration;
        if (std::abs(result.angular) > maxAngular)
            result.angular = (result.angular / std::abs(result.angular)) * maxAngular;
        return result;
    }
};

// Character
class Character
{
private:
    Kinematic kinematic;
    sf::Sprite sprite;
    sf::Texture boidTexture;
    Breadcrumb breadcrumbs;
    SteeringBehavior *currentBehavior;
    float maxSpeed;
    float maxRotation;

public:
    Character(sf::Vector2f startPos = sf::Vector2f(400.f, 300.f), sf::Color color = sf::Color::Red,
              int breadcrumbMax = 30, int breadcrumbInterval = 5)
        : breadcrumbs(breadcrumbMax, breadcrumbInterval, color), currentBehavior(nullptr), maxSpeed(150.0f), maxRotation(3.0f)
    {
        kinematic.position = startPos;
        kinematic.orientation = 0.f;

        // Try load texture; if failed, generate a tiny fallback image
        if (!boidTexture.loadFromFile("boid.png"))
        {
            sf::Image img({32u, 32u}, sf::Color::Transparent);
            for (unsigned y = 0; y < 32; ++y)
                for (unsigned x = 0; x < 32; ++x)
                    img.setPixel({x, y}, sf::Color(200, 200, 200));
            boidTexture.loadFromImage(img);
        }

        sprite = sf::Sprite(boidTexture);
        sprite.setOrigin({static_cast<float>(boidTexture.getSize().x) / 2.f, static_cast<float>(boidTexture.getSize().y) / 2.f});
        sprite.setScale({0.05f, 0.05f});
        sprite.setColor(color);
    }

    void setBehavior(SteeringBehavior *behavior) { currentBehavior = behavior; }

    void update(float deltaTime, const Kinematic &target)
    {
        if (!currentBehavior) return;

        SteeringOutput steering = currentBehavior->calculateSteering(kinematic, target);

        kinematic.position += kinematic.velocity * deltaTime;
        kinematic.orientation += kinematic.rotation * deltaTime;

        kinematic.velocity += steering.linear * deltaTime;
        kinematic.rotation += steering.angular * deltaTime;

        float speed = std::sqrt(kinematic.velocity.x*kinematic.velocity.x + kinematic.velocity.y*kinematic.velocity.y);
        if (speed > maxSpeed) kinematic.velocity = (kinematic.velocity / speed) * maxSpeed;

        if (std::abs(kinematic.rotation) > maxRotation)
            kinematic.rotation = (kinematic.rotation / std::abs(kinematic.rotation)) * maxRotation;

        kinematic.orientation = mapToRange(kinematic.orientation);

        breadcrumbs.update(kinematic.position);

        sprite.setPosition(kinematic.position);
        sprite.setRotation(sf::degrees(kinematic.orientation * 180.f / PI));
    }

    void updateWithBoundaryHandling(float deltaTime, const Kinematic &target)
    {
        if (!currentBehavior) return;

        SteeringOutput steering = currentBehavior->calculateSteering(kinematic, target);

        kinematic.position += kinematic.velocity * deltaTime;
        kinematic.orientation += kinematic.rotation * deltaTime;

        kinematic.velocity += steering.linear * deltaTime;
        kinematic.rotation += steering.angular * deltaTime;

        float speed = std::sqrt(kinematic.velocity.x*kinematic.velocity.x + kinematic.velocity.y*kinematic.velocity.y);
        if (speed > maxSpeed) kinematic.velocity = (kinematic.velocity / speed) * maxSpeed;

        if (std::abs(kinematic.rotation) > maxRotation)
            kinematic.rotation = (kinematic.rotation / std::abs(kinematic.rotation)) * maxRotation;

        const float margin = 4.0f;
        if (kinematic.position.x < margin) { kinematic.position.x = margin; kinematic.velocity.x = std::abs(kinematic.velocity.x) * 0.8f; }
        if (kinematic.position.x > WINDOW_WIDTH - margin) { kinematic.position.x = WINDOW_WIDTH - margin; kinematic.velocity.x = -std::abs(kinematic.velocity.x) * 0.8f; }
        if (kinematic.position.y < margin) { kinematic.position.y = margin; kinematic.velocity.y = std::abs(kinematic.velocity.y) * 0.8f; }
        if (kinematic.position.y > WINDOW_HEIGHT - margin) { kinematic.position.y = WINDOW_HEIGHT - margin; kinematic.velocity.y = -std::abs(kinematic.velocity.y) * 0.8f; }

        kinematic.orientation = mapToRange(kinematic.orientation);

        breadcrumbs.update(kinematic.position);

        sprite.setPosition(kinematic.position);
        sprite.setRotation(sf::degrees(kinematic.orientation * 180.f / PI));
    }

    void draw(sf::RenderWindow &window)
    {
        breadcrumbs.draw(window);
        window.draw(sprite);
    }

    Kinematic &getKinematic() { return kinematic; }
    void clearBreadcrumbs() { breadcrumbs.clear(); }
    void setMaxSpeed(float speed) { maxSpeed = speed; }
    void setPosition(sf::Vector2f pos) { kinematic.position = pos; sprite.setPosition(pos); }
};

// Boid
class Boid
{
public:
    Kinematic kinematic;
    sf::Sprite sprite;
    sf::Texture boidSmallTexture;
    Breadcrumb breadcrumbs;
    BlendedSteering *flockingBehavior;
    float maxSpeed;

    Boid(sf::Vector2f startPos, sf::Color color = sf::Color::Blue)
        : breadcrumbs(20, 10, color), flockingBehavior(nullptr), maxSpeed(200.0f)
    {
        kinematic.position = startPos;
        kinematic.velocity = sf::Vector2f(randomFloat(-50.f, 50.f), randomFloat(-50.f, 50.f));
        kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);

        if (!boidSmallTexture.loadFromFile("boid-sm.png"))
        {
            sf::Image img({16u, 16u}, sf::Color::Transparent);
            for (unsigned y = 0; y < 16; ++y)
                for (unsigned x = 0; x < 16; ++x)
                    img.setPixel({x, y}, sf::Color(180, 180, 180));
            boidSmallTexture.loadFromImage(img);
        }

        sprite = sf::Sprite(boidSmallTexture);
        sprite.setOrigin({static_cast<float>(boidSmallTexture.getSize().x) / 2.f, static_cast<float>(boidSmallTexture.getSize().y) / 2.f});
        sprite.setScale({1.5f, 1.5f});
        sprite.setColor(color);
    }

    void update(float deltaTime)
    {
        if (!flockingBehavior) return;
        SteeringOutput steering = flockingBehavior->calculateSteering(kinematic, kinematic);

        kinematic.position += kinematic.velocity * deltaTime;
        kinematic.velocity += steering.linear * deltaTime;

        float speed = std::sqrt(kinematic.velocity.x*kinematic.velocity.x + kinematic.velocity.y*kinematic.velocity.y);
        if (speed > maxSpeed) kinematic.velocity = (kinematic.velocity / speed) * maxSpeed;

        if (speed > 0.01f) kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);

        // wrap
        if (kinematic.position.x < 0.f) kinematic.position.x = WINDOW_WIDTH;
        if (kinematic.position.x > WINDOW_WIDTH) kinematic.position.x = 0.f;
        if (kinematic.position.y < 0.f) kinematic.position.y = WINDOW_HEIGHT;
        if (kinematic.position.y > WINDOW_HEIGHT) kinematic.position.y = 0.f;

        breadcrumbs.update(kinematic.position);
        sprite.setPosition(kinematic.position);
        sprite.setRotation(sf::degrees(kinematic.orientation * 180.f / PI));
    }

    void draw(sf::RenderWindow &window)
    {
        breadcrumbs.draw(window);
        window.draw(sprite);
    }
};

// Main
int main()
{
    // Create window with SFML 3 API (VideoMode + title)
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Steering Behaviors Demo (SFML 3)");
    window.setFramerateLimit(60);

    sf::Clock clock;
    int currentMode = 1;

    // Behaviors
    FastVelocityMatching fastVelMatch;

    ArriveAndAlign AAA1(
        300.0f, 120.0f, 5.0f, 100.0f, 0.2f,
        3.0f, 1.0f, 0.01f, 0.6f, 0.3f
    );

    ArriveAndAlign AAA2(
        100.0f, 120.0f, 5.0f, 200.0f, 0.2f,
        2.0f, 2.0f, 0.01f, 0.6f, 0.15f
    );

    Wander wanderSmooth(60.0f, 40.0f, 0.6f, 60.0f);
    WanderKinematic wanderKinematic1(60.0f, 40.0f, 0.6f, 60.0f, 2.5f);
    WanderKinematic wanderKinematic2(80.0f, 60.0f, 1.2f, 80.0f, 3.0f);
    WallAvoidance wallAvoid;

    BlendedSteering wanderWithWalls1(200.0f, 5.0f);
    wanderWithWalls1.addBehavior(&wanderSmooth, 1.0f);
    wanderWithWalls1.addBehavior(&wallAvoid, 2.0f);

    BlendedSteering wanderWithWalls2(200.0f, 5.0f);
    wanderWithWalls2.addBehavior(&wanderKinematic1, 1.0f);
    wanderWithWalls2.addBehavior(&wallAvoid, 2.0f);

    BlendedSteering wanderWithWalls3(200.0f, 5.0f);
    wanderWithWalls3.addBehavior(&wanderKinematic2, 1.0f);
    wanderWithWalls3.addBehavior(&wallAvoid, 2.0f);

    // Characters
    Character velMatchChar({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}, sf::Color(128,0,0,255));
    Character cyanAlignChar({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}, sf::Color(0,0,128,255));
    Character yellowArriveChar({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}, sf::Color(128,128,0,255));

    // Wander groups
    std::vector<std::unique_ptr<Character>> wanderSet1, wanderSet2, wanderSet3;
    auto initWander = [&]()
    {
        wanderSet1.clear(); wanderSet2.clear(); wanderSet3.clear();
        float centerX = WINDOW_WIDTH/2.f, centerY = WINDOW_HEIGHT/2.f;
        for (int i=0;i<3;++i)
        {
            auto c1 = std::make_unique<Character>(sf::Vector2f(centerX + randomFloat(-100.f,100.f), centerY + randomFloat(-100.f,100.f)), sf::Color::Blue);
            auto c2 = std::make_unique<Character>(sf::Vector2f(centerX + randomFloat(-100.f,100.f), centerY + randomFloat(-100.f,100.f)), sf::Color::Magenta);
            auto c3 = std::make_unique<Character>(sf::Vector2f(centerX + randomFloat(-100.f,100.f), centerY + randomFloat(-100.f,100.f)), sf::Color::Green);
            c1->getKinematic().orientation = randomFloat(-PI, PI);
            c2->getKinematic().orientation = randomFloat(-PI, PI);
            c3->getKinematic().orientation = randomFloat(-PI, PI);
            c1->setMaxSpeed(80.f); c2->setMaxSpeed(80.f); c3->setMaxSpeed(80.f);
            wanderSet1.push_back(std::move(c1));
            // wanderSet2.push_back(std::move(c2));
            wanderSet3.push_back(std::move(c3));
        }
    };

    // Flocking
    std::vector<std::unique_ptr<Boid>> flock;
    std::vector<Kinematic *> flockKinematics;
    auto initFlocking = [&]()
    {
        flock.clear(); flockKinematics.clear();
        int numBoids = 13;
        for (int i=0;i<numBoids;++i)
        {
            sf::Vector2f pos(randomFloat(100, WINDOW_WIDTH-100), randomFloat(100, WINDOW_HEIGHT-100));
            sf::Color color = (i%3==0) ? sf::Color::Cyan : (i%3==1) ? sf::Color::Magenta : sf::Color::Yellow;
            flock.push_back(std::make_unique<Boid>(pos, color));
            flockKinematics.push_back(&flock.back()->kinematic);
        }
        for (auto &boid : flock)
        {
            auto sep = new Separation(&flockKinematics, 40.0f, 5000.0f, 250.0f);
            auto coh = new Cohesion(&flockKinematics, 100.0f, 80.0f);
            auto ali = new Alignment(&flockKinematics, 80.0f, 100.0f);
            boid->flockingBehavior = new BlendedSteering();
            boid->flockingBehavior->addBehavior(sep, 5.0f);
            boid->flockingBehavior->addBehavior(coh, 0.7f);
            boid->flockingBehavior->addBehavior(ali, 0.7f);
        }
    };

    initWander();
    initFlocking();

    // Mouse
    Kinematic mouseTarget;
    sf::Vector2i mpos = sf::Mouse::getPosition(window);
    sf::Vector2f lastMousePos(static_cast<float>(mpos.x), static_cast<float>(mpos.y));
    sf::Clock mouseClock;
    Breadcrumb mouseBreadcrumbs(40, 3, sf::Color::White);

    // Font / text
    sf::Font font;
    if (!font.openFromFile("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"))
    {
        std::cerr << "Failed to load font; text might be empty.\n";
    }
    sf::Text modeText(font, "Case 1: Velocity Match Mouse", 14);
    modeText.setPosition({10.f, 10.f});

    // Reset helper
    auto resetCase = [&](int mode)
    {
        currentMode = mode;
        if (mode == 1)
        {
            velMatchChar.setPosition({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f});
            velMatchChar.clearBreadcrumbs();
        }
        else if (mode == 2)
        {
            cyanAlignChar.setPosition({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f});
            yellowArriveChar.setPosition({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f});
            cyanAlignChar.clearBreadcrumbs();
            yellowArriveChar.clearBreadcrumbs();
        }
        else if (mode == 3) initWander();
        else if (mode == 4) initFlocking();
    };

    // Main loop
    while (window.isOpen())
    {
        float dt = clock.restart().asSeconds();

        // Poll events (SFML 3 returns optional<Event>)
        while (true)
        {
            std::optional<sf::Event> opt = window.pollEvent();
            if (!opt) break;
            const sf::Event &event = *opt;

            if (event.is<sf::Event::Closed>()) window.close();

            if (event.is<sf::Event::KeyPressed>())
            {
                const auto &kp = event.get<sf::Event::KeyPressed>();
                // scancode enum
                if (kp.scancode == sf::Keyboard::Scancode::Escape) window.close();
                if (kp.scancode == sf::Keyboard::Scancode::Num1) resetCase(1);
                if (kp.scancode == sf::Keyboard::Scancode::Num2) resetCase(2);
                if (kp.scancode == sf::Keyboard::Scancode::Num3) resetCase(3);
                if (kp.scancode == sf::Keyboard::Scancode::Num4) resetCase(4);
            }

            if (event.is<sf::Event::MouseButtonPressed>() && currentMode == 2)
            {
                const auto &mb = event.get<sf::Event::MouseButtonPressed>();
                if (mb.button == sf::Mouse::Button::Left)
                {
                    mouseTarget.position = {static_cast<float>(mb.x), static_cast<float>(mb.y)};
                    mouseTarget.orientation = randomFloat(-PI, PI);
                }
            }
        }

        window.clear(sf::Color(30, 30, 40));

        switch (currentMode)
        {
            // Case 1: Velocity match mouse
            case 1:
            {
                modeText.setString("Case 1: Velocity Matching (Mouse)");
                sf::Vector2i mpInt = sf::Mouse::getPosition(window);
                sf::Vector2f mousePos(static_cast<float>(mpInt.x), static_cast<float>(mpInt.y));
                float elapsed = mouseClock.restart().asSeconds();
                if (elapsed > 0.f) mouseTarget.velocity = (mousePos - lastMousePos) / elapsed;
                mouseTarget.position = mousePos;
                lastMousePos = mousePos;

                mouseBreadcrumbs.update(mousePos);

                velMatchChar.setBehavior(&fastVelMatch);
                velMatchChar.setMaxSpeed(250.0f);
                velMatchChar.update(dt, mouseTarget);
                velMatchChar.draw(window);

                mouseBreadcrumbs.draw(window);

                sf::CircleShape targetShape(5.f);
                targetShape.setFillColor(sf::Color::White);
                targetShape.setOrigin({5.f, 5.f});
                targetShape.setPosition(mousePos);
                window.draw(targetShape);
                break;
            }

            // Case 2: Arrive + Align
            case 2:
            {
                modeText.setString("Case 2: Arrive + Align (Direction of Motion)");

                // allocate arrivers (as before - freed after)
                Arrive* quickArrive = new Arrive(500.0f, 120.0f, 5.0f, 150.0f, 0.12f);
                Arrive* slowArrive = new Arrive(200.0f, 100.0f, 7.0f, 150.0f, 0.08f);
                cyanAlignChar.setBehavior(quickArrive);
                yellowArriveChar.setBehavior(slowArrive);

                cyanAlignChar.update(dt, mouseTarget);
                yellowArriveChar.update(dt, mouseTarget);

                LookWhereYoureGoing lookWhere;
                SteeringOutput cyanOrient = lookWhere.calculateSteering(cyanAlignChar.getKinematic(), mouseTarget);
                SteeringOutput yellowOrient = lookWhere.calculateSteering(yellowArriveChar.getKinematic(), mouseTarget);

                cyanAlignChar.getKinematic().rotation += cyanOrient.angular * dt;
                cyanAlignChar.getKinematic().orientation += cyanAlignChar.getKinematic().rotation * dt;

                yellowArriveChar.getKinematic().rotation += yellowOrient.angular * dt;
                yellowArriveChar.getKinematic().orientation += yellowArriveChar.getKinematic().rotation * dt;

                cyanAlignChar.draw(window);
                yellowArriveChar.draw(window);

                delete quickArrive;
                delete slowArrive;
                break;
            }

            // Case 3: Wander
            case 3:
            {
                modeText.setString("Case 3: Wander (Blue: Circle+LookWhereYoureGoing, Magenta/Green: Direct Kinematic Rotation)");
                for (auto &c : wanderSet1)
                {
                    c->setBehavior(&wanderWithWalls1);
                    c->updateWithBoundaryHandling(dt, c->getKinematic());
                    c->draw(window);
                }
                for (auto &c : wanderSet2)
                {
                    c->setBehavior(&wanderWithWalls2);
                    c->updateWithBoundaryHandling(dt, c->getKinematic());
                    c->draw(window);
                }
                for (auto &c : wanderSet3)
                {
                    c->setBehavior(&wanderWithWalls3);
                    c->updateWithBoundaryHandling(dt, c->getKinematic());
                    c->draw(window);
                }
                break;
            }

            // Case 4: Boids
            case 4:
            {
                modeText.setString("Case 4: Reynolds Boids");
                for (auto &b : flock)
                {
                    b->update(dt);
                    b->draw(window);
                }
                break;
            }
        }

        window.draw(modeText);
        window.display();
    }

    return 0;
}
