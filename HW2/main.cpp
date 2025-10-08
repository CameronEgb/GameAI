// main.cpp - SFML 3.0.0 port of user's SFML 2.5.x main1.cpp
// All SFML 3.0 API changes applied.

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

// Forward
struct Kinematic;
struct SteeringOutput;

// Utilities
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

// Data structures
struct Kinematic
{
    sf::Vector2f position;
    float orientation;
    sf::Vector2f velocity;
    float rotation;
    Kinematic() : position(0.f,0.f), orientation(0.f), velocity(0.f,0.f), rotation(0.f) {}
};

struct SteeringOutput
{
    sf::Vector2f linear;
    float angular;
    SteeringOutput() : linear(0.f,0.f), angular(0.f) {}
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

// SteeringBehavior interface
class SteeringBehavior
{
public:
    virtual ~SteeringBehavior() {}
    virtual SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) = 0;
};

// --- Basic steering implementations (port from your SFML2 code) ---

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

class VelocityMatching : public SteeringBehavior
{
private:
    float maxAcceleration;
    float timeToTarget;
public:
    VelocityMatching(float maxAccel = 100.0f, float time = 0.1f) : maxAcceleration(maxAccel), timeToTarget(time) {}
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

class FastVelocityMatching : public VelocityMatching { public: FastVelocityMatching() : VelocityMatching(400.f, 0.05f) {} };

// Orientation / Rotation matching
class OrientationMatching : public SteeringBehavior
{
private:
    float maxAngularAcceleration;
    float maxRotation;
    float targetRadius;
    float slowRadius;
    float timeToTarget;
public:
    OrientationMatching(float maxAngAccel = 5.f, float maxRot = 2.f)
        : maxAngularAcceleration(maxAngAccel), maxRotation(maxRot),
          targetRadius(0.01f), slowRadius(0.5f), timeToTarget(0.1f) {}
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
            result.angular = (result.angular / angularAcceleration) * maxAngularAcceleration;
        }
        return result;
    }
};

class RotationMatching : public SteeringBehavior
{
private:
    float maxAngularAcceleration;
    float timeToTarget;
public:
    RotationMatching(float maxAngAccel = 5.f, float time = 0.1f) : maxAngularAcceleration(maxAngAccel), timeToTarget(time) {}
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
    float maxAcceleration, maxSpeed, targetRadius, slowRadius, timeToTarget;
public:
    Arrive(float maxAccel = 200.f, float maxSpd = 100.f, float targetRad = 5.f, float slowRad = 100.f, float time = 0.1f)
        : maxAcceleration(maxAccel), maxSpeed(maxSpd), targetRadius(targetRad), slowRadius(slowRad), timeToTarget(time) {}
    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) override
    {
        SteeringOutput result;
        sf::Vector2f direction = target.position - character.position;
        float distance = std::sqrt(direction.x*direction.x + direction.y*direction.y);
        if (distance < targetRadius) return result;
        float targetSpeed = (distance > slowRadius) ? maxSpeed : maxSpeed * distance / slowRadius;
        sf::Vector2f targetVelocity = (distance > 0.0001f) ? (direction / distance) * targetSpeed : sf::Vector2f(0.f,0.f);
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
    float maxAngularAcceleration, maxRotation, targetRadius, slowRadius, timeToTarget;
public:
    Align(float maxAngAccel = 5.f, float maxRot = 2.f, float targetRad = 0.01f, float slowRad = 0.5f, float time = 0.1f)
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
        if (angularAcceleration > maxAngularAcceleration) result.angular = (result.angular / angularAcceleration) * maxAngularAcceleration;
        return result;
    }
};

class SmoothAlign : public Align { public: SmoothAlign() : Align(2.5f,1.5f,0.01f,0.6f,0.1f) {} };

// ArriveAndAlign (composed)
class ArriveAndAlign : public SteeringBehavior
{
private:
    Arrive arrive;
    Align align;
public:
    ArriveAndAlign(float arrive_maxSpeed, float arrive_radius, float arrive_timeToTarget, float arrive_slowRadius, float arrive_maxAcceleration,
                   float align_maxRotation, float align_radius, float align_timeToTarget, float align_slowRadius, float align_maxAngularAcceleration)
        : arrive(arrive_maxAcceleration, arrive_maxSpeed, arrive_radius, arrive_slowRadius, arrive_timeToTarget),
          align(align_maxAngularAcceleration, align_maxRotation, align_radius, align_slowRadius, align_timeToTarget)
    {}
    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) override
    {
        SteeringOutput a = arrive.calculateSteering(character, target);
        SteeringOutput b = align.calculateSteering(character, target);
        SteeringOutput r;
        r.linear = a.linear;
        r.angular = b.angular;
        return r;
    }
};

// Face & LookWhereYoureGoing
class Face : public SteeringBehavior { Align align; public: Face():align(){} SteeringOutput calculateSteering(const Kinematic &c,const Kinematic &t) override { sf::Vector2f dir = t.position - c.position; if (dir.x==0 && dir.y==0) return SteeringOutput(); Kinematic targ; targ.orientation = std::atan2(dir.y, dir.x); return align.calculateSteering(c,targ); } };
class LookWhereYoureGoing : public SteeringBehavior { Align align; public: LookWhereYoureGoing():align(){} SteeringOutput calculateSteering(const Kinematic &c,const Kinematic &t) override { if (c.velocity.x==0 && c.velocity.y==0) return SteeringOutput(); Kinematic targ; targ.orientation = std::atan2(c.velocity.y, c.velocity.x); return align.calculateSteering(c,targ); } };

// WallAvoidance
class WallAvoidance : public SteeringBehavior
{
private:
    float wallMargin;
    float maxAcceleration;
    float detectionDistance;
public:
    WallAvoidance(float margin = 20.f, float maxAccel = 300.f, float detectDist = 120.f)
        : wallMargin(margin), maxAcceleration(maxAccel), detectionDistance(detectDist) {}
    SteeringOutput calculateSteering(const Kinematic &character, const Kinematic & /*target*/) override
    {
        SteeringOutput r;
        sf::Vector2f avoidance(0.f,0.f);
        float dl = character.position.x;
        float dr = WINDOW_WIDTH - character.position.x;
        float dt = character.position.y;
        float db = WINDOW_HEIGHT - character.position.y;
        if (dl < detectionDistance) avoidance.x += (detectionDistance - dl)/detectionDistance * maxAcceleration;
        if (dr < detectionDistance) avoidance.x -= (detectionDistance - dr)/detectionDistance * maxAcceleration;
        if (dt < detectionDistance) avoidance.y += (detectionDistance - dt)/detectionDistance * maxAcceleration;
        if (db < detectionDistance) avoidance.y -= (detectionDistance - db)/detectionDistance * maxAcceleration;
        r.linear = avoidance;
        return r;
    }
};

// Wander & WanderKinematic
class Wander : public SteeringBehavior
{
private:
    float wanderOffset, wanderRadius, wanderRate, wanderOrientation, maxAcceleration;
    LookWhereYoureGoing lookWhere;
public:
    Wander(float offset=60.f,float radius=40.f,float rate=0.5f,float maxAccel=80.f)
        : wanderOffset(offset),wanderRadius(radius),wanderRate(rate),wanderOrientation(0.f),maxAcceleration(maxAccel) {}
    SteeringOutput calculateSteering(const Kinematic &c,const Kinematic &t) override
    {
        wanderOrientation += randomBinomial()*wanderRate;
        float targetOrientation = wanderOrientation + c.orientation;
        sf::Vector2f orientVec(std::cos(c.orientation), std::sin(c.orientation));
        sf::Vector2f center = c.position + orientVec * wanderOffset;
        sf::Vector2f wanderTarget(center.x + wanderRadius*std::cos(targetOrientation), center.y + wanderRadius*std::sin(targetOrientation));
        SteeringOutput r;
        r.linear = wanderTarget - c.position;
        float len = std::sqrt(r.linear.x*r.linear.x + r.linear.y*r.linear.y);
        if (len>0) r.linear = (r.linear / len) * maxAcceleration;
        SteeringOutput look = lookWhere.calculateSteering(c,t);
        r.angular = look.angular;
        return r;
    }
};

class WanderKinematic : public SteeringBehavior
{
private:
    float wanderOffset, wanderRadius, wanderRate, wanderOrientation, maxAcceleration, maxRotation;
public:
    WanderKinematic(float offset=60.f,float radius=40.f,float rate=0.5f,float maxAccel=80.f,float maxRot=2.f)
        : wanderOffset(offset),wanderRadius(radius),wanderRate(rate),wanderOrientation(0.f),maxAcceleration(maxAccel),maxRotation(maxRot) {}
    SteeringOutput calculateSteering(const Kinematic &c,const Kinematic & /*t*/) override
    {
        wanderOrientation += randomBinomial()*wanderRate;
        float targetOrientation = wanderOrientation + c.orientation;
        sf::Vector2f orientVec(std::cos(c.orientation), std::sin(c.orientation));
        sf::Vector2f center = c.position + orientVec * wanderOffset;
        sf::Vector2f wanderTarget(center.x + wanderRadius*std::cos(targetOrientation), center.y + wanderRadius*std::sin(targetOrientation));
        SteeringOutput r;
        r.linear = wanderTarget - c.position;
        float len = std::sqrt(r.linear.x*r.linear.x + r.linear.y*r.linear.y);
        if (len>0) r.linear = (r.linear / len) * maxAcceleration;
        sf::Vector2f toTarget = wanderTarget - c.position;
        float desiredOrientation = std::atan2(toTarget.y,toTarget.x);
        float diff = mapToRange(desiredOrientation - c.orientation);
        r.angular = diff * 3.0f;
        if (std::abs(r.angular) > maxRotation) r.angular = (r.angular / std::abs(r.angular)) * maxRotation;
        return r;
    }
};

// Flocking: Separation, Cohesion, Alignment, BlendedSteering
class Separation : public SteeringBehavior
{
private:
    float threshold, decayCoefficient, maxAcceleration;
    std::vector<Kinematic*> *boids;
public:
    Separation(std::vector<Kinematic*> *b, float thresh=100.f, float decay=5000.f, float maxAccel=100.f)
        : threshold(thresh), decayCoefficient(decay), maxAcceleration(maxAccel), boids(b) {}
    SteeringOutput calculateSteering(const Kinematic &c,const Kinematic & /*t*/) override
    {
        SteeringOutput r;
        for (auto &b : *boids)
        {
            if (b->position == c.position) continue;
            sf::Vector2f dir = c.position - b->position;
            float dist = std::sqrt(dir.x*dir.x + dir.y*dir.y);
            if (dist < threshold && dist > 0.f)
            {
                float strength = std::min(decayCoefficient / (dist*dist), maxAcceleration);
                dir = (dir / dist) * strength;
                r.linear += dir;
            }
        }
        float len = std::sqrt(r.linear.x*r.linear.x + r.linear.y*r.linear.y);
        if (len > maxAcceleration) r.linear = (r.linear / len) * maxAcceleration;
        return r;
    }
};

class Cohesion : public SteeringBehavior
{
private:
    float neighborhoodRadius, maxAcceleration;
    std::vector<Kinematic*> *boids;
    Arrive arrive;
public:
    Cohesion(std::vector<Kinematic*> *b, float radius=150.f, float maxAccel=10.f)
        : neighborhoodRadius(radius), maxAcceleration(maxAccel), boids(b), arrive(maxAccel,100.f,10.f,50.f) {}
    SteeringOutput calculateSteering(const Kinematic &c,const Kinematic & /*t*/) override
    {
        sf::Vector2f com(0.f,0.f); int count=0;
        for (auto &b : *boids)
        {
            if (b->position == c.position) continue;
            sf::Vector2f dir = b->position - c.position;
            float dist = std::sqrt(dir.x*dir.x + dir.y*dir.y);
            if (dist < neighborhoodRadius) { com += b->position; ++count; }
        }
        if (count==0) return SteeringOutput();
        com /= static_cast<float>(count);
        Kinematic target; target.position = com;
        return arrive.calculateSteering(c,target);
    }
};

class Alignment : public SteeringBehavior
{
private:
    float neighborhoodRadius, maxAcceleration;
    std::vector<Kinematic*> *boids;
    VelocityMatching velocityMatch;
public:
    Alignment(std::vector<Kinematic*> *b, float radius=100.f, float maxAccel=50.f)
        : neighborhoodRadius(radius), maxAcceleration(maxAccel), boids(b), velocityMatch(maxAccel) {}
    SteeringOutput calculateSteering(const Kinematic &c,const Kinematic & /*t*/) override
    {
        sf::Vector2f avg(0.f,0.f); int count=0;
        for (auto &b : *boids)
        {
            if (b->position == c.position) continue;
            sf::Vector2f dir = b->position - c.position;
            float dist = std::sqrt(dir.x*dir.x + dir.y*dir.y);
            if (dist < neighborhoodRadius) { avg += b->velocity; ++count; }
        }
        if (count==0) return SteeringOutput();
        avg /= static_cast<float>(count);
        Kinematic target; target.velocity = avg;
        return velocityMatch.calculateSteering(c,target);
    }
};

class BlendedSteering : public SteeringBehavior
{
private:
    struct BW { SteeringBehavior *behavior; float weight; };
    std::vector<BW> behaviors;
    float maxAcceleration, maxAngular;
public:
    BlendedSteering(float maxAccel=200.f, float maxAng=5.f) : maxAcceleration(maxAccel), maxAngular(maxAng) {}
    void addBehavior(SteeringBehavior *b, float w) { behaviors.push_back({b,w}); }
    SteeringOutput calculateSteering(const Kinematic &c,const Kinematic &t) override
    {
        SteeringOutput r;
        for (auto &bw : behaviors)
        {
            SteeringOutput s = bw.behavior->calculateSteering(c,t);
            r.linear += s.linear * bw.weight;
            r.angular += s.angular * bw.weight;
        }
        float len = std::sqrt(r.linear.x*r.linear.x + r.linear.y*r.linear.y);
        if (len > maxAcceleration) r.linear = (r.linear / len) * maxAcceleration;
        if (std::abs(r.angular) > maxAngular) r.angular = (r.angular / std::abs(r.angular)) * maxAngular;
        return r;
    }
};

// Character (full) - adapted for SFML 3
class CharacterFull
{
private:
    Kinematic kinematic;
    sf::Sprite sprite;          // will be assigned after texture load
    sf::Texture boidTexture;
    Breadcrumb breadcrumbs;
    SteeringBehavior *currentBehavior;
    float maxSpeed;
    float maxRotation;

public:
    CharacterFull(sf::Vector2f startPos = sf::Vector2f(400.f,300.f), sf::Color color = sf::Color::Red,
                  int breadcrumbMax = 30, int breadcrumbInterval = 5)
        : breadcrumbs(breadcrumbMax, breadcrumbInterval, color),
          currentBehavior(nullptr), maxSpeed(150.f), maxRotation(3.f)
    {
        kinematic.position = startPos;
        kinematic.orientation = 0.f;

        // Attempt to load texture; if missing, create fallback image
        if (!boidTexture.loadFromFile("boid.png"))
        {
            sf::Image img({32u,32u}, sf::Color::Transparent);
            for (unsigned y=0;y<32;++y)
                for (unsigned x=0;x<32;++x)
                    img.setPixel({x,y}, sf::Color(200,200,200));
            (void)boidTexture.loadFromImage(img);
        }

        // Construct sprite with texture (SFML 3 requires texture at construction)
        sprite = sf::Sprite(boidTexture);
        sprite.setOrigin({ static_cast<float>(boidTexture.getSize().x)/2.f, static_cast<float>(boidTexture.getSize().y)/2.f });
        sprite.setScale({0.05f, 0.05f});
        sprite.setColor(color);
    }

    void setBehavior(SteeringBehavior *b) { currentBehavior = b; }
    Kinematic &getKinematic() { return kinematic; }
    void clearBreadcrumbs() { breadcrumbs.clear(); }
    void setMaxSpeed(float s) { maxSpeed = s; }
    void setPosition(sf::Vector2f p) { kinematic.position = p; sprite.setPosition(p); }

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

        const float margin = 4.f;
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
};

// Boid (flocking)
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
        : breadcrumbs(20,10,color), flockingBehavior(nullptr), maxSpeed(200.f)
    {
        kinematic.position = startPos;
        kinematic.velocity = sf::Vector2f(randomFloat(-50.f,50.f), randomFloat(-50.f,50.f));
        kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);

        if (!boidSmallTexture.loadFromFile("boid-sm.png"))
        {
            sf::Image img({16u,16u}, sf::Color::Transparent);
            for (unsigned y=0;y<16;++y) for (unsigned x=0;x<16;++x) img.setPixel({x,y}, sf::Color(180,180,180));
            (void)boidSmallTexture.loadFromImage(img);
        }

        sprite = sf::Sprite(boidSmallTexture);
        sprite.setOrigin({static_cast<float>(boidSmallTexture.getSize().x)/2.f, static_cast<float>(boidSmallTexture.getSize().y)/2.f});
        sprite.setScale({1.5f,1.5f});
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

// ----------------- main -----------------

int main()
{
    // VideoMode needs Vector2u in SFML 3
    sf::RenderWindow window(sf::VideoMode(sf::Vector2u(WINDOW_WIDTH, WINDOW_HEIGHT)),
                            "Steering Behaviors Demo (1: VelMatchMouse, 2: Align+Arrive, 3: Wander, 4: Boids)");
    window.setFramerateLimit(60);

    sf::Clock clock;
    int currentMode = 1;

    // Behaviors
    FastVelocityMatching fastVelMatch;
    ArriveAndAlign AAA1(300.f,120.f,5.f,100.f,0.2f, 3.f,1.f,0.01f,0.6f,0.3f);
    ArriveAndAlign AAA2(100.f,120.f,5.f,200.f,0.2f, 2.f,2.f,0.01f,0.6f,0.15f);
    Wander wanderSmooth(60.f,40.f,0.6f,60.f);
    WanderKinematic wanderKinematic1(60.f,40.f,0.6f,60.f,2.5f);
    WanderKinematic wanderKinematic2(80.f,60.f,1.2f,80.f,3.0f);
    WallAvoidance wallAvoid;

    BlendedSteering wanderWithWalls1(200.f,5.f);
    wanderWithWalls1.addBehavior(&wanderSmooth, 1.f); wanderWithWalls1.addBehavior(&wallAvoid, 2.f);
    BlendedSteering wanderWithWalls2(200.f,5.f);
    wanderWithWalls2.addBehavior(&wanderKinematic1, 1.f); wanderWithWalls2.addBehavior(&wallAvoid, 2.f);
    BlendedSteering wanderWithWalls3(200.f,5.f);
    wanderWithWalls3.addBehavior(&wanderKinematic2, 1.f); wanderWithWalls3.addBehavior(&wallAvoid, 2.f);

    // Characters
    CharacterFull velMatchChar({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}, sf::Color(128,0,0,255));
    CharacterFull cyanAlignChar({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}, sf::Color(0,0,128,255));
    CharacterFull yellowArriveChar({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}, sf::Color(128,128,0,255));

    // Wander groups init
    std::vector<std::unique_ptr<CharacterFull>> wanderSet1, wanderSet2, wanderSet3;
    auto initWander = [&]()
    {
        wanderSet1.clear(); wanderSet2.clear(); wanderSet3.clear();
        float cx = WINDOW_WIDTH/2.f, cy = WINDOW_HEIGHT/2.f;
        for (int i=0;i<3;++i)
        {
            auto c1 = std::make_unique<CharacterFull>(sf::Vector2f(cx + randomFloat(-100.f,100.f), cy + randomFloat(-100.f,100.f)), sf::Color::Blue);
            auto c2 = std::make_unique<CharacterFull>(sf::Vector2f(cx + randomFloat(-100.f,100.f), cy + randomFloat(-100.f,100.f)), sf::Color::Magenta);
            auto c3 = std::make_unique<CharacterFull>(sf::Vector2f(cx + randomFloat(-100.f,100.f), cy + randomFloat(-100.f,100.f)), sf::Color::Green);
            c1->getKinematic().orientation = randomFloat(-PI,PI);
            c2->getKinematic().orientation = randomFloat(-PI,PI);
            c3->getKinematic().orientation = randomFloat(-PI,PI);
            c1->setMaxSpeed(80.f); c2->setMaxSpeed(80.f); c3->setMaxSpeed(80.f);
            wanderSet1.push_back(std::move(c1)); //c2 unused in original; kept for completeness
            //wanderSet2.push_back(std::move(c2));
            wanderSet3.push_back(std::move(c3));
        }
    };

    // Flocking init
    std::vector<std::unique_ptr<Boid>> flock;
    std::vector<Kinematic*> flockKinematics;
    auto initFlocking = [&]()
    {
        flock.clear(); flockKinematics.clear();
        int numBoids = 13;
        for (int i=0;i<numBoids;++i)
        {
            sf::Vector2f p(randomFloat(100.f, WINDOW_WIDTH-100.f), randomFloat(100.f, WINDOW_HEIGHT-100.f));
            sf::Color color = (i%3==0) ? sf::Color::Cyan : (i%3==1) ? sf::Color::Magenta : sf::Color::Yellow;
            flock.push_back(std::make_unique<Boid>(p,color));
            flockKinematics.push_back(&flock.back()->kinematic);
        }
        for (auto &b : flock)
        {
            auto sep = new Separation(&flockKinematics, 40.f, 5000.f, 250.f);
            auto coh = new Cohesion(&flockKinematics, 100.f, 80.f);
            auto ali = new Alignment(&flockKinematics, 80.f, 100.f);
            b->flockingBehavior = new BlendedSteering();
            b->flockingBehavior->addBehavior(sep, 5.f);
            b->flockingBehavior->addBehavior(coh, 0.7f);
            b->flockingBehavior->addBehavior(ali, 0.7f);
        }
    };

    initWander();
    initFlocking();

    // Mouse state
    Kinematic mouseTarget;
    sf::Vector2i mposI = sf::Mouse::getPosition(window);
    sf::Vector2f lastMousePos(static_cast<float>(mposI.x), static_cast<float>(mposI.y));
    sf::Clock mouseClock;
    Breadcrumb mouseBreadcrumbs(40,3,sf::Color::White);

    // Font / text
    sf::Font font;
    if (!font.openFromFile("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"))
        std::cerr << "Failed to load font; text may be empty\n";
    sf::Text modeText(font, "Case 1: Velocity Match Mouse", 20);
    modeText.setPosition({10.f,10.f});

    // Reset helper
    auto resetCase = [&](int mode)
    {
        currentMode = mode;
        if (mode==1) { velMatchChar.setPosition({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}); velMatchChar.clearBreadcrumbs(); }
        else if (mode==2) { cyanAlignChar.setPosition({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}); yellowArriveChar.setPosition({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}); cyanAlignChar.clearBreadcrumbs(); yellowArriveChar.clearBreadcrumbs(); }
        else if (mode==3) initWander();
        else if (mode==4) initFlocking();
    };

    // Main loop
    while (window.isOpen())
    {
        float dt = clock.restart().asSeconds();

        // SFML3: pollEvent returns optional<sf::Event>
        while (true)
        {
            std::optional<sf::Event> opt = window.pollEvent();
            if (!opt) break;
            const sf::Event &event = *opt;

            if (event.is<sf::Event::Closed>()) window.close();
            else if (event.is<sf::Event::KeyPressed>())
            {
                const auto *kp = event.getIf<sf::Event::KeyPressed>();
                if (kp)
                {
                    if (kp->scancode == sf::Keyboard::Scancode::Escape) window.close();
                    if (kp->scancode == sf::Keyboard::Scancode::Num1) resetCase(1);
                    if (kp->scancode == sf::Keyboard::Scancode::Num2) resetCase(2);
                    if (kp->scancode == sf::Keyboard::Scancode::Num3) resetCase(3);
                    if (kp->scancode == sf::Keyboard::Scancode::Num4) resetCase(4);
                }
            }
            else if (event.is<sf::Event::MouseButtonPressed>() && currentMode==2)
            {
                const auto *mb = event.getIf<sf::Event::MouseButtonPressed>();
                if (mb && mb->button == sf::Mouse::Button::Left)
                {
                    mouseTarget.position = sf::Vector2f(static_cast<float>(mb->position.x), static_cast<float>(mb->position.y));
                    mouseTarget.orientation = randomFloat(-PI, PI);
                }
            }
        } // end events

        window.clear(sf::Color(30,30,40));

        switch (currentMode)
        {
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
                velMatchChar.setMaxSpeed(250.f);
                velMatchChar.update(dt, mouseTarget);
                velMatchChar.draw(window);

                mouseBreadcrumbs.draw(window);

                sf::CircleShape targetShape(5.f);
                targetShape.setFillColor(sf::Color::White);
                targetShape.setOrigin({5.f,5.f});
                targetShape.setPosition(mousePos);
                window.draw(targetShape);
                break;
            }

            case 2:
            {
                modeText.setString("Case 2: Arrive + Align (Direction of Motion)");
                Arrive* quickArrive = new Arrive(500.f,120.f,5.f,150.f,0.12f);
                Arrive* slowArrive  = new Arrive(200.f,100.f,7.f,150.f,0.08f);
                cyanAlignChar.setBehavior(quickArrive);
                yellowArriveChar.setBehavior(slowArrive);

                cyanAlignChar.update(dt, mouseTarget);
                yellowArriveChar.update(dt, mouseTarget);

                LookWhereYoureGoing lookWhere;
                SteeringOutput co = lookWhere.calculateSteering(cyanAlignChar.getKinematic(), mouseTarget);
                SteeringOutput yo = lookWhere.calculateSteering(yellowArriveChar.getKinematic(), mouseTarget);

                cyanAlignChar.getKinematic().rotation += co.angular * dt;
                cyanAlignChar.getKinematic().orientation += cyanAlignChar.getKinematic().rotation * dt;

                yellowArriveChar.getKinematic().rotation += yo.angular * dt;
                yellowArriveChar.getKinematic().orientation += yellowArriveChar.getKinematic().rotation * dt;

                cyanAlignChar.draw(window);
                yellowArriveChar.draw(window);

                delete quickArrive;
                delete slowArrive;
                break;
            }

            case 3:
            {
                modeText.setString("Case 3: Wander (Blue: Circle+LookWhereYoureGoing, Magenta/Green: Direct Kinematic Rotation)");
                for (auto &c : wanderSet1) { c->setBehavior(&wanderWithWalls1); c->updateWithBoundaryHandling(dt, c->getKinematic()); c->draw(window); }
                for (auto &c : wanderSet2) { c->setBehavior(&wanderWithWalls2); c->updateWithBoundaryHandling(dt, c->getKinematic()); c->draw(window); }
                for (auto &c : wanderSet3) { c->setBehavior(&wanderWithWalls3); c->updateWithBoundaryHandling(dt, c->getKinematic()); c->draw(window); }
                break;
            }

            case 4:
            {
                modeText.setString("Case 4: Reynolds Boids");
                for (auto &b : flock) { b->update(dt); b->draw(window); }
                break;
            }
        }

        window.draw(modeText);
        window.display();
    }

    return 0;
}
