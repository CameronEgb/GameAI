// main.cpp - SFML 3.0.0 port (full steering behaviors + boids)
// Copy-paste this entire file and compile with SFML 3.0.0 and -std=c++17

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

constexpr float PI = 3.14159265f;
constexpr int WINDOW_WIDTH = 800;
constexpr int WINDOW_HEIGHT = 600;

// ---------------- utilities ----------------
float mapToRange(float rotation)
{
    while (rotation > PI) rotation -= 2.f * PI;
    while (rotation < -PI) rotation += 2.f * PI;
    return rotation;
}

float randomBinomial()
{
    static std::mt19937 gen(std::random_device{}());
    static std::uniform_real_distribution<float> dis(0.f, 1.f);
    return dis(gen) - dis(gen);
}

float randomFloat(float a, float b)
{
    static std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<float> dis(a, b);
    return dis(gen);
}

// ---------------- data ----------------
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

// ---------------- breadcrumbs ----------------
class Breadcrumb {
    std::queue<sf::Vector2f> q;
    int maxCrumbs;
    int dropInterval;
    int counter;
    sf::Color color;
public:
    Breadcrumb(int maxCrumbs_=30, int dropInterval_=5, sf::Color c=sf::Color::Blue)
        : maxCrumbs(maxCrumbs_), dropInterval(dropInterval_), counter(0), color(c) {}

    void update(const sf::Vector2f &pos) {
        if (++counter >= dropInterval) {
            counter = 0;
            q.push(pos);
            if ((int)q.size() > maxCrumbs) q.pop();
        }
    }

    void draw(sf::RenderWindow &win) {
        std::queue<sf::Vector2f> temp = q;
        float alpha = 50.f;
        float inc = 200.f / std::max(1, maxCrumbs);
        while (!temp.empty()) {
            sf::CircleShape c(3.f);
            c.setPosition(temp.front() - sf::Vector2f(3.f,3.f));
            sf::Color col = color;
            col.a = static_cast<std::uint8_t>(std::min(255.f, alpha));
            c.setFillColor(col);
            win.draw(c);
            temp.pop();
            alpha += inc;
        }
    }

    void clear() {
        while (!q.empty()) q.pop();
        counter = 0;
    }
};

// ---------------- steering interface ----------------
class SteeringBehavior {
public:
    virtual ~SteeringBehavior() = default;
    virtual SteeringOutput calculateSteering(const Kinematic &character, const Kinematic &target) = 0;
};

// ---------------- basic steering algorithms ----------------

class PositionMatching : public SteeringBehavior {
    float maxAccel;
public:
    explicit PositionMatching(float m = 100.f) : maxAccel(m) {}
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override {
        SteeringOutput out;
        sf::Vector2f dir = t.position - c.position;
        float d = std::sqrt(dir.x*dir.x + dir.y*dir.y);
        if (d > 0.001f) {
            dir /= d;
            out.linear = dir * maxAccel;
        }
        return out;
    }
};

class VelocityMatching : public SteeringBehavior {
protected:
    float maxAccel;
    float timeToTarget;
public:
    VelocityMatching(float maxA=100.f, float time=0.1f) : maxAccel(maxA), timeToTarget(time) {}
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override {
        SteeringOutput out;
        out.linear = (t.velocity - c.velocity) / timeToTarget;
        float mag = std::sqrt(out.linear.x*out.linear.x + out.linear.y*out.linear.y);
        if (mag > maxAccel) out.linear = (out.linear / mag) * maxAccel;
        return out;
    }
};

class FastVelocityMatching : public VelocityMatching {
public:
    FastVelocityMatching() : VelocityMatching(400.f, 0.05f) {}
};

class OrientationMatching : public SteeringBehavior {
    float maxAngAccel;
    float maxRotation;
    float targetRadius;
    float slowRadius;
    float timeToTarget;
public:
    OrientationMatching(float maxAA=5.f, float maxR=2.f)
        : maxAngAccel(maxAA), maxRotation(maxR), targetRadius(0.01f), slowRadius(0.5f), timeToTarget(0.1f) {}
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override {
        SteeringOutput out;
        float rotation = t.orientation - c.orientation;
        rotation = mapToRange(rotation);
        float rotSize = std::abs(rotation);
        if (rotSize < targetRadius) return out;
        float targetRot = (rotSize > slowRadius) ? maxRotation : maxRotation * rotSize / slowRadius;
        targetRot *= rotation / rotSize;
        out.angular = (targetRot - c.rotation) / timeToTarget;
        float angularAccel = std::abs(out.angular);
        if (angularAccel > maxAngAccel) {
            out.angular = (out.angular / angularAccel) * maxAngAccel;
        }
        return out;
    }
};

class RotationMatching : public SteeringBehavior {
    float maxAngAccel;
    float timeToTarget;
public:
    RotationMatching(float maxA=5.f, float time=0.1f) : maxAngAccel(maxA), timeToTarget(time) {}
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override {
        SteeringOutput out;
        out.angular = (t.rotation - c.rotation) / timeToTarget;
        if (std::abs(out.angular) > maxAngAccel)
            out.angular = (out.angular / std::abs(out.angular)) * maxAngAccel;
        return out;
    }
};

// Arrive
class Arrive : public SteeringBehavior {
    float maxAcceleration;
    float maxSpeed;
    float targetRadius;
    float slowRadius;
    float timeToTarget;
public:
    Arrive(float maxAccel=200.f,float maxSpd=100.f,float tRad=5.f,float sRad=100.f,float time=0.1f)
        : maxAcceleration(maxAccel), maxSpeed(maxSpd), targetRadius(tRad), slowRadius(sRad), timeToTarget(time) {}
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override {
        SteeringOutput out;
        sf::Vector2f direction = t.position - c.position;
        float dist = std::sqrt(direction.x*direction.x + direction.y*direction.y);
        if (dist < targetRadius) return out;
        float targetSpeed = (dist > slowRadius) ? maxSpeed : maxSpeed * dist / slowRadius;
        sf::Vector2f targetVel(0.f, 0.f);
        if (dist > 0.0001f) targetVel = (direction / dist) * targetSpeed;
        out.linear = (targetVel - c.velocity) / timeToTarget;
        float mag = std::sqrt(out.linear.x*out.linear.x + out.linear.y*out.linear.y);
        if (mag > maxAcceleration) out.linear = (out.linear / mag) * maxAcceleration;
        return out;
    }
};

// Align
class Align : public SteeringBehavior {
    float maxAngularAcceleration;
    float maxRotation;
    float targetRadius;
    float slowRadius;
    float timeToTarget;
public:
    Align(float maxAngAccel=5.f, float maxRot=2.f, float tRad=0.01f, float sRad=0.5f, float time=0.1f)
        : maxAngularAcceleration(maxAngAccel), maxRotation(maxRot), targetRadius(tRad), slowRadius(sRad), timeToTarget(time) {}
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override {
        SteeringOutput out;
        float rotation = t.orientation - c.orientation;
        rotation = mapToRange(rotation);
        float rotationSize = std::abs(rotation);
        if (rotationSize < targetRadius) return out;
        float targetRotation = (rotationSize > slowRadius) ? maxRotation : maxRotation * rotationSize / slowRadius;
        targetRotation *= rotation / rotationSize;
        out.angular = (targetRotation - c.rotation) / timeToTarget;
        float angularAccel = std::abs(out.angular);
        if (angularAccel > maxAngularAcceleration) {
            out.angular = (out.angular / angularAccel) * maxAngularAcceleration;
        }
        return out;
    }
};

class SmoothAlign : public Align {
public:
    SmoothAlign() : Align(2.5f, 1.5f, 0.01f, 0.6f, 0.1f) {}
};

// Face
class Face : public SteeringBehavior {
    Align align;
public:
    Face() : align() {}
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override {
        sf::Vector2f dir = t.position - c.position;
        if (dir.x == 0.f && dir.y == 0.f) return {};
        Kinematic target;
        target.orientation = std::atan2(dir.y, dir.x);
        return align.calculateSteering(c, target);
    }
};

// LookWhereYoureGoing
class LookWhereYoureGoing : public SteeringBehavior {
    Align align;
public:
    LookWhereYoureGoing() : align() {}
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic & /*t*/) override {
        if (c.velocity.x == 0.f && c.velocity.y == 0.f) return {};
        Kinematic target;
        target.orientation = std::atan2(c.velocity.y, c.velocity.x);
        return align.calculateSteering(c, target);
    }
};

// Wall Avoidance (simple)
class WallAvoidance : public SteeringBehavior {
    float wallMargin;
    float maxAcceleration;
    float detectionDistance;
public:
    WallAvoidance(float margin=20.f, float maxAcc=300.f, float detectDist=120.f)
        : wallMargin(margin), maxAcceleration(maxAcc), detectionDistance(detectDist) {}
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic & /*t*/) override {
        SteeringOutput out;
        sf::Vector2f avoid(0.f, 0.f);
        float dl = c.position.x;
        float dr = WINDOW_WIDTH - c.position.x;
        float dt = c.position.y;
        float db = WINDOW_HEIGHT - c.position.y;
        if (dl < detectionDistance) avoid.x += (detectionDistance - dl)/detectionDistance * maxAcceleration;
        if (dr < detectionDistance) avoid.x -= (detectionDistance - dr)/detectionDistance * maxAcceleration;
        if (dt < detectionDistance) avoid.y += (detectionDistance - dt)/detectionDistance * maxAcceleration;
        if (db < detectionDistance) avoid.y -= (detectionDistance - db)/detectionDistance * maxAcceleration;
        out.linear = avoid;
        return out;
    }
};

// Wander behaviors
class Wander : public SteeringBehavior {
    float wanderOffset, wanderRadius, wanderRate, wanderOrientation, maxAcceleration;
    LookWhereYoureGoing lwg;
public:
    Wander(float offset=60.f, float radius=40.f, float rate=0.5f, float maxA=80.f)
        : wanderOffset(offset), wanderRadius(radius), wanderRate(rate), wanderOrientation(0.f), maxAcceleration(maxA) {}
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override {
        wanderOrientation += randomBinomial() * wanderRate;
        float targetOrientation = wanderOrientation + c.orientation;
        sf::Vector2f orientationVec(std::cos(c.orientation), std::sin(c.orientation));
        sf::Vector2f circleCenter = c.position + orientationVec * wanderOffset;
        sf::Vector2f wanderTarget(circleCenter.x + wanderRadius * std::cos(targetOrientation),
                                  circleCenter.y + wanderRadius * std::sin(targetOrientation));
        SteeringOutput out;
        out.linear = wanderTarget - c.position;
        float mag = std::sqrt(out.linear.x*out.linear.x + out.linear.y*out.linear.y);
        if (mag > 0.f) out.linear = (out.linear / mag) * maxAcceleration;
        SteeringOutput look = lwg.calculateSteering(c, t);
        out.angular = look.angular;
        return out;
    }
};

class WanderKinematic : public SteeringBehavior {
    float wanderOffset, wanderRadius, wanderRate, wanderOrientation, maxAcceleration, maxRotation;
public:
    WanderKinematic(float offset=60.f, float radius=40.f, float rate=0.5f, float maxA=80.f, float maxR=2.f)
        : wanderOffset(offset), wanderRadius(radius), wanderRate(rate), wanderOrientation(0.f), maxAcceleration(maxA), maxRotation(maxR) {}
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic & /*t*/) override {
        wanderOrientation += randomBinomial() * wanderRate;
        float targetOrientation = wanderOrientation + c.orientation;
        sf::Vector2f orientationVec(std::cos(c.orientation), std::sin(c.orientation));
        sf::Vector2f circleCenter = c.position + orientationVec * wanderOffset;
        sf::Vector2f wanderTarget(circleCenter.x + wanderRadius * std::cos(targetOrientation),
                                  circleCenter.y + wanderRadius * std::sin(targetOrientation));
        SteeringOutput out;
        out.linear = wanderTarget - c.position;
        float mag = std::sqrt(out.linear.x*out.linear.x + out.linear.y*out.linear.y);
        if (mag > 0.f) out.linear = (out.linear / mag) * maxAcceleration;
        sf::Vector2f toTarget = wanderTarget - c.position;
        float desiredOrientation = std::atan2(toTarget.y, toTarget.x);
        float rotation = mapToRange(desiredOrientation - c.orientation);
        out.angular = rotation * 3.f;
        if (std::abs(out.angular) > maxRotation) out.angular = (out.angular / std::abs(out.angular)) * maxRotation;
        return out;
    }
};

// ---------------- flocking behaviors ----------------
class Separation : public SteeringBehavior {
    float threshold;
    float decayCoefficient;
    float maxAcceleration;
    std::vector<Kinematic*> *boids;
public:
    Separation(std::vector<Kinematic*> *b, float thresh=100.f, float decay=5000.f, float maxA=100.f)
        : threshold(thresh), decayCoefficient(decay), maxAcceleration(maxA), boids(b) {}
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic & /*t*/) override {
        SteeringOutput out;
        for (auto b : *boids) {
            if (b->position == c.position) continue;
            sf::Vector2f direction = c.position - b->position;
            float dist = std::sqrt(direction.x*direction.x + direction.y*direction.y);
            if (dist < threshold && dist > 0.f) {
                float strength = std::min(decayCoefficient / (dist*dist), maxAcceleration);
                direction = (direction / dist) * strength;
                out.linear += direction;
            }
        }
        float mag = std::sqrt(out.linear.x*out.linear.x + out.linear.y*out.linear.y);
        if (mag > maxAcceleration) out.linear = (out.linear / mag) * maxAcceleration;
        return out;
    }
};

class Cohesion : public SteeringBehavior {
    float neighborhoodRadius;
    float maxAcceleration;
    std::vector<Kinematic*> *boids;
    Arrive arrive;
public:
    Cohesion(std::vector<Kinematic*> *b, float radius=150.f, float maxA=10.f)
        : neighborhoodRadius(radius), maxAcceleration(maxA), boids(b), arrive(maxA, 100.f, 10.f, 50.f) {}
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic & /*t*/) override {
        sf::Vector2f center(0.f, 0.f);
        int count = 0;
        for (auto b : *boids) {
            if (b->position == c.position) continue;
            sf::Vector2f dir = b->position - c.position;
            float dist = std::sqrt(dir.x*dir.x + dir.y*dir.y);
            if (dist < neighborhoodRadius) { center += b->position; ++count; }
        }
        if (count == 0) return {};
        center /= static_cast<float>(count);
        Kinematic target; target.position = center;
        return arrive.calculateSteering(c, target);
    }
};

class Alignment : public SteeringBehavior {
    float neighborhoodRadius;
    float maxAcceleration;
    std::vector<Kinematic*> *boids;
    VelocityMatching velocityMatch;
public:
    Alignment(std::vector<Kinematic*> *b, float radius=100.f, float maxA=50.f)
        : neighborhoodRadius(radius), maxAcceleration(maxA), boids(b), velocityMatch(maxA) {}
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic & /*t*/) override {
        sf::Vector2f avg(0.f, 0.f);
        int count = 0;
        for (auto b : *boids) {
            if (b->position == c.position) continue;
            sf::Vector2f dir = b->position - c.position;
            float dist = std::sqrt(dir.x*dir.x + dir.y*dir.y);
            if (dist < neighborhoodRadius) { avg += b->velocity; ++count; }
        }
        if (count == 0) return {};
        avg /= static_cast<float>(count);
        Kinematic target; target.velocity = avg;
        return velocityMatch.calculateSteering(c, target);
    }
};

// Blended steering
class BlendedSteering : public SteeringBehavior {
    struct BW { SteeringBehavior *behavior; float weight; };
    std::vector<BW> behaviors;
    float maxAcceleration;
    float maxAngular;
public:
    BlendedSteering(float maxA=200.f, float maxAng=5.f) : maxAcceleration(maxA), maxAngular(maxAng) {}
    void addBehavior(SteeringBehavior *b, float w) { behaviors.push_back({b,w}); }
    SteeringOutput calculateSteering(const Kinematic &c, const Kinematic &t) override {
        SteeringOutput out;
        for (auto &bw : behaviors) {
            SteeringOutput s = bw.behavior->calculateSteering(c,t);
            out.linear += s.linear * bw.weight;
            out.angular += s.angular * bw.weight;
        }
        float mag = std::sqrt(out.linear.x*out.linear.x + out.linear.y*out.linear.y);
        if (mag > maxAcceleration) out.linear = (out.linear / mag) * maxAcceleration;
        if (std::abs(out.angular) > maxAngular) out.angular = (out.angular / std::abs(out.angular)) * maxAngular;
        return out;
    }
};

// ---------------- Character class (full) ----------------
class Character {
    Kinematic kinematic;
    Breadcrumb breadcrumbs;
    sf::Texture texture;
    std::unique_ptr<sf::Sprite> sprite; // construct after texture
    SteeringBehavior *currentBehavior;
    float maxSpeed;
    float maxRotation;
public:
    Character(sf::Vector2f start = {400.f,300.f}, sf::Color color = sf::Color::Red,
              int bcmax = 30, int bcint = 5)
        : breadcrumbs(bcmax, bcint, color), currentBehavior(nullptr), maxSpeed(150.f), maxRotation(3.f)
    {
        kinematic.position = start;
        kinematic.orientation = 0.f;

        // try load a real texture, otherwise build fallback
        if (!texture.loadFromFile("boid.png")) {
            sf::Image img({32u, 32u}, sf::Color::Transparent);
            for (unsigned y=0;y<32;++y) for (unsigned x=0;x<32;++x) img.setPixel({x,y}, sf::Color(200,200,200));
            (void)texture.loadFromImage(img);
        }

        sprite = std::make_unique<sf::Sprite>(texture);
        sprite->setOrigin({ static_cast<float>(texture.getSize().x) / 2.f,
                            static_cast<float>(texture.getSize().y) / 2.f });
        sprite->setScale({0.05f, 0.05f});
        sprite->setColor(sf::Color::White);
    }

    void setBehavior(SteeringBehavior *b) { currentBehavior = b; }
    Kinematic &getKinematic() { return kinematic; }
    void clearBreadcrumbs() { breadcrumbs.clear(); }
    void setMaxSpeed(float s) { maxSpeed = s; }
    void setPosition(sf::Vector2f p) { kinematic.position = p; sprite->setPosition(p); }

    void update(float dt, const Kinematic &target) {
        if (currentBehavior) {
            SteeringOutput s = currentBehavior->calculateSteering(kinematic, target);
            kinematic.position += kinematic.velocity * dt;
            kinematic.orientation += kinematic.rotation * dt;
            kinematic.velocity += s.linear * dt;
            kinematic.rotation += s.angular * dt;

            float speed = std::sqrt(kinematic.velocity.x*kinematic.velocity.x + kinematic.velocity.y*kinematic.velocity.y);
            if (speed > maxSpeed) kinematic.velocity = (kinematic.velocity / speed) * maxSpeed;

            if (std::abs(kinematic.rotation) > maxRotation)
                kinematic.rotation = (kinematic.rotation / std::abs(kinematic.rotation)) * maxRotation;
        } else {
            kinematic.position += kinematic.velocity * dt;
            kinematic.orientation += kinematic.rotation * dt;
        }

        kinematic.orientation = mapToRange(kinematic.orientation);
        breadcrumbs.update(kinematic.position);
        sprite->setPosition(kinematic.position);
        sprite->setRotation(sf::degrees(kinematic.orientation * 180.f / PI));
    }

    void updateWithBoundaryHandling(float dt, const Kinematic &target) {
        if (currentBehavior) {
            SteeringOutput s = currentBehavior->calculateSteering(kinematic, target);
            kinematic.position += kinematic.velocity * dt;
            kinematic.orientation += kinematic.rotation * dt;
            kinematic.velocity += s.linear * dt;
            kinematic.rotation += s.angular * dt;

            float speed = std::sqrt(kinematic.velocity.x*kinematic.velocity.x + kinematic.velocity.y*kinematic.velocity.y);
            if (speed > maxSpeed) kinematic.velocity = (kinematic.velocity / speed) * maxSpeed;

            if (std::abs(kinematic.rotation) > maxRotation)
                kinematic.rotation = (kinematic.rotation / std::abs(kinematic.rotation)) * maxRotation;
        } else {
            kinematic.position += kinematic.velocity * dt;
            kinematic.orientation += kinematic.rotation * dt;
        }

        const float margin = 4.f;
        if (kinematic.position.x < margin) { kinematic.position.x = margin; kinematic.velocity.x = std::abs(kinematic.velocity.x) * 0.8f; }
        if (kinematic.position.x > WINDOW_WIDTH - margin) { kinematic.position.x = WINDOW_WIDTH - margin; kinematic.velocity.x = -std::abs(kinematic.velocity.x) * 0.8f; }
        if (kinematic.position.y < margin) { kinematic.position.y = margin; kinematic.velocity.y = std::abs(kinematic.velocity.y) * 0.8f; }
        if (kinematic.position.y > WINDOW_HEIGHT - margin) { kinematic.position.y = WINDOW_HEIGHT - margin; kinematic.velocity.y = -std::abs(kinematic.velocity.y) * 0.8f; }

        kinematic.orientation = mapToRange(kinematic.orientation);
        breadcrumbs.update(kinematic.position);
        sprite->setPosition(kinematic.position);
        sprite->setRotation(sf::degrees(kinematic.orientation * 180.f / PI));
    }

    void draw(sf::RenderWindow &win) {
        breadcrumbs.draw(win);
        win.draw(*sprite);
    }
};

// ---------------- Boid class ----------------
class Boid {
public:
    Kinematic kinematic;
    Breadcrumb breadcrumbs;
    sf::Texture texture;
    std::unique_ptr<sf::Sprite> sprite;
    BlendedSteering *flockingBehavior; // assigned externally
    float maxSpeed;

    Boid(sf::Vector2f start, sf::Color color = sf::Color::Blue)
        : breadcrumbs(20, 10, color), flockingBehavior(nullptr), maxSpeed(200.f)
    {
        kinematic.position = start;
        kinematic.velocity = sf::Vector2f(randomFloat(-50.f, 50.f), randomFloat(-50.f, 50.f));
        kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);

        if (!texture.loadFromFile("boid-sm.png")) {
            sf::Image img({16u,16u}, sf::Color::Transparent);
            for (unsigned y=0;y<16;++y) for (unsigned x=0;x<16;++x) img.setPixel({x,y}, sf::Color(180,180,180));
            (void)texture.loadFromImage(img);
        }

        sprite = std::make_unique<sf::Sprite>(texture);
        sprite->setOrigin({ static_cast<float>(texture.getSize().x)/2.f, static_cast<float>(texture.getSize().y)/2.f });
        sprite->setScale({1.5f, 1.5f});
        sprite->setColor(color);
    }

    void update(float dt) {
        if (flockingBehavior) {
            SteeringOutput s = flockingBehavior->calculateSteering(kinematic, kinematic);
            kinematic.position += kinematic.velocity * dt;
            kinematic.velocity += s.linear * dt;
            float speed = std::sqrt(kinematic.velocity.x*kinematic.velocity.x + kinematic.velocity.y*kinematic.velocity.y);
            if (speed > maxSpeed) kinematic.velocity = (kinematic.velocity / speed) * maxSpeed;
            if (speed > 0.01f) kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
        } else {
            kinematic.position += kinematic.velocity * dt;
            if (std::sqrt(kinematic.velocity.x*kinematic.velocity.x + kinematic.velocity.y*kinematic.velocity.y) > 0.01f)
                kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
        }

        // wrap
        if (kinematic.position.x < 0.f) kinematic.position.x = WINDOW_WIDTH;
        if (kinematic.position.x > WINDOW_WIDTH) kinematic.position.x = 0.f;
        if (kinematic.position.y < 0.f) kinematic.position.y = WINDOW_HEIGHT;
        if (kinematic.position.y > WINDOW_HEIGHT) kinematic.position.y = 0.f;

        breadcrumbs.update(kinematic.position);
        sprite->setPosition(kinematic.position);
        sprite->setRotation(sf::degrees(kinematic.orientation * 180.f / PI));
    }

    void draw(sf::RenderWindow &win) {
        breadcrumbs.draw(win);
        win.draw(*sprite);
    }
};

// ---------------- main ----------------
int main() {
    sf::RenderWindow window(sf::VideoMode(sf::Vector2u(WINDOW_WIDTH, WINDOW_HEIGHT)),
                            "Steering Behaviors Demo (SFML 3.0.0)");
    window.setFramerateLimit(60);

    sf::Clock clock;
    int currentMode = 1;

    // create behaviors to reuse
    FastVelocityMatching fastVelMatch;
    Arrive arriveFast(500.f, 500.f, 20.f, 150.f, 0.12f); // example
    Arrive arriveSlow(200.f, 200.f, 10.f, 150.f, 0.08f);
    ArriveAndAlign arriveAndAlign; // simple combined
    Wander wanderSmooth(60.f, 40.f, 0.6f, 60.f);
    WanderKinematic wanderKin1(60.f, 40.f, 0.6f, 60.f, 2.5f);
    WanderKinematic wanderKin2(80.f, 60.f, 1.2f, 80.f, 3.0f);
    WallAvoidance wallAvoid;

    BlendedSteering wanderWithWalls1(200.f, 5.f);
    wanderWithWalls1.addBehavior(&wanderSmooth, 1.f);
    wanderWithWalls1.addBehavior(&wallAvoid, 2.f);

    BlendedSteering wanderWithWalls2(200.f, 5.f);
    wanderWithWalls2.addBehavior(&wanderKin1, 1.f);
    wanderWithWalls2.addBehavior(&wallAvoid, 2.f);

    BlendedSteering wanderWithWalls3(200.f, 5.f);
    wanderWithWalls3.addBehavior(&wanderKin2, 1.f);
    wanderWithWalls3.addBehavior(&wallAvoid, 2.f);

    // Characters
    Character velMatchChar({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}, sf::Color(128,0,0,255));
    Character cyanAlignChar({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}, sf::Color(0,0,128,255));
    Character yellowArriveChar({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}, sf::Color(128,128,0,255));

    // Wander sets
    std::vector<std::unique_ptr<Character>> wanderSet1, wanderSet2, wanderSet3;
    auto initWander = [&]() {
        wanderSet1.clear(); wanderSet2.clear(); wanderSet3.clear();
        float cx = WINDOW_WIDTH/2.f, cy = WINDOW_HEIGHT/2.f;
        for (int i=0;i<3;++i) {
            auto a = std::make_unique<Character>(sf::Vector2f(cx + randomFloat(-100.f,100.f), cy + randomFloat(-100.f,100.f)), sf::Color::Blue);
            auto b = std::make_unique<Character>(sf::Vector2f(cx + randomFloat(-100.f,100.f), cy + randomFloat(-100.f,100.f)), sf::Color::Magenta);
            auto c = std::make_unique<Character>(sf::Vector2f(cx + randomFloat(-100.f,100.f), cy + randomFloat(-100.f,100.f)), sf::Color::Green);
            a->getKinematic().orientation = randomFloat(-PI, PI);
            b->getKinematic().orientation = randomFloat(-PI, PI);
            c->getKinematic().orientation = randomFloat(-PI, PI);
            a->setMaxSpeed(80.f); b->setMaxSpeed(80.f); c->setMaxSpeed(80.f);
            wanderSet1.push_back(std::move(a));
            // wanderSet2.push_back(std::move(b)); // original left this unused
            wanderSet3.push_back(std::move(c));
        }
    };

    // Flocking
    std::vector<std::unique_ptr<Boid>> flock;
    std::vector<Kinematic*> flockKinematics;
    auto initFlocking = [&]() {
        flock.clear(); flockKinematics.clear();
        int numBoids = 13;
        for (int i=0;i<numBoids;++i) {
            sf::Vector2f p(randomFloat(100.f, WINDOW_WIDTH-100.f), randomFloat(100.f, WINDOW_HEIGHT-100.f));
            sf::Color color = (i%3==0) ? sf::Color::Cyan : (i%3==1) ? sf::Color::Magenta : sf::Color::Yellow;
            flock.push_back(std::make_unique<Boid>(p, color));
            flockKinematics.push_back(&flock.back()->kinematic);
        }
        // give each boid a blended steering behavior
        for (auto &b : flock) {
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

    // Mouse target tracking
    Kinematic mouseTarget;
    sf::Vector2i mposI = sf::Mouse::getPosition(window);
    sf::Vector2f lastMousePos(static_cast<float>(mposI.x), static_cast<float>(mposI.y));
    sf::Clock mouseClock;
    Breadcrumb mouseBreadcrumbs(40, 3, sf::Color::White);

    // Text / font
    sf::Font font;
    if (!font.openFromFile("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf")) {
        std::cerr << "Font load failed; text may be empty\n";
    }
    sf::Text modeText(font, "Mode 1", 16);
    modeText.setPosition({10.f, 10.f});
    modeText.setFillColor(sf::Color::White);

    // reset helper
    auto resetCase = [&](int mode) {
        currentMode = mode;
        if (mode == 1) { velMatchChar.setPosition({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}); velMatchChar.clearBreadcrumbs(); }
        if (mode == 2) { cyanAlignChar.setPosition({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}); yellowArriveChar.setPosition({WINDOW_WIDTH/2.f, WINDOW_HEIGHT/2.f}); cyanAlignChar.clearBreadcrumbs(); yellowArriveChar.clearBreadcrumbs(); }
        if (mode == 3) initWander();
        if (mode == 4) initFlocking();
    };

    // main loop
    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        if (dt <= 0.f) dt = 0.016f;

        // events (SFML 3)
        while (true) {
            std::optional<sf::Event> opt = window.pollEvent();
            if (!opt) break;
            const sf::Event &event = *opt;

            if (event.is<sf::Event::Closed>()) {
                window.close();
            } else if (event.is<sf::Event::KeyPressed>()) {
                const auto *kp = event.getIf<sf::Event::KeyPressed>();
                if (kp) {
                    if (kp->scancode == sf::Keyboard::Scancode::Escape) window.close();
                    if (kp->scancode == sf::Keyboard::Scancode::Num1) resetCase(1);
                    if (kp->scancode == sf::Keyboard::Scancode::Num2) resetCase(2);
                    if (kp->scancode == sf::Keyboard::Scancode::Num3) resetCase(3);
                    if (kp->scancode == sf::Keyboard::Scancode::Num4) resetCase(4);
                }
            } else if (event.is<sf::Event::MouseButtonPressed>()) {
                const auto *mb = event.getIf<sf::Event::MouseButtonPressed>();
                if (mb && currentMode == 2) {
                    if (mb->button == sf::Mouse::Button::Left) {
                        mouseTarget.position = sf::Vector2f(static_cast<float>(mb->position.x), static_cast<float>(mb->position.y));
                        mouseTarget.orientation = randomFloat(-PI, PI);
                    }
                }
            }
        }

        // update + render by mode
        window.clear(sf::Color(30, 30, 40));

        switch (currentMode) {
            case 1: {
                modeText.setString("Case 1: Velocity Matching (Mouse)");
                sf::Vector2i mouseInt = sf::Mouse::getPosition(window);
                sf::Vector2f mousePos(static_cast<float>(mouseInt.x), static_cast<float>(mouseInt.y));
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
                targetShape.setOrigin({5.f, 5.f});
                targetShape.setPosition(mousePos);
                window.draw(targetShape);
                break;
            }

            case 2: {
                modeText.setString("Case 2: Arrive + Align (Mouse)");
                Arrive quickArrive(500.f, 500.f, 20.f, 150.f, 0.12f);
                Arrive slowArrive(200.f, 200.f, 10.f, 150.f, 0.08f);
                cyanAlignChar.setBehavior(&quickArrive);
                yellowArriveChar.setBehavior(&slowArrive);
                cyanAlignChar.update(dt, mouseTarget);
                yellowArriveChar.update(dt, mouseTarget);
                LookWhereYoureGoing lwg;
                SteeringOutput s1 = lwg.calculateSteering(cyanAlignChar.getKinematic(), mouseTarget);
                SteeringOutput s2 = lwg.calculateSteering(yellowArriveChar.getKinematic(), mouseTarget);
                cyanAlignChar.getKinematic().rotation += s1.angular * dt;
                cyanAlignChar.getKinematic().orientation += cyanAlignChar.getKinematic().rotation * dt;
                yellowArriveChar.getKinematic().rotation += s2.angular * dt;
                yellowArriveChar.getKinematic().orientation += yellowArriveChar.getKinematic().rotation * dt;
                cyanAlignChar.draw(window);
                yellowArriveChar.draw(window);
                break;
            }

            case 3: {
                modeText.setString("Case 3: Wander (variants)");
                for (auto &c : wanderSet1) {
                    c->setBehavior(&wanderWithWalls1);
                    c->updateWithBoundaryHandling(dt, c->getKinematic());
                    c->draw(window);
                }
                for (auto &c : wanderSet2) {
                    c->setBehavior(&wanderWithWalls2);
                    c->updateWithBoundaryHandling(dt, c->getKinematic());
                    c->draw(window);
                }
                for (auto &c : wanderSet3) {
                    c->setBehavior(&wanderWithWalls3);
                    c->updateWithBoundaryHandling(dt, c->getKinematic());
                    c->draw(window);
                }
                break;
            }

            case 4: {
                modeText.setString("Case 4: Reynolds Boids");
                for (auto &b : flock) {
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
